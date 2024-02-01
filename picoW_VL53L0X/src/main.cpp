
#include <SPI.h>
#include <Adafruit_TCS34725.h>
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "commands.h"

#define TOF_SDA 16
#define TOF_SCL 17
#define RGB_SDA 2
#define RGB_SCL 3

#define BASE_PIN   9
#define ARM_PIN   11
#define FARM_PIN  13
#define CLAW_PIN  15
#define SQUARE(x) (x*x)
#define BUFFER_MAX_SIZE 20
VL53L0X tof;
float distance, prev_distance;

#include "robot.h"

int LED_state;
unsigned long interval;
unsigned long currentMicros, previousMicros;
int loop_count;
float speed = 15;
float CurrPos[3];
float targetPos[3];
float sensorPos[3];
float GreenPos[3];
float RedPos[3];
float BluePos[3];
float initpos[3];
int targetComplete = 0;
int numTargets = 0;
int cycle;
int mode;
bool reached, c_open, start, ok;
int color;
float rMin, thetaMin;
bool hasMin;
float defaut[3];

typedef struct{  
  String command = "";
  float value[12][3] = {{0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}};
  // float value[4] = {0,0,0,0};
  boolean on = false;
}serialCommand_t;
typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_1X);
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}
void getRawData_noDelay(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  *c = tcs.read16(TCS34725_CDATAL);
  *r = tcs.read16(TCS34725_RDATAL);
  *g = tcs.read16(TCS34725_GDATAL);
  *b = tcs.read16(TCS34725_BDATAL);
}

fsm_t fsm1, fsm2, fsm3, fsm4, fsm5;

serialCommand_t serialCommand;

String buff;
int dataProcessed = 0;

void processChar(char b);
void setPosition(void);
void clearLastPos(void);
void setBasic(void);

void setup() 
{
  robot.setServos();
  robot.base.attach(BASE_PIN, 530, 2400);   // Base Servo 760(Left -60º) - 2225(Right 60º)
  robot.arm.attach(ARM_PIN, 970, 2450);     // Arm Servo 970(Backward 120º) - 2440(Forward 0º)
  robot.farm.attach(FARM_PIN, 870, 2100);   // Forearm Servo 2070(Downward 0º) - 910(Upward 90º)
  robot.claw.attach(CLAW_PIN, 560, 1630);   // Claw Servo 560 (Opened) - 1630(closed)
  robot.openclaw();
  cycle=40;
  interval = 40*1000;
  mode=0;
  color=0;
  c_open=true;
  reached=false;
  start=false;
  rMin = 1000;
  thetaMin = 1000;
  hasMin = false;
  ok = false;
  Serial.begin(115200);
 

  Wire.setSDA(TOF_SDA);
  Wire.setSCL(TOF_SCL);

  Wire.begin();

  Wire1.setSDA(RGB_SDA);  // Connect TCS34725 SDA to gpio 20
  Wire1.setSCL(RGB_SCL);  // Connect TCS34725 SCL to gpio 21

  Wire1.begin();

  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  } 
   

  while (!tcs.begin(TCS34725_ADDRESS, &Wire1)) {
    Serial.println("No TCS34725 found ... check your connections");
    delay(500);
  }
  // Start new distance measure
  tof.startReadRangeMillimeters();  
  targetPos[0]=robot.rel_x;
	targetPos[1]=robot.rel_y;
	targetPos[2]=robot.rel_z;
  // sensorPos[0]=36.00;
  // sensorPos[1]=-86.00;
  // sensorPos[2]=82.00;
  sensorPos[0]=4.00;
  sensorPos[1]=-62.00;
  sensorPos[2]=117.00;
  RedPos[0]=58.00;
  RedPos[1]=-54.00;
  RedPos[2]=110.00;
  GreenPos[0]=47.00;
  GreenPos[1]=-84.00;
  GreenPos[2]=110.00;
  BluePos[0]=76.00;
  BluePos[1]=-97.00;
  BluePos[2]=95.00;
  initpos[0]=80;
  initpos[1]=0;
  initpos[2]=138;
  defaut[0]=80;
  defaut[1]=0;
  defaut[2]=115;
  // Start all state machines
  set_state(fsm1, 0);
  set_state(fsm2, 0);
  set_state(fsm3, 0);
  set_state(fsm4, 0);
  set_state(fsm5, 0);
 
  
}


void loop() 
{

  
  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();    
    Serial.write(b);

    if(b == '\t') {
      serialCommand.on = !serialCommand.on;
      targetPos[0] = 80;
      targetPos[1] = 0;
      targetPos[2] = 138;
      }

    if(!serialCommand.on) robot.sendCommand(b);
    else if(b != '\t') processChar(b);
    // robot.sendCommand(b);
    
  } 
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    uint16_t r, g, b, c, colorTemp, lux;
    //tcs.getRawData(&r, &g, &b, &c);
      getRawData_noDelay(&r, &g, &b, &c);
      // colorTemp = tcs.calculateColorTemperature(r, g, b);
      colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
      lux = tcs.calculateLux(r, g, b);

    // if(serialCommand.command.equals("op1") || (serialCommand.command.equals("go") && dataProcessed == 0)){
    //   if(robot.moveToTarget(targetPos,speed,cycle)) {
    //     if(numTargets > 0)
    //       targetComplete++;
    //     setPosition();
    //   }
    // }
    // else if(serialCommand.command.equals("delete")) if(numTargets >= 1) clearLastPos();

    previousMicros = currentMicros;

    if (tof.readRangeAvailable()) {
      prev_distance = distance;
      distance = tof.readRangeMillimeters() * 1e-3;
      distance = distance - 0.032;
    }
 
    // Start new distance measure
    tof.startReadRangeMillimeters(); 

    // Toggle builtin LED    
    loop_count++;
    if (loop_count > 5) {
      LED_state = !LED_state;
     // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, LED_state);
      loop_count = 0;
    }
    unsigned long cur_time = millis();   // Just one call to millis()

    fsm1.tis = cur_time - fsm1.tes;
    fsm2.tis = cur_time - fsm2.tes;
    fsm3.tis = cur_time - fsm3.tes;
    fsm4.tis = cur_time - fsm4.tes;
    fsm5.tis = cur_time - fsm5.tes;


    //Mode State machine 1
    if(fsm1.state==0 && serialCommand.command.equals("1")){
      
      fsm1.new_state=1;
    }
    else if(fsm1.state==0 && serialCommand.command.equals("2")){
      fsm1.new_state=2;
    }
    else if(fsm1.state==0 && serialCommand.command.equals("3")){
      fsm1.new_state=3;
    }
    else if((fsm1.state==1 && serialCommand.command.equals("end"))||(fsm1.state==2 && serialCommand.command.equals("end"))||(fsm1.state==3 && serialCommand.command.equals("end"))){
      fsm1.new_state=0;
    }

    //Basic Mode State machine 2
    if(fsm2.state==0 && (fsm1.state==1 || fsm1.state == 2)){
      fsm2.new_state=1;
    }
    else if(fsm2.state==1 && start){
      if(fsm1.state == 2){
        setBasic();
      }
      setPosition();
      fsm2.new_state=2;
    }
    else if(fsm2.state== 1 && (serialCommand.command.equals("stop") || fsm1.state==0)){
      fsm2.new_state=0;
    }
    else if(fsm2.state==2 && fsm3.state==5){
      targetComplete = 0;
      fsm2.new_state=1;
    }
    
    // //Basic Mode State machine 6
    // if(fsm2.state==0 && fsm1.state==1){
    //   fsm2.new_state=1;
    // }
    // else if(fsm2.state==1 && start){
    //   if(fsm1.state == 2){
    //     setBasic();
    //   }
    //   setPosition();
    //   fsm2.new_state=2;
    // }
    // else if(fsm2.state== 1 && (serialCommand.command.equals("stop") || fsm1.state==0)){
    //   fsm2.new_state=0;
    // }
    // else if(fsm2.state==2 && fsm3.state==5){
    //   targetComplete = 0;
    //   fsm2.new_state=1;
    // }

    //Move State machine 3
    if(fsm3.state==0 && fsm2.state==2){
      fsm3.new_state=1;
    }
    else if(fsm3.state==1 && reached){
      fsm3.new_state=2;
      reached=false;
    }
    else if(fsm3.state== 2 && !c_open){   // We need to change this later
      fsm3.new_state=3;
    }
    else if(fsm3.state==3 && fsm4.state==7 ){ 
      fsm3.new_state=4;
    }
    else if(fsm3.state==4 && targetComplete< numTargets){
      fsm3.new_state=1;
    }
    else if(fsm3.state==4 && targetComplete>= numTargets){
      fsm3.new_state=5;
    }
    else if(fsm3.state==5){
      fsm3.new_state=0;
    }

    //Piece State machine 4
    if(fsm4.state==0 && fsm3.state==3){
      fsm4.new_state=1;
    }
    else if(fsm4.state==1 && reached){
      fsm4.new_state=2;
      reached=false;
      ok = false;
    }
    else if(fsm4.state== 2 && color==1){
      fsm4.new_state=3;
    }
    else if(fsm4.state==2 && color==2){
      fsm4.new_state=4;
    }
    else if(fsm4.state==2 && color==3){
      fsm4.new_state=5;
    }
    else if((fsm4.state==3 && reached) || (fsm4.state==4 && reached) || (fsm4.state==5 && reached)){
      fsm4.new_state=6;
      reached=false;
    }
    else if(fsm4.state==6 && c_open){
      fsm4.new_state=7;
    }
    else if(fsm4.state==7){
      fsm4.new_state=0;
    }

    //Piece State machine 5
    if(fsm5.state == 0 && fsm1.state == 3){
      fsm5.new_state = 1;
    }
    else if(fsm5.state == 1 && robot.ang_base <= radians(20) && robot.rel_z >= 110){
      ok = false;
      fsm5.new_state = 2;
    }
    else if(fsm5.state == 2 && robot.ang_base >= radians(140) && hasMin){
      robot.rad2pos(rMin, thetaMin, targetPos);
      Serial.print("targetPos[0 1 2]: "); Serial.print(targetPos[0]); Serial.print(" "); 
      Serial.print(targetPos[1]); Serial.print(" "); 
      Serial.println(targetPos[2]); Serial.print("]"); 
      hasMin = false;
      fsm5.new_state = 3;
    }
    else if(fsm5.state == 2 && robot.ang_base >= radians(140) && !hasMin){
      fsm5.new_state == 0;
    }
    else if(fsm5.state == 3 && reached){
      reached = false;
      fsm5.new_state = 4;
    }
    else if(fsm5.state == 4 && fsm5.tis >= 500 && robot.pwm_claw >= 1190){
      fsm5.new_state = 5;
    }
    else if(fsm5.state == 5 && color == 1 && reached){
      reached = false;
      fsm5.new_state = 6;
    }
    else if(fsm5.state == 5 && color == 2 && reached){
      reached = false;
      fsm5.new_state = 7;
    }
    else if(fsm5.state == 5 && color == 3 && reached){
      reached = false;
      fsm5.new_state = 8;
    }
    else if(fsm5.state == 6 && reached){
      reached = false;
      fsm5.new_state = 9;
    }
    else if(fsm5.state == 7 && reached){
      reached = false;
      fsm5.new_state = 9;
    }
    else if(fsm5.state == 8 && reached){
      reached = false;
      fsm5.new_state = 9;
    }
    else if(fsm5.state == 9 && c_open){
      ok = false;
      fsm5.new_state = 1;
    }

    set_state(fsm1, fsm1.new_state);
    set_state(fsm2, fsm2.new_state);
    set_state(fsm3, fsm3.new_state);
    set_state(fsm4, fsm4.new_state);
    set_state(fsm5, fsm5.new_state);

    //Actions
    if(fsm1.state==0){
    }
    //fsm2
    if(fsm2.state==0){}
    else if(fsm2.state==1){//save positions
      // clearLastPos();
      start = false;
      targetComplete = 0;
      robot.moveToTarget(initpos, speed, cycle);
    }
    else if(fsm2.state==2){//move sm
    }
    //fsm3
    if(fsm3.state==0){}
    else if(fsm3.state==1){
      // Serial.println("Entering moveToTarget");
      reached=robot.moveToTarget(targetPos, speed, cycle);
      // Serial.print("Reached: ");
      // Serial.println(reached);
      if(reached) {
        targetComplete++;
        if(targetComplete < numTargets) setPosition();
      }
    }
    else if(fsm3.state==2){//closeclaw
      // c_open=robot.Closeclaw();
      robot.step = 22;
      robot.cClaw();
      if(robot.pwm_claw >= 1218) {
        c_open = false;
        robot.pwm_claw = 604;
        }
    }
    else if(fsm3.state==3){//piece sm
    }
    else if(fsm3.state==4){//verify if there is more positions
    }
    else if(fsm3.state==5){//positionsended
    }
    //fsm4
    if(fsm4.state==0){}
    else if(fsm4.state==1){
      //move 2 sensor
      if(!ok) {
        ok = robot.moveToTarget(defaut, speed, cycle);
      }
      else {
        reached=robot.moveToTarget(sensorPos, speed, cycle);
      }
      
    }
    else if(fsm4.state==2){//color defining 
      if(r > 30 || g > 30 || b > 30){
        if(g>b && g>r) color=1;
        else if(b>r && b>g) color = 2;
        else if(r>g && r>b) color = 3;
      }
      if(fsm4.tis >= 10000) color = 3;
    }
    else if(fsm4.state==3){//move to green contentor 
      reached=robot.moveToTarget(GreenPos, speed, cycle);
    }
    else if(fsm4.state==4){//move to blue contentor 
      reached=robot.moveToTarget(BluePos, speed, cycle);
    }
    else if(fsm4.state==5){//move to red contentor 
      reached=robot.moveToTarget(RedPos, speed, cycle);
    }
    else if(fsm4.state==6){//drop piece
      c_open=robot.openclaw();
      color=0;
    }
    else if(fsm4.state==7){//dropped piece
    }

    //fsm5
    if(fsm5.state == 0){}
    if(fsm5.state == 1){
      robot.step = 2;
      if(!ok) {
        ok = robot.moveToTarget(defaut, speed, cycle);
      }
      else{
        robot.turn2Angle(radians(10));
      }
    }
    if(fsm5.state == 2){
      robot.step = 0.25;
      robot.turn2Angle(radians(160));
      if(distance < rMin && robot.ang_base >= radians(65)) {
        rMin = distance;
        thetaMin = robot.ang_base;
        hasMin = true;
      }
      if(rMin >= 0.15) {
        hasMin = false;
        rMin = 1000;
        thetaMin = 1000;
      }
    }
    if(fsm5.state == 3){
      Serial.println("        Get piece");
      reached = robot.moveToTarget(targetPos, speed, cycle);
    }
    if(fsm5.state == 4){
      robot.step = 22;
      robot.cClaw();
      if(robot.pwm_claw >= 1218) {
        c_open = false;
        robot.pwm_claw = 604;
        }
    }
    if(fsm5.state == 5){
      if(!ok) {
        ok = robot.moveToTarget(defaut, speed, cycle);
      }
      else{
        reached = robot.moveToTarget(sensorPos, speed, cycle);
      }
      if(r > 30 || g > 30 || b > 30){
        if(g>b && g>r) color=1;
        else if(b>r && b>g) color = 2;
        else if(r>g && r>b) color = 3;
      }
      if(fsm4.tis >= 10000) color = 3;
    }
    if(fsm5.state == 6){
      reached = robot.moveToTarget(GreenPos, speed, cycle);
    }
    if(fsm5.state == 7){
      reached =robot.moveToTarget(BluePos, speed, cycle);
    }
    if(fsm5.state == 8){
      reached = robot.moveToTarget(RedPos, speed, cycle);
    }
    if(fsm5.state == 9){
      robot.openclaw();
    }


    if(serialCommand.on){
      Serial.print(" cmd: ");
      if(buff.length() > 0) Serial.print(buff);
      Serial.print("  Command: ");
      Serial.print(serialCommand.command);
      if(!serialCommand.command.equals("3")){
        Serial.print("  Pos ");
        Serial.print(numTargets);
        Serial.print(" (x y z): (");
        Serial.printf("%0.1f", serialCommand.value[numTargets][0]); Serial.print(" ");
        Serial.printf("%0.1f", serialCommand.value[numTargets][1]); Serial.print(" ");
        Serial.printf("%0.1f", serialCommand.value[numTargets][2]); Serial.print(")");
      }
    }
    else {

      Serial.print("  mode: ");
      Serial.print(robot.mode);

      // Serial.print("  step: ");
      // Serial.print(robot.step);
      // Serial.print("  base_pwm: ");
      // Serial.print(robot.pwm_base);

      // Serial.print("  arm_pwm: ");
      // Serial.print(robot.pwm_arm);

      // Serial.print("  farm_pwm: ");
      // Serial.print(robot.pwm_farm);

    }
      Serial.print("  tof: ");
      Serial.print(distance, 3);


      // Serial.print("  claw_pwm: ");
      // Serial.print(robot.pwm_claw);
    // Serial.print("  ang(b a f): (");
    // Serial.print((int)robot.rad2Degree(robot.ang_base)); Serial.print(" ");
    // Serial.print((int)robot.rad2Degree(robot.ang_arm)); Serial.print(" ");
    // Serial.print((int)robot.rad2Degree(robot.ang_farm)); Serial.print(")");

    Serial.print("  rel(x y z): (");
    Serial.printf("%0.1f", robot.rel_x); Serial.print(" ");
    Serial.printf("%0.1f", robot.rel_y); Serial.print(" ");
    Serial.printf("%0.1f", robot.rel_z); Serial.print(")");

    Serial.print("  fsm1: ");
    Serial.print(fsm1.state);

    Serial.print("  fsm2: ");
    Serial.print(fsm2.state);

    Serial.print("  fsm3: ");
    Serial.print(fsm3.state);

    Serial.print("  fsm4: ");
    Serial.print(fsm4.state);

    Serial.print("  fsm5: ");
    Serial.print(fsm5.state);

    // Serial.print("  start: ");
    // Serial.print(start);

    // Serial.print("  c_open: ");
    // Serial.print(c_open);

    // Serial.print("  c: ");
    // if (color == 1) Serial.print("g");
    // else if (color == 2) Serial.print("b");
    // else if (color == 3) Serial.print("r");
    // else Serial.print("WTF");

    Serial.print("  (R G B): ("); 
    Serial.print(r, DEC); Serial.print(" ");
    Serial.print(g, DEC); Serial.print(" ");
    Serial.print(b, DEC); Serial.print(")");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");

    // Serial.print("  rMin: ");
    // Serial.print(rMin);

    // Serial.print("  thMin: ");
    // Serial.print(thetaMin);

    Serial.print("  Reach: ");
    Serial.print(reached);

    Serial.println();
  }
  
  


}


void processChar(char b){
  switch (b)
  {
  case 'r':
    dataProcessed = 0;
    Serial.println("Delete");
    buff = "";
    serialCommand.command = "";
    clearLastPos();

    break;

  case '\b':
    dataProcessed = 0;
    Serial.println("Backspace");
    buff = "";
    serialCommand.command = "";
    serialCommand.value[numTargets][0] = 0;
    serialCommand.value[numTargets][1] = 0;
    serialCommand.value[numTargets][2] = 0;

    break;
  
  case '\n':
    Serial.println("Enter");
    Serial.print("Data processed: ");
    Serial.println(dataProcessed);
    if(dataProcessed == 3){
      serialCommand.value[numTargets][dataProcessed-1] = buff.toFloat();
      numTargets++;
      
      if(serialCommand.command.equals("save")){
        Serial.print("Position "); Serial.print(numTargets-1); Serial.print(" saved:(x y z): (");
        Serial.print(serialCommand.value[numTargets-1][0]); Serial.print(" ");
        Serial.print(serialCommand.value[numTargets-1][1]); Serial.print(" ");
        Serial.print(serialCommand.value[numTargets-1][2]); Serial.println(")");
      }
      else if(serialCommand.command.equals("op1")){
        Serial.print("Target set to:(x y z): (");
        Serial.print(serialCommand.value[numTargets-1][0]); Serial.print(" ");
        Serial.print(serialCommand.value[numTargets-1][1]); Serial.print(" ");
        Serial.print(serialCommand.value[numTargets-1][2]); Serial.println(")");
        setPosition();
      }
    }
    if(serialCommand.command.equals("go"))
      start = true;
    // if(dataProcessed != 1)
      // serialCommand.value[dataProcessed-1] = buff.toFloat();
    // targetPos[0] = serialCommand.value[numTargets][0];
    // targetPos[1] = serialCommand.value[numTargets][1];
    // targetPos[2] = serialCommand.value[numTargets][2];
    dataProcessed = 0;
    buff = "";
    break;

  case 0x20:
    if(dataProcessed>0) {buff = ""; break;}
    Serial.print("Space: ");
    serialCommand.command = buff;
    Serial.println(serialCommand.command);
    buff = "";
    dataProcessed++; // This is the address of the data
    // if(serialCommand.command.equals("go"))
      // setPosition();
    break;
  
  case ',':
    if(dataProcessed>2) {buff = ""; break;}
    else if(serialCommand.command.equals("save") && dataProcessed == 0) dataProcessed++;
    // serialCommand.value[dataProcessed-1] = buff.toFloat();
    serialCommand.value[numTargets][dataProcessed-1] = buff.toFloat();
    dataProcessed++;
    buff = "";
    break;

  default:
    buff.concat(b);
    break;
  }
}

void setPosition(void){
  if(targetComplete < numTargets){
    targetPos[0] = serialCommand.value[targetComplete][0];
    targetPos[1] = serialCommand.value[targetComplete][1];
    targetPos[2] = serialCommand.value[targetComplete][2];
    Serial.print("Moving to position: ");
    Serial.print(" x:");
    Serial.print(serialCommand.value[targetComplete][0]);
    Serial.print(" y:");
    Serial.print(serialCommand.value[targetComplete][1]);
    Serial.print(" z:");
    Serial.print(serialCommand.value[targetComplete][2]);
  }
}

void clearLastPos(void){
  if(numTargets >= 1){
    Serial.print("Position "); Serial.print(numTargets-1); Serial.print(" cleared:(x y z): (");
    Serial.print(serialCommand.value[targetComplete][0]); Serial.print(" ");
    Serial.print(serialCommand.value[targetComplete][1]); Serial.print(" ");
    Serial.print(serialCommand.value[targetComplete][2]); Serial.println(")");
    serialCommand.value[numTargets-1][0] = 0;
    serialCommand.value[numTargets-1][1] = 0;
    serialCommand.value[numTargets-1][2] = 0;
    numTargets--;
  }
  else {
    Serial.println("All positions cleared!");
    targetPos[0] = 80;
    targetPos[1] = 0;
    targetPos[2] = 138;
    }
}

void setBasic(){
    serialCommand.value[numTargets][0] = 56;  //a
    serialCommand.value[numTargets][1] = -24;
    serialCommand.value[numTargets][2] = 33;

    numTargets++;
    serialCommand.value[numTargets][0] = 41;  //b
    serialCommand.value[numTargets][1] = 0;
    serialCommand.value[numTargets][2] = 42;

    numTargets++;
    serialCommand.value[numTargets][0] = 54;  //c
    serialCommand.value[numTargets][1] = 24;
    serialCommand.value[numTargets][2] = 38;

    numTargets++;    
    serialCommand.value[numTargets][0] = 88;  //d
    serialCommand.value[numTargets][1] = -26;
    serialCommand.value[numTargets][2] = 33;

    numTargets++;
    serialCommand.value[numTargets][0] = 90;  //e
    serialCommand.value[numTargets][1] = 0;
    serialCommand.value[numTargets][2] = 36;

    numTargets++;
    serialCommand.value[numTargets][0] = 88;  //f
    serialCommand.value[numTargets][1] = 28;
    serialCommand.value[numTargets][2] = 28;

    numTargets++;
    serialCommand.value[numTargets][0] = 95;    //g
    serialCommand.value[numTargets][1] = -27;
    serialCommand.value[numTargets][2] = 7;

    numTargets++;
    serialCommand.value[numTargets][0] = 103;   //h
    serialCommand.value[numTargets][1] = 1;
    serialCommand.value[numTargets][2] = 11;

    numTargets++;
    serialCommand.value[numTargets][0] = 99;   //i
    serialCommand.value[numTargets][1] = 26;
    serialCommand.value[numTargets][2] = 1;

    numTargets++;   
}