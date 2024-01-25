// 2022 Paulo Costa
// Pico W LED access

#include <Arduino.h>
//#include <WiFi.h>

//#include "pico/cyw43_arch.h"

#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>


#define TOF_SDA 16
#define TOF_SCL 17

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
float speed = 20;
float CurrPos[3];
float targetPos[3];
float sensorPos[3];
float GreenPos[3];
float RedPos[3];
float BluePos[3];
int targetComplete = 0;
int numTargets = 0;
int cycle;
int mode;
bool reached, c_open;
int color;
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

void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state chnanged tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

fsm_t fsm1, fsm2, fsm3, fsm4;

serialCommand_t serialCommand;

String buff;
int dataProcessed = 0;

void processChar(char b);
void setPosition(void);
void clearLastPos(void);

void setup() 
{
  robot.setServos();
  robot.base.attach(BASE_PIN, 530, 2400);   // Base Servo 760(Left -60º) - 2225(Right 60º)
  robot.arm.attach(ARM_PIN, 970, 2450);     // Arm Servo 970(Backward 120º) - 2440(Forward 0º)
  robot.farm.attach(FARM_PIN, 870, 2100);   // Forearm Servo 2070(Downward 0º) - 910(Upward 90º)
  robot.claw.attach(CLAW_PIN, 560, 1630);   // Claw Servo 560 (Opened) - 1630(closed)
  cycle=40;
  interval = 40*1000;
  mode=0;
  c_open=true;
  reached=false;
  Serial.begin(115200);
 

  Wire.setSDA(TOF_SDA);
  Wire.setSCL(TOF_SCL);

  Wire.begin();

  tof.setTimeout(500);
  while (!tof.init()) {
    Serial.println(F("Failed to detect and initialize VL53L0X!"));
    delay(100);
  }  

  // Start new distance measure
  tof.startReadRangeMillimeters();  
  targetPos[0]=robot.rel_x;
	targetPos[1]=robot.rel_y;
	targetPos[2]=robot.rel_z;

  // Start all state machines
  set_state(fsm1, 0);
  set_state(fsm2, 0);
  set_state(fsm3, 0);
  set_state(fsm4, 0);
 
  
}


void loop() 
{

  
  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();    
    Serial.write(b);

    if(b == '\t') serialCommand.on = !serialCommand.on;

    if(!serialCommand.on) robot.sendCommand(b);
    else if(b != '\t') processChar(b);
    // robot.sendCommand(b);
    
  } 
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
    

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

    if(serialCommand.on){
      Serial.print(" cmd: ");
      if(buff.length() > 0) Serial.print(buff);
      Serial.print(" Command: ");
      Serial.print(serialCommand.command);
      Serial.print(" Pos:");
      Serial.print(numTargets);
      Serial.print(" x:");
      Serial.print(serialCommand.value[numTargets][0]);
      Serial.print(" y:");
      Serial.print(serialCommand.value[numTargets][1]);
      Serial.print(" z:");
      Serial.print(serialCommand.value[numTargets][2]);
    }
    else {
      Serial.print(" serialOn: ");
      Serial.print(serialCommand.on);

      Serial.print(" tof: ");
      Serial.print(distance, 3);

      Serial.print("  mode: ");
      Serial.print(robot.mode);

      Serial.print("  step: ");
      Serial.print(robot.step);
      // Serial.print("  base_pwm: ");
      // Serial.print(robot.pwm_base);

      // Serial.print("  arm_pwm: ");
      // Serial.print(robot.pwm_arm);

      // Serial.print("  farm_pwm: ");
      // Serial.print(robot.pwm_farm);

      Serial.print("  claw_pwm: ");
      Serial.print(robot.pwm_claw);

      // Serial.print("  ang_b: ");
      // Serial.print(robot.rad2Degree(robot.ang_base));

      // Serial.print("  ang_a: ");
      // Serial.print(robot.rad2Degree(robot.ang_arm));

      // Serial.print("  ang_f: ");
      // Serial.print(robot.rad2Degree(robot.ang_farm));
    }

    Serial.print("  x_rel: ");
    Serial.print(robot.rel_x);

    Serial.print("  y_rel: ");
    Serial.print(robot.rel_y);

    Serial.print("  z_rel: ");
    Serial.print(robot.rel_z);

    Serial.println();
  }
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
  if(fsm2.state==0 && fsm1.state==1){
    fsm2.new_state=1;
  }
  else if(fsm2.state==1 && serialCommand.command.equals("go")){
    fsm2.new_state=2;
  }
  else if(fsm2.state== 1 && serialCommand.command.equals("stop")){
    fsm2.new_state=0;
  }
   else if(fsm2.state==2 && fsm3.state==5){
    fsm1.new_state=1;
  }

  //Move State machine 3
  if(fsm3.state==0 && fsm2.state==2){
    fsm3.new_state=1;
  }
  else if(fsm3.state==1 && reached){
    fsm3.new_state=2;
    reached==false;
  }
  else if(fsm3.state== 2 && !c_open){
    fsm3.new_state=3;
  }
  else if(fsm3.state==3 && fsm4.state==7){
    fsm3.new_state=4;
  }
   else if(fsm3.state==4 && targetComplete<= numTargets){
    fsm3.new_state=1;
  }
  else if(fsm3.state==4 && targetComplete> numTargets){
    fsm3.new_state=5;
  }
  else if(fsm3.state==5){
    fsm3.new_state=0;
  }

  //Piece State machine 4
  if(fsm4.state==0 && fsm3.state==2){
    fsm4.new_state=1;
  }
  else if(fsm4.state==1 && reached){
    fsm4.new_state=2;
    reached=false;
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
   else if(fsm4.state==3 && reached || fsm4.state==4 && reached || fsm4.state==5 && reached){
    fsm4.new_state=6;
    reached=false;
  }
   else if(fsm4.state==6 && c_open){
    fsm4.new_state=7;
  }
   else if(fsm4.state==7){
    fsm4.new_state=0;
  }
  set_state(fsm1, fsm1.new_state);
  set_state(fsm2, fsm2.new_state);
  set_state(fsm3, fsm3.new_state);
  set_state(fsm4, fsm4.new_state);

  //Actions
  if(fsm2.state==0){}
  else if(fsm2.state==1){//save positions
  }
  else if(fsm2.state==2){//move sm
  }
  //fsm3
   if(fsm3.state==0){}
   else if(fsm3.state==1){
    reached=robot.moveToTarget(targetPos, speed, cycle);
  }
  else if(fsm3.state==2){//closeclaw
    c_open=robot.Closeclaw();
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
    reached=robot.moveToTarget(sensorPos, speed, cycle);
    if(reached) {
        if(numTargets > 0)
          targetComplete++;
          setPosition();
      }
  }
  else if(fsm4.state==2){//color defining 
  }
   else if(fsm4.state==3){//move to green contentor 
    robot.moveToTarget(GreenPos, speed, cycle);
  }
   else if(fsm4.state==4){//move to blue contentor 
    robot.moveToTarget(BluePos, speed, cycle);
  }
  else if(fsm4.state==5){//move to red contentor 
    robot.moveToTarget(RedPos, speed, cycle);
  }
  else if(fsm4.state==6){//drop piece
    c_open=robot.openclaw();
  }
  else if(fsm4.state==7){//dropped piece
  }
  


}


void processChar(char b){
  switch (b)
  {
  case '-':
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
      // if(dataProcessed != 1)
        // serialCommand.value[dataProcessed-1] = buff.toFloat();
      // targetPos[0] = serialCommand.value[numTargets][0];
      // targetPos[1] = serialCommand.value[numTargets][1];
      // targetPos[2] = serialCommand.value[numTargets][2];
      dataProcessed = 0;
      buff = "";
    }
    break;

  case 0x20:
    if(dataProcessed>0) {buff = ""; break;}
    Serial.print("Space: ");
    serialCommand.command = buff;
    Serial.println(serialCommand.command);
    buff = "";
    dataProcessed++; // This is the address of the data
    if(serialCommand.command.equals("go"))
      setPosition();
    break;
  
  case ',':
    if(dataProcessed>2) {buff = ""; break;}
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
    Serial.print("x:");
    Serial.print(serialCommand.value[targetComplete][0]);
    Serial.print("y:");
    Serial.print(serialCommand.value[targetComplete][1]);
    Serial.print("z:");
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
  else Serial.println("All positions cleared!");
}