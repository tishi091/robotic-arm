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
VL53L0X tof;
float distance, prev_distance;

#include "robot.h"

int LED_state;
unsigned long interval;
unsigned long currentMicros, previousMicros;
int loop_count;
float speed = 5;
float CurrPos[3];
float targetPos[3];
int cycle;


void setup() 
{
  robot.setServos();
  robot.base.attach(BASE_PIN, 530, 2400);   // Base Servo 760(Left -60º) - 2225(Right 60º)
  robot.arm.attach(ARM_PIN, 970, 2450);     // Arm Servo 970(Backward 120º) - 2440(Forward 0º)
  robot.farm.attach(FARM_PIN, 870, 2100);   // Forearm Servo 2070(Downward 0º) - 910(Upward 90º)
  robot.claw.attach(CLAW_PIN, 560, 1610);   // Claw Servo 560 (Opened) - 1610(closed)
  cycle=40;
  interval = 40*1000;

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
 
}


void loop() 
{

  
  uint8_t b;
  if (Serial.available()) {  // Only do this if there is serial data to be read
  
    b = Serial.read();    
    Serial.write(b);
    if(b == '7'){
      targetPos[0]=100;
	    targetPos[1]=50;
	    targetPos[2]=40;
    }
    if(b == '8'){
      targetPos[0]=80;
	    targetPos[1]=-20;
	    targetPos[2]=90;
    }
    robot.sendCommand(b);
    
  } 
  currentMicros = micros();

  // THE Control Loop
  if (currentMicros - previousMicros >= interval) {
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

     
     robot.moveToTarget(targetPos,speed,cycle);
    
    

    Serial.print(" tof: ");
    Serial.print(distance, 3);

    Serial.print("  mode: ");
    Serial.print(robot.mode);

    Serial.print("  step: ");
    Serial.print(robot.step);

    Serial.print("  base_pwm: ");
    Serial.print(robot.pwm_base);

    Serial.print("  arm_pwm: ");
    Serial.print(robot.pwm_arm);

    Serial.print("  farm_pwm: ");
    Serial.print(robot.pwm_farm);

    Serial.print("  claw_pwm: ");
    Serial.print(robot.pwm_claw);

    Serial.print("  ang_b: ");
    Serial.print(robot.rad2Degree(robot.ang_base));

    Serial.print("  ang_a: ");
    Serial.print(robot.rad2Degree(robot.ang_arm));

    Serial.print("  ang_f: ");
    Serial.print(robot.rad2Degree(robot.ang_farm));

    Serial.print("  x_rel: ");
    Serial.print(robot.rel_x);

    Serial.print("  y_rel: ");
    Serial.print(robot.rel_y);

    Serial.print("  z_rel: ");
    Serial.print(robot.rel_z);

    Serial.println();
  }
}
