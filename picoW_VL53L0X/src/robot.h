
#ifndef ROBOT_H
  #define ROBOT_H
#endif


#include <Arduino.h>
#include <Servo.h>
#define E (float) 2.718282
typedef enum {
    m_pwm,
    m_angle,
    m_invKin
} control_mode_t;

class robot_t{
    public:
    // Setup
    int mode;
    char input;
    Servo base, arm, farm, claw;

    // Relative position from sensor
    float rel_x, rel_y, rel_z;

    // Movement values
    float ang_base, ang_arm, ang_farm, ang_claw;
    float pwm_base, pwm_arm, pwm_farm, pwm_claw;
    float step;

    // Position values
    float xe, ye, ze;
    float tof_dist;

    // Set the robot initial conditions
    robot_t();
    void sendCommand(char command);
    void setServos(void);
    
    // Incremental PW movement
    void increaseStep(void);
    void decreaseStep(void);
    void moveRight(void);
    void moveLeft(void);
    void moveForward(void);
    void moveBackwards(void);
    void moveUpwards(void);
    void moveDownward(void);
    void cClaw(void);
    void oClaw(void);

    // Incremental angle movement
    float rad2Degree(float rad);
    float deg2Rad (float deg);
    float increaseAngle(float rad);
    float decreaseAngle(float rad);
    void angle2PW(float angle, int joint);
    void turn2Angle(float angle);
    // Incremental cartesian movement
    float incrementPos(float pos);
    float decrementPos(float pos);
    void cartesian2Angle(float x, float y, float z);
    bool moveToTarget(float* targetP, float speed, int cylclePeriod);
    // Selected movement functions
    void setBase(int micros);
    void setArm(int micros);
    void setFarm(int micros);
    void setClaw(int micros);
    bool openclaw();
    bool Closeclaw();
    void inverseKin(void);
    void rad2pos(float r, float theta, float* target);
  
    float pitagoras(float a, float b, float c);
};

extern robot_t robot;