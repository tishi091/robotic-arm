#include <Arduino.h>

#include "robot.h"
#define SQUARE(x) (x*x)
#define CUBE(x) (x*x*x)
#define POS_MARGIN (float) 0.5
robot_t robot;
robot_t::robot_t()
{
    mode = m_pwm;

    // Distance from tof sensor
    // rel_x = 0.08;
    // rel_y = 0.00;
    // rel_z = 0.138;
    rel_x = 80;
    rel_y = 0;
    rel_z = 138;

    pwm_base = 1523;
    ang_base = PI/2;
    pwm_arm = 1478;
    ang_arm = PI/2;
    pwm_farm = 2070;
    ang_farm = 0;
    pwm_claw = 1600;

    step = 11;
}


void robot_t::sendCommand(char command)
{
    if(command == '0') {
        robot.mode = m_pwm;
        step = 11;
        setServos();
    }
    else if(command == '1') {
        robot.mode = m_angle;
        step = 1;
        setServos();
    }
    else if(command == '2') {
        robot.mode = m_invKin;
        // step = 0.01;
        step = 1;

        setServos();
    }

    if(robot.mode == m_pwm){
        switch (command)
        {
        case 'g':
        robot.moveLeft();
        break;
        
        case 'j':
        robot.moveRight();
        break;
        
        case 'y':
        robot.moveForward();
        break;
        
        case 'h':
        robot.moveBackwards();
        break;
        
        case 'i':
        robot.moveUpwards();
        break;
        
        case 'k':
        robot.moveDownward();
        break;
        
        case '+':
        robot.increaseStep();
        break;
        
        case '-':
        robot.decreaseStep();
        break;

        default:
        break;
        }
    }
    else if(robot.mode == m_angle){
        switch (command)
        {
        case 'g':
        robot.angle2PW(decreaseAngle(robot.ang_base), 1);
        break;
        
        case 'j':
        robot.angle2PW(increaseAngle(robot.ang_base), 1);
        break;
        
        case 'y':
        robot.angle2PW(increaseAngle(robot.ang_arm), 2);
        break;
        
        case 'h':
        robot.angle2PW(decreaseAngle(robot.ang_arm), 2);
        break;
        
        case 'i':
        robot.angle2PW(increaseAngle(robot.ang_farm), 3);
        break;
        
        case 'k':
        robot.angle2PW(decreaseAngle(robot.ang_farm), 3);
        break;
        
        case '+':
        robot.increaseStep();
        break;
        
        case '-':
        robot.decreaseStep();
        break;

        default:
        break;
        }
    }
    else if(robot.mode == m_invKin){
        switch (command)
        {
        case 'g':
        robot.cartesian2Angle(robot.rel_x, decrementPos(robot.rel_y), robot.rel_z);
        break;
        
        case 'j':
        robot.cartesian2Angle(robot.rel_x, incrementPos(robot.rel_y), robot.rel_z);
        break;
        
        case 'y':
        robot.cartesian2Angle(incrementPos(robot.rel_x), robot.rel_y, robot.rel_z);
        break;
        
        case 'h':
        robot.cartesian2Angle(decrementPos(robot.rel_x), robot.rel_y, robot.rel_z);
        break;
        
        case 'i':
        robot.cartesian2Angle(robot.rel_x, robot.rel_y, incrementPos(robot.rel_z));
        break;
        
        case 'k':
        robot.cartesian2Angle(robot.rel_x, robot.rel_y, decrementPos(robot.rel_z));
        break;
        
        case '+':
        robot.increaseStep();
        break;
        
        case '-':
        robot.decreaseStep();
        break;

        default:
        break;
        }
    }
    command = '?';
}

void robot_t::setServos(void){
    pwm_base = 1523;
    ang_base = PI/2;
    pwm_arm = 1478;
    ang_arm = PI/2;
    pwm_farm = 2070;
    ang_farm = 0;
    pwm_claw = 1600;
    rel_x = 80;
    rel_y = 0;
    rel_z = 138;
    setBase(pwm_base);
    setArm(pwm_arm);
    setFarm(pwm_farm);
    setClaw(pwm_claw);
}

// ############### Incremental movement ###############
void robot_t::increaseStep(void){       // Step configuration
    step += 1;
}
void robot_t::decreaseStep(void){
    step -= 1;
}

void robot_t::moveLeft(void){           // Base
    if(robot.mode == m_pwm){
        if(pwm_base <= 546) pwm_base = 546;
        pwm_base -= step;
        base.writeMicroseconds(pwm_base);
    }
}
void robot_t::moveRight(void){
    if(robot.mode == m_pwm){
        if (pwm_base >= 2270) pwm_base = 2270;
        pwm_base += step;
        base.writeMicroseconds(pwm_base);
    }
}

void robot_t::moveBackwards(void){      // Arm
    if(robot.mode == m_pwm){
        if(pwm_arm <= 970) pwm_arm = 970;
        else pwm_arm -= step;
        arm.writeMicroseconds(pwm_arm);
    }
}
void robot_t::moveForward(void){
    if(robot.mode == m_pwm){
        if (pwm_arm >= 2446) pwm_arm = 2446;
        else pwm_arm += step;
        arm.writeMicroseconds(pwm_arm);
    }
}

void robot_t::moveDownward(void){       // Forearm
    if(robot.mode == m_pwm){
        if (pwm_farm <= 910) pwm_farm = 910;
        else pwm_farm -= step;
        farm.writeMicroseconds(pwm_farm);
    }
}
void robot_t::moveUpwards(void){
    if(robot.mode == m_pwm){
        if (pwm_farm >= 2070) pwm_farm = 2070;
        else pwm_farm += step;
        farm.writeMicroseconds(pwm_farm);
    }
}

void robot_t::closeClaw(void)           // Claw
{
    pwm_claw += step;
    claw.writeMicroseconds(pwm_claw);
}
void robot_t::openClaw(void)
{
    pwm_claw -= step;
    claw.writeMicroseconds(pwm_claw);
}
// ################################################

// ############### Setting position ###############
void robot_t::setBase(int micros)       // Base
{
    robot.base.writeMicroseconds(micros);
}

void robot_t::setArm(int micros)        // Arm
{
    robot.arm.writeMicroseconds(micros);
}

void robot_t::setFarm(int micros)       // Forearm
{
    robot.farm.writeMicroseconds(micros);
}

void robot_t::setClaw(int micros)       // Claw
{
    robot.claw.writeMicroseconds(micros);
}
// ################################################


// ############# Servo PW from angle ##############
float robot_t::rad2Degree(float rad){
    return rad*180/PI;
}

float robot_t::deg2Rad(float deg){
    return deg*PI/180;
}

float robot_t::increaseAngle(float rad)
{
    float deg = rad2Degree(rad);
    deg += step;
    float res = deg2Rad(deg);
    return res;
}

float robot_t::decreaseAngle(float rad)
{
    float deg = rad2Degree(rad);
    deg -= step;
    float res = deg2Rad(deg);
    return res;
}

void robot_t::angle2PW(float angle, int joint)
{
    if(joint == 1){
        pwm_base =( 546 + (628*angle) - (3.71*SQUARE(angle)));
        if(pwm_base >= 2270) pwm_base = 2270;
         else if(pwm_base <= 546) pwm_base = 546;
        ang_base = angle;
        if(ang_base <= 0) ang_base = 0;
        else if(ang_base >= radians(160)) ang_base = radians(160);
        base.writeMicroseconds(pwm_base);
    }
    
    if(joint == 2){
        pwm_arm = (-320 + (1408*angle) - (168*SQUARE(angle)));
         if(pwm_arm >= 2446) pwm_arm = 2446;
         else if(pwm_arm <= 970) pwm_arm = 970;
         ang_arm = angle;
         if(ang_arm <= 0.087266) ang_arm = 0.087266;
         else if(ang_arm >= 2.094395) ang_arm = 2.094395;
        arm.writeMicroseconds(pwm_arm);
    }

    if(joint == 3){
        // pwm_farm = (2094 - (768*angle) + (76*angle*angle));
        pwm_farm = (2094 - (1072*angle) + (228*SQUARE(angle)));
         if(pwm_farm >= 2200) pwm_farm = 22000;
         else if(pwm_farm <= 910) pwm_farm = 910;
        ang_farm = angle;
         if(ang_farm <= 0) ang_farm = 0;
         else if(ang_farm >= 1.570796) ang_farm = 1.570796;
        farm.writeMicroseconds(pwm_farm);
    }
}
// ################################################


// ############# Inverse Kinematics ###############

float robot_t::incrementPos(float pos)
{
    pos += step;
    return pos;
}

float robot_t::decrementPos(float pos)
{
    pos -= step;
    return pos;
}

void robot_t::cartesian2Angle(float x, float y, float z)
{   
    
    if(x <= 1) x = 1;
    if(z <= 1) z = 1;
    else if(z >= 152) z = 152;
    
    // Defining new coordinates
    robot.rel_x = x;
    robot.rel_y = y;
    robot.rel_z = z;

    

    // Defining base angle
    float ang_ebase = atan2(y, x)+(PI/2);
    if(ang_ebase <= 0) ang_ebase = 0;
    else if(ang_ebase >= deg2Rad(160)) ang_ebase = deg2Rad(160);
    robot.ang_base = ang_ebase;

    // Defining arm and forearm angle

    float l = 80; //lenght of arms in meters
    float r = sqrt(SQUARE(x) + SQUARE(y)); //Dist to object
    if(r <= 20) r = 20;
    float hp = z - 60; // 0.060 altura até o sensor
    float s = sqrt(SQUARE(hp) + SQUARE(r));// distância garra base
    if(s >= 155) s = 155;
    Serial.print("S is equal to: ");
    Serial.println(s);
    float ecos = 1 - (SQUARE(s)/(2*SQUARE(l)));
    if(ecos <= -1) ecos = -1;
    else if(ecos >= 1) ecos = 1;
    float q1 = acos(ecos);    //s^2 = x^2 + z^2 ; cossen law s^2 = 2l^2-2*cos(q1)
    float q2_1 = (PI-q1)/2;   // Isoceles triangle, q1 = 180-(2*q2_1)
    float q2_2 = atan2(hp, r);             // Pitagoras q2_2 is the angle between the z and x
    float ang_earm = PI-(q2_1+q2_2);
    if(ang_earm <= 0) ang_earm = 0;
    else if(ang_earm >= PI) ang_earm = PI;
    float ang_efarm = robot.ang_arm-q1;
    if(ang_efarm <= 0) ang_efarm = 0;
    else if(ang_efarm >= PI/2) ang_efarm = PI/2;
    robot.ang_arm = ang_earm;          // Arm angle is q2 = 180 - (q2_1+q2_2)
    robot.ang_farm = ang_efarm; // Forearm angle is the suplementary of q1+q2
    Serial.print("Base angle: ");
    Serial.println(robot.ang_base);
    Serial.print("Arm angle: ");
    Serial.println(robot.ang_arm);
    Serial.print("Forearm angle: ");
    Serial.println(robot.ang_farm);
    inverseKin();
}

bool robot_t::moveToTarget(float* targetP, float speed, int cylclePeriod){
    float x,y,z;
    if(robot.rel_x< targetP[0]+POS_MARGIN && (robot.rel_x> targetP[0]-POS_MARGIN)){
		if(robot.rel_y < targetP[1]+POS_MARGIN && robot.rel_y > targetP[1]-POS_MARGIN){
			if(robot.rel_z < targetP[2]+POS_MARGIN && robot.rel_z > targetP[2]-POS_MARGIN){
			return true;
            }

		}
	}
	float sVec[3] = {targetP[0] - robot.rel_x, targetP[1] - robot.rel_y, targetP[2] - robot.rel_z};
	// Serial.printf("SVec: %.6f %.6f %.6f", sVec[0], sVec[1], sVec[2]);
	float sMod = pitagoras(sVec[0], sVec[1], sVec[2]);
	// Serial.printf("   SMod: %.6f", sMod); 
	float sUnit[3] = {sVec[0]/sMod, sVec[1]/sMod, sVec[2]/sMod};
	// Serial.printf("   SUnit: %.6f %.6f %.6f", sUnit[0], sUnit[1], sUnit[2]);

    Serial.println("            MOVING TO TARGET");
	speed = (speed*powf(E, sMod*0.025) > 150) ? 150 : speed*powf(E, sMod*0.025);
    Serial.print("              Speed is: ");
    Serial.println(speed);

	x= sUnit[0]*speed*1e-3*cylclePeriod + robot.rel_x;
    y = sUnit[1]*speed*1e-3*cylclePeriod + robot.rel_y;
	z = sUnit[2]*speed*1e-3*cylclePeriod + robot.rel_z;
    
    robot.cartesian2Angle(x,y,z);
    return false;
}

void robot_t::inverseKin(void)
{
    angle2PW(robot.ang_base, 1);
    angle2PW(robot.ang_arm, 2);
    angle2PW(robot.ang_farm, 3);
}


float robot_t::pitagoras(float a, float b, float c){
	return sqrt(a*a+b*b+c*c);
}
// ################################################