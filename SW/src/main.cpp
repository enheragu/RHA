/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */



// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include <Arduino.h>
#include "debug.h"
#include "rha_types.h"
#include "utilities.h"
#include "servo_rha.h"
#include "joint_handler.h"
#include "robot_rha.h"
// #include "joint_rha.h"

 #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
 #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

//JHUtilitiesJH joint_handler;
RobotRHA robo_health_arm;

void setup() {
	printf("Esto es un setup corriendo en una raspy\n");
	#if !defined(__RASPBERRY_PI_3B__)
		delay(2000);
		Serial.begin(921600); //115200 230400 250000 460800 921600
	#endif
	printf("lalala"); printf("\n");
	printf("lalala"); printf("\n");
	printf("# Setup begin");

	robo_health_arm.initJointHandler();
	robo_health_arm.initChuckHandler();

	/*joint_handler.setTimer(50);
	outputln("# Init G15 serial port");
	joint_handler.initSerial(19,18,8,460800);  // baudrate 460800 means 57.6 bytes/milisecond
	outputln("# Init Joints");
	joint_handler.initJoints();
	outputln("# Exit wheel mode");
	joint_handler.sendExitWheelModeAll();
	delay(100);
	outputln("# Set wheel mode");
	joint_handler.sendSetWheelModeAll();
	//output("# Id for servo is: "); outputln(joint_handler.joint_[0].servo_.getID());
	joint_handler.updateJointInfo();
	delay(5000);
	outputln("# Setup done");

	RHATypes::SpeedGoal speed_goal(1,150,0,CW);  // Id, speed, speed_slope
	joint_handler.setSpeedGoal(speed_goal);
	RHATypes::SpeedGoal speed_goal2(2,150,0,CW);  // Id, speed, speed_slope
	joint_handler.setSpeedGoal(speed_goal2);
	*/
	/*outputln("# Set wheel speed in single mode");
	joint_handler.sendSetWheelSpeedAll(500,CW);
	delay(5000);*/
	outputln("# Init loop");

}

unsigned int i = 0, c = 0;
long timeNow = 0;
long timeBuffer[800];
float average = 0;
float standard_deviation = 0;

void loop() {
    //joint_handler.extractRegulatorData(0);
    //joint_handler.extractStepInputData(0);
    //delay(500);
    //RASPYTEST: robo_health_arm.handleWithChuck();
    printf("Inicio del loop\n");
    if (i % 1000 == 0){
		printf("%d....", i);
        //RASPYTEST: output(robo_health_arm.joint_handler_.joint_[0].servo_.getGoalTorque()); output(",\t");
        //RASPYTEST: output(robo_health_arm.joint_handler_.joint_[2].servo_.getGoalTorque()); output(",\t"); outputln(robo_health_arm.joint_handler_.joint_[1].servo_.getGoalTorque());
    }
    i++;
    /*for (i = 0; i < 800; i++) {
        timeNow = millis();
        robo_health_arm.handleWithChuck();
        timeBuffer[i] = millis() - timeNow;
    }

    MeasureUtilities::averageChauvenet(timeBuffer, 800, average, standard_deviation);
    output("Average time of 800 mesaures is : "); output(average);outputln("ms");

    for (i = 0; i < 800; i++) {
        timeNow = millis();
        robo_health_arm.joint_handler_.controlLoop();
        timeBuffer[i] = millis() - timeNow;
    }

    MeasureUtilities::averageChauvenet(timeBuffer, 800, average, standard_deviation);
    output("Average time of 800 mesaures is : "); output(average); outputln("ms");

    while (true)
    {
         delay(100);
    }*/
    //if (i % 1000 == 0){outputln(" "); output("############  "); output (c); output("  ############"); outputln(" "); c++;}
    //i++;
    //joint_handler.controlLoop();
    //joint_handler.checkComSucces(150);
    //joint_handler.extractSlopeInputData(0);
    //outputln(" ");
    //outputln("###############################################################");
    //outputln("######################### End of test #########################");
    //outputln("###############################################################");
    //while (true) {
    //    delay(150);
    //}
}
/**
void loop(){
    RHATypes::SpeedGoal speed_goal(1,50,0, CW);  // Id, speed, speed_slope
    joint_handler.setSpeedGoal(speed_goal);
    joint_handler.joint_[0].servo_.speed_regulator_.setKRegulator(kp_samples[sample],0,0);
    output("n_data"); output(sample); output(" = "); outputln(SAMPLE_REGULATOR);
    output("speed_target"); output(sample); output("  = "); outputln(joint_handler.joint_[0].getSpeedTarget());
    output("regulatorTest"); output(sample); output("  = [");
    output("['"); output(joint_handler.joint_[0].servo_.speed_regulator_.getKp()); output("']");

    int counter = 0;
    while(true){
        unsigned long time_init = millis();
        joint_handler.controlLoop();
        output(",['"); output(joint_handler.joint_[0].servo_.getSpeed()); output("','");
        output(time_init); outputln("']\\");
        counter ++;
        if (counter > SAMPLE_REGULATOR) break;

    }

    outputln("]");
    sample++;
    if (sample >= SAMPLE_KP)
        while(true) {}

}*/


// #endif
