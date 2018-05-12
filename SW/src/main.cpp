/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */

//#ifndef UNIT_TEST  // disable program main loop while unit testing in progress


// #ifndef UNIT_TEST  // disable program main loop while unit testing in progress
#include <Arduino.h>
<<<<<<< HEAD
// #include "rha_types.h"
// #include "utilities.h"
// #include "servo_rha.h"
// #include "joint_handler.h"
=======
#include "debug.h"
#include "rha_types.h"
#include "utilities.h"
#include "servo_rha.h"
#include "joint_handler.h"
#include "robot_rha.h"
>>>>>>> 38ec39dc79ac6a91e8c048533a58de7588515ba8
// #include "joint_rha.h"
#include "robot_rha.h"

#include <wiringSerial.h>

<<<<<<< HEAD
=======
 #define DEBUG_SERVO_RHA
// #define DEBUG_TEST_SERVO_RHA
 #define DEBUG_CYTRON_G15_SERVO
// #define DEBUG_TEST_CYTRON_G15_SERVO
>>>>>>> 38ec39dc79ac6a91e8c048533a58de7588515ba8

// CYTRON_G15_SERVO g15(1, 2, 3, 8);

RobotRHA robo_health_arm;

void setup() {
<<<<<<< HEAD
    pinMode(LED_BUILTIN, OUTPUT);
    delay(2000);
=======
>>>>>>> 38ec39dc79ac6a91e8c048533a58de7588515ba8
	printf("Esto es un setup corriendo en una raspy\n");
	#if !defined(__RASPBERRY_PI_3B__)
		delay(2000);
		Serial.begin(921600); //115200 230400 250000 460800 921600
	#endif
<<<<<<< HEAD
    printf("# Start setup\n");
    printf("# TX_BUFFER = ");
    printf(SERIAL_TX_BUFFER_SIZE);
    printf("\n");
    printf("# RX_BUFFER = ");
    printf(SERIAL_RX_BUFFER_SIZE);
    printf("\n");

    robo_health_arm.initJointHandler();
    robo_health_arm.initPynterface();
    printf("#Joint Handler initialiced\n");
    // robo_health_arm.initChuckHandler();

    printf("# Init loop\n");
=======
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
>>>>>>> 38ec39dc79ac6a91e8c048533a58de7588515ba8

    /*RHATypes::Point3 cartesian_pos;
    cartesian_pos.x = 0.8;
    cartesian_pos.z = 0.1;
    robo_health_arm.goToCartesianPos(cartesian_pos);*/
}

// int i = 0;
 int k = 0;

void loop() {
<<<<<<< HEAD
    robo_health_arm.handleWithPynterface();
    /*robo_health_arm.joint_handler_.updateJointInfo();
    robo_health_arm.handleRobot();

    printf("\n");
    if (robo_health_arm.isError()) {
        while (true) {
            printf("[Error] Some error ocurred while running, arm in error mode. Needs reset to work again\ns");
            digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
            delay(500);                       // wait for a second
            digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
            delay(500);
            break;
        }
    }*/
     /*if (k >= 100) {
        k = 0;
*/
        /*printf("freeMemory()= ");
        printf(freeMemory());
		printf("\n");
 
		printf("\n");
        printf("Servo 0:"); printf("\t");
        robo_health_arm.joint_handler_.joint_[0].servo_.printCheckVar();
        printf("Servo 1:"); printf("\t");
        robo_health_arm.joint_handler_.joint_[1].servo_.printCheckVar();
        printf("Servo 2:"); printf("\t");
        robo_health_arm.joint_handler_.joint_[2].servo_.printCheckVar();
        printf("joint 0:"); printf("\t");
        robo_health_arm.joint_handler_.joint_[0].printCheckVar();
        printf("joint 1:"); printf("\t");
        robo_health_arm.joint_handler_.joint_[1].printCheckVar();
        printf("joint 2:"); printf("\t");
        robo_health_arm.joint_handler_.joint_[2].printCheckVar();
        printf("joint handler:"); printf("\t");
        robo_health_arm.joint_handler_.printCheckVar();
		printf("\n");*/

        /*printf("ID Servo:"); printf("\t\t");
        printf(robo_health_arm.joint_handler_.joint_[0].servo_.getID()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].servo_.getID()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[2].servo_.getID());
		printf("\n");

        printf("Pot pin:"); printf("\t\t");
        printf("-"); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].getPotentiometerPin()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[2].getPotentiometerPin());
		printf("\n");

        printf("Pot value:"); printf("\t\t");
        printf("-"); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].getAnalogReadPot()); printf(",\t");
        printfln(robo_health_arm.joint_handler_.joint_[2].getAnalogReadPot());

        printf(F("Goal pos:")); printf("\t\t");
        printf("-"); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].getPosTarget()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[2].getPosTarget());
		printf("\n");

        printf(F("Articulation position:")); printf("\t");
        printf("-"); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].getPosition()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[2].getPosition());
		printf("\n");

        printf(F("Speed calculated:")); printf("\t");
        printf("-"); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].getGoalSpeed()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[2].getGoalSpeed());
		printf("\n");

        printf(F("Error calculated:")); printf("\t");
        printf("-"); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].getError()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[2].getError());
		printf("\n");

        printf(F("Goal Torque:")); printf("\t\t");
        printf(robo_health_arm.joint_handler_.joint_[0].servo_.getGoalTorque()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].servo_.getGoalTorque()); printf(",\t"); printf(robo_health_arm.joint_handler_.joint_[2].servo_.getGoalTorque());
		printf("\n");

        printf(F("Goal dir:")); printf("\t\t");
        printf(robo_health_arm.joint_handler_.joint_[0].servo_.getDirectionTarget()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].servo_.getDirectionTarget()); printf(",\t"); printf(robo_health_arm.joint_handler_.joint_[2].servo_.getDirectionTarget());
		printf("\n");

        printf(F("Speed in servos:")); printf("\t");
        printf(robo_health_arm.joint_handler_.joint_[0].servo_.getSpeed()); printf(",\t");
        printf(robo_health_arm.joint_handler_.joint_[1].servo_.getSpeed()); printf(",\t"); printf(robo_health_arm.joint_handler_.joint_[2].servo_.getSpeed());
		printf("\n");

		printf("\n");
		printf("\n");
     }

     k++;*/

//int fd ;

//void setup() {
	////wiringPiSetup();
	///*if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
	//{
		//fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		//return;
	//}*/
	////Serial.begin(115200);
	//printf("Inicio del setup\n");

//}

//#endif



//#include <Arduino.h>
//#include <wiringSerial.h>


//#include <stdio.h>
//#include <unistd.h>			//Used for UART
//#include <fcntl.h>			//Used for UART
//#include <termios.h>		//Used for UART

//int fd ;

//void setup() {  
	////wiringPiSetup();
	///*if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
	//{
		//fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		//return;
	//}*/
	////Serial.begin(115200);
	//printf("Inicio del setup\n");
//}


//void loop() {	
    //int i = 0;
    //printf("##############################");
    //printf("\nInicio del loop\n");

	//printf("Ahora en otro ");    
	////Serial.write("cacafuti");
	///*serialPrintf(fd, "Ahora en uno") ;
    //fflush(stdout) ;*/
    //for (i = 0; i < 100; i++) {
		//printf("Ahora en otro ");
		//delay(100);
    //}
    //printf("\nFin del loop\n");
    //printf("##############################");
    
    
    
    //int uart0_filestream = -1;
    //uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	//if (uart0_filestream == -1)
	//{
		////ERROR - CAN'T OPEN SERIAL PORT
		//printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	//}
	//struct termios options;
	//tcgetattr(uart0_filestream, &options);
	//options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	//options.c_iflag = IGNPAR;
	//options.c_oflag = 0;
	//options.c_lflag = 0;
	//tcflush(uart0_filestream, TCIFLUSH);
	//tcsetattr(uart0_filestream, TCSANOW, &options);
	
	////----- TX BYTES -----
	//unsigned char tx_buffer[20];
	//unsigned char *p_tx_buffer;
	
	//p_tx_buffer = &tx_buffer[0];
	//*p_tx_buffer++ = 0xFF;
	////*p_tx_buffer++ = 'H';
	////*p_tx_buffer++ = 'e';
	////*p_tx_buffer++ = 'l';
	////*p_tx_buffer++ = 'l';
	////*p_tx_buffer++ = 'o';
	
	//if (uart0_filestream != -1)
	//{
		//int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		//if (count < 0)
		//{
			//printf("UART TX error\n");
		//}
	//}
	
	//close(uart0_filestream);
//}

//void loop() {
    //int i = 0;
    //printf("##############################");
    //printf("\nInicio del loop\n");

	//printf("Ahora en otro ");
	////Serial.write("cacafuti");
	///*serialPrintf(fd, "Ahora en uno") ;
    //fflush(stdout) ;*/
    //for (i = 0; i < 100; i++) {
		//printf("Ahora en otro ");
		//delay(100);
    //}
    //printf("\nFin del loop\n");
    //printf("##############################");



    //int uart0_filestream = -1;
    //uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	//if (uart0_filestream == -1)
	//{
		////ERROR - CAN'T OPEN SERIAL PORT
		//printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	//}
	//struct termios options;
	//tcgetattr(uart0_filestream, &options);
	//options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	//options.c_iflag = IGNPAR;
	//options.c_oflag = 0;
	//options.c_lflag = 0;
	//tcflush(uart0_filestream, TCIFLUSH);
	//tcsetattr(uart0_filestream, TCSANOW, &options);

	////----- TX BYTES -----
	//unsigned char tx_buffer[20];
	//unsigned char *p_tx_buffer;

	//p_tx_buffer = &tx_buffer[0];
	//*p_tx_buffer++ = 0xFF;
	////*p_tx_buffer++ = 'H';
	////*p_tx_buffer++ = 'e';
	////*p_tx_buffer++ = 'l';
	////*p_tx_buffer++ = 'l';
	////*p_tx_buffer++ = 'o';

	//if (uart0_filestream != -1)
	//{
		//int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		//if (count < 0)
		//{
			//printf("UART TX error\n");
		//}
	//}

	//close(uart0_filestream);
//}

=======
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
>>>>>>> 38ec39dc79ac6a91e8c048533a58de7588515ba8
