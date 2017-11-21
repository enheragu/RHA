<<<<<<< HEAD
/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   2017_Sep_08
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   quique
 * @Last modified time: 29-Oct-2017
 */

//#ifndef UNIT_TEST  // disable program main loop while unit testing in progress

#include <Arduino.h>
// #include "rha_types.h"
// #include "utilities.h"
// #include "servo_rha.h"
// #include "joint_handler.h"
// #include "joint_rha.h"
#include "robot_rha.h"

#include <wiringSerial.h>

#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

RobotRHA robo_health_arm;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    delay(2000);
    Serial.begin(921600);  // 115200 230400 250000 460800 921600
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

    /*RHATypes::Point3 cartesian_pos;
    cartesian_pos.x = 0.8;
    cartesian_pos.z = 0.1;
    robo_health_arm.goToCartesianPos(cartesian_pos);*/
}

// int i = 0;
 int k = 0;

void loop() {
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
}

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
