/**
 * @Author: Enrique Heredia Aguado <enheragu>
 * @Date:   18_Nov_2017
 * @Project: RHA
 * @Filename: main.cpp
 * @Last modified by:   enheragu
 * @Last modified time: 23_Nov_2017
 */



#include <Arduino.h>
#include <wiringSerial.h>


#include <stdio.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

int fd ;

void setup() {
	//wiringPiSetup();
	/*if ((fd = serialOpen ("/dev/ttyAMA0", 115200)) < 0)
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		return;
	}*/
	//Serial.begin(115200);
	printf("Inicio del setup\n");
}


void loop() {
    int i = 0;
    printf("##############################");
    printf("\nInicio del loop\n");

	printf("Ahora en otro ");
	//Serial.write("cacafuti");
	/*serialPrintf(fd, "Ahora en uno") ;
    fflush(stdout) ;*/
    for (i = 0; i < 100; i++) {
		printf("Ahora en otro ");
		delay(100);
    }
    printf("\nFin del loop\n");
    printf("##############################");



    int uart0_filestream = -1;
    uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;

	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 0xFF;
	//*p_tx_buffer++ = 'H';
	//*p_tx_buffer++ = 'e';
	//*p_tx_buffer++ = 'l';
	//*p_tx_buffer++ = 'l';
	//*p_tx_buffer++ = 'o';

	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}

	close(uart0_filestream);
}
