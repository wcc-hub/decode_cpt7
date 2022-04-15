#include "ReadCPT.h"

#include <math.h>


int main(int argc,char **argv)
{

  	int fd;
    	printf("\n +----------------------------------+");
    	printf("\n |        Serial Port Read          |");
    	printf("\n +----------------------------------+");
	string filename;
        if(argc!=3)
	{
	 printf("please input the serial port number.\nsudo ./ReadCPTData /dev/tty* DeviceType\n");
	}


 	uart_int(argv[1]);

	/*------------------------------- Opening the Serial Port -------------------------------*/
    	//fd = open("/dev/ttyUSB4", O_RDWR | O_NOCTTY | O_NDELAY);
        fd = open(argv[1], O_RDWR | O_NOCTTY | O_NDELAY);

   	if(fd == -1)						/* Error Checking */
        	printf("\n  Error! in Opening %s  ",argv[1]);
   	else
        	printf("\n  %s Opened Successfully ",argv[1]);

        printf("\nstart run!\n");


	ReadCPTData();

}
