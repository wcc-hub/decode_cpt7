#include "ReadCPT.h"


void GetCPTINSFileName(string &filename)
{
    time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    filename = "../data/"+to_string(timeinfo->tm_year+1900)+"-"+to_string(timeinfo->tm_mon+1)+"-"+to_string(timeinfo->tm_mday)+"-"+to_string(timeinfo->tm_hour)+"-"+to_string(timeinfo->tm_min)+"-"+to_string(timeinfo->tm_sec)+"-CPT.txt";

    printf ( "\nThe current date/time is: %s\n", asctime (timeinfo) );

    //printf ( "##################" );

}
void GetCPTGNSSFileName(string &filename)
{
    time_t rawtime;
    struct tm * timeinfo;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    filename = "../data/"+to_string(timeinfo->tm_year+1900)+"-"+to_string(timeinfo->tm_mon+1)+"-"+to_string(timeinfo->tm_mday)+"-"+to_string(timeinfo->tm_hour)+"-"+to_string(timeinfo->tm_min)+"-"+to_string(timeinfo->tm_sec)+"-GNSS.txt";

    printf ( "\nThe current date/time is: %s\n", asctime (timeinfo) );

    //printf ( "##################" );

}



int uart_int(const char* serialport)
{

    /*------------------------------- Opening the Serial Port -------------------------------*/
    //printf("serial: %s",&serialport);
    // fd = open("/dev/ttyUSB4", O_RDWR | O_NOCTTY | O_NDELAY);	/* ttyUSB0 is the FT232 based USB2SERIAL Converter  |O_NDELAY */
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    /* O_RDWR   - Read/Write access to serial port       */
    /* O_NOCTTY - No terminal will control the process   */
    /* Open in blocking mode,read will wait              */



    if(fd == -1)						/* Error Checking */
    {
        printf("\n  Error! in Opening ttyTHS0\n  ");
        return -1;
    }
    else
    {
        printf("\n  ttyTHS0 Opened Successfully\n ");

    }
    /*---------- Setting the Attributes of the serial port using termios structure --------- */

    /*RX init*/
    struct termios SerialPortSettings;	/* Create the structure                          */

    tcgetattr(fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&SerialPortSettings, B460800); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings, B460800); /* Set Write Speed as 9600                       */

    /* 8N1 Mode */
    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;      /* Set the data bits = 8                                 */

    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */


    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~ICRNL;
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */


    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing   raw  format  output*/

    /* Setting Time outs */
    SerialPortSettings.c_cc[VMIN] = 0; /* Read at least 10 characters */
    SerialPortSettings.c_cc[VTIME] = 1; /* Wait indefinetly   */


    if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) /* Set the attributes to the termios structure*/
        printf("\n  ERROR ! in Setting attributes\n");
    else
        printf("\n  BaudRate = 460800 \n  StopBits = 1 \n  Parity   = none\n");

    /*------------------------------- Read data from serial port -----------------------------*/

    tcflush(fd, TCIFLUSH);   /* Discards old data in the rx buffer            */

    close(fd); /* Close the serial port */




    return 0;

}

void DecodeINSPVAXB()
{

    fprintf(CPTINSfpuart,"#CPT,%d,%d,%f,%d,%d,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf\n",//,%f,%f,%f,%f,%f,%f,%f,%f,%f
            INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.Week,
            INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec,
            utc,
            //gpstime_end,
            //UtcTime,
            INSPVAXBDATA.Inspvaxb_Data.INSStatus,
            INSPVAXBDATA.Inspvaxb_Data.PosType,
            INSPVAXBDATA.Inspvaxb_Data.latitude,
            INSPVAXBDATA.Inspvaxb_Data.longitude,
            INSPVAXBDATA.Inspvaxb_Data.height,
            INSPVAXBDATA.Inspvaxb_Data.northvelocity,
            INSPVAXBDATA.Inspvaxb_Data.eastvelocity,
            INSPVAXBDATA.Inspvaxb_Data.vertvelocity,
            INSPVAXBDATA.Inspvaxb_Data.roll,
            INSPVAXBDATA.Inspvaxb_Data.pitch,
            INSPVAXBDATA.Inspvaxb_Data.azimuth
           );
    //usleep(100);
    printf("#CPT,%d,%d,%f,%d,%d,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf,%.9lf\n",//,%f,%f,%f,%f,%f,%f,%f,%f,%f
           INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.Week,
           INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec,
           utc,
           //gpstime_end,
           //UtcTime,
           INSPVAXBDATA.Inspvaxb_Data.INSStatus,
           INSPVAXBDATA.Inspvaxb_Data.PosType,
           INSPVAXBDATA.Inspvaxb_Data.latitude,
           INSPVAXBDATA.Inspvaxb_Data.longitude,
           INSPVAXBDATA.Inspvaxb_Data.height,
           INSPVAXBDATA.Inspvaxb_Data.northvelocity,
           INSPVAXBDATA.Inspvaxb_Data.eastvelocity,
           INSPVAXBDATA.Inspvaxb_Data.vertvelocity,
           INSPVAXBDATA.Inspvaxb_Data.roll,
           INSPVAXBDATA.Inspvaxb_Data.pitch,
           INSPVAXBDATA.Inspvaxb_Data.azimuth
          );
}


void DecodeBESTGNSSPOSB()
{

    fprintf(CPTGNSSfpuart,"p,%d,%d,%f,%d,%d,%.9lf,%.9lf,%.9lf\n",//,%f,%f,%f,%f,%f,%f,%f,%f,%f
            BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.Week,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec,
            utc,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.SolStatus,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.PosType,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.latitude,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.longitude,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.height

           );
    //usleep(100);
    printf("p,%d,%d,%f,%d,%d,%.9lf,%.9lf,%.9lf\n",//,%f,%f,%f,%f,%f,%f,%f,%f,%f
           BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.Week,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec,
            utc,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.SolStatus,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.PosType,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.latitude,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.longitude,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.height
          );
}

void DecodeBESTGNSSVELB()
{

    fprintf(CPTGNSSfpuart,"v,%d,%d,%f,%d,%d,%.9lf,%.9lf,%.9lf\n",//,%f,%f,%f,%f,%f,%f,%f,%f,%f
            BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.Week,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec,
            utc,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.SolStatus,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.PosType,
            BESTGNSSVELBDATA.Bestgnssvelb_Data.HorSpd,
            BESTGNSSVELBDATA.Bestgnssvelb_Data.TrkGnd,
            BESTGNSSVELBDATA.Bestgnssvelb_Data.VertSpd

           );
    //usleep(100);
    printf("v,%d,%d,%f,%d,%d,%.9lf,%.9lf,%.9lf\n",//,%f,%f,%f,%f,%f,%f,%f,%f,%f
           BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.Week,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec,
            utc,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.SolStatus,
            BESTGNSSPOSBDATA.Bestgnssposb_Data.PosType,
            BESTGNSSVELBDATA.Bestgnssvelb_Data.HorSpd,
            BESTGNSSVELBDATA.Bestgnssvelb_Data.TrkGnd,
            BESTGNSSVELBDATA.Bestgnssvelb_Data.VertSpd
          );
}


void ReadCPTData()
{
    GetCPTINSFileName(CPTINSFileName);
    GetCPTGNSSFileName(CPTGNSSFileName);
    CPTINSfpuart = fopen(CPTINSFileName.c_str(),"w");
    CPTGNSSfpuart = fopen(CPTGNSSFileName.c_str(),"w");

    while(1)
    {
        bytes_read = read(fd, &read_buffer, sizeof(read_buffer));


        if(bytes_read>0)
        {
            switch(decodeid)
            {
            case 0:

                if(read_buffer[0]==0xAA)
                {
                    decodeid = 1;//return 0;
                    BESTGNSSPOSBDATA.buffer[0]  = read_buffer[0];
                    BESTGNSSVELBDATA.buffer[0]  = read_buffer[0];
                    INSPVAXBDATA.buffer[0] = read_buffer[0];

                }
                else
                {
                    decodeid = 0;
                }
                break;
            case 1:
                if(read_buffer[0]==0x44)
                {
                    decodeid = 2;
                    BESTGNSSPOSBDATA.buffer[1]  = read_buffer[0];
                    BESTGNSSVELBDATA.buffer[1]  = read_buffer[0];
                    INSPVAXBDATA.buffer[1] = read_buffer[0];
                }
                else
                {
                    decodeid = 0;
                }
                break;
            case 2:
                if(read_buffer[0]==0x12)
                {
                    decodeid = 3;
                    BESTGNSSPOSBDATA.buffer[2]  = read_buffer[0];
                    BESTGNSSVELBDATA.buffer[2]  = read_buffer[0];
                    INSPVAXBDATA.buffer[2] = read_buffer[0];

                }
                else
                {
                    decodeid = 0;
                }
                break;
            case 3:
                decodeid = 4;
                BESTGNSSPOSBDATA.buffer[3]  = read_buffer[0];
                BESTGNSSVELBDATA.buffer[3]  = read_buffer[0];
                INSPVAXBDATA.buffer[3] = read_buffer[0];
                break;
            case 4:
                BESTGNSSPOSBDATA.buffer[4]  = read_buffer[0];
                BESTGNSSVELBDATA.buffer[4]  = read_buffer[0];
                INSPVAXBDATA.buffer[4] = read_buffer[0];

                if(BESTGNSSPOSBDATA.buffer[4] == 0x05 && BESTGNSSPOSBDATA.buffer[3] == 0x95)
                {
                    while(headcount < BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.HeaderLength-5)
                    {
                        bytes_read = read(fd, &read_buffer, 1);
                        if(bytes_read>0)
                        {
                            BESTGNSSPOSBDATA.buffer[5+headcount] = read_buffer[0];
                            headcount++;

                        }
                    }

                    decodeid =5;
                    headcount=0;
                }
                if(BESTGNSSPOSBDATA.buffer[4] == 0x05 && BESTGNSSPOSBDATA.buffer[3] == 0x96)
                {
                    while(headcount < BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.HeaderLength-5)
                    {
                        bytes_read = read(fd, &read_buffer, 1);
                        if(bytes_read>0)
                        {
                            BESTGNSSVELBDATA.buffer[5+headcount] = read_buffer[0];
                            headcount++;

                        }
                    }
                    decodeid = 6;
                    headcount=0;
                }
                if(INSPVAXBDATA.buffer[4] == 0x05 && INSPVAXBDATA.buffer[3] == 0xB9)
                {
                    while(headcount < INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.HeaderLength-5)
                    {
                        bytes_read = read(fd, &read_buffer, 1);
                        if(bytes_read>0)
                        {
                            INSPVAXBDATA.buffer[5+headcount] = read_buffer[0];
                            headcount++;

                        }
                    }
                    decodeid = 7;
                    headcount=0;
                }
                else
                {
                    decodeid = 0;
                }
                break;
            case 5:
                BESTGNSSPOSBDATA.buffer[BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.HeaderLength] = read_buffer[0];
                while(Bestgnssposbcount < BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.MessageLength+4-1)
                {
                    bytes_read = read(fd, &read_buffer, 1);

                    if(bytes_read>0)
                    {
                        BESTGNSSPOSBDATA.buffer[BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.HeaderLength+1+Bestgnssposbcount] = read_buffer[0];
                        Bestgnssposbcount++;
                    }
                }
                if(Bestgnssposbcount==BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.MessageLength+4-1)
                {
                        a      = floor(BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec/1000/3600);
                        hour   = a %24;
                        minute = floor((floor(BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec/1000)-floor(BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec/1000/3600)*3600)/60);
                        sec    = (double)BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec/1000 - floor(BESTGNSSPOSBDATA.Bestgnssposb_Data.Bestgnssposb_Header.GPSec/1000/3600)*3600-minute*60-18;
                        if (sec<0)
                        {
                            minute = minute-1;
                            sec = sec+60;
                        }
                        if (minute<0)
                        {
                            hour = hour-1;
                            minute = minute+60;
                        }
                        if (hour<0)
                        {
                            hour = hour+24;
                        }
                        utc =hour*10000+ minute*100 + sec;

                    DecodeBESTGNSSPOSB();

                }
                Bestgnssposbcount=0;

                decodeid = 0;
            case 6:
                BESTGNSSVELBDATA.buffer[BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.HeaderLength] = read_buffer[0];
                while(Bestgnssposbcount < BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.MessageLength+4-1)
                {
                    bytes_read = read(fd, &read_buffer, 1);

                    if(bytes_read>0)
                    {
                        BESTGNSSVELBDATA.buffer[BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.HeaderLength+1+Bestgnssvelbcount] = read_buffer[0];
                        Bestgnssvelbcount++;
                    }
                }
                if(Bestgnssvelbcount==BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.MessageLength+4-1)
                {
                        a      = floor(BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.GPSec/1000/3600);
                        hour   = a %24;
                        minute = floor((floor(BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.GPSec/1000)-floor(BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.GPSec/1000/3600)*3600)/60);
                        sec    = (double)BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.GPSec/1000 - floor(BESTGNSSVELBDATA.Bestgnssvelb_Data.Bestgnssvelb_Header.GPSec/1000/3600)*3600-minute*60-18;
                        if (sec<0)
                        {
                            minute = minute-1;
                            sec = sec+60;
                        }
                        if (minute<0)
                        {
                            hour = hour-1;
                            minute = minute+60;
                        }
                        if (hour<0)
                        {
                            hour = hour+24;
                        }
                        utc =hour*10000+ minute*100 + sec;

                    DecodeBESTGNSSVELB();

                }
                Bestgnssvelbcount=0;
                decodeid = 0;
                break;
            case 7:
                INSPVAXBDATA.buffer[INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.HeaderLength] = read_buffer[0];
                while(inspvxbcount < INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.MessageLength+4-1)
                {
                    bytes_read = read(fd, &read_buffer, 1);

                    if(bytes_read>0)
                    {
                        INSPVAXBDATA.buffer[INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.HeaderLength+1+inspvxbcount] = read_buffer[0];
                        inspvxbcount++;
                    }
                }
                if(inspvxbcount==INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.MessageLength+4-1)
                {
                        a      = floor(INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec/1000/3600);
                        hour   = a %24;
                        minute = floor((floor(INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec/1000)-floor(INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec/1000/3600)*3600)/60);
                        sec    = (double)INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec/1000 - floor(INSPVAXBDATA.Inspvaxb_Data.Inspvaxb_Header.GPSec/1000/3600)*3600-minute*60-18;
                        if (sec<0)
                        {
                            minute = minute-1;
                            sec = sec+60;
                        }
                        if (minute<0)
                        {
                            hour = hour-1;
                            minute = minute+60;
                        }
                        if (hour<0)
                        {
                            hour = hour+24;
                        }
                        utc =hour*10000+ minute*100 + sec;

                    DecodeINSPVAXB();

                }
                decodeid = 0;
                inspvxbcount=0;
                break;
            default:
                break;
            }
        }
    }
fclose(CPTINSfpuart);
fclose(CPTGNSSfpuart);
}

