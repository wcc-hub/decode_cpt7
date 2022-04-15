#include <stdio.h>
#include <fcntl.h>   /* File Control Definitions           */
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */
#include <errno.h>   /* ERROR Number Definitions           */
#include <stdlib.h>
#include <string.h>
#include <string>
#pragma pack(1)
using namespace std;

typedef struct
{

  	unsigned char  Head[3];             //3
  	unsigned char  HeaderLength;        //1
  	uint16_t MessageID;          //2
	char MessageType;                   //1
  	unsigned char PortAddress;         //1
  	uint16_t MessageLength;       //2
  	uint16_t Sequence;            //2
  	unsigned char IdleTime;             //1
  	unsigned char TimeStatus;           //1
 	uint16_t Week;                //2
 	uint32_t  GPSec;               //4
 	uint32_t  ReceiverStatus;      //4
 	uint16_t Reserved;            //2
 	uint16_t ReceiverSWVersion;   //2

}Header;

typedef struct
{
        Header Inspvaxb_Header;  //28
      //unsigned char Head[28];      //28
        uint32_t INSStatus;      //4
        uint32_t  PosType;      //4
  	double latitude;             //8byte
  	double longitude;            //8
  	double height;               //8
  	float  undulation;           //4
  	double northvelocity;        //8  start with 37B
  	double eastvelocity;         //8
  	double vertvelocity;         //8
  	double roll;                 //8
  	double pitch;                //8
 	double azimuth;              //8
 	float latsigma;              //4
 	float longsigma;             //4
 	float heightsigma;           //4
 	float Nvelsigma;             //4
 	float Evelsigma;             //4
 	float Uvelsigma;             //4
 	float rollsigma;             //4
 	float pitchsigma;            //4
 	float azimuthsigma;          //4
 	uint32_t Ess;          //4
 	uint16_t TSU;         //2
 	uint32_t Hex;          //4

}InspvaxbData;

union InspvaxbMsg
{
  InspvaxbData Inspvaxb_Data;
  uint8_t buffer[158];
};
InspvaxbMsg INSPVAXBDATA;

typedef struct
{
    Header Bestgnssposb_Header;  //28
    uint32_t SolStatus;      //4
    uint32_t  PosType;      //4
  	double latitude;             //8byte
  	double longitude;            //8
  	double height;               //8
  	float  undulation;           //4
  	uint32_t DatumID;        //4
 	float latsigma;              //4
 	float longsigma;             //4
 	float heightsigma;           //4
 	char  StnID[4];             //4
 	float Diff_age;             //4
 	float Sol_age;             //4
 	unsigned char SVs;             //1
 	unsigned char solnSVs;            //1
 	unsigned char solnL1SVs;          //1
 	unsigned char solnMultiSVs;          //1
 	unsigned char Reserved;         //1
 	unsigned char extsolstat;          //1
 	unsigned char GalileoandBeiDousigmask;          //1
 	unsigned char GPSandGLONASSsigmask;          //1
 	unsigned char  CRC[4];             //4


}BestgnssposbData;

union BestgnssposbMsg
{
  BestgnssposbData Bestgnssposb_Data;
  uint8_t buffer[104];
};
BestgnssposbMsg BESTGNSSPOSBDATA;

typedef struct
{
    Header Bestgnssvelb_Header;  //28
    uint32_t SolStatus;      //4
    uint32_t  VelType;      //4
  	float Latency;             //4
  	float Age;            //4
  	double HorSpd;               //8
  	double  TrkGnd;           //8
  	double VertSpd;        //8
 	float Reserved;              //4
 	unsigned char  CRC[4];             //4

}BestgnssvelbData;

union BestgnssvelbMsg
{
  BestgnssvelbData Bestgnssvelb_Data;
  uint8_t buffer[76];
};
BestgnssvelbMsg BESTGNSSVELBDATA;



#pragma pack()
void DecodeINSPVAXB();
void DecodeBESTGNSSVELB();
void DecodeBESTGNSSPOSB();

string  CPTINSFileName,CPTGNSSFileName;
FILE *CPTINSfpuart,*CPTGNSSfpuart;

int fd;                 /* Buffer to store the data received              */
int  bytes_read = 0;    /* Number of bytes read by the read() system call */
int decodeid=0;
int headcount=0;
int Bestgnssposbcount =0;
unsigned char read_buffer[1];

int Bestgnssvelbcount =0;

int inspvxbcount =0;

int a,hour;
int minute;
double sec;
double utc;

void GetCPTINSFileName(string &filename);
void GetCPTGNSSFileName(string &filename);
int  uart_int(const char* serialport);
void ReadCPTData();

