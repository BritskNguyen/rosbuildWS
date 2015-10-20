//_____________________________________________________________________________
//
// Copyright 2011-2015 Time Domain Corporation
//
//
// rcmSampleApp.c
//
//   Sample code showing how to interface to P400 RCM module.
//
//   This code uses the functions in rcm.c to:
//      - make sure the RCM is awake and in the correct mode
//      - get the configuration from the RCM and print it
//      - get the status/info from the RCM and print it
//      - range to another RCM node
//      - broadcast that range in a data packet
//
// This sample can communicate with the RCM over Ethernet, the 3.3V serial port,
// or the USB interface (which acts like a serial port).
//
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// #includes
//_____________________________________________________________________________

#define ROS_MASTER_URI		"http://localhost:11311"
#define ROS_ROOT		"/opt/ros/indigo/share/ros"
#include <iostream>
#include "stdio.h"
#include <stdlib.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <cstring>
#include <unistd.h>
#include <sstream>
#include <signal.h>
#include <math.h>
#include <uwb_localization/rcmEkfStateMsg.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign/list_of.hpp>
#include <string.h>
#include "rcmIf.h"
#include "rcm.h"

//ekf ert generated code
#include "ekf_mf.h"
#include "trilat.h"

//log file
#include <fstream>

//_____________________________________________________________________________
//
// #defines
//_____________________________________________________________________________

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

//#define DEFAULT_DEST_NODE_ID    301
#define     DFLT_PORT           "/dev/ttyUSB0"
#define     DFLT_NODE_RATE      "10"
#define     TRILAT_TIMES        20

//_____________________________________________________________________________
//
// Vicon defines
//_____________________________________________________________________________
#define BAUDRATE		57600
#define BAUD_MACRO		B57600
#define DFLT_PORT	"/dev/ttyUSB0"
#define RCV_THRESHOLD	40					//Receive frame's length
#define FPS				40					//Frames per second
#define	UP				0xFF
#define DOWN			0x00
#define HEADER_NAN		0x7FAAAAAA

uint8_t		frame32[RCV_THRESHOLD];
uint8_t		rcvdFrame[RCV_THRESHOLD];
uint8_t		backupFrame[RCV_THRESHOLD];
uint8_t		flagIncompleteFrame = 0;
int			rcvdBytesCount = 0;
uint8_t		backupRcvBytesCount = 0;
uint8_t		misAlgnFlag = DOWN;
uint8_t		msgFlag = DOWN;
uint8_t		viconUpdateFlag = DOWN;
uint8_t		headerIndex = 0xFF;
float viconX , viconY, viconZ, viconXd, viconYd, viconZd;

void signal_handler_IO(int status)
{
    msgFlag = UP;
}

//_____________________________________________________________________________
//
// static data
//_____________________________________________________________________________

//initial conditions for the ekf
static ekf_mfClass ekf_Obj;
static double ancs[12] =
{
    -3.0, 3.0, 3.0, -3.0,
    -3.0, -3.0, 3.0, 3.0,
    -1.78, -1.15, -1.31, -1.31
    //-1.78, -0.1 , -1.31, -0.21
};
static double R = 0.2;
//inputs for the ekf
static double dists[4] = { 0, 0, 0, 0 };
static double deltat = 0;
//static double imus = 0;
static unsigned int nodeId = 1;		//WARNING: MATLAB GENERATED FUNCTION USES INDEX 1 FOR 1ST ELEMENT
//output of the ekf
static double x_est[6];


//initial conditions for the trilaterator
static trilatModelClass trilat_Obj;
static double ancsTranspose[12] =
{
    -3.0, -3.0, -1.71,
    3.0, -3.0, -1.15,/*-0.1,//*/
    3.0, 3.0, -1.31,
    -3.0, 3.0, -1.31/*-0.21//*/
};
static double tempDists[4] = { 0, 0, 0, 0 };//{ 7.51, 13.03, 12.69, 6.90 };
static double trilatPos[3] = { 0, 0, 0 };
static double initialPos[3];

static uint8_T uwb2FccBuff[32];
#define U2F_HEADER1     (uint8_T)0x46;  //'F'
#define U2F_HEADER2     (uint8_T)0x43;  //'C'
#define U2F_ID          (*(uint8_T *)(uwb2FccBuff + 2))
#define U2F_LENGTH      (*(uint8_T *)(uwb2FccBuff + 3))
#define U2F_X           (*(uint8_T *)(uwb2FccBuff + 4))
#define U2F_Y           (*(uint8_T *)(uwb2FccBuff + 8))
#define U2F_Z           (*(uint8_T *)(uwb2FccBuff + 12))
#define U2F_DX          (*(uint8_T *)(uwb2FccBuff + 16))
#define U2F_DY          (*(uint8_T *)(uwb2FccBuff + 20))
#define U2F_DZ          (*(uint8_T *)(uwb2FccBuff + 24))
#define U2F_REV1        (*(uint8_T *)(uwb2FccBuff + 28))
#define U2F_REV2        (*(uint8_T *)(uwb2FccBuff + 29))
#define U2F_CS1         (*(uint8_T *)(uwb2FccBuff + 30))
#define U2F_CS2         (*(uint8_T *)(uwb2FccBuff + 31))
#define ID_TO_RES2_LENGTH   27

//_____________________________________________________________________________
//
// local function prototypes
//_____________________________________________________________________________


//_____________________________________________________________________________
//
// main - sample app entry point
//_____________________________________________________________________________
using namespace std;

int main(int argc, char *argv[])
{
    //Create ros handler to node
    ros::init(argc, argv, "uwb_localization");
    ros::NodeHandle uwbViconNodeHandle("~");
    string serialPortName = string(DFLT_PORT);
    string rcmEkfRate = string(DFLT_NODE_RATE);
    uint32_T nodeRate = 10;

    if(uwbViconNodeHandle.getParam("rcmSerialPort", serialPortName))
        printf(KBLU"Retrieved value %s for param 'rcmSerialPort'!\n"RESET, serialPortName.data());
    else
    {
        //serialPortName = string(DFLT_PORT);
        printf(KRED "Couldn't retrieve param 'rcmSerialPort', program closed!\n"RESET);
        return 0;
    }

    if(uwbViconNodeHandle.getParam("rcmLocalizationRate", rcmEkfRate))
        printf(KBLU"Retrieved value %s for param 'rcmLocalizationRate'\n"RESET, rcmEkfRate.data());
    else
        printf(KYEL "Couldn't retrieve param 'rcmLocalizationRate', applying default value %sHz\n"RESET, rcmEkfRate.data());

//-------------------------------------Vicon Serial Parameter Collection-------------------------------------------
    string viconSerialPortName = string(DFLT_PORT);
    bool handshakingMode = true;
    if(uwbViconNodeHandle.getParam("viconSerialPort", viconSerialPortName))
        printf(KBLU"Retrieved value %s for param 'viconSerialPort'!\n"RESET, viconSerialPortName.data());
    else
    {
        printf(KRED "Couldn't retrieve param 'viconSerialPort', program closed!\n"RESET);
        return 0;
    }

    if(uwbViconNodeHandle.getParam("viconhandshakingModeEnabled", handshakingMode))
    {
        if(handshakingMode)
            printf(KBLU"Retrieved value 'true' for param 'viconhandshakingModeEnabled'\n"RESET);
        else
            printf(KBLU"Retrieved value 'false' for param 'viconhandshakingModeEnabled'\n"RESET);
    }
    else
        printf(KBLU"Couldn't retrieve param 'viconhandshakingModeEnabled'. Using default mode with handshake\n"RESET);

//-------------------Initialize Vicon Serial Connection---------------------------------------------------
    int fd = -1;
    struct termios newtio;
    struct sigaction saio;           //definition of signal action
    FILE *fpSerial = NULL;

    //Open the serial port as a file descriptor for low level configuration
    //read/write, not controlling terminal for process,
    //fd = open(viconSerialPortName.data(), O_RDWR | O_NOCTTY | O_NDELAY);	//program won't be blocked during read
    fd = open(viconSerialPortName.data(), O_RDWR | O_NOCTTY);			//program will be blocked in during read
    if ( fd < 0 )
    {
        printf(KRED "serialInit: Could not open serial device %s\n" RESET, viconSerialPortName.data());
        return 0;
    }

//    //Install the signal handler before making the device asynchronous
//    saio.sa_handler = signal_handler_IO;
//    sigemptyset(&saio.sa_mask);
//    saio.sa_flags = 0;
//    saio.sa_restorer = NULL;
//    sigaction(SIGIO,&saio,NULL);

//    //allow the process to receive SIGIO
//    fcntl(fd, F_SETOWN, getpid());
//    //Make the file descriptor asynchronous
//    fcntl(fd, F_SETFL, O_ASYNC | O_NONBLOCK);

    //Set up serial settings
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag =  CS8 | CLOCAL | CREAD;		//no parity, 1 stop bit
    newtio.c_iflag |= IGNBRK;  					//ignore break condition
    newtio.c_oflag = 0;							//all options off
    //set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    //non-canonical condition, RCV_THRESHOLD is the number of bytes to return
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = RCV_THRESHOLD;

    //Flush the residual data and activate new settings
    tcflush(fd, TCIFLUSH);
    if (cfsetispeed(&newtio, BAUD_MACRO) < 0 || cfsetospeed(&newtio, BAUD_MACRO) < 0)
    {
        printf(KRED"Cannot set baudrate for %s\n", serialPortName.data());
        close(fd);
        return 0;
    }
    else
    {
        tcsetattr(fd, TCSANOW, &newtio);
        tcflush(fd, TCIOFLUSH);
        printf(KBLU "Connection established\n\n" RESET);
    }
    //Open file as a standard I/O stream
    fpSerial = fdopen(fd, "r+");
    if (!fpSerial)
    {
        printf(KRED "serialInit: Failed to open %s as serial stream\n" RESET, serialPortName.data());
        fpSerial = NULL;
    }
        tcflush(fd, TCIOFLUSH);
////-----------------------------------vicon Serial Initialization Done-------------------------------------

    int destNodeId;
    ros::Time timeStart, timeEnd;
    int status;
    rcmIfType   rcmIf;
    rcmConfiguration config;
    rcmMsg_GetStatusInfoConfirm statusInfo;
    rcmMsg_FullRangeInfo rangeInfo;
    rcmMsg_DataInfo dataInfo;
    rcmMsg_ScanInfo scanInfo;
    rcmMsg_FullScanInfo fullScanInfo;
    ofstream fout("/home/britsk/rosbuildWS/sandbox/uwb_localization/log/ekfmflog.m");

//----------------------------------initialize P410 serial interface--------------------------------------
    {
        printf("RCM Sample App\n\n");

        rcmIf = rcmIfUsb;

        // initialize the interface to the RCM
        if (rcmIfInit(rcmIf, &serialPortName[0]) != OK)
        {
            printf("Initialization failed.\n");
            exit(0);
        }

        // Make sure RCM is awake
        if (rcmSleepModeSet(RCM_SLEEP_MODE_ACTIVE) != 0)
        {
            printf("Time out waiting for sleep mode set.\n");
            exit(0);
        }

        // Make sure opmode is RCM
        if (rcmOpModeSet(RCM_OPMODE_RCM) != 0)
        {
            printf("Time out waiting for opmode set.\n");
            exit(0);
        }


        // execute Built-In Test - verify that radio is healthy
        if (rcmBit(&status) != 0)
        {
            printf("Time out waiting for BIT.\n");
            exit(0);
        }

        if (status != OK)
        {
            printf("Built-in test failed - status %d.\n", status);
            exit(0);
        }
        else
        {
            printf("Radio passes built-in test.\n\n");
        }

        // retrieve config from RCM
        if (rcmConfigGet(&config) != 0)
        {
            printf("Time out waiting for config confirm.\n");
            exit(0);
        }

        // print out configuration
        printf("Configuration:\n");
        printf("\tnodeId: %d\n", config.nodeId);
        printf("\tintegrationIndex: %d\n", config.integrationIndex);
        printf("\tantennaMode: %d\n", config.antennaMode);
        printf("\tcodeChannel: %d\n", config.codeChannel);
        printf("\telectricalDelayPsA: %d\n", config.electricalDelayPsA);
        printf("\telectricalDelayPsB: %d\n", config.electricalDelayPsB);
        printf("\tflags: 0x%X\n", config.flags);
        printf("\ttxGain: %d\n", config.txGain);

        // retrieve status/info from RCM
        if (rcmStatusInfoGet(&statusInfo) != 0)
        {
            printf("Time out waiting for status info confirm.\n");
            exit(0);
        }

        // print out status/info
        printf("\nStatus/Info:\n");
        printf("\tPackage version: %s\n", statusInfo.packageVersionStr);
        printf("\tRCM version: %d.%d build %d\n", statusInfo.appVersionMajor,
               statusInfo.appVersionMinor, statusInfo.appVersionBuild);
        printf("\tUWB Kernel version: %d.%d build %d\n", statusInfo.uwbKernelVersionMajor,
               statusInfo.uwbKernelVersionMinor, statusInfo.uwbKernelVersionBuild);
        printf("\tFirmware version: %x/%x/%x ver %X\n", statusInfo.firmwareMonth,
               statusInfo.firmwareDay, statusInfo.firmwareYear,
               statusInfo.firmwareVersion);
        printf("\tSerial number: %08X\n", statusInfo.serialNum);
        printf("\tBoard revision: %c\n", statusInfo.boardRev);
        printf("\tTemperature: %.2f degC\n\n", statusInfo.temperature / 4.0);
    }
//---------------------------------------------P410 serial interface initialization done-------------------------------

    //initialize trilateration object
    trilat_Obj.initialize();

    //Reset the intial position
    for (int i = 0; i < 3; i++)
    {
        trilatPos[i] = 0;
        initialPos[i] = 0;
    }

    long failedInitiationCount = 0;

    //get the initial position by multiple trilaterations
    for (int i = 0; i < TRILAT_TIMES; i++)
    {

        //do the trilateration everytime a quadruple of distances is aquired
        for (int nodeId = 1; nodeId < 5; nodeId++)
        {
            //interpreting nodeID to anchor's ID
            switch (nodeId)
            {
            case 1:
                destNodeId = 101;
                break;
            case 2:
                destNodeId = 102;
                break;
            case 3:
                destNodeId = 105;
                break;
            case 4:
                destNodeId = 106;
                break;
            default:
                break;
            }

            //Make the measurement
            rcmRangeTo(destNodeId, RCM_ANTENNAMODE_TXA_RXA, 0, NULL, &rangeInfo, &dataInfo, &scanInfo, &fullScanInfo);

            //Value must be less than 15m
            if ((rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION) && (rangeInfo.precisionRangeMm < 15000))
            {
                if (i == 0)
                    tempDists[nodeId - 1] = rangeInfo.precisionRangeMm / 1000.0; //Range measurements are in mm
                else if (abs(rangeInfo.precisionRangeMm / 1000.0 - tempDists[nodeId - 1]) < 1)
                {
                    tempDists[nodeId - 1] = rangeInfo.precisionRangeMm / 1000.0; //Range measurements are in mm
                }
                else
                {
                    nodeId = 0;
                    failedInitiationCount++;
                    if (failedInitiationCount == 10)
                    {
                        //if failing to go a head due to too much difference with previous value, then previous range
                        //appears to be wrong, reverse to previous update
                        i--;
                        failedInitiationCount = 0;
                        continue;
                    }
                    continue;
                }
            }
            //if any ranging fails, restart from the 1st range
            else
            {
                nodeId = 0;
                continue;
            }
        }
        printf("iteration #%d:\td1 = %f, d2 = %f, d3 = %f, d4 = %f;\n", i, tempDists[0], tempDists[1], tempDists[2], tempDists[3]);
        //add up the measurements to cancel noise
        for (int j = 0; j < 4; j++)
            dists[j] += tempDists[j];
    }

    //averaging the measurements to get the initial position for the ekf
    for (int j = 0; j < 4; j++)
        dists[j] /= TRILAT_TIMES;
    printf("average ranges , d1 = %f; d2 = %f; d3 = %f; d4 = %f\n", dists[0], dists[1], dists[2], dists[3]);

    //trilatering
    trilat_Obj.step(ancs, dists, initialPos);

    printf(" init pos: x = %f, y = %f, z = %f\n", initialPos[0], initialPos[1], initialPos[2]);

    //Assign initial positions and anchor locations to the ekf
    for (int i = 0; i < 12; i++)
        ekf_Obj.ekf_mf_P.ancs[i] = ancsTranspose[i];
    for (int i = 0; i < 3; i++)
        ekf_Obj.ekf_mf_P.ekf_mf_x_hat0[i] = initialPos[i];
    ekf_Obj.ekf_mf_P.R_InitialValue = 0.04;
    ekf_Obj.ekf_mf_P.acc_xy_InitialValue = 5.0;
    ekf_Obj.ekf_mf_P.acc_z_InitialValue = 2.0;
    for (int i = 0; i < 5; i++)
    {
        ekf_Obj.ekf_mf_P.initDists_InitialValue[i * 4] = dists[0];
        ekf_Obj.ekf_mf_P.initDists_InitialValue[i * 4 + 1] = dists[1];
        ekf_Obj.ekf_mf_P.initDists_InitialValue[i * 4 + 2] = dists[2];
        ekf_Obj.ekf_mf_P.initDists_InitialValue[i * 4 + 3] = dists[3];
    }

    //initialize ekf object
    ekf_Obj.initialize();

    fout << "i=" << 1 << "; d=" << dists[0] << "; x=" << initialPos[0] << "; y=" << initialPos[1] << "; z=" << initialPos[2] << "; ";
    fout << "p1=" << ekf_Obj.ekf_mf_P.P_0[0] << "; p2=" << ekf_Obj.ekf_mf_P.P_0[7] << "; p3=" << ekf_Obj.ekf_mf_P.P_0[14] << "; ";
    fout << "X=" << initialPos[0] << "; Y=" << initialPos[1] << "; Z=" << initialPos[2] << "; ";
    fout << "dt=" << deltat << "; loss=" << 0 << "; " << endl;

    fout << "i=[i " << 2 << "]; d=[d " << dists[1] << "]; x=[x " << initialPos[0] << "]; y=[y " << initialPos[1] << "]; z=[z " << initialPos[2] << "]; ";
    fout << "p1=[p1 " << ekf_Obj.ekf_mf_P.P_0[0] << "]; p2=[p2 " << ekf_Obj.ekf_mf_P.P_0[7] << "]; p3=[p3 " << ekf_Obj.ekf_mf_P.P_0[14] << "]; ";
    fout << "X=[X " << initialPos[0] << "]; Y=[Y " << initialPos[1] << "]; Z=[Z " << initialPos[2] << "]; ";
    fout << "dt=[dt " << deltat << "]; loss=[loss " << 0 << "]; " << endl;

    fout << "i=[i " << 3 << "]; d=[d " << dists[2] << "]; x=[x " << initialPos[0] << "]; y=[y " << initialPos[1] << "]; z=[z " << initialPos[2] << "]; ";
    fout << "p1=[p1 " << ekf_Obj.ekf_mf_P.P_0[0] << "]; p2=[p2 " << ekf_Obj.ekf_mf_P.P_0[7] << "]; p3=[p3 " << ekf_Obj.ekf_mf_P.P_0[14] << "]; ";
    fout << "X=[X " << initialPos[0] << "]; Y=[Y " << initialPos[1] << "]; Z=[Z " << initialPos[2] << "]; ";
    fout << "dt=[dt " << deltat << "]; loss=[loss " << 0 << "]; " << endl;

    fout << "i=[i " << 4 << "]; d=[d " << dists[3] << "]; x=[x " << initialPos[0] << "]; y=[y " << initialPos[1] << "]; z=[z " << initialPos[2] << "]; ";
    fout << "p1=[p1 " << ekf_Obj.ekf_mf_P.P_0[0] << "]; p2=[p2 " << ekf_Obj.ekf_mf_P.P_0[7] << "]; p3=[p3 " << ekf_Obj.ekf_mf_P.P_0[14] << "]; ";
    fout << "X=[X " << initialPos[0] << "]; Y=[Y " << initialPos[1] << "]; Z=[Z " << initialPos[2] << "]; ";
    fout << "dt=[dt " << deltat << "]; loss=[loss " << 0 << "]; " << endl;

    nodeId = 1;
    bool lastRangingSuccesful = true;
    int loopCount = 1;
    int faultyRangingCount = 0;

    // enter loop to a ranging a node and broadcasting the resulting range
    nodeRate = atoi(rcmEkfRate.data());
    ros::Rate rate(nodeRate);
    while(ros::ok())
    {
        if (loopCount > 200)
        {
            loopCount = 0;
            faultyRangingCount = 0;
            //lastRangingSuccesful = true;
        }
        else
            loopCount++;

        //reset timer if last ranging was successful
        if (lastRangingSuccesful)
            timeStart = ros::Time::now();
        else
            faultyRangingCount++;

        switch (nodeId)
        {
        case 1:
            destNodeId = 101;
            break;
        case 2:
            destNodeId = 102;
            break;
        case 3:
            destNodeId = 105;
            break;
        case 4:
            destNodeId = 106;
            break;
        default:
            break;
        }

//        //tcflush(fd, TCIOFLUSH);
//        //Send request to vicon
//        if(handshakingMode)
//            write(fd,"d", 1);

//        //rate.sleep();
//        ros::Duration tempoSleep(0.020);
//        tempoSleep.sleep();

//        //Check if vicon data has arrived
//        if(msgFlag == UP)
//        {
////            ros::Duration tempoSleep(0.020);
////            tempoSleep.sleep();
//            msgFlag = DOWN;
//            rcvdBytesCount = read(fd,rcvdFrame, RCV_THRESHOLD);
//            //First, check if read is successful
//            if(rcvdBytesCount == -1)
//            {
//                printf(KRED"Buffer not ready... :( \n" RESET);
//                ros::Time recoveryTime = ros::Time::now();
//                uint8_T residue = RCV_THRESHOLD;
//                while(true)
//                {
//                    rcvdBytesCount = read(fd,rcvdFrame, residue);

///*                    if(rcvdBytesCount == RCV_THRESHOLD)
//                    {
//                        printf(KGRN"Frame comes after waiting\n" RESET);
//                        for(int i = 0; i < rcvdBytesCount; i++)
//                        {
//                            backupFrame[i] = rcvdFrame[i];
//                            printf(KGRN"%2x ", rcvdFrame[i]);
//                        }
//                        printf("\n"RESET);
//                    }
//                    else*/ if(rcvdBytesCount != -1)
//                    {
//                        printf(KYEL"Fragment of %d byte(s) arrives\n"RESET, rcvdBytesCount);
////                        for(int i = 0; i < rcvdBytesCount; i++)
////                        {
////                            backupFrame[RCV_THRESHOLD - residue + i] = rcvdFrame[i];
////                            printf(KGRN"%2x ", rcvdFrame[i]);
////                        }
////                        printf("\n"RESET);
//                        residue = residue - rcvdBytesCount;
//                        if(residue == 0)
//                        {
//                            viconUpdateFlag = UP;
////                            printf(KGRN"Concantenated frame: "RESET);
////                            for(int i = 0; i < RCV_THRESHOLD; i++)
////                                printf(KYEL"%2x ", backupFrame[i]);
////                            printf("\n"RESET);
//                            break;
//                        }
//                        //tcflush(fd, TCIOFLUSH);
//                        //break;
//                    }
//                    else if((ros::Time::now() - recoveryTime).toSec() > 0.01)
//                    {
//                        printf(KRED"Time out waiting for this frame!\n"RESET);
//                        tcflush(fd, TCIOFLUSH);
//                        break;
//                    }
//                }
//            }
//            //Second check if read returns full frame
//            else if(rcvdBytesCount < RCV_THRESHOLD)
//            {
//                //printf(KRED "%d\n" RESET, rcvdBytesCount);
//                printf(KRED "Only %d/%d bytes! Attempting to fetch the rest.. >,< \n" RESET, rcvdBytesCount, RCV_THRESHOLD);
//                for(int i = 0; i < rcvdBytesCount; i++)
//                //{
//                    backupFrame[i] = rcvdFrame[i];
//                    //printf(KYEL"%2x ", rcvdFrame[i]);
//                //}
//                //printf("\n"RESET);
////                ros::Duration tempoSleep(0.02);
////                tempoSleep.sleep();
//                ros::Time recoveryTime = ros::Time::now();
//                uint8_T residue = RCV_THRESHOLD - rcvdBytesCount;
//                uint8_T miniBuff = 0, retrievalCount = 0;
//                for (int i = 0; i < residue; i++)
//                {
//                    rcvdBytesCount = read(fd,&miniBuff, 1);
//                    if(rcvdBytesCount == -1)
//                    {
//                        if((ros::Time::now() - recoveryTime).toSec() > 0.01)
//                            i = residue;
//                        else
//                            i--;
//                    }
//                    else
//                    {
//                        retrievalCount++;
//                        backupFrame[RCV_THRESHOLD - residue + i] = miniBuff;
//                        //printf(KYEL"%2x ", miniBuff);
//                    }
//                }
//                //printf("\n"RESET);
//                if(retrievalCount == residue)
//                {
//                    printf(KGRN"Full retrival \n"RESET);
////                    for(int i = 0; i < RCV_THRESHOLD; i++)
////                        printf(KGRN"%2x ", backupFrame[i]);
////                    printf("\n"RESET);
//                    viconUpdateFlag = UP;
//                }
//                else
//                {
//                    printf(KRED"Frame segment lost forever!\n\n"RESET);
//                    tcflush(fd, TCIOFLUSH);
//                }

//            }
//            //Third, check if frame is valid
//            else
//            {
//                //Tracing the header
//                if(*(uint32_t *)(rcvdFrame) != (HEADER_NAN))
//                {
//                    printf(KYEL"Jumbled frame: "RESET);
//                    for(int i = 0; i < RCV_THRESHOLD; i++)
//                    {
//                        backupFrame[i] = rcvdFrame[i];
//                        printf(KYEL"%2x ", rcvdFrame[i]);
//                    }
//                    printf("\n"RESET);
//                    viconUpdateFlag = UP;
//                }
//                else // header = HEADER_NAN
//                {
//                    printf(KBLU "Recieved well-aligned message... ^,^!\n");
//                    for(int i = 0; i< RCV_THRESHOLD; i++)
//                        backupFrame[i] = rcvdFrame[i];
//                    viconUpdateFlag = UP;
//                }
//            }
//        }
//        //If no vicon data has arrived then notify and go on
//        else
//        {
//            printf(KWHT "No message has arrived...\n\n\n" RESET);
//        }

////        backupFrame[20] = 0xAA;
////        backupFrame[21] = 0xAA;
////        backupFrame[22] = 0x7F;
////        backupFrame[23] = 1;
////        backupFrame[24] = 2;
////        backupFrame[25] = 3;
////        backupFrame[26] = 4;
////        backupFrame[27] = 5;
////        backupFrame[28] = 6;
////        backupFrame[29] = 7;
////        backupFrame[30] = 8;
////        backupFrame[31] = 9;
////        backupFrame[32] = 10;
////        backupFrame[33] = 11;
////        backupFrame[34] = 12;
////        backupFrame[35] = 13;
////        backupFrame[36] = 14;
////        backupFrame[37] = 15;
////        backupFrame[38] = 16;
////        backupFrame[39] = 17;
////        backupFrame[0] = 18;
////        backupFrame[1] = 19;
////        backupFrame[2] = 20;
////        backupFrame[3] = 21;
////        backupFrame[4] = 22;
////        backupFrame[5] = 23;
////        backupFrame[6] = 24;
////        backupFrame[7] = 25;
////        backupFrame[8] = 26;
////        backupFrame[9] = 27;
////        backupFrame[10] = 28;
////        backupFrame[11] = 29;
////        backupFrame[12] = 30;
////        backupFrame[13] = 31;
////        backupFrame[14] = 32;
////        backupFrame[15] = 33;
////        backupFrame[16] = 34;
////        backupFrame[17] = 35;
////        backupFrame[18] = 36;
////        backupFrame[19] = 0xAA;

//        if(viconUpdateFlag == UP)
//        {
//            viconUpdateFlag = DOWN;

//            //Tracing the header
//            headerIndex = 0xFF;
//            //check if header is in the middle of frame
//            for(int i = 0; i < RCV_THRESHOLD - 4; i++)
//                if(*(uint32_t *)(backupFrame + i) == HEADER_NAN)
//                {
//                    headerIndex = i;
//                    //swapping segments
//                    for(int j = 0; j < RCV_THRESHOLD - headerIndex; j++)
//                        rcvdFrame[j] = backupFrame[headerIndex + j];
//                    for(int j = RCV_THRESHOLD - headerIndex; j < RCV_THRESHOLD; j++)
//                        rcvdFrame[j] = backupFrame[j - RCV_THRESHOLD + headerIndex];
//                    for(int j = 0; j < RCV_THRESHOLD; j++)
//                        backupFrame[j] = rcvdFrame[j];

////                    for(int j = 0; j < RCV_THRESHOLD; j++)
////                        printf(KYEL"%2x ", backupFrame[j]);
////                    printf("\n"RESET);

//                    break;
//                }
//            if(headerIndex == 0xFF)
//            {
//                printf(KYEL"Possible seperated header!\n");
//                for(int j = 0; j < 3; j++)
//                {
//                    //check if header is seperated to the two ends, only three cases
//                    uint8_T backupFrameLastByte = backupFrame[RCV_THRESHOLD-1];
//                    for(int i = RCV_THRESHOLD - 1; i > 0; i--)
//                        backupFrame[i] = backupFrame[i-1];
//                    backupFrame[0] = backupFrameLastByte;

//                    if(*(uint32_t *)(backupFrame) == HEADER_NAN)
//                    {
//                        headerIndex = RCV_THRESHOLD - j - 1;
//                        break;
//                    }
//                }
//            }
//            printf(KGRN"Header index: %d\n"RESET, headerIndex);

//            printf(KGRN"#");
//            printf("%#8x$", *(uint32_t *)(backupFrame));
//            for(int i = 1; i < RCV_THRESHOLD/4; i++)
//                printf("%f$", *(float *)(backupFrame + i*4));
//            printf("\n"RESET);

//            viconX = *(float *)(backupFrame + 4);
//            viconY = *(float *)(backupFrame + 8);
//            viconZ = *(float *)(backupFrame + 12);
//            viconXd = *(float *)(backupFrame + 16);
//            viconYd = *(float *)(backupFrame + 20);
//            viconZd = *(float *)(backupFrame + 24);
//        }

        //get the range to an anchor
        rcmRangeTo(destNodeId, RCM_ANTENNAMODE_TXA_RXA, 0, NULL, &rangeInfo, &dataInfo, &scanInfo, &fullScanInfo);
        //rate.sleep();
        //if measurement succeeds proceed to the ekf, otherwise move to the next anchor
        if ((rangeInfo.rangeMeasurementType & RCM_RANGE_TYPE_PRECISION) && rangeInfo.precisionRangeMm < 12000)
        {
            //calculate deltat
            timeEnd = ros::Time::now();
            deltat = (ros::Time::now() - timeStart).toSec();

            dists[nodeId - 1] = rangeInfo.precisionRangeMm / 1000.0; //Range measurements are in mm

            //step the model
            ekf_Obj.step(dists, deltat, 0, nodeId, x_est, 1.0, 3.0);
            //trilaterating to compare
            trilat_Obj.step(ancs, dists, trilatPos);

            //Print and log state values
            if (lastRangingSuccesful)
            {
                printf("i=[i %d]; d=[d %6.4f]; x=[x %6.4f]; y=[y %6.4f]; z=[z %6.4f]; p1=[p1 %6.4f]; p2=[p2 %6.4f]; p3=[p3 %6.4f]; dt=[dt %6.4f]; loss=[loss %4.2f];\n",
                       nodeId,
                       dists[nodeId - 1],
                        x_est[0], x_est[1], x_est[2],
                        ekf_Obj.ekf_mf_B.P_pre[0],
                        ekf_Obj.ekf_mf_B.P_pre[7],
                        ekf_Obj.ekf_mf_B.P_pre[14],
                        deltat,
                        faultyRangingCount / (double)loopCount * 100
                        );
                printf("X=[X %6.4f]; Y=[Y %6.4f]; Z=[Z %6.4f]\n", trilatPos[0], trilatPos[1], trilatPos[2]);
            }
            else
                printf("\nI=[I %d]; D=[D %6.4f]; X=[X %6.4f]; Y=[Y %6.4f]; Z=[Z %6.4f]; P1=[P1 %6.4f]; P2=[P2 %6.4f]; P3=[P3 %6.4f]; DT=[DT %6.4f]; LOSS=[LOSS %4.2f];\n\n",
                       nodeId,
                       dists[nodeId - 1],
                        x_est[0], x_est[1], x_est[2],
                        ekf_Obj.ekf_mf_B.P_pre[0],
                        ekf_Obj.ekf_mf_B.P_pre[7],
                        ekf_Obj.ekf_mf_B.P_pre[14],
                        deltat,
                        faultyRangingCount / (double)loopCount * 100
                        );

            //Prepare the frame to send to FCC
            U2F_ID      +=U2F_ID;
            U2F_LENGTH  = 26;
            U2F_X       = (float)x_est[0];
            U2F_Y       = (float)x_est[1];
            U2F_Z       = (float)x_est[2];
            U2F_DX      = (float)x_est[3];
            U2F_DY      = (float)x_est[4];
            U2F_DZ      = (float)x_est[5];

            //Find the maximum covariance of postions
            double maxP = ekf_Obj.ekf_mf_B.P_pre[0];
            if(maxP < ekf_Obj.ekf_mf_B.P_pre[1])
                maxP = ekf_Obj.ekf_mf_B.P_pre[1];
            if(maxP < ekf_Obj.ekf_mf_B.P_pre[2])
                maxP = ekf_Obj.ekf_mf_B.P_pre[2];

            uint16_T ambiRadius = (uint16_T)sqrt(2*maxP);;

            //Sphere of ambiguity
            U2F_REV1    = (uint8_t)(ambiRadius & 0x0F);
            U2F_REV2    = (uint8_t)ambiRadius>>8;

            //Checksum
            uint8_t CSA = 0, CSB = 0;
            for (uint8_T i = 0; i < ID_TO_RES2_LENGTH; i++)
            {
                CSA = CSA + uwb2FccBuff[2+i];
                CSB = CSB + CSA;
            }
            U2F_CS1 = CSA;
            U2F_CS2 = CSB;

            fout << "i = [i " << nodeId << "]; d = [d " << dists[nodeId - 1] << "]; x = [x " << x_est[0] << "]; y = [y " << x_est[1] << "]; z = [z " << x_est[2] << "]; ";
            fout << "p1 = [p1 " << ekf_Obj.ekf_mf_B.P_pre[0] << "]; p2 = [p2 "<<ekf_Obj.ekf_mf_B.P_pre[7]<<"]; p3 = [p3 "<<ekf_Obj.ekf_mf_B.P_pre[14]<<"]; ";
            fout << "X=[X " << trilatPos[0] << "]; Y=[Y " << trilatPos[1] << "]; Z=[Z " << trilatPos[2] << "]; ";
            fout << "dt = [dt "<<deltat<<"]; loss = [loss "<<faultyRangingCount / (double)loopCount * 100<<"]; " << endl;

            lastRangingSuccesful = true;
        }
        else
            //Raise this flag to not to reset timer on next loop
            lastRangingSuccesful = false;

        nodeId++;
        if (nodeId > 4)
            nodeId = 1;
    }

    //cleanup
    ekf_Obj.terminate();
    getchar();
    rcmIfClose();
    return 0;
}
