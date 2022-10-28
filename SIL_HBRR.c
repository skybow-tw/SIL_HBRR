#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>
#include <time.h>
#include <math.h>

#include "./includes/ADS131A0X/ADS131A0x.h"
#include "FFT_SIL.h"

#define SIZE_SERIAL_BUFFER 10

//======FFT parameters====
// FFT_STAGE=15, FFT_SIZE=2^15=32768
#define FFT_STAGE 15
#define FFT_SIZE 32768
// uint32_t FFT_size = (1u << FFT_STAGE);

// Input data array (from ADC, only real number)
double Volt_I[FFT_SIZE], Volt_Q[FFT_SIZE];

uint64_t max_time;
uint16_t num_FFT_exec;

double fs = 500.0; // smapling rate (Hz)

double SpctmFreq[FFT_SIZE];
double SpctmValue_I_chl[FFT_SIZE], SpctmValue_Q_chl[FFT_SIZE], SpctmValue_Mod_IQ[FFT_SIZE];

double *aryTF = NULL;
complex_t *aryRF = NULL;

// //=====Human vital signs analysis algorithm parameters=====
// int index_RR, index_HR;

// int lower_limit_RR, upper_limit_RR, lower_limit_HR, upper_limit_HR;
// int size_RR_data, size_HR_data;
// int index_freq;
vital_t HRRR_I, HRRR_Q, HRRR_MOD_IQ;

//===pigpio parameters=====
int status_ISR_Register;

//===============General purpose parameters===============
char DataIn[SIZE_SERIAL_BUFFER]; // Serial in data buffer;
char arySerialOutMsg[100] = {0}; // Traditional C char array for serial communication about ADS131A04 SysCmd

// loop count
uint8_t isBufferAvailable = 0;
uint32_t countDataAcq = 0;
// uint32_t countWhileLoop = 0;
// uint32_t countDataAcq_Th = 500 * 10;
// loop wait time
int time_message_wait; // milliseconds
int time_loop_halt;    // milliseconds
double START, END;     // for calculating processing time

uint8_t Status_SerialOut; // Flag to start reading (Status_SerialOut:1=>output on;   Status_SerialOut:0=>output off!)
uint8_t Size_Serial;

// ADC 4-channel current data
float aData_ADC[4] = {0};
float ADC_Mod_IQ = 0.0;

// datalog file name
FILE *pFile_ADC, *pFile_FFT, *pFile_HRRR;
char strAry_filename_rawdata[64] = {};
char strary_filename_FFT[64] = {};
char strary_filename_HRRR[64] = {};
//=============Serial UART===========
int SerialStatus = -1;
char aryUARTData[100] = {0};
//===============General purpose Functions Declaration====================

time_t t1;       //  Declare a "time_t" type variable
struct tm *nPtr; // Declare a "struct tm" type pointer

void BufferClean(char *);

//================ISR for pigpio=======================
// NOTE: when RPi GPIO#0 detect Falling Edge, it would trigger ISR,
// that is, calling "myInterrupt0" function ,and pass 3 parameters to it (gpio,level,tick)

void myInterrupt0(int gpio, int level, uint32_t tick)
{
  // This function would be called on both Rising or Falling Edge,
  // but it would receive "level" parameter indicating the edge type(0=Falling ,1=Rising)

  // level=0 means Falling Edge
  if (level == 0)
  {
    ADS131A0x_GetADCData(1, aData_ADC);  // Mode1= Real ADC
    Volt_I[countDataAcq] = aData_ADC[1]; // channel 2
    Volt_Q[countDataAcq] = aData_ADC[2]; // channel 3
    ADC_Mod_IQ = sqrt(pow(aData_ADC[1], 2) + pow(aData_ADC[2], 2));

    sprintf(aryUARTData, "@DI%6.3f,Q%6.3f,M%6.3f\n", aData_ADC[1], aData_ADC[2], ADC_Mod_IQ);
    serWrite(SerialStatus, aryUARTData, 27);

    countDataAcq++;

    if ((countDataAcq % 500) == 0)
    {
      printf("%2d\n", countDataAcq / 500); // To be replaced by RPi serial library
    }
    // if ((countDataAcq % 250) == 0)
    // {
    //   printf("Volt_I:%f, Volt_Q:%f\n", Volt_I[countDataAcq], Volt_Q[countDataAcq]);
    // }

    // printf("%d\n", ++countDataAcq); //To be replaced by RPi serial library

    // printf("%6.3f,%6.3f \n", aData_ADC[1], aData_ADC[2]);

    // CHANGE to "fwrite()"" would be faster if output data total length longer than 5 cahracter, e.g. "56789"=5Bytes,
    // but fwrite use its integer size, i.e. "1"=4Bytes, "56789" still = 4Bytes!
    fprintf(pFile_ADC, "%6.3f,%6.3f,%6.3f\n", aData_ADC[1], aData_ADC[2], ADC_Mod_IQ);
  }
}

// /=================================================================================
int main(int argc, char *argv[])
{
  complex_t I_Signal[FFT_SIZE];
  complex_t Q_Signal[FFT_SIZE];
  complex_t Mod_IQ[FFT_SIZE];

  // printf("argc=%d,", argc);
  // printf("argv=%s,%s,%s\n", argv[0], argv[1], argv[2]);

  // When user execute the program by typing "sudo ./SIL_HBRR 3 77",
  // the first argument argv[0] is program 's name "./SIL_HBRR",
  // and the second argument argv[1] is "3"
  // and the third argument argv[2] is "77" ,and so on...

  if (argc == 2)
    max_time = atoi(argv[1]);
  else
    max_time = 30;

  t1 = time(NULL);
  nPtr = localtime(&t1);

  // Get current time, and generate "the first" datalog file name
  // Format tm type to string literal
  strftime(strAry_filename_rawdata, 64, "Datalog/%Y_%m%d_%H%M%S_rawdata.csv", nPtr);
  strftime(strary_filename_FFT, 64, "Datalog/%Y_%m%d_%H%M%S_FFT.csv", nPtr);
  strftime(strary_filename_HRRR, 64, "Datalog/%Y_%m%d_%H%M%S_HRRR.csv", nPtr);

  pFile_ADC = fopen(strAry_filename_rawdata, "w");
  pFile_FFT = fopen(strary_filename_FFT, "w");
  pFile_HRRR = fopen(strary_filename_HRRR, "w");

  // Generate column name for HRRR file
  fprintf(pFile_HRRR, "%s,%s,%s,%s,%s,%s,%s\n", "NUM", "I_RR", "I_HR", "Q_RR", "Q_HR", "IQ_RR", "IQ_HR");

  // pigpio.h initializing Function
  if (gpioInitialise() < 0)
  {
    printf("PI_INIT_FAILED!\n");
    return 1;
  }
  else
  {
    if (ads131_debug_on == 1)
      printf("pigpio initialize success!\n");
  }

  // pigpio library: Open new Com port, and save its handle variable
  SerialStatus = serOpen("/dev/ttyAMA1", 115200, 0);

  if (SerialStatus >= 0)
    printf("SERIAL open success at handle: %d!\n", SerialStatus);
  else
    printf("SERIAL open fail with error: %d!\n", SerialStatus); //-24= PI_NO_HANDLE, -72=PI_SER_OPEN_FAILED

  // serial write test
  //  for (int i = 0; i < 100; i++)
  //  {
  //    // sprintf(aryUARTData, "@D%5.3f\n", sin(i));
  //    sprintf(aryUARTData, "@D%6.3f\n", sin(i / 6.2832));

  //   // serWrite(SerialStatus, "data # \n", 7);
  //   serWrite(SerialStatus, aryUARTData, 9);
  //   // delay(100);
  //   usleep(50000);

  //   // sleep(1);
  // }
  // printf("SERIAL close success at handle: %d!\n", serClose(SerialStatus));

  //======

  // Initialize SPI with CS=0,speed=2MHz (SPI Mode is fixed to Mode1)
  ADS131A0x_setSPI(CS_0, 2000000);

  // Simulation
  isBufferAvailable = 1; // unused
  DataIn[0] = 'S';       // unused

  // Follwoing part should be put inside the while loop,
  // and it would start acquiring data after get "START" command form keyboard or RS-232 "%S"

  ADS131A0x_InitialADC();
  ADS131A0x_Start();
  printf("Start Data Acquiring!\n ===================\n");

  // Enable interruption at GPIO_0
  status_ISR_Register = gpioSetISRFunc(17, FALLING_EDGE, 0, myInterrupt0);

  // show ISR initialization error message according to return value
  if (status_ISR_Register == 0)
  {
    if (ads131_debug_on == 1)
      printf("ISR Registered OK!\n");
  }
  else
  {
    switch (status_ISR_Register)
    {
    case -3:
      printf("PI_BAD_GPIO!\n");
      break;
    case -123:
      printf("PI_BAD_ISR_INIT!\n");
      break;
    case -122:
      printf("PI_BAD_EDGE!\n");
      break;

    default:
      printf("error!\n");
    }
  }

  // This should be change to infinite loop, such as while(1){}?
  // NO! here, it should be a thread that detect
  // if the newest 500 point in the circular buffer had been update?
  // then it would do the FFT and HB/RR analysis again

  // for DFT
  // This TF should be calculated only once in the beginning
  // aryTF = GenTwiddleFactor(FFT_SIZE);

  aryRF = GenRF(FFT_STAGE);

  // =====Initialize temp variable for FFT=====

  // FFT_resl = fs / FFT_SIZE;         // fs/N=0.01526
  // lower_limit_RR = 0.08 / FFT_resl; // RR=0.08~0.7HZ(4.8~42 pm), so 0.08/0.01526=5.24 ~=5th
  // upper_limit_RR = 0.7 / FFT_resl;  // 0.7/0.01526=45.8752 ~=46th
  // size_RR_data = upper_limit_RR - lower_limit_RR + 1;

  // lower_limit_HR = 0.9 / FFT_resl; // HB =0.9~3.5HZ (54~210bpm), so 0.9/0.01526=58.98 ~=59th
  // upper_limit_HR = 3.5 / FFT_resl; // 3.5/0.01526 = 229.38 ~=229th
  // size_HR_data = upper_limit_HR - lower_limit_HR + 1;

  num_FFT_exec = 0;

  // START infinite loop!!!
  // while (1)
  while (num_FFT_exec < max_time)
  {
    // Continuously detect GPIO interruption until gathering FFT_SIZE points data from ADC.
    if (countDataAcq >= 1)
    {

      // Perform FFT only if ADC has collected "enough number" of data points (say, 32768 points)
      if ((countDataAcq % FFT_SIZE) == 0)
      {

        num_FFT_exec++;
        countDataAcq = 0;

        // t1 = time(NULL);
        // nPtr = localtime(&t1);

        // // Format tm type to string literal
        // strftime(strAry_filename_rawdata, 64, "Datalog/%Y_%m%d_%H%M%S_rawdata.csv", nPtr);
        // strftime(strary_filename_FFT, 64, "Datalog/%Y_%m%d_%H%M%S_DFT.csv", nPtr);

        // pFile_ADC = fopen(strAry_filename_rawdata, "w");
        // pFile_FFT = fopen(strary_filename_FFT, "w");

        // Mark start time
        // START = clock();

        HRRR_I.RespRate = 0;
        HRRR_I.HrtRate = 0;
        HRRR_Q.RespRate = 0;
        HRRR_Q.HrtRate = 0;
        HRRR_MOD_IQ.RespRate = 0;
        HRRR_MOD_IQ.HrtRate = 0;

        // DFT(Volt_I, FFT_SIZE, fs, SpctmFreq, SpctmValue_I_chl, aryTF, 1);

        // FFT for SIL HB/RR detection

        // FFT_SIL(Volt_I, FFT_STAGE, fs, SpctmFreq, SpctmValue_I_chl, aryRF, 1);
        // FFT_SIL(Volt_Q, FFT_STAGE, fs, SpctmFreq, SpctmValue_Q_chl, aryRF, 1);

        // NEW
        dbl_to_cplx(FFT_SIZE, Volt_I, I_Signal);
        dbl_to_cplx(FFT_SIZE, Volt_Q, Q_Signal);
        cplx_Demod(FFT_SIZE, Volt_I, Volt_Q, Mod_IQ);

        HRRR_I = SIL_get_HRRR(FFT_STAGE, FFT_SIZE, I_Signal, fs, SpctmFreq, SpctmValue_I_chl, aryRF, 1);
        HRRR_Q = SIL_get_HRRR(FFT_STAGE, FFT_SIZE, Q_Signal, fs, SpctmFreq, SpctmValue_Q_chl, aryRF, 1);
        HRRR_MOD_IQ = SIL_get_HRRR(FFT_STAGE, FFT_SIZE, Mod_IQ, fs, SpctmFreq, SpctmValue_Mod_IQ, aryRF, 1);

        // Mark end time
        // END = clock();

        // Show processing time
        // printf("FFT costs %6.3f s! \n", (END - START) / CLOCKS_PER_SEC);

        // TBD: This part should be move to the inside of the FFT function!!!
        // Find the maximum value in the specific spectrum segment , and assume the value as HR or RR

        // I channel
        /*
        index_RR = FindMax(SpctmValue_I_chl + lower_limit_RR, size_RR_data);
        index_HR = FindMax(SpctmValue_I_chl + lower_limit_HR, size_HR_data);
        HRRR_I.RespRate = round((SpctmFreq[index_RR + lower_limit_RR] * 60));
        HRRR_I.HrtRate = round((SpctmFreq[index_HR + lower_limit_HR] * 60));
        */

        // Q channel
        /*
        index_RR = FindMax(SpctmValue_Q_chl + lower_limit_RR, size_RR_data);
        index_HR = FindMax(SpctmValue_Q_chl + lower_limit_HR, size_HR_data);
        HRRR_Q.RespRate = round((SpctmFreq[index_RR + lower_limit_RR] * 60));
        HRRR_Q.HrtRate = round((SpctmFreq[index_HR + lower_limit_HR] * 60));
        */
        printf("I_chl RR:%d,HR:%d\n", HRRR_I.RespRate, HRRR_I.HrtRate);
        printf("Q_chl RR:%d,HR:%d\n", HRRR_Q.RespRate, HRRR_Q.HrtRate);
        printf("Demod RR:%d,HR:%d\n", HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate);
        fprintf(pFile_HRRR, "%d,%d,%d,%d,%d,%d,%d\n", num_FFT_exec, HRRR_I.RespRate, HRRR_I.HrtRate, HRRR_Q.RespRate, HRRR_Q.HrtRate, HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate);
        // fprintf(pFile_HRRR, "%d,%d,%d\n", num_FFT_exec, HRRR_Q.RespRate, HRRR_Q.HrtRate);

        // Output data to csv file
        for (int index_freq = 0; index_freq < FFT_SIZE / 2; index_freq++)
          fprintf(pFile_FFT, "%d,%6.4f,%6.3f,%6.3f,%6.3f\n", index_freq, SpctmFreq[index_freq], SpctmValue_I_chl[index_freq], SpctmValue_Q_chl[index_freq], SpctmValue_Mod_IQ[index_freq]);

        // open new file for next round datalog (rawdata and FFT analysis result)
        if (num_FFT_exec >= 1 && num_FFT_exec < max_time)
        {
          // Get current time, and generate datalog file name
          t1 = time(NULL);
          nPtr = localtime(&t1);
          strftime(strAry_filename_rawdata, 64, "Datalog/%Y_%m%d_%H%M%S_rawdata.csv", nPtr);
          strftime(strary_filename_FFT, 64, "Datalog/%Y_%m%d_%H%M%S_FFT.csv", nPtr);
          // strftime(strary_filename_HRRR, 64, "Datalog/%Y_%m%d_%H%M%S_HRRR.csv", nPtr);

          pFile_ADC = fopen(strAry_filename_rawdata, "w");
          pFile_FFT = fopen(strary_filename_FFT, "w");
          // pFile_HRRR = fopen(strary_filename_HRRR, "w");

          // reset spectrum X-axis and Y-axis datas
          memset(SpctmFreq, 0, FFT_SIZE);
          memset(SpctmValue_I_chl, 0, FFT_SIZE);
          memset(SpctmValue_Q_chl, 0, FFT_SIZE);
          memset(SpctmValue_Mod_IQ, 0, FFT_SIZE);
        }
        // end
      }
    }
  }
  // close file reference
  fclose(pFile_ADC);
  fclose(pFile_FFT);
  fclose(pFile_HRRR);

  // Disable interruption at GPIO_0 through calling  "gpioSetISRFunc" function again
  // by passing a NULL function pointer
  gpioSetISRFunc(17, FALLING_EDGE, 0, NULL);

  printf("Stop Data Acquiring!\n ===================\n");

  // calculate processing time

  /*
  // DFT, input I signal=Volt_I,Outpust spectrum=SpctmValue_I_chl
  // Set isRRHB=1 to perform FFT at human RR/HB detection mode
  DFT(Volt_I, FFT_SIZE, fs, SpctmFreq, SpctmValue_I_chl, aryTF, 1);
  DFT(Volt_Q, FFT_SIZE, fs, SpctmFreq, SpctmValue_Q_chl, aryTF, 1);
  */
  return 0;
}

//-------------------------------------------------------
// This function clean the UART input data buffer
void BufferClean(char *CharArray)
{
  int i;
  for (i = SIZE_SERIAL_BUFFER - 1; i >= 0; i--)
  {
    // can't use sizeof?
    CharArray[i] = 0;
  }
}
