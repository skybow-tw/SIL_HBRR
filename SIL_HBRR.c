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
// STAGE=15, SIZE_DATA=2^15=32768
// #define SIZE_DATA 32768
#define SIZE_DATA 65536
#define STAGE 16

double fs = 500.0; // smapling rate (Hz)
double FFT_resl;

double SpctmValue_I_chl[SIZE_DATA], SpctmValue_Q_chl[SIZE_DATA];
double W_N[SIZE_DATA];
double *aryTF = NULL;
complex_t *aryRF = NULL;

double SpctmFreq[SIZE_DATA];
// Input data array (from ADC, only real number)
double Volt_I[SIZE_DATA], Volt_Q[SIZE_DATA];

//=====Human vital signs analysis algorithm parameters=====
int index_RR, index_HR;

int lower_limit_RR, upper_limit_RR, lower_limit_HR, upper_limit_HR;
int size_RR_data, size_HR_data;
int index_freq;
vital_t hrrr_I_chl, hrrr_Q_chl;

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

float aData_ADC[4] = {0};
FILE *pFile_ADC, *pFile_DFT;
char strAry_filename_rawdata[64] = {};
char strary_filename_DFT[64] = {};
//===============General purpose Functions Declaration====================
void BufferClean(char *);
int compare_double(const void *arg1, const void *arg2);
int compare_Spctm(const void *arg1, const void *arg2);
int FindMax(double *array, uint32_t size);

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

    countDataAcq++;

    if ((countDataAcq % 500) == 0)
      printf("%d\n", countDataAcq / 500); // To be replaced by RPi serial library

    // printf("%d\n", ++countDataAcq); //To be replaced by RPi serial library

    // printf("%6.3f,%6.3f \n", aData_ADC[1], aData_ADC[2]);

    // CHANGE to "fwrite()"" would be faster if output data total length longer than 5 cahracter, e.g. "56789"=5Bytes,
    // but fwrite use its integer size, i.e. "1"=4Bytes, "56789" still = 4Bytes!
    fprintf(pFile_ADC, "%6.3f,%6.3f \n", aData_ADC[1], aData_ADC[2]);
  }
}

// /=================================================================================
int main(int argc, char *argv[])
{

  //  Declare a "time_t" type variable
  time_t t1 = time(NULL);

  // Declare a "struct tm" type pointer
  struct tm *nPtr = localtime(&t1);

  // Format tm type to string literal
  strftime(strAry_filename_rawdata, 64, "Datalog/%Y_%m%d_%H%M%S_rawdata.csv", nPtr);
  strftime(strary_filename_DFT, 64, "Datalog/%Y_%m%d_%H%M%S_DFT.csv", nPtr);

  pFile_ADC = fopen(strAry_filename_rawdata, "w");
  pFile_DFT = fopen(strary_filename_DFT, "w");

  // pigpio.h initializing Function
  if (gpioInitialise() < 0)
  {
    printf("PI_INIT_FAILED!\n");
    return 1;
  }
  else
  {
    printf("pigpio initialize success!\n");
  }

  // int SerialStatus = 999;

  // SerialStatus = serOpen("/dev/ttyS0", 9600, 0);

  // if (SerialStatus >= 0)
  //   printf("SERIAL open success at handle: %d!\n", SerialStatus);
  // else
  //   printf("SERIAL open fail with error: %d!\n", SerialStatus); //-24= PI_NO_HANDLE, -72=PI_SER_OPEN_FAILED

  // for (int i = 0; i < 5; i++)
  // {
  //   serWrite(SerialStatus, "skybow", 7);
  //   sleep(1);
  //   printf("write # %d\n", i);
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
  while (countDataAcq < SIZE_DATA)
  {
    // Continuously detect GPIO interruption until gathering SIZE_DATA points data from ADC.
  }

  // Disable interruption at GPIO_0 through calling  "gpioSetISRFunc" function again
  // by passing a NULL function pointer
  gpioSetISRFunc(17, FALLING_EDGE, 0, NULL);

  printf("Stop Data Acquiring!\n ===================\n");

  // calculate processing time

  /* DFT calculation
  // This TF should be calculated only once in the beginning
  aryTF = GenTwiddleFactor(SIZE_DATA);

  // DFT, input I signal=Volt_I,Outpust spectrum=SpctmValue_I_chl
  // Set isRRHB=1 to perform FFT at human RR/HB detection mode
  DFT(Volt_I, SIZE_DATA, fs, SpctmFreq, SpctmValue_I_chl, aryTF, 1);
  DFT(Volt_Q, SIZE_DATA, fs, SpctmFreq, SpctmValue_Q_chl, aryTF, 1);
  */

  // FFT for SIL HB/RR detection
  aryRF = GenRF(STAGE);

  // Mark start time
  START = clock();

  hrrr_I_chl.RespRate = 0;
  hrrr_I_chl.HrtRate = 0;
  hrrr_Q_chl.RespRate = 0;
  hrrr_Q_chl.HrtRate = 0;

  FFT_SIL(Volt_I, STAGE, fs, SpctmFreq, SpctmValue_I_chl, aryRF, 1);
  FFT_SIL(Volt_Q, STAGE, fs, SpctmFreq, SpctmValue_Q_chl, aryRF, 1);

  // Mark end time
  END = clock();

  // Show processing time
  printf("Complete! It costs %f seconds! \n", (END - START) / CLOCKS_PER_SEC);

  // fs/N=0.01526
  FFT_resl = fs / SIZE_DATA;
  lower_limit_RR = 0.05 / FFT_resl; // RR=0.05~0.5HZ(3~30 pm), so 0.05/0.01526=3.276 ~=3
  upper_limit_RR = 0.5 / FFT_resl;  // 0.5/0.01526=32.765 ~=33
  size_RR_data = upper_limit_RR - lower_limit_RR + 1;
  double *SpctmValue_RR_I_chl = calloc(size_RR_data, sizeof(double));
  double *SpctmValue_RR_Q_chl = calloc(size_RR_data, sizeof(double));

  lower_limit_HR = 0.8 / FFT_resl; // HB =0.8~4HZ (48~240bpm), so 0.8/0.01526=52.42 ~=52
  upper_limit_HR = 4 / FFT_resl;   // 4/0.01526 = 262.12 ~=262
  size_HR_data = upper_limit_HR - lower_limit_HR + 1;
  double *SpctmValue_HR_I_chl = calloc(size_HR_data, sizeof(double));
  double *SpctmValue_HR_Q_chl = calloc(size_HR_data, sizeof(double));

  for (index_freq = 0; index_freq < size_RR_data; index_freq++)
  {
    SpctmValue_RR_I_chl[index_freq] = SpctmValue_I_chl[index_freq + lower_limit_RR];
    SpctmValue_RR_Q_chl[index_freq] = SpctmValue_Q_chl[index_freq + lower_limit_RR];
  }

  for (index_freq = 0; index_freq < size_HR_data; index_freq++)
  {
    SpctmValue_HR_I_chl[index_freq] = SpctmValue_I_chl[index_freq + lower_limit_HR];
    SpctmValue_HR_Q_chl[index_freq] = SpctmValue_Q_chl[index_freq + lower_limit_HR];
  }

  // I channel
  index_RR = FindMax(SpctmValue_RR_I_chl, size_RR_data);
  index_HR = FindMax(SpctmValue_HR_I_chl, size_HR_data);

  hrrr_I_chl.RespRate = (int)(SpctmFreq[index_RR + lower_limit_RR] * 60);
  hrrr_I_chl.HrtRate = (int)(SpctmFreq[index_HR + lower_limit_HR] * 60);

  // Q channel
  index_RR = FindMax(SpctmValue_RR_Q_chl, size_RR_data);
  index_HR = FindMax(SpctmValue_HR_Q_chl, size_HR_data);

  hrrr_Q_chl.RespRate = (int)(SpctmFreq[index_RR + lower_limit_RR] * 60);
  hrrr_Q_chl.HrtRate = (int)(SpctmFreq[index_HR + lower_limit_HR] * 60);

  // printf("i_HR:%d,i_RR:%d\n", index_RR, index_HR);
  printf("I_chl RR:%d,HR:%d\n", hrrr_I_chl.RespRate, hrrr_I_chl.HrtRate);
  printf("Q_chl RR:%d,HR:%d\n", hrrr_Q_chl.RespRate, hrrr_Q_chl.HrtRate);
  free(SpctmValue_RR_I_chl);
  free(SpctmValue_RR_Q_chl);
  free(SpctmValue_HR_I_chl);
  free(SpctmValue_HR_Q_chl);

  for (index_freq = 0; index_freq < SIZE_DATA; index_freq++)
    // for (int index_freq = 0; index_freq < 100; index_freq++)
    fprintf(pFile_DFT, "%d,%6.4f,%6.3f,%6.3f\n", index_freq, SpctmFreq[index_freq], SpctmValue_I_chl[index_freq], SpctmValue_Q_chl[index_freq]);

  fclose(pFile_ADC);
  fclose(pFile_DFT);

  // Quick sort the power spectrum array
  // int Spctm_index[SIZE_DATA];
  // qsort(Spctm_index, SIZE_DATA, sizeof(int), compare_Spctm);

  // printf("MAX freq value is %6.4f \n", SpctmValue_I_chl[SIZE_DATA - 1]);

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

int compare_double(const void *arg1, const void *arg2)
{
  double ret = *(double *)(arg1) - *(double *)(arg2);
  if (ret == 0)
    return 0;
  else if (ret > 0)
    return 1;
  else
    return -1;
  // return (*(double *)(arg1) - *(double *)(arg2));//NOT for double variable
}

int compare_Spctm(const void *arg1, const void *arg2)
{
  double ret = *(double *)(arg1) - *(double *)(arg2);
  if (ret == 0)
    return 0;
  else if (ret > 0)
    return 1;
  else
    return -1;
  // return (*(double *)(arg1) - *(double *)(arg2));//NOT for double variable
}

int FindMax(double *array, uint32_t size)
{
  double max = 0.0;
  unsigned int i, index_max = 0;

  for (i = 0; i < size; i++)
  {
    if (array[i] > max)
    {
      // If current value is greater than max
      // value then replace it with max value
      max = array[i];
      index_max = i;
    }
  }
  return index_max;
}
