#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>
#include <time.h>
#include <math.h>

#include "./includes/ADS131A0X/ADS131A0x.h"
#include "FFT_SIL.h"

#define SIZE_SERIAL_BUFFER 40
#define ADC_BUFFER_SIZE 5
#define ADC_SAMPLE_RATE 500.0 // smapling rate(Hz)

//======FFT parameters====
// FFT_STAGE=15, FFT_SIZE=2^15=32768
#define FFT_STAGE 15
#define FFT_SIZE 32768 // FFT_size = (1u << FFT_STAGE)

uint16_t flag_FFT = 0;

// Input data array (from ADC, only real number)
double Volt_I[FFT_SIZE], Volt_Q[FFT_SIZE];
double Volt_Mod_IQ[FFT_SIZE];

double avg_I = 0.0, avg_Q = 0.0;
double max_I = 0.0, max_Q = 0.0;

// for Motion detection
uint16_t isMmotionDetected;
double thr_Motion = 0.01; // threshold of motiion detection from time-domain signal change(volt)

uint32_t max_time;
uint32_t num_FFT_exec = 0;

double SpctmFreq[FFT_SIZE];
double SpctmValue_I_chl[FFT_SIZE], SpctmValue_Q_chl[FFT_SIZE], SpctmValue_Mod_IQ[FFT_SIZE];

double *aryTF = NULL;
complex_t *aryRF = NULL;

// index of current SpctmFreq[i] for serial output
int index_freq = 0;

// The maximum index of SpctmFreq[i] of FFT spectrum

int index_freq_max;

vital_t HRRR_I, HRRR_Q, HRRR_MOD_IQ;

//===pigpio parameters=====
int status_ISR_Register;

//===============General purpose parameters===============

// loop count
uint32_t countDataAcq = 0;

// for calculating processing time
time_t t1;       //  Declare a "time_t" type variable
struct tm *nPtr; // Declare a "struct tm" type pointer
double START, END;

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

char serial_Data[SIZE_SERIAL_BUFFER] = {0};
char serial_Msg[SIZE_SERIAL_BUFFER] = {0};
char serial_Spctm[SIZE_SERIAL_BUFFER] = {0};

//===============General purpose Functions Declaration====================

void ISR_ADC(int gpio, int level, uint32_t tick);

// /=================================================================================
int main(int argc, char *argv[])
{
  complex_t I_Signal[FFT_SIZE];
  complex_t Q_Signal[FFT_SIZE];
  complex_t Mod_IQ[FFT_SIZE];

  // printf("argc=%d,", argc);
  // printf("argv=%s,%s,%s\n", argv[0], argv[1], argv[2]);

  /*
  When user execute the program by typing "sudo ./SIL_HBRR 3 77",
  the first argument argv[0] is program 's name "./SIL_HBRR",
  and the second argument argv[1] is "3"
  and the third argument argv[2] is "77" ,and so on...
  */

  if (argc == 2)
    max_time = atoi(argv[1]);
  else
    max_time = 300;

  // for RRHR only (0Hz ~ 3Hz) ; since FFT resolution= fs/N, if max. freq= 3Hz, index_max= 3/(fs/N)=3.0/0.01526 = 196.59 ~=197th
  // index_freq_max = (int)(3.0 / (ADC_SAMPLE_RATE / FFT_SIZE) + 0.5);
  index_freq_max = round(3.0 / (ADC_SAMPLE_RATE / FFT_SIZE));

  // Nyquist frequency = fs/2
  // index_freq_max = FFT_SIZE / 2;

  printf("Freq max= %d\n", index_freq_max);

  /*
  // Generate 1st datalog file name
  t1 = time(NULL);// Format tm type to string literal
  nPtr = localtime(&t1);
  strftime(strAry_filename_rawdata, 64, "Datalog/%Y_%m%d_%H%M%S_rawdata.csv", nPtr);
  strftime(strary_filename_FFT, 64, "Datalog/%Y_%m%d_%H%M%S_FFT.csv", nPtr);
  strftime(strary_filename_HRRR, 64, "Datalog/%Y_%m%d_%H%M%S_HRRR.csv", nPtr);

  pFile_ADC = fopen(strAry_filename_rawdata, "w");
  pFile_FFT = fopen(strary_filename_FFT, "w");
  pFile_HRRR = fopen(strary_filename_HRRR, "w");

  // Generate column name for HRRR file
  fprintf(pFile_HRRR, "%s,%s,%s\n", "NUM", "IQ_RR", "IQ_HR");
  */

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

  // pigpio library: Open new Com port, and save its handle variable
  SerialStatus = serOpen("/dev/ttyAMA1", 115200, 0);

  if (SerialStatus >= 0)
    printf("SERIAL open success at handle: %d!\n", SerialStatus);
  else
    printf("SERIAL open fail with error: %d!\n", SerialStatus); //-24= PI_NO_HANDLE, -72=PI_SER_OPEN_FAILED

  // serial write test
  /*
  for (int i = 0; i < 100; i++)
  {
    sprintf(serial_Msg, "@D%6.3f\n", sin(i / 6.2832));
    serWrite(SerialStatus, serial_Msg, 9);
    usleep(50000);
  }
  */
  // printf("SERIAL close success at handle: %d!\n", serClose(SerialStatus));

  // Initialize SPI with CS=0,speed=2MHz (SPI Mode is fixed to Mode1)
  ADS131A0x_setSPI(CS_0, 2000000);

  // Follwoing part should be put inside the while loop,
  // and it would start acquiring data after get "START" command form keyboard or RS-232 "%S"

  ADS131A0x_InitialADC(); // TBD: add fs=500Hz ,Vref=4.0 or 2.442,NCP=0 or 1 as input parameters
  ADS131A0x_Start();
  printf("Start Data Acquiring!\n ===================\n");

  // Enable interruption at GPIO_0
  status_ISR_Register = gpioSetISRFunc(17, FALLING_EDGE, 0, ISR_ADC);

  // show ISR initialization error message according to return value
  if (status_ISR_Register == 0)
  {
    // printf("ISR Registered OK!\n");
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

  aryRF = GenRF(FFT_STAGE);

  // TBD: it should be a thread that detect
  // if the newest FFT_SIZE point in the circular buffer had been update?
  // then it would do the FFT and RRHR analysis repeatedly

  /*
  //calculate processing time
  START = clock(); // Mark start time
  END = clock(); // Mark end time
  printf("FFT costs %6.3f s! \n", (END - START) / CLOCKS_PER_SEC); // Show processing time
  */

  // START infinite loop
  // while (1)
  while (num_FFT_exec < max_time)
  {
    // Perform FFT only if ADC has collected "enough number" of data points (say, 32768 points)
    if (flag_FFT == 1)
    {

      // increse the number of FFT execution by 1
      num_FFT_exec++;

      // reset count number of ADC data acquisition
      countDataAcq = 0;

      HRRR_I.RespRate = 0;
      HRRR_I.HrtRate = 0;
      HRRR_Q.RespRate = 0;
      HRRR_Q.HrtRate = 0;
      HRRR_MOD_IQ.RespRate = 0;
      HRRR_MOD_IQ.HrtRate = 0;

      // Convert raw data to complex-number
      // dbl_to_cplx(FFT_SIZE, Volt_I, I_Signal);
      // dbl_to_cplx(FFT_SIZE, Volt_Q, Q_Signal);
      avg_I = getAvg(Volt_I, FFT_SIZE);
      avg_Q = getAvg(Volt_Q, FFT_SIZE);
      max_I = Volt_I[FindMax(Volt_I, FFT_SIZE)];
      max_Q = Volt_Q[FindMax(Volt_Q, FFT_SIZE)];

      if (((max_I - avg_I) >= thr_Motion) | ((max_Q - avg_Q) >= thr_Motion))
        isMmotionDetected = 2;
      else
        isMmotionDetected = 0;

      cplx_Demod(FFT_SIZE, Volt_I, Volt_Q, Mod_IQ, avg_I, avg_Q);

      // HRRR_I = SIL_get_HRRR(FFT_STAGE, FFT_SIZE, I_Signal, fs, SpctmFreq, SpctmValue_I_chl, aryRF, 1);
      // HRRR_Q = SIL_get_HRRR(FFT_STAGE, FFT_SIZE, Q_Signal, fs, SpctmFreq, SpctmValue_Q_chl, aryRF, 1);
      HRRR_MOD_IQ = SIL_get_HRRR(FFT_STAGE, FFT_SIZE, Mod_IQ, ADC_SAMPLE_RATE, SpctmFreq, SpctmValue_Mod_IQ, aryRF, 1);

      // printf("I_chl RR:%d,HR:%d\n", HRRR_I.RespRate, HRRR_I.HrtRate);
      // printf("Q_chl RR:%d,HR:%d\n", HRRR_Q.RespRate, HRRR_Q.HrtRate);
      printf("IQ-Demod RR:%u,HR:%u, Motion:%u\n", HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate, isMmotionDetected);

      // Serial output RR/HR data
      sprintf(serial_Data, "@R%u,%u,%u\n", HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate, isMmotionDetected);
      serWrite(SerialStatus, serial_Data, strlen(serial_Data) + 1);
      // memset(serial_Data, 0, 40);

      flag_FFT = 2;

      // Output RR/HR data to csv file
      //  fprintf(pFile_HRRR, "%d,%u,%u\n", num_FFT_exec, HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate);

      // Dprecated!
      // fprintf(pFile_HRRR, "%d,%d,%d,%d,%d,%d,%d\n", num_FFT_exec, HRRR_I.RespRate, HRRR_I.HrtRate, HRRR_Q.RespRate, HRRR_Q.HrtRate, HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate);
      // fprintf(pFile_HRRR, "%d,%d,%d\n", num_FFT_exec, HRRR_Q.RespRate, HRRR_Q.HrtRate);

      // open new file for next round datalog (rawdata and FFT analysis result)
      // NOTE! TO BE DELETED! Datalog function will be moved to PC S/W
      if (num_FFT_exec >= 1 && num_FFT_exec < max_time)
      {
        // Generate 2nd~ final datalog file name
        /*
        t1 = time(NULL);
        nPtr = localtime(&t1);

        strftime(strAry_filename_rawdata, 64, "Datalog/%Y_%m%d_%H%M%S_rawdata.csv", nPtr);
        strftime(strary_filename_FFT, 64, "Datalog/%Y_%m%d_%H%M%S_FFT.csv", nPtr);

        // pFile_ADC = fopen(strAry_filename_rawdata, "w");
        // pFile_FFT = fopen(strary_filename_FFT, "w");
        // pFile_HRRR = fopen(strary_filename_HRRR, "w");
        */

        // reset spectrum X-axis and Y-axis datas
        /*
        memset(SpctmFreq, 0, FFT_SIZE);
        memset(SpctmValue_I_chl, 0, FFT_SIZE);
        memset(SpctmValue_Q_chl, 0, FFT_SIZE);
        memset(SpctmValue_Mod_IQ, 0, FFT_SIZE);
        */
      }
    }
  }
  // close file reference
  // fclose(pFile_ADC);
  // fclose(pFile_FFT);
  // fclose(pFile_HRRR);

  // Disable interruption at GPIO_0 through calling  "gpioSetISRFunc" function again
  // by passing a NULL function pointer
  gpioSetISRFunc(17, FALLING_EDGE, 0, NULL);

  printf("Stop Data Acquiring!\n ===================\n");
  return 0;
}

//================ISR for pigpio=======================
// NOTE: when RPi GPIO#0 detect Falling Edge, it would trigger ISR,
// that is, calling "ISR_ADC" function ,and pass 3 parameters to it (gpio,level,tick)

void ISR_ADC(int gpio, int level, uint32_t tick)
{
  // This function would be called on both Rising or Falling Edge,
  // but it would receive "level" parameter indicating the edge type(0=Falling ,1=Rising)

  // level=0 means Falling Edge
  if (level == 0)
  {

    ADS131A0x_GetADCData(1, aData_ADC);  // Mode1= Real ADC
    Volt_I[countDataAcq] = aData_ADC[0]; // channel 0
    Volt_Q[countDataAcq] = aData_ADC[1]; // channel 1

    // Volt_Mod_IQ[countDataAcq] = sqrt(pow(Volt_I[countDataAcq], 2) + pow(Volt_Q[countDataAcq], 2));

    // Here, the IQ complex signal= sqrt(I^2+Q^2)
    // However, it should be ADC1-avg_1, but the average value avg_1,avg_2 is still unknown at this time
    // so it lacks the DC offset calibration process
    ADC_Mod_IQ = sqrt(pow(Volt_I[countDataAcq], 2) + pow(Volt_Q[countDataAcq], 2));

    // Datalog: use fprintf to save data
    // fprintf(pFile_ADC, "%6.3f,%6.3f,%6.3f\n", aData_ADC[1], aData_ADC[2], ADC_Mod_IQ);

    /*
    NOTE: use "fwrite()"" would be faster if output data total length longer than 5 cahracter,
    e.g. "56789"=5Bytes, but fwrite use its integer size, i.e. "1"=4Bytes, "56789" still = 4Bytes!
    */

    /*Serial output*/
    sprintf(serial_Data, "@D%6.3f,%6.3f,%6.3f\n", Volt_I[countDataAcq], Volt_Q[countDataAcq], ADC_Mod_IQ);
    serWrite(SerialStatus, serial_Data, strlen(serial_Data) + 1);
    // memset(serial_Data, 0, 40);

    countDataAcq++;

    if (countDataAcq == FFT_SIZE)
    {
      flag_FFT = 1;
      countDataAcq = 0;
    }

    if ((countDataAcq % 500) == 0)
    {
      printf("%2d\n", countDataAcq / 500); // To be replaced by progress bar (*****----- 99%)

      // Test message serial out
      /*
      sprintf(serial_Msg, "@MRaw#%2d\n", countDataAcq / 500);
      serWrite(SerialStatus, serial_Msg, strlen(serial_Msg) + 1);
      memset(serial_Msg, 0, 40);
      */
    }

    if (flag_FFT == 2)
    {
      // Serial output FFT spectrum data

      if (index_freq == 0)
        sprintf(serial_Spctm, "@F0,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
      else if (index_freq > 0 && index_freq < index_freq_max - 1)
        sprintf(serial_Spctm, "@F1,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
      else if (index_freq == (index_freq_max - 1))
        sprintf(serial_Spctm, "@F2,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);

      serWrite(SerialStatus, serial_Spctm, strlen(serial_Spctm) + 1);
      index_freq++;

      if (index_freq == index_freq_max)
      {
        printf("FFT Spectrum output OK!\n");
        flag_FFT = 0;
        index_freq = 0;
      }
      /*
      for (int index_freq = 0; index_freq < index_freq_max; index_freq++)
      {
        // Output spectrum data to csv file
        // fprintf(pFile_FFT, "%d,%6.4f,%6.3f,%6.3f,%f\n", index_freq, SpctmFreq[index_freq], SpctmValue_I_chl[index_freq], SpctmValue_Q_chl[index_freq], SpctmValue_Mod_IQ[index_freq]);

        if (index_freq == 0)
        {
          sprintf(serial_Spctm, "@F0,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
          serWrite(SerialStatus, serial_Spctm, strlen(serial_Spctm) + 1);
          // memset(serial_Spctm, 0, 40);
        }
        else if (index_freq > 0 && index_freq < index_freq_max - 1)
        {
          sprintf(serial_Spctm, "@F1,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
          serWrite(SerialStatus, serial_Spctm, strlen(serial_Spctm) + 1);
          // memset(serial_Spctm, 0, 40);
        }
        else if (index_freq == (index_freq_max - 1))
        {
          sprintf(serial_Spctm, "@F2,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
          serWrite(SerialStatus, serial_Spctm, strlen(serial_Spctm) + 1);
          // memset(serial_Spctm, 0, 40);
        }
      }*/
    }
  }
}

//-------------------------------------------------------
