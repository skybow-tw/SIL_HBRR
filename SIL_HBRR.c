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
// double Volt_Mod_IQ[FFT_SIZE];

// complex_t I_Signal[FFT_SIZE], Q_Signal[FFT_SIZE];
complex_t Mod_IQ[FFT_SIZE];

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
int index_freq;

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
int flag_Serial_out = 0;
int handle_Serial = -1;
int num_Serial_in = 0;

// Serial Device name full path setting
// NOTE: Add "dtoverlay=miniurt-bt" to the "/boot/config.txt" file while using the UART0 to transfer data,
//       which would switch the Bluetooth to use the mini-UART, and make UART0 as the Primary UART.
char *SerialDevName_UART0 = "/dev/ttyAMA0"; // UART0

// NOTE: Add "dtoverlay=uart3" to the "/boot/config.txt" file while using the UART3 to transfer data.
// Besides, the number "n" of the device name "/dev/ttyAMAn" does not equal to the "UARTn" number,
// e.g. UART3 is the first newly open UART, so it is named "AMA1"
char *SerialDevName_UART3 = "/dev/ttyAMA1"; // UART3

char arySerIn_Data[SIZE_SERIAL_BUFFER] = {0};
char arySerOut_ADC[SIZE_SERIAL_BUFFER] = {0};
char arySerOut_Pct[SIZE_SERIAL_BUFFER] = {0};
char arySerOut_Msg[SIZE_SERIAL_BUFFER] = {0};
char arySerOut_Spctm[SIZE_SERIAL_BUFFER] = {0};

//===============General purpose Functions Declaration====================

void ISR_ADC(int gpio, int level, uint32_t tick);

// /=================================================================================
int main(int argc, char *argv[])
{

  // printf("argc=%d,", argc);
  // printf("argv=%s,%s,%s\n", argv[0], argv[1], argv[2]);

  /*
  When user execute the program by typing "sudo ./SIL_HBRR 3 77",
  the first argument argv[0] is program 's name "./SIL_HBRR",
  and the second argument argv[1] is "3"
  and the third argument argv[2] is "77"
  */

  if (argc == 2)
    max_time = atoi(argv[1]);
  else
    max_time = 300;

  index_freq = 0;

  // for RRHR only (0Hz ~ 3Hz) ; since FFT resolution= fs/N, if max. freq= 3Hz, index_max= 3/(fs/N)=3.0/0.01526 = 196.59 ~=197th
  index_freq_max = round(3.0 / (ADC_SAMPLE_RATE / FFT_SIZE));
  // index_freq_max = FFT_SIZE / 2; // Nyquist frequency = fs/2, so index_freq_max= 16384

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
  // handle_Serial = serOpen("/dev/ttyAMA1", 115200, 0);
  handle_Serial = serOpen(SerialDevName_UART0, 115200, 0);

  if (handle_Serial >= 0)
    printf("Serial device open success at handle: %d!\n", handle_Serial);
  else
    printf("Serial device open failed with error: %d!\n", handle_Serial); //-24= PI_NO_HANDLE, -72=PI_SER_OPEN_FAILED

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

  /*
  //calculate processing time
  START = clock(); // Mark start time
  END = clock(); // Mark end time
  printf("FFT costs %6.3f s! \n", (END - START) / CLOCKS_PER_SEC); // Show processing time
  */

  // TBD: To be replaced by a POSIX thread
  // START infinite loop
  while (1)
  // while (num_FFT_exec < max_time)
  {

    num_Serial_in = serDataAvailable(handle_Serial);

    if (num_Serial_in >= 4)
    {
      // printf("Serial Data available=%d\n", num_Serial_in);
      serRead(handle_Serial, arySerIn_Data, num_Serial_in);

      // The enter of WINDOWS-10 would produce \r\n=CR(0x0D), LF(0x0A)
      // therefore, if PC output "S1" and press enter, Linux would receive "S1\r\n", that is 4 char.

      if (arySerIn_Data[0] == 'S')
      {
        switch (arySerIn_Data[1])
        {
        case '0':
          flag_Serial_out = 0;
          break;

        case '1':
          flag_Serial_out = 1;
          break;

        default:

          break;
        }
      }
      else
      {
        // printf("Serial input=%s\n", arySerIn_Data);
        printf("Unknown command!\n");
      }
    }
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
      if (flag_Serial_out == 1)
      {
        sprintf(arySerOut_ADC, "@R%u,%u,%u\n", HRRR_MOD_IQ.RespRate, HRRR_MOD_IQ.HrtRate, isMmotionDetected);
        serWrite(handle_Serial, arySerOut_ADC, strlen(arySerOut_ADC) + 1);
        // memset(arySerOut_ADC, 0, 40);
      }

      flag_FFT = 2; // trigger ISR to output FFT spectrum to PC through serial port(TTL to USB)
    }
  }

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

    // Here, the IQ complex signal= sqrt(I^2+Q^2);
    // however, it should be ADC1-avg_1 or ADC2-avg_2,
    // but the average value avg_1 and avg_2 were still unknown at this time,
    // so it lacks the DC offset calibration process.
    ADC_Mod_IQ = sqrt(pow(Volt_I[countDataAcq], 2) + pow(Volt_Q[countDataAcq], 2));

    // Datalog: use fprintf to save data
    // fprintf(pFile_ADC, "%6.3f,%6.3f,%6.3f\n", aData_ADC[1], aData_ADC[2], ADC_Mod_IQ);

    /*
    NOTE: use "fwrite()"" would be faster if output data total length longer than 5 cahracter,
    e.g. "56789"=5Bytes, but fwrite use its integer size, i.e. "1"=4Bytes, "56789" still = 4Bytes!
    */

    /*Serial output*/
    if (flag_Serial_out == 1)
    {
      sprintf(arySerOut_ADC, "@D%6.3f,%6.3f,%6.3f\n", Volt_I[countDataAcq], Volt_Q[countDataAcq], ADC_Mod_IQ);
      serWrite(handle_Serial, arySerOut_ADC, strlen(arySerOut_ADC) + 1);
      // memset(arySerOut_ADC, 0, 40);
    }
    countDataAcq++; // increse ADC data count number

    if ((countDataAcq % 500) == 0 && flag_Serial_out == 1)
    {

      // printf("@P%d\n", (int)(countDataAcq * 100.0 / 32768));
      sprintf(arySerOut_Pct, "@P%d\n", (int)(countDataAcq * 100.0 / 32768));
      serWrite(handle_Serial, arySerOut_Pct, strlen(arySerOut_Pct) + 1);
    }

    // Complete data acquiring, change flag for FFT calculation
    if (countDataAcq == FFT_SIZE)
    {
      flag_FFT = 1;
      countDataAcq = 0;
    }

    if (flag_FFT == 2)
    {

      // Serial output FFT spectrum data

      // METHOD 1: Divide and conquer
      if (index_freq == 0)
      {
        sprintf(arySerOut_Spctm, "@F0,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
        index_freq++;
      }
      else if (index_freq > 0 && index_freq < index_freq_max - 1)
      {
        sprintf(arySerOut_Spctm, "@F1,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
        index_freq++;
      }
      else if (index_freq == (index_freq_max - 1))
      {
        sprintf(arySerOut_Spctm, "@F2,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
        flag_FFT = 0;
        index_freq = 0;
      }

      if (flag_Serial_out == 3)
        serWrite(handle_Serial, arySerOut_Spctm, strlen(arySerOut_Spctm) + 1);

      /*
      //METHOD 2: Too time-consuming
      // NOTE: This would results in an obvious delay of GUI chart displaying of ADC datas output,
      // the reason is still uncertain, it might be the size of the spectrum magnitude are too large, say, >1,000,000

      for (int index_freq = 0; index_freq < index_freq_max; index_freq++)
      {
        // Output spectrum data to csv file
        // fprintf(pFile_FFT, "%d,%6.4f,%6.3f,%6.3f,%f\n", index_freq, SpctmFreq[index_freq], SpctmValue_I_chl[index_freq], SpctmValue_Q_chl[index_freq], SpctmValue_Mod_IQ[index_freq]);

        if (index_freq == 0)
        {
          sprintf(arySerOut_Spctm, "@F0,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
          serWrite(handle_Serial, arySerOut_Spctm, strlen(arySerOut_Spctm) + 1);
          // memset(arySerOut_Spctm, 0, 40);
        }
        else if (index_freq > 0 && index_freq < index_freq_max - 1)
        {
          sprintf(arySerOut_Spctm, "@F1,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
          serWrite(handle_Serial, arySerOut_Spctm, strlen(arySerOut_Spctm) + 1);
          // memset(arySerOut_Spctm, 0, 40);
        }
        else if (index_freq == (index_freq_max - 1))
        {
          sprintf(arySerOut_Spctm, "@F2,%6.4f,%f\n", SpctmFreq[index_freq], SpctmValue_Mod_IQ[index_freq]);
          serWrite(handle_Serial, arySerOut_Spctm, strlen(arySerOut_Spctm) + 1);
          // memset(arySerOut_Spctm, 0, 40);
        }
      }*/
    }
  }
}

//-------------------------------------------------------
