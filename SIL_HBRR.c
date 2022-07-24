#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>
#include <time.h>
#include <math.h>

#include "./includes/ADS131A0X/ADS131A0x.h"

#define SIZE_SERIAL_BUFFER 10

//======DFT parameters====
#define SIZE_DATA 4096
#define TwoPI 6.2832

int index_data;
double fs = 500.0; // smapling rate
double freq_resolution, freq_bin;
double W[SIZE_DATA]; // Twiddle Factor array
double Volt_I[SIZE_DATA], Volt_Q[SIZE_DATA];
double DFT_I_Re[SIZE_DATA / 2 + 1], DFT_I_Im[SIZE_DATA / 2 + 1];
double PwrSpctm_I[SIZE_DATA / 2 + 1];
// double Xp_Odd_Ch1[SIZE_DATA], Xp_Even_Ch1[SIZE_DATA];
// double Xp_Odd_Ch2[SIZE_DATA], Xp_Even_Ch2[SIZE_DATA];

//===pigpio parameters=====
int status_ISR_Register;

//===============General purpose parameters===============
char DataIn[SIZE_SERIAL_BUFFER]; // Serial in data buffer;
char arySerialOutMsg[100] = {0}; // Traditional C char array for serial communication about ADS131A04 SysCmd
double START, END;               // for calculating processing time

// loop count
uint8_t isBufferAvailable = 0;
uint32_t countDataAcq = 0;
// uint32_t countWhileLoop = 0;
uint32_t countDataAcq_Th = 500 * 10;
// loop wait time
int time_message_wait; // milliseconds
int time_loop_halt;    // milliseconds

uint8_t Status_SerialOut; // Flag to start reading (Status_SerialOut:1=>output on;   Status_SerialOut:0=>output off!)
uint8_t Size_Serial;

float aData_ADC[4] = {0};
FILE *pFile_ADC, *pFile_DFT;
char strAry_filename_rawdata[64] = {};
char strary_filename_DFT[64] = {};
//===============General purpose Functions Declaration====================
void BufferClean(char *);
int compare_double(const void *arg1, const void *arg2);

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

  // Declare a "struct tm" type variable
  struct tm *nPtr = localtime(&t1);

  // Format tm type to string literal
  strftime(strAry_filename_rawdata, 64, "%Y_%m%d_%H%M%S_rawdata_", nPtr);
  strftime(strary_filename_DFT, 64, "%Y_%m%d_%H%M%S_DFT", nPtr);

  // concatenate filename string with .csv
  strcat(strAry_filename_rawdata, ".csv");
  strcat(strary_filename_DFT, ".csv");

  pFile_ADC = fopen(strAry_filename_rawdata, "w");
  pFile_DFT = fopen(strary_filename_DFT, "w");

  // set DFT frequency resolution
  freq_resolution = fs / (double)SIZE_DATA;

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

  //======

  // Initialize SPI with CS=0,speed=2MHz (SPI Mode is fixed to Mode1)
  ADS131A0x_setSPI(CS_0, 2000000);

  // Simulation
  isBufferAvailable = 1;
  DataIn[0] = 'S';

  ADS131A0x_InitialADC();
  ADS131A0x_Start();

  printf("Start Data Acquiring!\n ===================\n");

  // Enable interruption at GPIO_0
  status_ISR_Register = gpioSetISRFunc(17, FALLING_EDGE, 0, myInterrupt0);
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
  /*
  NOTE: pigpio ISR use BCM2835 pin number!!!
  Initialize GPIO_0(BCM #17) with interrupt
  GPIO_0=Connector_#11=BCM_#17
  GPIO_1=Connector_#12=BCM_#18
  GPIO_2=Connector_#13=BCM_#27
  GPIO_3=Connector_#15=BCM_#22
  GPIO_4=Connector_#16=BCM_#23
  GPIO_5=Connector_#18=BCM_#24
  GPIO_6=Connector_#22=BCM_#25
  GPIO_7=Connector_#07=BCM_#04
  */

  // This should be change to infinite loop, such as while(1){}
  while (countDataAcq < SIZE_DATA)
  {
    // Continuously detect GPIO interruption until gathering SIZE_DATA points data from ADC.
  }

  // Disable interruption at GPIO_0 through calling  "gpioSetISRFunc" function again by passing a NULL function pointer
  gpioSetISRFunc(17, FALLING_EDGE, 0, NULL);

  printf("Stop Data Acquiring!\n ===================\n");
  START = clock();
  // fprintf(pFile_ADC, "%6.3f,%6.3f \n", aData_ADC[1], aData_ADC[2]);

  //==================
  // Generate Twiddle Factor array
  printf("Generating %dth-order W...\n", SIZE_DATA);

  for (index_data = 0; index_data < SIZE_DATA; ++index_data)
  {
    W[index_data] = cos(index_data * TwoPI / SIZE_DATA);
  }

  END = clock();
  printf(" It costs %f s. \n", (END - START) / CLOCKS_PER_SEC);

  //====
  // Calculta DFT
  // time and frequency domain data arrays
  int index_freq; // time and frequency domain indices

  // Calculate DFT and power spectrum up to Nyquist frequency
  int to_sin = 3 * SIZE_DATA / 4; // index offset for sin

  //

  //---for Real Part (COS(Theta))
  int index_W;
  //---for Imaginary Part 's offset (SIN(Theta)), because COS(Theta+3*PI/2)=SIN(Theta);
  // On the unit circle of COS(Theta) value, just go forward counterclockwise (offset) 3*PI/2,
  // you will get the value for SIN(Theta)
  // So, if 2PI->N points, then (3/2)*PI->3N/4 points

  int Offset_CosToSin = 3 * SIZE_DATA / 4;

  // calculate processing time
  printf("Starting DFT with %d th order...\n", SIZE_DATA);
  START = clock();

  for (index_freq = 0; index_freq <= SIZE_DATA / 2; ++index_freq)
  {
    DFT_I_Re[index_freq] = 0;
    DFT_I_Im[index_freq] = 0;

    for (index_data = 0; index_data < SIZE_DATA; ++index_data)
    {
      index_W = index_freq * index_data;

      DFT_I_Re[index_freq] += Volt_I[index_data] * W[index_W % SIZE_DATA];
      DFT_I_Im[index_freq] -= Volt_I[index_data] * W[(index_W + Offset_CosToSin) % SIZE_DATA];
    }
    PwrSpctm_I[index_freq] = DFT_I_Re[index_freq] * DFT_I_Re[index_freq] + DFT_I_Im[index_freq] * DFT_I_Im[index_freq];

    freq_bin = freq_resolution * (double)index_freq;
    fprintf(pFile_DFT, "%d,%6.4f,%6.3f\n", index_freq, freq_bin, PwrSpctm_I[index_freq]);
  }

  fclose(pFile_ADC);
  fclose(pFile_DFT);

  END = clock();
  printf("Complete! It costs %f seconds! \n", (END - START) / CLOCKS_PER_SEC);

  // Quick sort the power spectrum array
  // qsort(PwrSpctm_I, SIZE_DATA, sizeof(double), compare_double);

  // printf("MAX freq value is %6.4f \n", PwrSpctm_I[SIZE_DATA - 1]);

  return 0;

  /*
  while (countWhileLoop <= 100000)
  {
    //Replaced by "fgets" to receive input string! Maybe ot doesn't need Serial in command anymore?
    //=>TEST RESULT:
    //NO GOOD! The fgets would wait for input only one time,it doesn't acts like the Arduino Serial input/ouput
    //So we need a interrupt serial function to receive users' manual commands at any moment!!

    // printf("Loop#_%d_Input Command:", countDataAcq);
    // fgets(DataIn, SIZE_SERIAL_BUFFER, stdin);


    if (isBufferAvailable == 1) //should be replaced by  serial buffer input ISR event!
    {

      //Read Buffer and store at DataIn

      switch (DataIn[0])
      {
      case 'A':
        sprintf(arySerialOutMsg, "TEST!\n");
        break;
      case 'I':
        ADS131A0x_InitialADC();
        break;

      case 'S':
        countDataAcq = 0;
        Status_SerialOut = 1;

        ADS131A0x_InitialADC();
        ADS131A0x_Start();

        sprintf(arySerialOutMsg, "@MStart Data Acquiring!\n ===================\n");
        isBufferAvailable = 0; //Simulation,trigger only one time!

        //Initialize GPIO_0 with interrupt
        break;

      case 'E':
        Status_SerialOut = 0;
        sprintf(arySerialOutMsg, "@MStop Data Acquiring!\n ");

        ADS131A0x_Stop();

        break;

      default:
        sprintf(arySerialOutMsg, "Undefined command!\n");
        break;
      }
      printf(arySerialOutMsg);
    }


    //BufferClean(DataIn); //Drop 1 byte command?
    if (Status_SerialOut == 1) //ADC continuous DAQ on ,and dump first 2 sets of ADC datas
    {
      if ((countDataAcq > 1))
      {
        ADS131A0x_GetADCData(1, aData_ADC); //Mode1= Real ADC

        printf("Count:%d;", countDataAcq); //To be replaced by RPi serial library

        // printf("ADC2=%6.3f,ADC3=%6.3f \n", aData_ADC[1], aData_ADC[2]);
        sprintf(arySerialOutMsg, "ADC2=%6.3f,ADC3=%6.3f \n", aData_ADC[1], aData_ADC[2]);
        printf(arySerialOutMsg); //To be replaced by RPi serial library

        // delay(time_loop_halt);
        // delayMicroseconds(2000);//max accurate value=16383 us
      }

      countDataAcq++;
    }

    countWhileLoop++;
    // printf("Loop:%d\n", countWhileLoop);
  }*/
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
  // double ret = *(double *)(arg1) - *(double *)(arg2);
  // if (ret > 0)
  //   return 1;
  // if (ret < 0)
  //   return -1;
  return (*(double *)(arg1) - *(double *)(arg2));
}

int FindMax(double *array, unsigned int size)
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