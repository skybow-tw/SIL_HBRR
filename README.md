# HeartBeat and Respiration Rate analysis by SIL radar module
//========pigpio example========
/*
ALL GPIO are identified by their Broadcom number.

*Usage*
Include <pigpio.h> in your source files.

Assuming your source is in prog.c use the following command to build and
run the executable.

. .
gcc -Wall -pthread -o prog prog.c -lpigpio -lrt
sudo ./prog
. .

For examples of usage see the C programs within the pigpio archive file.



*Notes*

All the functions which return an int return < 0 on error.

[*gpioInitialise*] must be called before all other library functions
with the following exceptions:

. .
[*gpioCfg**]
[*gpioVersion*]
[*gpioHardwareRevision*]
. .

If the library is not initialised all but the [*gpioCfg**],
[*gpioVersion*], and [*gpioHardwareRevision*] functions will
return error PI_NOT_INITIALISED.

If the library is initialised the [*gpioCfg**] functions will return
error PI_INITIALISED.

If you intend to rely on signals sent to your application, you should
turn off the internal signal handling as shown in this example:

. .
int cfg = gpioCfgGetInternals();
cfg |= PI_CFG_NOSIGHANDLER;  // (1<<10)
gpioCfgSetInternals(cfg);
int status = gpioInitialise();
. .

*/
//==============================
/*
  Using "time.h"  to get current time and apply to datalog filename!
  3 functions and 2 data type are used here.

  NOTE: time_t"calendar time"-> struct tm "structure time data " -> formatted time string

  Function "time_t time(time_t *timer)":
  return current calendar time.

  Function "struct tm *localtime(const time_t *timer)":
  convert the "calendar time" to "tm" type which is expressed in the local time zone.

  Function "size_t strftime(char *str, size_t maxsize, const char *format, const struct tm *timeptr)":
  formats the time represented in the structure "timeptr"
  according to the formatting rules defined in "format" and stored into "str".

  NOTE: "time_t" is a type suitable for storing the calendar time.
  NOTE: "tm" is a structure used to hold the time and date.


  */

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
