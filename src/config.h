// Cpu frequency for util/delay.h
#define F_CPU 8000000

// Half of delay in main execution loop, given in milliseconds.
#define LOOP_DELAY 50

// Half length of indicator led on-off sequence. In other words, the time
// the indicator is lit of darkened. Given in units of LOOP_DELAY.
#define INDICATOR_HALF_PERIOD 10

// How ofter magnet values are changed. Given in units of LOOP_DELAY.
#define MAGNET_RUN_INTERVAL 5

//Enables debug messaging
//#define DEBUG
