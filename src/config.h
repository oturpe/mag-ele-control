// Cpu frequency for util/delay.h
#define F_CPU 8000000

// Half of delay in main execution loop, given in milliseconds.
#define LOOP_DELAY 50

// Half length of indicator led on-off sequence. In other words, the time
// the indicator is lit of darkened. Given in units of LOOP_DELAY.
#define INDICATOR_HALF_PERIOD 3

// Pwm duty cycle for running the motor. 0 is 0 %, 255 is 100 %.
#define HUMAN_MOTOR_DUTY_CYCLE 200

//Enables debug messaging
//#define DEBUG
