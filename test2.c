/*
  This example rotates 4 motors, each of them in normal and then reversed direction.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

extern void motorImplementationInitialize(int motorPins[], int motorMax) ;
extern void motorImplementationFinalize(int motorPins[], int motorMax) ;
extern void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) ;
extern void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) ;
extern void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int repeatCounter) ;

int 	motorPins[32];
double 	throttles[32];

int main(int argc, char **argv) {
    int i, n, k;

    if (argc < 2) {
	printf("usage:   ./test2 <gpio_pin_1> ... <gpio_pin_n>\n");
	printf("example: sudo ./test2 16 19 20 21\n");
	exit(1);
    }

    n = 0;
    for(i=1; i<argc; i++) {
	motorPins[n] = atoi(argv[i]);
	if (motorPins[n] > 0 && motorPins[n] < 28) {
	    n ++;
	} else {
	    printf("pin %d out of range 1..27\n", motorPins[n]);
	}
    }
    
    motorImplementationInitialize(motorPins, n);
    
    printf("Setting 3d mode and rotation direction. It may take a few seconds.\n");
    motorImplementationSet3dModeAndSpinDirection(motorPins, n, 1, 0);

    // make spinning 1 motor after another
    for(k=0;k<n;k++) {
	// spin
	printf("Spinning motor %d.\n", k%n);
	throttles[k%n] = 0.15;
	for(i=0; i<1000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
	}
	// stop
	printf("Stop.\n");
	throttles[k%n] = 0;
	for(i=0; i<1000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
	}
	// inverse spin
	printf("Inverse spin of motor %d.\n", k%n);
	throttles[k%n] = -0.15;
	for(i=0; i<1000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
	}
	// stop
	printf("Stop.\n");
	throttles[k%n] = 0;
	for(i=0; i<1000; i++) {
	    motorImplementationSendThrottles(motorPins, n, throttles);
	    usleep(1000);
	}
    }
    
    // stop motors
    for(i=0; i<n; i++) throttles[i] = 0;
    motorImplementationSendThrottles(motorPins, n, throttles);
    usleep(1000);

    // Send beeps
    for(k=0;k<10;k++) {
	printf("Beacon %d.\n", k%5+1);
	// beacons are dshot commands 1 to 5.
	dshotRepeatSendCommand(motorPins, n, k%5+1, 1, 1);
	// Not sure how exactly it works. There has to be a pause after having sent beacon command.
	// However, if I do not send anything for a long time, motors are disarmed. So wait a bit
	// and then send command 0 for some time.
	usleep(10000);
	dshotRepeatSendCommand(motorPins, n, 0, 0, 300);
    }
    
    // finalize
    motorImplementationFinalize(motorPins, n);
    
    return(0);
}
