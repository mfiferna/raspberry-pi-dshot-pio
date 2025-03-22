/*
Code to generate DSHOT protocol signals to a Raspberry Pi GPIO
output. This code uses 'pio' library allowing to generate signal
while not loading the main CPU.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "dshot.pio.h"

// #include "../motor-common.h"

// Strangely, my ESC does not conform to the specificaion. Its timing
// is 18% faster than the spec. DSHOT_CLOCK_DIVIDER_FACTOR is
// introduced to handle it. If set to the value 1.0, it will produce
// DSHOT frames with the exact timing as specified in the
// standard. Values above 1.0 will make broadcasting slower and values
// under 1.0 faster.
#define DSHOT_CLOCK_DIVIDER_FACTOR 	1.0

// DMA support for RP1 is little documented and maybe a bit experimental.
// If it does not work for you, You may disable it by setting this to 0.
#define DSHOT_USE_DMA		1

// Maximal number of PIO state machines. This number comes from RP1 hardware.
#define DSHOT_PIO_SM_MAX	4

// Each PIO state machine generates DSHOT to (max) 2, 4, 8 or 16
// continuous GPIO pins depending on the value of the following
// macro. Higher value means that you may use more pins and less state
// machines (if your motors are connected to continuous GPIO
// numbers). However, it also means that more data is sent to FIFO. If
// you restrict PINS_PER_SM to 2 then each state machine can be
// updated by a single push. The optimal case is when you have all
// your motors connected to a single continuous range of GPIOs and you
// set PINS_PER_SM to the nearest power of 2.
// !!! If you change this value, you also have to change it in  !!!
// !!! 'dshot.pio' and recompile to dshot.pio.h                 !!!
#define PINS_PER_SM              8

// Max GPIO pins we can handle
#define DSHOT_NUM_PINS           32
// DSHOT frame length from dshot specification
#define DSHOT_FRAME_LEN          16

// DSOT version we generate, aka the bitrate.
// It can be one of 150, 300, 600, 1200, ...
// This value may be modified from the program. However, any changes
// made after 'motorImplementationInitialize' do not take effect.
unsigned dshotVersion = 300;

////////////////////////////////////////////////////////////////////////////

// DSHOT special commands.
enum {
    DSHOT_CMD_MOTOR_STOP = 0,
    DSHOT_CMD_BEACON1,
    DSHOT_CMD_BEACON2,
    DSHOT_CMD_BEACON3,
    DSHOT_CMD_BEACON4,
    DSHOT_CMD_BEACON5,
    DSHOT_CMD_ESC_INFO, // V2 includes settings
    DSHOT_CMD_SPIN_DIRECTION_1,
    DSHOT_CMD_SPIN_DIRECTION_2,
    DSHOT_CMD_3D_MODE_OFF,
    DSHOT_CMD_3D_MODE_ON,
    DSHOT_CMD_SETTINGS_REQUEST, // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,
    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    DSHOT_CMD_LED0_ON, // BLHeli32 only
    DSHOT_CMD_LED1_ON, // BLHeli32 only
    DSHOT_CMD_LED2_ON, // BLHeli32 only
    DSHOT_CMD_LED3_ON, // BLHeli32 only
    DSHOT_CMD_LED0_OFF, // BLHeli32 only
    DSHOT_CMD_LED1_OFF, // BLHeli32 only
    DSHOT_CMD_LED2_OFF, // BLHeli32 only
    DSHOT_CMD_LED3_OFF, // BLHeli32 only
    DSHOT_CMD_AUDIO_STREAM_MODE_ON_OFF = 30, // KISS audio Stream mode on/Off
    DSHOT_CMD_SILENT_MODE_ON_OFF = 31, // KISS silent Mode on/Off
    DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    DSHOT_CMD_MAX = 47
};

struct dshotPioPins {
    int pinBase;
    int pinCount;
};

struct pinToSmStr {
    int sm;
    int mask;
};

// the /dev/pio0 link.
static PIO  dshotPio;
// state machines index acquired from pio lib
static int  dshotPioSmi[DSHOT_PIO_SM_MAX];
// pin distribution among state machines
static struct dshotPioPins dshotSm[DSHOT_PIO_SM_MAX];
// the state machine and offset for each pin
static struct pinToSmStr dshotPinToSm[DSHOT_NUM_PINS];
// offset where our binary is loaded into state machines memory.
static uint dshotLoadedOffset;
// Flag whether we are in 3D mode, if dshot3dMode != 0 then reverse rotation is enabled.
static int dshot3dMode = 0;


////////////////////////////////////////////////////////////////////////

static unsigned dshotAddChecksumAndTelemetry(int packet, int telem) {
    unsigned pp, ss;
    pp = (packet << 1) | (telem & 1);
    ss = (pp ^ (pp>>4) ^ (pp>>8)) & 0x0f;
    return ((pp<<4) | ss);
}

// precompute continuous areas of pins so that all pins are covered by 4 state machines
static int dshotAssignPinsToStateMachines(int motorPins[], int motorMax) {
    int		i, smi, pin, base, count;
    int8_t 	pinMap[DSHOT_NUM_PINS];

    memset(pinMap, 0, sizeof(pinMap));
    for(i=0; i<motorMax; i++) {
	pin = motorPins[i];
	assert(pin >= 0 && pin < DSHOT_NUM_PINS);
	pinMap[pin] = 1;
    }
    // assign continuos regions of pins to state machines
    memset(dshotSm, 0, sizeof(dshotSm));
    i = 0;
    while(pinMap[i] == 0 && i<DSHOT_NUM_PINS) i++;
    // No pin's found
    if (i>=DSHOT_NUM_PINS) return(-2);
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	base = i;
	while(pinMap[i] != 0 && i<base+PINS_PER_SM && i<DSHOT_NUM_PINS) i++;
	count = i-base;
	dshotSm[smi].pinBase = base;
	dshotSm[smi].pinCount = count;
	while(pinMap[i] == 0 && i<DSHOT_NUM_PINS) i++;
	if (i>=DSHOT_NUM_PINS) return(0);
    }
    // Hmm I am running out of state machines
    return(-1);
}

// precompute mapping from pin number to assigned state machine and pin's bit masks
static void dshotPinToSmTableInit() {
    int smi, j, pin;

    memset(dshotPinToSm, 0, sizeof(dshotPinToSm));
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	for(j=0; j<dshotSm[smi].pinCount; j++) {
	    pin = dshotSm[smi].pinBase + j;
	    dshotPinToSm[pin].sm = smi;
	    dshotPinToSm[pin].mask = (1 << j);
	}
    }
}

// Pack DSHOT frames from buffer 'b' into 32bits words and send it 'i'-th state machine TX FIFO.
static void dshotPackFramesIntoFifo(uint16_t *b, int i) {
    int 	k, j, sm;
    uint32_t    bb;
    
    sm = dshotPioSmi[i];
    for(j=0; j<DSHOT_FRAME_LEN; j+=32/PINS_PER_SM) {
	bb = 0;
	for(k=0; k<32/PINS_PER_SM; k++) {
	    bb |= (b[j+k] << k*PINS_PER_SM);
	}
	pio_sm_put(dshotPio, sm, bb);
    }
}

// Pack DSHOT frames from buffer 'b' into 32bits words and send it 'i'-th state machine TX FIFO.
// Use DMA transfer.
static void dshotPackFramesIntoFifoUsingDma(uint16_t *b, int i) {
    static uint32_t 	d[DSHOT_PIO_SM_MAX][DSHOT_FRAME_LEN];
    int 		r, dlen, k, j, sm;
    uint32_t    	bb;
    
    sm = dshotPioSmi[i];
    dlen = 0;
    for(j=0; j<DSHOT_FRAME_LEN; j+=32/PINS_PER_SM) {
	bb = 0;
	for(k=0; k<32/PINS_PER_SM; k++) {
	    bb |= (b[j+k] << k*PINS_PER_SM);
	}
	d[i][dlen++] = bb;
    }
    r = pio_sm_xfer_data(dshotPio, sm, PIO_DIR_TO_SM, dlen * sizeof(d[0][0]), d[i]);
    if (r) {
	fprintf(stderr, "Warning: pio_sm_xfer_data returned %d\n", r);
    }
}

// send dshot 'frames' to 'motorPins' respectively.
static void dshotSendFrames(int motorPins[], int motorMax, unsigned frames[]) {
    int         smi, bi;
    unsigned    bit;
    uint16_t    b[DSHOT_PIO_SM_MAX][DSHOT_FRAME_LEN];
    int	      	pin, sm, mask;
    
    assert(motorMax < DSHOT_NUM_PINS);

    memset(b, 0, sizeof(b));

    // precompute broadcasting to state machines for each message bit
    for(smi=0; smi<motorMax; smi++) {
	pin = motorPins[smi];
	assert(pin < DSHOT_NUM_PINS);
	sm = dshotPinToSm[pin].sm;
	mask = dshotPinToSm[pin].mask;
	bit = 0x8000;
	for(bi=0; bi<DSHOT_FRAME_LEN; bi++) {
	    if ((frames[smi] & bit)) b[sm][bi] |= mask;
	    bit = (bit >> 1);
	}
    }

    // send precomputed frames to each state machine
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	sm = dshotPioSmi[smi];
	if (sm >= 0 && dshotSm[smi].pinCount > 0) {
	    if (pio_sm_is_tx_fifo_empty(dshotPio, sm)) {
		if (DSHOT_USE_DMA) {
		    dshotPackFramesIntoFifoUsingDma(b[smi], smi);
		} else {
		    // non DMA transfer to TX FIFO seems to be slow. To make a fluent
		    // broadcasting disable the state machine, fill FIFO, and re-enable it.
		    pio_sm_set_enabled(dshotPio, sm, false);
		    dshotPackFramesIntoFifo(b[smi], smi);
		    pio_sm_set_enabled(dshotPio, sm, true);
		}
	    } else {
		fprintf(stderr, "Warning: a dshot frame skipped\n");
	    }
	}
    }
}

static void dshotPioStateMachineInit(PIO pio, uint sm, uint offset, uint pin, uint pincount, double clkDivider) {
    pio_sm_set_consecutive_pindirs(pio, sm, pin, pincount, true);
    pio_sm_config c = dshot_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin, pincount);
    sm_config_set_set_pins(&c, pin, pincount);
    sm_config_set_out_shift (&c, true, true, 32);
    sm_config_set_fifo_join (&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, clkDivider);
    pio_sm_init(pio, sm, offset, &c);
}

static unsigned dshotThrottleToDshotFrame(double tt) {
    int 	ff;
    unsigned 	res;
    
    if (dshot3dMode) {
	// translate throttles ranging <-1, 1> to dshot frames.
	if (tt >= 0) {
	    ff = tt * 999 + 1048;
	} else {
	    ff = -tt * 999 + 48;
	}
    } else {
	// translate throttles ranging <0, 1> to dshot frames.
	ff = tt * 1999 + 48;
    }
    // I used to issue DSHOT_CMD_MOTOR_STOP if thrust == 0. It is not possible anymore
    // because in 3d mode it seems to reset the motor. So, you have to arm motors in some different way.
    // Now the DSHOT_CMD_MOTOR_STOP is issued only for values out of range.
    if (/*tt == 0 || */ ff < 48 || ff >= 2048) ff = DSHOT_CMD_MOTOR_STOP;
    res = dshotAddChecksumAndTelemetry(ff, 0);
    return(res);
}

// Send a command repeatedly 
void dshotRepeatSendCommand(int motorPins[], int motorMax, int cmd, int telemetry, int repeatCounter) {
    unsigned    ff;
    unsigned    frame[DSHOT_NUM_PINS+1];
    int         i;

    ff = dshotAddChecksumAndTelemetry(cmd, telemetry);
    for(i=0; i<motorMax; i++) frame[i] = ff;
    for(i=0; i<repeatCounter; i++) {
        dshotSendFrames(motorPins, motorMax, frame);
        usleep(1000);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Main exported functions of the module implementing raspilot motor instance.
//////////////////////////////////////////////////////////////////////////////////////////////

// This function allows to set bidirectional rotation (mode3dFlag!=0) and reverse rotation logic (reverseDirectionFlag!=0).
// Changing 3D mode is interfering with rotation direction (at least on my ESC), so always reset the direction when changing 3D
void motorImplementationSet3dModeAndSpinDirection(int motorPins[], int motorMax, int mode3dFlag, int reverseDirectionFlag) {
    int         repeatCounter;

    // First, stop/arm motors for around 3 seconds. Strangely my ESC requires so much time to re-init.
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_MOTOR_STOP, 0, 3000);

    // Those commands have to be sent repeatedly in order to be applied.
    repeatCounter = 10;

    dshot3dMode = mode3dFlag;
    // Set 3d mode. Some sources say that the telemetry bit shall be set in this command. It seems to
    // work both ways.
    if (dshot3dMode) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_ON, 1, repeatCounter);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_3D_MODE_OFF, 1, repeatCounter);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatCounter);
    usleep(50000);

    // Set rotation direction mode. Telemetry as previously.
    if (reverseDirectionFlag) {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_REVERSED, 1, repeatCounter);
    } else {
        dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SPIN_DIRECTION_NORMAL, 1, repeatCounter);
    }
    dshotRepeatSendCommand(motorPins, motorMax, DSHOT_CMD_SAVE_SETTINGS, 0, repeatCounter);
    usleep(50000);
}

void motorImplementationInitialize(int motorPins[], int motorMax) {
    int    smi, r;
    double divider;
    
    // initialize pio library
    stdio_init_all();
    dshotPio = pio0;
    
    r = dshotAssignPinsToStateMachines(motorPins, motorMax) ;
    if (r != 0) {
	fprintf(stderr, "Error: MotorPins aren't in %d continuous intervals. Can't map.\n", DSHOT_PIO_SM_MAX);
    }
    
    dshotPinToSmTableInit() ;

    for(smi=0; smi<motorMax; smi++) {
	pio_gpio_init(dshotPio, motorPins[smi]);
	// I guess the following line maybe useless, unless set by other software, pull down should be the default.
	gpio_set_pulls(motorPins[smi], false, true);  // up, down
    }

    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotSm[smi].pinCount > 0) {
	    dshotPioSmi[smi] = pio_claim_unused_sm(dshotPio, false);
	    if (dshotPioSmi[smi] < 0) fprintf(stderr, "Error: Can't claim state machine number %d\n", smi);
	}
    }

    dshotLoadedOffset = pio_add_program(dshotPio, &dshot_program);

    // Set clock divider to the value specified in dshot specification (we use 8 ticks to broadcast one bit).
    divider = (double)clock_get_hz(clk_sys) / (dshotVersion * 1000.0 * 8);
    // My cheap ESC is nearly 20% off the standard. Adjust divider by an ad-hoc value.
    divider *= DSHOT_CLOCK_DIVIDER_FACTOR;
    // printf("clock hz == %d divider == %f\n", clock_get_hz(clk_sys), divider);

    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotSm[smi].pinCount > 0 && dshotPioSmi[smi] >= 0) {
	    if (DSHOT_USE_DMA) pio_sm_config_xfer(dshotPio, dshotPioSmi[smi], PIO_DIR_TO_SM, 256, 1);
	    // printf("init program in sm %d, offset %d, pinbase %d, pincount %d\n", smi[smi], loaded_offset, dshotSm[smi].pinBase, dshotSm[smi].pinCount);
	    dshotPioStateMachineInit(dshotPio, dshotPioSmi[smi], dshotLoadedOffset, dshotSm[smi].pinBase, dshotSm[smi].pinCount, divider);
	    pio_sm_set_enabled(dshotPio, dshotPioSmi[smi], true);
	}
    }
    
}

void motorImplementationFinalize(int motorPins[], int motorMax) {
    /* Not yet implemented. 
    */
    int smi;
    
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotPioSmi[smi] >= 0) {
	    pio_sm_set_enabled(dshotPio, dshotPioSmi[smi], false);
	}
    }

    /*
    // Strangely, this code makes state machines to freeze
    // TODO: figure out the proper deinitialization of RP1 state machines

    // pio_remove_program (pio, &dshot_program, loaded_offset);
    for(smi=0; smi<DSHOT_PIO_SM_MAX; smi++) {
	if (dshotPioSmi[smi] >= 0) {
	    pio_sm_unclaim(pio, dshotPioSmi[smi]);
	}
    }
    */
}

void motorImplementationSendThrottles(int motorPins[], int motorMax, double motorThrottle[]) {
    int         i;
    unsigned    frames[DSHOT_NUM_PINS+1];

    assert(motorMax < DSHOT_NUM_PINS);

    for(i=0; i<motorMax; i++) {
        frames[i] = dshotThrottleToDshotFrame(motorThrottle[i]);
    }
    dshotSendFrames(motorPins, motorMax, frames);
}
