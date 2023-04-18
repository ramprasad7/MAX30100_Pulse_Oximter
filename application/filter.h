#ifndef _FILTER_H_
#define _FILTER_H_

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <time.h>
#include <sys/ioctl.h>

/*IOCLT Commands to set mode configurations*/
#define HEART_RATE_MODE _IOW('a', 'b', int)
#define SPO2_MODE _IOW('a', 'a', int)
#define TEMPERATURE_MODE _IOW('a', 'c', int)
#define LED_CURRENT_SETTING _IOW('a', 'd', int)

/*Maximum number of samples FIFO can Hold*/
#define MAX_SAMPLES 16

/*Maximum Data Size in Bytes*/
#define BUF_SIZE (4 * MAX_SAMPLES)

/* SpO2 parameters */
#define RESET_SPO2_EVERY_N_PULSES 4

/* Filter parameters */
#define ALPHA 0.95 // dc filter alpha value
#define MEAN_FILTER_SIZE 15

/* Pulse detection parameters */
#define PULSE_MIN_THRESHOLD 100 // 300 is good for finger, but for wrist you need like 20, and there is shitloads of noise
// #define PULSE_MIN_THRESHOLD 20
#define PULSE_MAX_THRESHOLD 2000
// #define PULSE_MAX_THRESHOLD 3000 // new max threshold value
#define PULSE_GO_DOWN_THRESHOLD 1

#define PULSE_BPM_SAMPLE_SIZE 10 // Moving average size

/* Adjust RED LED current balancing*/
#define MAGIC_ACCEPTABLE_INTENSITY_DIFF 65000
#define RED_LED_CURRENT_ADJUSTMENT_MS 500 // MILLI SECONDS

#define DEFAULT_IR_LED_CURRENT MAX30100_LED_CURRENT_50MA
#define STARTING_RED_LED_CURRENT MAX30100_LED_CURRENT_27_1MA

/* Enums, data structures and typdefs. DO NOT EDIT */
typedef struct pulseoxymeter_t
{
    bool pulseDetected;
    float heartBPM;

    float irCardiogram;

    float irDcValue;
    float redDcValue;

    float SaO2;

    uint32_t lastBeatThreshold;

    float dcFilteredIR;
    float dcFilteredRed;
} pulseoxymeter_t;

typedef enum PulseStateMachine
{
    PULSE_IDLE,
    PULSE_TRACE_UP,
    PULSE_TRACE_DOWN
} PulseStateMachine;

typedef enum LEDCurrent
{
    MAX30100_LED_CURRENT_0MA = 0x00,
    MAX30100_LED_CURRENT_4_4MA = 0x01,
    MAX30100_LED_CURRENT_7_6MA = 0x02,
    MAX30100_LED_CURRENT_11MA = 0x03,
    MAX30100_LED_CURRENT_14_2MA = 0x04,
    MAX30100_LED_CURRENT_17_4MA = 0x05,
    MAX30100_LED_CURRENT_20_8MA = 0x06,
    MAX30100_LED_CURRENT_24MA = 0x07,
    MAX30100_LED_CURRENT_27_1MA = 0x08,
    MAX30100_LED_CURRENT_30_6MA = 0x09,
    MAX30100_LED_CURRENT_33_8MA = 0x0A,
    MAX30100_LED_CURRENT_37MA = 0x0B,
    MAX30100_LED_CURRENT_40_2MA = 0x0C,
    MAX30100_LED_CURRENT_43_6MA = 0x0D,
    MAX30100_LED_CURRENT_46_8MA = 0x0E,
    MAX30100_LED_CURRENT_50MA = 0x0F
} LEDCurrent;

typedef struct fifo_t
{
    uint16_t rawIR;
    uint16_t rawRed;
} fifo_t;

typedef struct dcFilter_t
{
    float w;
    float result;
} dcFilter_t;

typedef struct butterworthFilter_t
{
    float v[2];
    float result;
} butterworthFilter_t;

typedef struct meanDiffFilter_t
{
    float values[MEAN_FILTER_SIZE];
    uint8_t index;
    float sum;
    uint8_t count;
} meanDiffFilter_t;

void init();
pulseoxymeter_t update();

#endif