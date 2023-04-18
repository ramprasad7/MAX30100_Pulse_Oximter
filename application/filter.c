#include "filter.h"

/*read buffer*/
extern char read_buffer[BUF_SIZE];

/*one sample buffer*/
extern char filter_buf[4];

/*file desciptor*/
extern int fd;

extern int op;

/*global variables*/
static dcFilter_t dcFilterIR;
static dcFilter_t dcFilterRed;
static butterworthFilter_t lpbFilterIR;
static meanDiffFilter_t meanDiffIR;
static fifo_t prevFifo;
static LEDCurrent IrLedCurrent;

static float irACValueSqSum;
static float redACValueSqSum;
static uint16_t samplesRecorded;
static uint16_t pulsesDetected;
static float currentSaO2Value;

static uint8_t redLEDCurrent;
static float lastREDLedCurrentCheck;

static uint8_t currentPulseDetectorState;
static float currentBPM;
static float valuesBPM[PULSE_BPM_SAMPLE_SIZE];
static float valuesBPMSum;
static uint8_t valuesBPMCount;
static uint8_t bpmIndex;
static uint32_t lastBeatThreshold;

extern bool debug;
#if 0
/*millis*/
static unsigned int millis()
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return (t.tv_sec * 1000) + (t.tv_usec + 500) / 1000;
}
#endif

#if 1
static struct timeval __millis_start;

void init_millis()
{
    gettimeofday(&__millis_start, NULL);
}

unsigned long int millis()
{
    long mtime, seconds, useconds;
    struct timeval end;
    gettimeofday(&end, NULL);
    seconds = end.tv_sec - __millis_start.tv_sec;
    useconds = end.tv_usec - __millis_start.tv_usec;

    mtime = ((seconds)*1000 + useconds / 1000.0) + 0.5;
    return mtime;
}
#endif

static fifo_t readFIFO()
{
    fifo_t result;
#if 0
    for (int i = 0; i < 4; i++)
    {
        printf("filter buf = %d ", filter_buf[i]);
    }
    printf("\n");
#endif
    result.rawIR = (filter_buf[0] << 8) | filter_buf[1]; // changed according to endianess
    result.rawRed = (filter_buf[2] << 8) | filter_buf[3];
    return result;
}

static dcFilter_t dcRemoval(float x, float prev_w, float alpha)
{
    dcFilter_t filtered;
    filtered.w = x + alpha * prev_w;
    filtered.result = filtered.w - prev_w;
    return filtered;
}

static void lowPassButterworthFilter(float x, butterworthFilter_t *filterResult)
{
    filterResult->v[0] = filterResult->v[1];
    // Fs = 100Hz and Fc = 10Hz
    filterResult->v[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * filterResult->v[0]);

    filterResult->result = filterResult->v[0] + filterResult->v[1];
}

static float meanDiff(float M, meanDiffFilter_t *filterValues)
{
    float avg = 0.0;
    filterValues->sum -= filterValues->values[filterValues->index];
    filterValues->values[filterValues->index] = M;
    filterValues->sum += filterValues->values[filterValues->index];

    filterValues->index++;
    filterValues->index = filterValues->index % MEAN_FILTER_SIZE;

    if (filterValues->count < MEAN_FILTER_SIZE)
        filterValues->count++;

    avg = filterValues->sum / filterValues->count;
    return avg - M;
}

static bool detectPulse(float sensor_value)
{
    static float prev_sensor_value = 0;
    static uint8_t values_went_down = 0;
    static uint32_t currentBeat = 0;
    static uint32_t lastBeat = 0;
#if 0
    if (sensor_value > PULSE_MAX_THRESHOLD)
    {
        currentPulseDetectorState = PULSE_IDLE;
        prev_sensor_value = 0;
        lastBeat = 0;
        currentBeat = 0;
        values_went_down = 0;
        lastBeatThreshold = 0;
        // printf("Sensor value exceeded max threshold = %.3f\n", sensor_value);
        return false;
    }
#endif
    // printf("pulse state = %d\n", currentPulseDetectorState);

    switch (currentPulseDetectorState)
    {
    case PULSE_IDLE:
        if (sensor_value >= PULSE_MIN_THRESHOLD)
        {
            currentPulseDetectorState = PULSE_TRACE_UP;
            values_went_down = 0;
        }
        // printf("Pulse IDLE\n");
        break;

    case PULSE_TRACE_UP:
        if (sensor_value > prev_sensor_value)
        {
            currentBeat = millis();
            lastBeatThreshold = sensor_value;
        }
        else
        {
            if (debug == true)
            {
                printf("Peak reached: ");
                printf("%f", sensor_value);
                printf(" ");
                printf("%f\n", prev_sensor_value);
            }

            uint32_t beatDuration = currentBeat - lastBeat;
            // printf("current beat = %d\nlast beat = %d\n", currentBeat, lastBeat);
            // printf("beat duration = %d\n", beatDuration);
            lastBeat = currentBeat;

            float rawBPM = 0;

            if (beatDuration > 0)
                rawBPM = 60000.0 / (float)beatDuration;
            if (debug == true)
                printf("raw BPM = %f\n", rawBPM);

            // This method sometimes glitches, it's better to go through whole moving average everytime
            // IT's a neat idea to optimize the amount of work for moving avg. but while placing, removing finger it can screw up
            valuesBPMSum -= valuesBPM[bpmIndex];
            valuesBPM[bpmIndex] = rawBPM;
            valuesBPMSum += valuesBPM[bpmIndex];

            valuesBPM[bpmIndex] = rawBPM;
            valuesBPMSum = 0;
            for (int i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++)
            {
                valuesBPMSum += valuesBPM[i];
            }

            if (debug == true)
            {
                printf("CurrentMoving Avg: ");
                for (int i = 0; i < PULSE_BPM_SAMPLE_SIZE; i++)
                {
                    printf("%.3f", valuesBPM[i]);
                    printf(" ");
                }
                printf(" \n");
            }

            bpmIndex++;
            bpmIndex = bpmIndex % PULSE_BPM_SAMPLE_SIZE;

            if (valuesBPMCount < PULSE_BPM_SAMPLE_SIZE)
                valuesBPMCount++;

            currentBPM = valuesBPMSum / valuesBPMCount;
            if (debug == true)
            {
                printf("Avg. BPM: ");
                printf("%.3f\n", currentBPM);
            }
            currentPulseDetectorState = PULSE_TRACE_DOWN;
            // printf("Pulse Traced Up\n");
            return true;
        }
        break;

    case PULSE_TRACE_DOWN:
        if (sensor_value < prev_sensor_value)
        {
            values_went_down++;
        }

        if (sensor_value < PULSE_MIN_THRESHOLD)
        {
            currentPulseDetectorState = PULSE_IDLE;
        }
        // printf("Pulse Traced Down\n");
        break;
    }
    prev_sensor_value = sensor_value;
    // printf("Pulse Not Detected (%s())\n", __func__);
    return false;
}

void init()
{
    currentPulseDetectorState = PULSE_IDLE;
    IrLedCurrent = MAX30100_LED_CURRENT_50MA;

    redLEDCurrent = (uint8_t)STARTING_RED_LED_CURRENT;

    lastREDLedCurrentCheck = 0;

    prevFifo.rawIR = 0;
    prevFifo.rawRed = 0;

    dcFilterIR.w = 0;
    dcFilterIR.result = 0;

    dcFilterRed.w = 0;
    dcFilterRed.result = 0;

    lpbFilterIR.v[0] = 0;
    lpbFilterIR.v[1] = 0;
    lpbFilterIR.result = 0;

    meanDiffIR.index = 0;
    meanDiffIR.sum = 0;
    meanDiffIR.count = 0;

    valuesBPM[0] = 0;
    valuesBPMSum = 0;
    valuesBPMCount = 0;
    bpmIndex = 0;

    irACValueSqSum = 0;
    redACValueSqSum = 0;
    samplesRecorded = 0;
    pulsesDetected = 0;
    currentSaO2Value = 0;

    lastBeatThreshold = 0;
}

static void balanceIntesities(float redLedDC, float IRLedDC)
{

    if (millis() - lastREDLedCurrentCheck >= RED_LED_CURRENT_ADJUSTMENT_MS)
    {
        // Serial.println( redLedDC - IRLedDC );
        if (IRLedDC - redLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent < MAX30100_LED_CURRENT_50MA)
        {
            redLEDCurrent++;
            // setLEDCurrents(redLEDCurrent, IrLedCurrent);
            if (ioctl(fd, LED_CURRENT_SETTING, (redLEDCurrent << 4) | IrLedCurrent) < 0)
            {
                perror("failed to set led currents\n");
                return;
            }
            if (debug == true)
                printf("RED LED Current +\n");
        }
        else if (redLedDC - IRLedDC > MAGIC_ACCEPTABLE_INTENSITY_DIFF && redLEDCurrent > 0)
        {
            redLEDCurrent--;
            // setLEDCurrents(redLEDCurrent, IrLedCurrent);
            if (ioctl(fd, LED_CURRENT_SETTING, (redLEDCurrent << 4) | IrLedCurrent) < 0)
            {
                perror("failed to set led currents\n");
                return;
            }
            if (debug == true)
                printf("RED LED Current -\n");
        }

        lastREDLedCurrentCheck = millis();
    }
}

pulseoxymeter_t update()
{
    pulseoxymeter_t result = {
        /*bool pulseDetected*/ false,
        /*float heartBPM*/ 0.0,
        /*float irCardiogram*/ 0.0,
        /*float irDcValue*/ 0.0,
        /*float redDcValue*/ 0.0,
        /*float SaO2*/ currentSaO2Value,
        /*uint32_t lastBeatThreshold*/ 0,
        /*float dcFilteredIR*/ 0.0,
        /*float dcFilteredRed*/ 0.0};

    fifo_t rawData = readFIFO();

    dcFilterIR = dcRemoval((float)rawData.rawIR, /*dcFilterIR.w*/ prevFifo.rawIR, ALPHA);
    dcFilterRed = dcRemoval((float)rawData.rawRed, /*dcFilterRed.w*/ prevFifo.rawRed, ALPHA);
    // printf("dc removed IR = %.3f\n", dcFilterIR.result);
    // printf("dc removed Red = %.3f\n", dcFilterRed.result);

    prevFifo = rawData;

    float meanDiffResIR = meanDiff(dcFilterIR.result, &meanDiffIR);
    //  printf("mean filter = %.3f\n", meanDiffResIR);

    lowPassButterworthFilter(meanDiffResIR /* -dcFilterIR.result*/, &lpbFilterIR);
    // printf("LPB Flilter = %.3f\n", lpbFilterIR.result);

    irACValueSqSum += dcFilterIR.result * dcFilterIR.result;
    redACValueSqSum += dcFilterRed.result * dcFilterRed.result;
    samplesRecorded++;
    // printf("lpd filter value = %.3f\n", lpbFilterIR.result);
    // printf("samples recorded = %d\n", samplesRecorded);
    if (detectPulse(lpbFilterIR.result) && samplesRecorded > 0)
    {
        result.pulseDetected = true;
        pulsesDetected++;

        float ratioRMS = log(sqrt(redACValueSqSum / samplesRecorded)) / log(sqrt(irACValueSqSum / samplesRecorded));

        if (debug == true)
        {
            printf("RMS Ratio: ");
            printf("%f\n", ratioRMS);
        }

        // This is my adjusted standard model, so it shows 0.89 as 94% saturation. It is probably far from correct, requires proper empircal calibration
        currentSaO2Value = 110.0 - (18.0 * ratioRMS);
        result.SaO2 = currentSaO2Value;

        if (pulsesDetected % RESET_SPO2_EVERY_N_PULSES == 0)
        {
            irACValueSqSum = 0;
            redACValueSqSum = 0;
            samplesRecorded = 0;
        }
    }
    // if (op == 2)
    //{
    balanceIntesities(dcFilterRed.w, dcFilterIR.w);
    //}
    result.heartBPM = currentBPM;
    result.irCardiogram = lpbFilterIR.result;
    result.irDcValue = dcFilterIR.w;
    result.redDcValue = dcFilterRed.w;
    result.lastBeatThreshold = lastBeatThreshold;
    result.dcFilteredIR = dcFilterIR.result;
    result.dcFilteredRed = dcFilterRed.result;

    return result;
}