#include "statemachine.h"

#define SAMPLE_FREQUENCY    5000

// sample buffer
#define BUF_SIZE    1500
static uint8_t buffer[BUF_SIZE];
static volatile uint16_t bufr = 0;
static volatile uint16_t bufw = 0;
static volatile bool overflow = false;

ISR(TIMER0_COMPA_vect)
{
    if (!overflow) {
        uint16_t next = (bufw + 1) % BUF_SIZE;

        if (next != bufr) {
            buffer[bufw] = analogRead(A0) >> 2;
            digitalWrite(LED_BUILTIN, !(micros() & 0x40000));
            bufw = next;
        } else {
            overflow = true;
        }
    }
}

static void adc_init(uint16_t hz)
{
    // with help from http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html

    // TIMER 0 for interrupt frequency 'hz' Hz:
    cli(); // stop interrupts
    TCCR0A = 0; // set entire TCCR0A register to 0
    TCCR0B = 0; // same for TCCR0B
    TCNT0  = 0; // initialize counter value to 0
    // set compare match register for 'hz' Hz increments
    OCR0A = 16000000ll / (64 * hz) - 1; // (must be <256)
    // turn on CTC mode
    TCCR0B |= (1 << WGM01);
    // Set CS02, CS01 and CS00 bits for 64 prescaler
    TCCR0B |= (0 << CS02) | (1 << CS01) | (1 << CS00);
    // enable timer compare interrupt
    TIMSK0 |= (1 << OCIE0A);
    sei(); // allow interrupts
}

static int compare_uint8(const void *v1, const void *v2)
{
    uint8_t u1 = *((uint8_t *) v1);
    uint8_t u2 = *((uint8_t *) v2);
    return u1 - u2;
}

static void sample_reset(void)
{
    bufr = 0;
    bufw = 0;
    overflow = false;
}

static bool sample_get(Double * pval)
{
    if (bufr == bufw)
        return false;

    int next = (bufr + 1) % BUF_SIZE;
    *pval = Double(buffer[bufr]);
    bufr = next;
    return true;
}

static int do_freq()
{
    // take stats
    sample_reset();

    while (!overflow);

    qsort(buffer, BUF_SIZE, sizeof(buffer[0]), compare_uint8);
    uint8_t q1  = buffer[2 * BUF_SIZE / 8];
    uint8_t med = buffer[4 * BUF_SIZE / 8];
    uint8_t q3  = buffer[6 * BUF_SIZE / 8];
    Serial.print(F("stats: "));
    Serial.print(q1);
    Serial.print('-');
    Serial.print(med);
    Serial.print('-');
    Serial.println(q3);

    // determine zero crossings
    sample_reset();
    StateMachine sm(q1 - med, q3 - med);
    uint16_t t = 0;
    uint32_t start = millis();
    Double first;
    Double last;
    uint16_t count = 0;
    bool done = false;
    while (!done && ((millis() - start) < 3000)) {
        Double value;
        if (sample_get(&value)) {
            Double time = Double(t) / Double(uint16_t(SAMPLE_FREQUENCY));
            if (sm.process(time, value - med)) {
                switch (count) {
                case 0:
                    first = sm.get_result();
                    break;
                case 50:
                    last = sm.get_result();
                    done = true;
                    break;
                default:
                    break;
                }
                count++;
            }
            t++;
        }
    }
    Double freq = Double(50.0) / (last - first);
    Serial.print(F("overflow="));
    Serial.print(overflow);
    Serial.print(F(",done="));
    Serial.print(done);
    Serial.print(F(",n="));
    Serial.print(count);
    Serial.print(F(",first="));
    Serial.print(first.toString());
    Serial.print(F(",last="));
    Serial.print(last.toString());
    Serial.print(F(",freq="));
    Serial.println(freq.toString());

    return 1;
}

void setup(void) {
    Serial.begin(115200);
    Serial.println(F("AVR328SAMPLER"));

    pinMode(LED_BUILTIN, OUTPUT);

    adc_init(SAMPLE_FREQUENCY);
}

void loop(void) {
    do_freq();
}
