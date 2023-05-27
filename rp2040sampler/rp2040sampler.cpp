#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
//#include <lwip/apps/mqtt_priv.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"

//#include "TrueRMS/src/TrueRMS.h"

#include "statemachine.h"

#define PIN_50HZ_INPUT      A0
#define SAMPLE_FREQUENCY    10000

#define W_SSID ""
#define W_PASS ""

const ip_addr_t MQTT_HOST = IPADDR4_INIT_BYTES(192, 168, 64, 1);

static mqtt_client_t static_client;

static char line[120];

static uint32_t value = 0;
static volatile uint32_t int_count = 0;

// sample buffer
#define BUF_SIZE    2048
static uint16_t buffer[BUF_SIZE];
static volatile uint32_t bufr = 0;
static volatile uint32_t bufw = 0;
static volatile bool overflow = false;
static volatile uint16_t latest_reading = 0;

static repeating_timer timer;

static int tt = 0;

bool timer_isr(struct repeating_timer *t)
{
    int_count++;
    uint32_t next = (bufw + 1) % BUF_SIZE;
    if (!overflow) {
        if (next != bufr) {
            latest_reading = buffer[bufw] = analogRead(PIN_50HZ_INPUT);
            tt++;
            bufw = next;
        } else {
            overflow = true;
        }
    }

    return true;
}

static void adc_init()
{
    analogReadResolution(12);  // 12bit conversion

    add_repeating_timer_us(-1000000 / SAMPLE_FREQUENCY, timer_isr, nullptr, &timer);
}

static int compare_uint16(const void *v1, const void *v2)
{
    uint16_t u1 = *((uint16_t *) v1);
    uint16_t u2 = *((uint16_t *) v2);
    return u1 - u2;
}

static void sample_reset(void)
{
    bufr = 0;
    bufw = 0;
    overflow = false;
}

static bool sample_get(uint16_t * pval)
{
    if (bufr == bufw) {
        return false;
    }
    int next = (bufr + 1) % BUF_SIZE;
    *pval = buffer[bufr];
    bufr = next;
    return true;
}

// https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
class welford
{
private:
	int count { 0 };
	double mean { 0. };
	double M2 { 0. };

public:
	welford() {
	}

	void update(double newValue) {
		count++;
		double delta = newValue - mean;
		mean += delta / count;
		double delta2 = newValue - mean;
		M2 += delta * delta2;
	}

	// mean, variance, sample variance
	std::optional<std::tuple<double, double, double> > get() {
		if (count < 2)
			return { };

		return { { mean, M2 / count, M2 / (count - 1) } };
	}
};

static void do_measure(double *const hz, double *ac_volt_rms, double *volt_dc_bias, double *variance)
{
    // take stats
    sample_reset();
    while (!overflow) yield();
    qsort(buffer, BUF_SIZE, sizeof(uint16_t), compare_uint16);
    uint16_t q1 = buffer[2 * BUF_SIZE / 8];
    uint16_t med = buffer[4 * BUF_SIZE / 8];
    uint16_t q3 = buffer[6 * BUF_SIZE / 8];

    constexpr int window = SAMPLE_FREQUENCY / 50 * 4;

//    constexpr int RMS_WINDOW = window;  // "TrueRMS" library limited this variable to 8 bits
//    constexpr float acVoltRange = 500;  // peak-to-peak voltage scaled down to 0-5V is 700V (=700/2*sqrt(2) = 247.5Vrms max).
//    Rms readRms;
//    readRms.begin(acVoltRange, RMS_WINDOW, ADC_12BIT, BLR_ON, SGL_SCAN);
//    readRms.start();

    welford w;

    // determine zero crossings
    sample_reset();
    StateMachine sm(q1 - med, q3 - med);
    int t = 0;
    uint32_t start = millis();
    double first = 0.0;
    double last = 0.0;
    int count = 0;
    bool done = false;
    bool do_rms = true;
    bool rms_started = false;
    while (!done && ((millis() - start) < 3000)) {
        uint16_t temp = 0;
        if (sample_get(&temp)) {
            int16_t signed_value = temp - 2048;
            double frag = signed_value / 2048.;
            if (rms_started) {
                 //readRms.update(signed_value);
	         w.update(frag);
            }
            double value = temp;
            double time = (double)t / SAMPLE_FREQUENCY;
            if (sm.process(time, value - med)) {
                switch (count) {
                case 0:
                    first = sm.get_result();
                    rms_started = do_rms;
                    break;
                case 4:
                    rms_started = false;
                    do_rms = false;
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

    //readRms.publish();

    *hz = 50.0 / (last - first);

    auto w_resul = w.get();

    if (w_resul.has_value())
        *variance = std::get<1>(w_resul.value());
    else
        *variance = -1.;

    // 3.205: assuming voltage divider of 5.6k and 10k ohm
    //*ac_volt_rms = readRms.rmsVal * (3.3 / 3.205);
	*ac_volt_rms = -1.0;

    //*volt_dc_bias = readRms.dcBias;
	*volt_dc_bias = -1.0;
}

void setup(void)
{
    pinMode(LED_BUILTIN, OUTPUT);

    adc_init();

    // platformio currently (2023/05/24) does not implement
    // the setup()/setup1() pair like the arduino ide does
    multicore_launch_core1(thread2);
}

void loop()
{
	delay(16383);
}

void thread2()
{
	setup1();

	for(;;)
		loop1();
}

int main(int argc, char *argv[])
{
	if (cyw43_arch_init()) {
		printf("failed to initialise\n");
	        return 1;
	}

	cyw43_arch_enable_sta_mode();

	if (cyw43_arch_wifi_connect_timeout_ms(W_SSID, W_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
		printf("failed to connect\n");
		return 1;
	}

	setup();

	for(;;)
		loop();

	cyw43_arch_deinit();

	return 0;
}

static void mqtt_pub_request_cb(void *arg, err_t result)
{
    if (result != ERR_OK)
        Serial.println(F("Publish failed"));
}

void publish_mqtt(mqtt_client_t *client, const char *topic, const char *what)
{
    u8_t qos = 2;  // 0 1 or 2, see MQTT specification
    u8_t retain = 0;  // don't retain

    err_t err = mqtt_publish(client, topic, what, strlen(what), qos, retain, mqtt_pub_request_cb, nullptr);
    if (err != ERR_OK)
        Serial.println(F("mqtt_publish failed"));
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    if (status == MQTT_CONNECT_ACCEPTED)
        printf("mqtt_connection_cb: Successfully connected\n");
    else {
        printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);

        do_mqtt_connect(client);
    }
}

static void do_mqtt_connect(mqtt_client_t *client)
{
    mqtt_connect_client_info_t ci { 0 };

    ci.client_id = "50hz";

    err_t err = mqtt_client_connect(client, &MQTT_HOST, MQTT_PORT, mqtt_connection_cb, 0, &ci);

    if (err != ERR_OK)
        Serial.println(F("Failed to connect to MQTT server"));
}

void setup1()
{
    Serial.begin(115200);

    pinMode(16, OUTPUT);

    WiFi.mode(WIFI_STA);

    WiFi.begin(W_SSID, W_PASS);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println(F(""));
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println(F(""));
    Serial.println(F("STM32SAMPLER"));
}

void loop1()
{
    digitalWrite(16, HIGH);

    double ac_volt_rms = 0.;
    double dc_bias = 0.;
    double hz = 0.;
    double variance = 0.;
    do_measure(&hz, &ac_volt_rms, &dc_bias, &variance);

    digitalWrite(16, LOW);

    Serial.print(variance, 6);
    Serial.print(' ');

    char buffer_hz[16] { 0 };
    snprintf(buffer_hz, sizeof buffer_hz, "%.6f", hz);
    Serial.print(buffer_hz);

    publish_mqtt(&static_client, "mains/frequency", buffer_hz);

    Serial.print(' ');

    char buffer_ac_volt_rms[16] { 0 };
    snprintf(buffer_ac_volt_rms, sizeof buffer_ac_volt_rms, "%.6f", ac_volt_rms);
    Serial.println(buffer_ac_volt_rms);

    publish_mqtt(&static_client, "mains/ac-voltager-rms", buffer_ac_volt_rms);
}
