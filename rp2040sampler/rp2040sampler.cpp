#include <optional>
#include <tuple>

#include "lwipopts.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "statemachine.h"

#include "PZEM-004T-v30/src/PZEM004Tv30.h"

#include "TrueRMS/src/TrueRMS.h"

#define LED1_PIN 25
#define LED2_PIN 17

static inline uint32_t millis(void) { return to_ms_since_boot(get_absolute_time()); }

#define SAMPLE_FREQUENCY    10000

#include "config.h"

static ip_addr_t MQTT_HOST;

static mqtt_client_t static_client { 0 };

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

char sys_id[5] { 0 };

bool timer_isr(struct repeating_timer *t)
{
	int_count++;
	uint32_t next = (bufw + 1) % BUF_SIZE;
	if (!overflow) {
		if (next != bufr) {
			latest_reading = buffer[bufw] = adc_read();
			tt++;
			bufw = next;
		} else {
			overflow = true;
		}
	}

	return true;
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

static void do_measure(double *const hz, double *ac_volt_rms, double *volt_dc_bias, double *variance, double *diff)
{
	// take stats
	sample_reset();
	while (!overflow) sleep_ms(1);
	qsort(buffer, BUF_SIZE, sizeof(uint16_t), compare_uint16);
	uint16_t q1 = buffer[2 * BUF_SIZE / 8];
	uint16_t med = buffer[4 * BUF_SIZE / 8];
	uint16_t q3 = buffer[6 * BUF_SIZE / 8];

	constexpr int window = SAMPLE_FREQUENCY / 50 * 4;

	constexpr int RMS_WINDOW = window;  // "TrueRMS" library limited this variable to 8 bits
	constexpr float acVoltRange = 500;  // peak-to-peak voltage scaled down to 0-5V is 700V (=700/2*sqrt(2) = 247.5Vrms max).
	Rms readRms;
	readRms.begin(acVoltRange, RMS_WINDOW, ADC_12BIT, BLR_ON, SGL_SCAN);
	readRms.start();

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
	double max_ = -65535, min_ = 65535;
	while (!done && ((millis() - start) < 3000)) {
		uint16_t temp = 0;
		if (sample_get(&temp)) {
			int16_t signed_value = temp - 2048;
			double frag = signed_value / 2048.;
			if (rms_started) {
				readRms.update(signed_value);
				w.update(frag);
				if (frag > max_) max_ = frag;
				if (frag < min_) min_ = frag;
			}
			double value = temp;
			double time = (double)t / SAMPLE_FREQUENCY;
			if (sm.process(time, value - med)) {
				switch (count) {
					case 0:
						gpio_put(LED2_PIN, 1);
						first = sm.get_result();
						rms_started = do_rms;
						break;
					case 4:
						rms_started = false;
						do_rms = false;
						break;
					case 25:
						gpio_put(LED2_PIN, 0);
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

	readRms.publish();

	*hz = 50.0 / (last - first);

	auto w_resul = w.get();

	if (w_resul.has_value())
		*variance = std::get<1>(w_resul.value());
	else
		*variance = -1.;

	// 3.205: assuming voltage divider of 5.6k and 10k ohm
	*ac_volt_rms = readRms.rmsVal * (3.3 / 3.205);

	*volt_dc_bias = readRms.dcBias;

	*diff = max_ - min_;
}

static void mqtt_pub_request_cb(void *arg, err_t result)
{
	if (result != ERR_OK)
		printf("Publish failed\n");
}

static int mqtt_failures = 0;

static void publish_mqtt(mqtt_client_t *client, const char *sys_id, const char *topic, const char *what)
{
	u8_t qos = 2;  // 0 1 or 2, see MQTT specification
	u8_t retain = 0;  // don't retain

	char buffer[128];
	snprintf(buffer, sizeof buffer, "mains/%s/%s", sys_id, topic);

	printf("%s -> %s\n", buffer, what);

	err_t err = mqtt_publish(client, buffer, what, strlen(what), qos, retain, mqtt_pub_request_cb, nullptr);
	if (err != ERR_OK)
		printf("mqtt_publish failed: %d\n", err), mqtt_failures++;
	else
	{
		static int pin_state = 1;

		gpio_put(LED1_PIN, pin_state);

		pin_state = !pin_state;
	}
}

static void publish_mqtt(mqtt_client_t *client, const char *sys_id, const char *topic, const double value)
{
	char buffer[16] { 0 };
	snprintf(buffer, sizeof buffer, "%.06f", value);

	publish_mqtt(client, sys_id, topic, buffer);
}

static void do_mqtt_connect(mqtt_client_t *client);

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
	if (status == MQTT_CONNECT_ACCEPTED)
		printf("mqtt_connection_cb: successfully connected\n"), mqtt_failures = 0;
	else {
		printf("mqtt_connection_cb: disconnected, reason: %d\n", status);

		do_mqtt_connect(client);
	}
}

bool hostname_found = false;

static void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
	hostname_found = true;

	ip_addr_t *tgt = (ip_addr_t *)callback_arg;

	if (ipaddr) {
		*tgt = *ipaddr;
		printf("dns lookup finished\n");
	}
	else {
		printf("dns lookup failed\n");
	}
}

static void do_mqtt_connect(mqtt_client_t *client)
{
	mqtt_connect_client_info_t ci { 0 };

	static char client_id[16] { 0 };
	snprintf(client_id, sizeof client_id, "MainsData-%s", sys_id);
	ci.client_id = client_id;

	err_t err = mqtt_client_connect(client, &MQTT_HOST, MQTT_PORT, mqtt_connection_cb, 0, &ci);

	if (err != ERR_OK)
		printf("Failed to connect to MQTT server\n"), mqtt_failures++;
}

static void software_reset()
{
	watchdog_enable(1, 1);
	while(1);
}

void thread2()
{
	if (cyw43_arch_init_with_country(CYW43_COUNTRY_NETHERLANDS))
		printf("failed to initialise\n");

	cyw43_arch_enable_sta_mode();

	netif_set_hostname(netif_default, "MainsData");

	int rc = -1;
	do {
		rc = cyw43_arch_wifi_connect_timeout_ms(W_SSID, W_PASS, CYW43_AUTH_WPA2_MIXED_PSK, 10000);
		if (rc)
			printf("failed to connect %d\n", rc);
	}
	while(rc != 0);

	printf("connect to wifi\n");

	printf("IP: %s\n", ip4addr_ntoa(netif_ip_addr4(netif_default)));
	printf("Mask: %s\n", ip4addr_ntoa(netif_ip_netmask4(netif_default)));
	printf("Gateway: %s\n", ip4addr_ntoa(netif_ip_gw4(netif_default)));
	printf("Host name: %s\n", netif_get_hostname(netif_default));

	ip_addr_t dnsServerIp;
	IP_ADDR4(&dnsServerIp, 8, 8, 8, 8);
	dns_setserver(0, &dnsServerIp);

	//	cyw43_arch_lwip_begin();

	printf("start dns lookup\n");

	dns_gethostbyname("vps001.vanheusden.com", &MQTT_HOST, dns_found, &MQTT_HOST);

	watchdog_enable(2000, 1);

	uint32_t start_ts = millis();

	while(!hostname_found) {
#if PICO_CYW43_ARCH_POLL
		cyw43_arch_poll();
#endif
		sleep_ms(10);

		watchdog_update();

		if (millis() - start_ts > 5000) {
			printf("DNS not responding?\n");

			software_reset();
		}
	}

	printf("hostname found: %s\n", ipaddr_ntoa(&MQTT_HOST));

	//	cyw43_arch_lwip_end();

	PZEM004Tv30 pzem;

	printf("addr: %02x\n", pzem.readAddress(true));

	//	pzem.search();

	for(;;) {
		watchdog_update();

		if (mqtt_failures >= 3) {
			if (mqtt_failures >= 6) {
				// reboot
				printf("failure count too high (%d)\n", mqtt_failures);
				software_reset();
			}

			printf("Forcing mqtt reconnect (%d)\n", mqtt_failures);
			mqtt_disconnect(&static_client);
		}

		if (mqtt_client_is_connected(&static_client) == 0)
			do_mqtt_connect(&static_client);

		watchdog_update();

		double ac_volt_rms = 0.;
		double dc_bias = 0.;
		double hz = 0.;
		double variance = 0.;
		double diff = 0;
		do_measure(&hz, &ac_volt_rms, &dc_bias, &variance, &diff);

		publish_mqtt(&static_client, sys_id, "diff", double(diff));

		publish_mqtt(&static_client, sys_id, "variance", variance);

		publish_mqtt(&static_client, sys_id, "frequency", hz);

		publish_mqtt(&static_client, sys_id, "ac-voltager-rms", ac_volt_rms);

		publish_mqtt(&static_client, sys_id, "dc-bias", dc_bias);

		float voltage = pzem.voltage();
		if (voltage > 0 && voltage < 300)
			publish_mqtt(&static_client, sys_id, "pzem/voltage", voltage);

		float frequency2 = pzem.frequency();
		if (frequency2 > 10 && frequency2 < 100)
			publish_mqtt(&static_client, sys_id, "pzem/frequency", frequency2);

		float current = pzem.current();
		if (current >= 0 && current < 100)
			publish_mqtt(&static_client, sys_id, "pzem/current", current);

		publish_mqtt(&static_client, sys_id, "pzem/power", pzem.power());

		publish_mqtt(&static_client, sys_id, "pzem/energy", pzem.energy());

		publish_mqtt(&static_client, sys_id, "pzem/factor", pzem.pf());
	}

	cyw43_arch_deinit();
}

int main(int argc, char *argv[])
{
	stdio_init_all();

	uart_init(uart0, 9600);

	gpio_set_function(0, GPIO_FUNC_UART);
	gpio_set_function(1, GPIO_FUNC_UART);

	if (watchdog_caused_reboot())
		printf("Rebooted by watchdog!\n");

	watchdog_update();

	printf("Init ADC\n");

	adc_init();
	adc_gpio_init(26);
	adc_select_input(0);

	printf("Init GPIO\n");

	gpio_init(LED1_PIN);
	gpio_set_dir(LED1_PIN, GPIO_OUT);

	gpio_init(LED2_PIN);
	gpio_set_dir(LED2_PIN, GPIO_OUT);

	printf("Start timer\n");

	add_repeating_timer_us(-1000000 / SAMPLE_FREQUENCY, timer_isr, nullptr, &timer);

	uint8_t sys_id_temp[8];
	flash_get_unique_id(sys_id_temp);
	// fold in half
	for(int i=0; i<4; i++)
		sys_id_temp[i] ^= sys_id_temp[i + 4];
	// and again
	for(int i=0; i<2; i++)
		sys_id_temp[i] ^= sys_id_temp[i + 2];
	snprintf(sys_id, sizeof sys_id, "%02x%02x", sys_id_temp[0], sys_id_temp[1]);

	printf("System ID: %s\n", sys_id);

	printf("Start core 1\n");

	multicore_launch_core1(thread2);

	printf("Core 0 - sleep for timer\n");

	for(;;)
		sleep_ms(1000);

	return 0;
}
