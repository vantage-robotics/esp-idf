/* OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "soc/uart_struct.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "mavlink.h"

#include "ubx.h"
#include "ms5611.h"
#include "bq27421.h"

extern uint32_t pvtUpdates;

#define EXAMPLE_MAX_STA_CONN 1
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD
//#define EXAMPLE_SERVER_IP   CONFIG_SERVER_IP
#define EXAMPLE_SERVER_IP   "192.168.4.2"
#define EXAMPLE_SERVER_PORT CONFIG_SERVER_PORT
#define EXAMPLE_FILENAME CONFIG_EXAMPLE_FILENAME
#define BUFFSIZE 1024
#define TEXT_BUFFSIZE 1024

ubx_payload_rx_nav_pvt_t _currPvt;

static const char *TAG = "ota";
ms5611_t baro;
bq27421_t gauge;

/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
/*an packet receive buffer*/
static char text[BUFFSIZE + 1] = { 0 };
/* an image total length*/
static int binary_file_length = 0;
/*socket id*/
static int socket_id = -1;
static bool join_ap = false;
static bool charging = false;
static bool connected = false;
static float baroAltitude = 0.0f;
char my_ip[32];
/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

#define LEDC_TEST_CH_NUM 3
#define LEDC_TEST_FADE_TIME    (50)
#define LEDC_OFF_DUTY 8192
#define LEDC_BRIGHT_DUTY 4000

ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = 19,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_1,
            .duty       = 0,
            .gpio_num   = 22,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
        {
            .channel    = LEDC_CHANNEL_2,
            .duty       = 0,
            .gpio_num   = 27,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
        },
    };

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
    	if (join_ap)
    		esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        connected = true;
        sprintf(my_ip,IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
        ESP_LOGI(TAG, "got ip");
        break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
    	xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    	connected = true;
    	break;
    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                 MAC2STR(event->event_info.sta_connected.mac),
                 event->event_info.sta_connected.aid);
        break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                 MAC2STR(event->event_info.sta_disconnected.mac),
                 event->event_info.sta_disconnected.aid);
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        connected = false;
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
    	if (join_ap)
    		esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        connected = false;
        break;
    default:
        break;
    }
    return ESP_OK;
}

// start as station, connect to any Snap_XXXX
void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();
    wifi_ap_record_t scan_results[20];
    uint16_t num_scans = 20;

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS
        },
    };
    wifi_scan_config_t scan_config;
	scan_config.ssid = 0;
	scan_config.bssid = 0;
	scan_config.channel = 6;
	scan_config.scan_type = WIFI_SCAN_TYPE_ACTIVE;
	scan_config.scan_time.passive = 1000;
	scan_config.scan_time.active.min = 100;
	scan_config.scan_time.active.max = 1000; // 1.5 sec

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    //ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true) );

    ESP_LOGI(TAG, "scan completed");
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_scans, scan_results) );
    ESP_ERROR_CHECK(esp_wifi_stop() );

    for (int i=0; i < num_scans; i++)
    {
    	ESP_LOGI(TAG, "num:%d,ssid:%s",i,scan_results[i].ssid);
    	if (scan_results[i].ssid[0] == 'S' && scan_results[i].ssid[1] == 'n' && scan_results[i].ssid[2] == 'a' && scan_results[i].ssid[3] == 'p')
    	{
    		strcpy((char*)wifi_config.sta.ssid, (const char*)scan_results[i].ssid);
    		join_ap = true;
    		break;
    	}
    }

    if (join_ap)
    {
		ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
		ESP_ERROR_CHECK(esp_wifi_start() );

		ESP_LOGI(TAG, "wifi_init_sta finished.");
		ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
				 wifi_config.sta.ssid, EXAMPLE_WIFI_PASS);
    }
    else
    {

    }
}

static void initialise_wifi(void)
{
    //tcpip_adapter_init();
    //wifi_event_group = xEventGroupCreate();
    //ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    //wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    //ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    //ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );

    // stop DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP));

    // assign a static IP to the network interface
    tcpip_adapter_ip_info_t info;
    memset(&info, 0, sizeof(info));
    IP4_ADDR(&info.ip, 192, 168, 4, 1);
    IP4_ADDR(&info.gw, 192, 168, 4, 1);
    IP4_ADDR(&info.netmask, 255, 255, 255, 0);
    ESP_ERROR_CHECK(tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_AP, &info));

    // start the DHCP server
    ESP_ERROR_CHECK(tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP));

    wifi_config_t wifi_config = {
            .ap = {
                .ssid = EXAMPLE_WIFI_SSID,
                .ssid_len = strlen(EXAMPLE_WIFI_SSID),
                .password = EXAMPLE_WIFI_PASS,
                .max_connection = EXAMPLE_MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA_WPA2_PSK
            },
        };
        if (strlen(EXAMPLE_WIFI_PASS) == 0) {
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        }

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
        ESP_ERROR_CHECK(esp_wifi_start());

        ESP_LOGI(TAG, "wifi_init_softap finished.SSID:%s password:%s",
                 EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
}

/*read buffer by byte still delim ,return read bytes counts*/
static int read_until(char *buffer, char delim, int len)
{
//  /*TODO: delim check,buffer check,further: do an buffer length limited*/
    int i = 0;
    while (buffer[i] != delim && i < len) {
        ++i;
    }
    return i + 1;
}

/* resolve a packet from http socket
 * return true if packet including \r\n\r\n that means http packet header finished,start to receive packet body
 * otherwise return false
 * */
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle)
{
    /* i means current position */
    int i = 0, i_read_len = 0;
    while (text[i] != 0 && i < total_len) {
        i_read_len = read_until(&text[i], '\n', total_len);
        // if we resolve \r\n line,we think packet header is finished
        if (i_read_len == 2) {
            int i_write_len = total_len - (i + 2);
            memset(ota_write_data, 0, BUFFSIZE);
            /*copy first http packet body to write buffer*/
            memcpy(ota_write_data, &(text[i + 2]), i_write_len);

            esp_err_t err = esp_ota_write( update_handle, (const void *)ota_write_data, i_write_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed (%s)!", esp_err_to_name(err));
                return false;
            } else {
                ESP_LOGI(TAG, "esp_ota_write header OK");
                binary_file_length += i_write_len;
            }
            return true;
        }
        i += i_read_len;
    }
    return false;
}

static bool connect_to_http_server()
{
    ESP_LOGI(TAG, "Server IP: %s Server Port:%s", EXAMPLE_SERVER_IP, EXAMPLE_SERVER_PORT);

    int  http_connect_flag = -1;
    struct sockaddr_in sock_info;

    socket_id = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_id == -1) {
        ESP_LOGE(TAG, "Create socket failed!");
        return false;
    }

    // set connect info
    memset(&sock_info, 0, sizeof(struct sockaddr_in));
    sock_info.sin_family = AF_INET;
    sock_info.sin_addr.s_addr = inet_addr(EXAMPLE_SERVER_IP);
    sock_info.sin_port = htons(atoi(EXAMPLE_SERVER_PORT));

    // connect to http server
    http_connect_flag = connect(socket_id, (struct sockaddr *)&sock_info, sizeof(sock_info));
    if (http_connect_flag == -1) {
        ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
        close(socket_id);
        return false;
    } else {
        ESP_LOGI(TAG, "Connected to server");
        return true;
    }
    return false;
}

static void __attribute__((noreturn)) task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    close(socket_id);
    (void)vTaskDelete(NULL);

    while (1) {
        ;
    }
}

static void ota_example_task(void *pvParameter)
{
    esp_err_t err;
    /* update handle : set by esp_ota_begin(), must be freed via esp_ota_end() */
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;

    ESP_LOGI(TAG, "Starting OTA example...");

    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    if (configured != running) {
        ESP_LOGW(TAG, "Configured OTA boot partition at offset 0x%08x, but running from offset 0x%08x",
                 configured->address, running->address);
        ESP_LOGW(TAG, "(This can happen if either the OTA boot data or preferred boot image become corrupted somehow.)");
    }
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    /* Wait for the callback to set the CONNECTED_BIT in the
       event group.
    */
    while (!connected)
    {
    	if (gpio_get_level(35) == 0)
    	{
    		charging = true;
    		esp_wifi_stop();
    		vTaskSuspend(NULL);
    	}
    	vTaskDelay( 100 / portTICK_PERIOD_MS );
    	ESP_LOGI(TAG, ".");
    }
    //xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
    //                    false, true, portMAX_DELAY);

    ESP_LOGI(TAG, "Connect to Wifi ! Start to Connect to Server....");

    /*connect to http server*/
    if (connect_to_http_server()) {
        ESP_LOGI(TAG, "Connected to http server");
    } else {
        ESP_LOGE(TAG, "Connect to http server failed!");
        task_fatal_error();
    }

    /*send GET request to http server*/
    const char *GET_FORMAT =
        "GET %s HTTP/1.0\r\n"
        "Host: %s:%s\r\n"
        "User-Agent: esp-idf/1.0 esp32\r\n\r\n";

    char *http_request = NULL;
    int get_len = asprintf(&http_request, GET_FORMAT, EXAMPLE_FILENAME, EXAMPLE_SERVER_IP, EXAMPLE_SERVER_PORT);
    if (get_len < 0) {
        ESP_LOGE(TAG, "Failed to allocate memory for GET request buffer");
        task_fatal_error();
    }
    int res = send(socket_id, http_request, get_len, 0);
    free(http_request);

    if (res < 0) {
        ESP_LOGE(TAG, "Send GET request to server failed");
        task_fatal_error();
    } else {
        ESP_LOGI(TAG, "Send GET request to server succeeded");
    }

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
        task_fatal_error();
    }
    ESP_LOGI(TAG, "esp_ota_begin succeeded");

    bool resp_body_start = false, socket_flag = true, http_200_flag = false;
    /*deal with all receive packet*/
    while (socket_flag) {
        memset(text, 0, TEXT_BUFFSIZE);
        memset(ota_write_data, 0, BUFFSIZE);
        int buff_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
        if (buff_len < 0) { /*receive error*/
            ESP_LOGE(TAG, "Error: receive data error! errno=%d", errno);
            task_fatal_error();
        } else if (buff_len > 0 && !resp_body_start) {  /*deal with response header*/
            // only start ota when server response 200 state code
            if (strstr(text, "200") == NULL && !http_200_flag) {
                ESP_LOGE(TAG, "ota url is invalid or bin is not exist");
                task_fatal_error();
            }
            http_200_flag = true;
            memcpy(ota_write_data, text, buff_len);
            resp_body_start = read_past_http_header(text, buff_len, update_handle);
        } else if (buff_len > 0 && resp_body_start) { /*deal with response body*/
            memcpy(ota_write_data, text, buff_len);
            err = esp_ota_write( update_handle, (const void *)ota_write_data, buff_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed (%s)!", esp_err_to_name(err));
                task_fatal_error();
            }
            binary_file_length += buff_len;
            ESP_LOGI(TAG, "Have written image length %d", binary_file_length);
        } else if (buff_len == 0) {  /*packet over*/
            socket_flag = false;
            ESP_LOGI(TAG, "Connection closed, all packets received");
            close(socket_id);
        } else {
            ESP_LOGE(TAG, "Unexpected recv result");
        }
    }

    ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);

    if (esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        task_fatal_error();
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        task_fatal_error();
    }
    ESP_LOGI(TAG, "Prepare to restart system!");
    esp_restart();
    return ;
}
static void charging_task(void *pvParameter)
{
	static bool was_charging = false;
	int ch;
	static int brightness = 8000;

	while(1)
	{
		if (charging && !was_charging)
		{
			ESP_LOGI(TAG, "starting charge power down");
			was_charging = charging;
			powerdown_pam(1);
			for (ch = 0; ch < 3; ch++) {
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 8192);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
			}
			esp_sleep_enable_timer_wakeup(500);
		}

		if (charging)
		{
			//ESP_LOGI(TAG, "charging");
			brightness -= 1;
			if (brightness < 5000)
				brightness = 8000;
			for (ch = 0; ch < 1; ch++) {
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, brightness);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
			}

			esp_light_sleep_start();

			for (ch = 0; ch < 1; ch++) {
				ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, brightness);
				ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
			}

		}

		charging = !gpio_get_level(35);

		if (!charging && was_charging)
			esp_restart();

		if (!charging)
			vTaskDelay( 1000 / portTICK_PERIOD_MS );
	}

}
static void i2c_task(void *pvParameter)
{
	int32_t pressure;
	float temperature;
	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000.0;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	const double p1 = 101325.0 / 1000.0;

	while (1)
	{
		ms5611_get_sensor_data(&baro, &pressure, &temperature);

		/* measured pressure in kPa */
		double p = (double)pressure / 1000.0;

		baroAltitude = (float)((((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a);

		vTaskDelay( 100 / portTICK_PERIOD_MS );
		ESP_LOGI(TAG, "baro alt:%0.3f", (float)baroAltitude);
		ESP_LOGI(TAG, "baro pres:%d", pressure);
	}
}
static void gps_task(void *pvParameter)
{
	#define udpAddress "192.168.20.1"
	const int udpPort = 44444;
	static uint32_t lastPvtUpdates = 0;
	static const char *RX_TASK_TAG = "RX_TASK";
	esp_log_level_set(RX_TASK_TAG, ESP_LOG_DEBUG);

    int socket_fd;
    struct sockaddr_in sa,ra;

    int sent_data;
    int ch, duty = LEDC_OFF_DUTY;

    ledc_fade_func_install(0);

    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
                        false, true, portMAX_DELAY);

    /* Creates an UDP socket (SOCK_DGRAM) with Internet Protocol Family (PF_INET).
     * Protocol family and Address family related. For example PF_INET Protocol Family and AF_INET family are coupled.
    */
    socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

    if ( socket_fd < 0 )
    {
        printf("socket call failed");
        exit(0);
    }

    int yes = 1;
    if (setsockopt(socket_fd,SOL_SOCKET,SO_REUSEADDR,&yes,sizeof(yes)) < 0) {
	  printf("could not set socket option: %d", errno);
	  exit(0);
    }

    memset(&sa, 0, sizeof(struct sockaddr_in));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr(my_ip);
    sa.sin_port = htons(udpPort);

    /* Bind the TCP socket to the port SENDER_PORT_NUM and to the current
    * machines IP address (Its defined by SENDER_IP_ADDR).
    * Once bind is successful for UDP sockets application can operate
    * on the socket descriptor for sending or receiving data.
    */
    if (bind(socket_fd, (struct sockaddr *)&sa, sizeof(struct sockaddr_in)) == -1)
    {
      printf("Bind to Port Number %d ,IP address %s failed\n",udpPort,my_ip /*SENDER_IP_ADDR*/);
      close(socket_fd);
      exit(1);
    }
    fcntl(socket_fd, F_SETFL, O_NONBLOCK);
    printf("Bind to Port Number %d ,IP address %s SUCCESS!!!\n",udpPort,my_ip);

    memset(&ra, 0, sizeof(struct sockaddr_in));
    ra.sin_family = AF_INET;
    ra.sin_addr.s_addr = inet_addr(udpAddress);
    ra.sin_port = htons(udpPort);

	while(1)
	{
		receive(10);

		if (pvtUpdates > lastPvtUpdates)
		{
			lastPvtUpdates = pvtUpdates;

			mavlink_message_t mavlink_msg_out;
			char mavBuf[256];

			mavlink_msg_tracker_data_pack(0, 0, &mavlink_msg_out, 6,
			                                  _currPvt.iTOW, _currPvt.fixType, _currPvt.numSV,
			                                  _currPvt.lon, _currPvt.lat, (float) _currPvt.height/1000,
			                                  baroAltitude, _currPvt.velN, _currPvt.velE, _currPvt.velD,
			                                  _currPvt.sAcc, _currPvt.hAcc, _currPvt.vAcc);
			uint16_t len = mavlink_msg_to_send_buffer((uint8_t *) mavBuf, &mavlink_msg_out);

			sent_data = sendto(socket_fd, mavBuf, (size_t)len, 0, (struct sockaddr*)&ra, sizeof(ra));

			if(sent_data<0)
				ESP_LOGI(TAG,"gps update, udp error:%d",errno);
			else
			{
				//ESP_LOGI(TAG, "gps update, sent %d bytes on udp",sent_data);

				if (duty == LEDC_OFF_DUTY)
					duty = LEDC_BRIGHT_DUTY;
				else
					duty = LEDC_OFF_DUTY;

				for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
					ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
							ledc_channel[ch].channel, duty, LEDC_TEST_FADE_TIME);
					ledc_fade_start(ledc_channel[ch].speed_mode,
							ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
				}
			}

		}
		vTaskDelay( 50 / portTICK_PERIOD_MS );

	}
}
void led_init(void)
{
	int ch;

	/*
	 * Prepare and set configuration of timers
	 * that will be used by LED Controller
	 */
	ledc_timer_config_t ledc_timer = {
		.duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
		.freq_hz = 5000,                      // frequency of PWM signal
		.speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
		.timer_num = LEDC_TIMER_0            // timer index
	};
	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);

	// Set LED Controller with previously prepared configuration
	    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
	        ledc_channel_config(&ledc_channel[ch]);
	        ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 8192);
	       	ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
	    }


}
void app_main()
{
    // Initialize NVS.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // try connect to snap
    wifi_init_sta();

    led_init();

    gpio_set_direction(35, GPIO_MODE_INPUT);
    xTaskCreate(&charging_task, "charging_task", 2048, NULL, configMAX_PRIORITIES - 5, NULL);
    ESP_LOGI(TAG, "charging task created");

    // if failed, turn to AP for config/OTA
    if (!join_ap)
    {
    	initialise_wifi();
    	xTaskCreate(&ota_example_task, "ota_example_task", 8192, NULL, 5, NULL);
    } else {
    	 while (i2cdev_init() != ESP_OK)
    	    {
    	        printf("Could not init I2Cdev library\n");
    	        vTaskDelay(250 / portTICK_PERIOD_MS);
    	    }
    	//ESP_LOGI(TAG, "gauge init");
    	//bq27421_init_desc(&gauge, I2C_NUM_0 , GPIO_NUM_5, GPIO_NUM_18);
    	//bq27421_battery_configure_capacity(&gauge, 0);

    	ESP_LOGI(TAG, "ms5611 init desc");
    	ms5611_init_desc(&baro, 0x76, I2C_NUM_0 , GPIO_NUM_5, GPIO_NUM_18);
    	ESP_LOGI(TAG, "ms5611 init");
    	ms5611_init(&baro, MS5611_OSR_1024);

    	configure_pam(&_currPvt);
    	ESP_LOGI(TAG, "gps initialized");
    	xTaskCreate(&gps_task, "gps_rx_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    	ESP_LOGI(TAG, "gps task created");
    	xTaskCreate(&i2c_task, "i2c_task", 2048, NULL, configMAX_PRIORITIES - 2, NULL);
    	ESP_LOGI(TAG, "i2c task created");

    }
}
