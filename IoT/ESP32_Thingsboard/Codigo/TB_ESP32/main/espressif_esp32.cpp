#include <esp_netif.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <esp_random.h>
#include <string.h>

// Whether the given script is using encryption or not,
// generally recommended as it increases security (communication with the server is not in clear text anymore),
// it does come with an overhead tough as having an encrypted session requires a lot of memory,
// which might not be avaialable on lower end devices.
#define ENCRYPTED false// Para usar puerto 1883 sin encriptación
  

#include <Espressif_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>
// ************DS18B20**************** */
#include "ds18b20.h"  //Header sensor temperatura 
#include "onewire_bus.h"
#include "driver/gpio.h"
// Temp Sensors are on GPIO26
#define EXAMPLE_ONEWIRE_BUS_GPIO    26
#define EXAMPLE_ONEWIRE_MAX_DS18B20 2  
#define LED 2
#define HIGH 1
#define digitalWrite gpio_set_level
static int s_ds18b20_device_num = 0;
static float s_temperature = 0.0;
static ds18b20_device_handle_t s_ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
static int H_temperature = 0;
static const char *TAG1 = "DS18B20";
/*************************************** */
/**************Max31865***************** */
#include <max31865.h>
#include "esp_idf_lib_helpers.h"

#define HOST HELPER_SPI_HOST_DEFAULT

#if CONFIG_EXAMPLE_FILTER_50
#define FILTER MAX31865_FILTER_50HZ
#endif
#if CONFIG_EXAMPLE_FILTER_60
#define FILTER MAX31865_FILTER_60HZ
#endif
#if CONFIG_EXAMPLE_SCALE_ITS90
#define RTD_STANDARD MAX31865_ITS90
#endif
#if CONFIG_EXAMPLE_SCALE_DIN43760
#define RTD_STANDARD MAX31865_DIN43760
#endif
#if CONFIG_EXAMPLE_SCALE_US
#define RTD_STANDARD MAX31865_US_INDUSTRIAL
#endif

#if CONFIG_EXAMPLE_CONN_2WIRE
#define RTD_CONNECTION MAX31865_2WIRE
#endif
#if CONFIG_EXAMPLE_CONN_3WIRE
#define RTD_CONNECTION MAX31865_3WIRE
#endif
#if CONFIG_EXAMPLE_CONN_4WIRE
#define RTD_CONNECTION MAX31865_4WIRE
#endif
/**************************************** */
/************************Salida Rele************* */
#define LED 2
#define HIGH 1
#define digitalWrite gpio_set_level
#define GPIO_COIL_0 GPIO_NUM_2 // Ejemplo: usar el GPIO 2
#define GPIO_COIL_1 GPIO_NUM_4 // Ejemplo: usar el GPIO 4
/********************************************** */
// See https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/memory-types.html#drom-data-stored-in-flash
// for more information about the aforementioned feature
constexpr char WIFI_SSID[] = "Casa 2.4";
constexpr char WIFI_PASSWORD[] = "C4B4Family";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
constexpr char TOKEN[] = "75831rprfjpof6q3vlc7";//"75831rprfjpof6q3vlc7";//"r10pyxvqzmmjf5rpsbbj";

// Thingsboard we want to establish a connection too
constexpr char THINGSBOARD_SERVER[] = "demo.thingsboard.io";

// MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port,
// whereas 8883 would be the default encrypted SSL MQTT port
#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

// Maximum size packets will ever be sent or received by the underlying MQTT client,
// if the size is to small messages might not be sent or received messages will be discarded
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 128U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 128U;

#if ENCRYPTED
// See https://comodosslstore.com/resources/what-is-a-root-ca-certificate-and-how-do-i-download-it/
// on how to get the root certificate of the server we want to communicate with,
// this is needed to establish a secure connection and changes depending on the website.
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";
#endif
/******************Constantes MQTT*********** */
constexpr char TEMPERATURE_KEY[] = "temp1";
constexpr char HUMIDITY_KEY[] = "temp2";
/****************Constantes RPC************** */
constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr char RPC_SWITCH_METHOD[] = "example_set_switch";
constexpr char RPC_SWITCH_DOS_METHOD[] = "set_switch2";
constexpr char RPC_TEMPERATURE_KEY[] = "temp";
constexpr char RPC_SWITCH_KEY[] = "switch";
constexpr char RPC_RESPONSE_KEY[] = "example_response";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

// Initalize the Mqtt client instance
Espressif_MQTT_Client<> mqttClient;
// Initialize ThingsBoard instance with the maximum needed buffer size
//ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE);
// Initialize used apis
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// Status for successfully connecting to the given WiFi
bool wifi_connected = false;
bool subscribed = false;


/// @brief Callback method that is called if we got an ip address from the connected WiFi meaning we successfully established a connection
/// @param event_handler_arg User data registered to the event
/// @param event_base Event base for the handler
/// @param event_id The id for the received event
/// @param event_data The data for the event, esp_event_handler_t
void on_got_ip(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    wifi_connected = true;
}

/// @brief Initalizes WiFi connection,
// will endlessly delay until a connection has been successfully established
void InitWiFi() {
  const wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));

  esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();
  esp_netif_t *netif = esp_netif_new(&netif_config);
  assert(netif);

  ESP_ERROR_CHECK(esp_netif_attach_wifi_station(netif));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ip_event_t::IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
  ESP_ERROR_CHECK(esp_wifi_set_default_wifi_sta_handlers());
  ESP_ERROR_CHECK(esp_wifi_set_storage(wifi_storage_t::WIFI_STORAGE_RAM));

  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), WIFI_SSID, strlen(WIFI_SSID) + 1);
  strncpy(reinterpret_cast<char*>(wifi_config.sta.password), WIFI_PASSWORD, strlen(WIFI_PASSWORD) + 1);

  ESP_LOGI("MAIN", "Connecting to %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(wifi_mode_t::WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(wifi_interface_t::WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_connect());
}
//*******************Funciones DS18B20********************* */
static void sensor_detect(void)
{
    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG1, "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &s_ds18b20s[s_ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI(TAG1, "Found a DS18B20[%d], address: %016llX", s_ds18b20_device_num, next_onewire_device.address);
                s_ds18b20_device_num++;
            } else {
                ESP_LOGI(TAG1, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG1, "Searching done, %d DS18B20 device(s) found", s_ds18b20_device_num);
}

void sensor_read(void)
{
    for (int i = 0; i < s_ds18b20_device_num; i ++) {
        ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(s_ds18b20s[i]));
        ESP_ERROR_CHECK(ds18b20_get_temperature(s_ds18b20s[i], &s_temperature));
        //ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", i, s_temperature);
        //printf("Temperatura Hexa : %4X \n", H_temperature);
    }
}
/***************************************************************** */
/************************Funciones Max31865 y SPI*********************** */
static max31865_config_t config =
{
    .mode = MAX31865_MODE_SINGLE,
    .connection = RTD_CONNECTION,
    .v_bias = true,
    .filter = FILTER

};

max31865_t ConfigMax31865(void  )
{
    // Configure SPI bus
    spi_bus_config_t cfg =
    {
        .mosi_io_num = CONFIG_EXAMPLE_MOSI_GPIO,
        .miso_io_num = CONFIG_EXAMPLE_MISO_GPIO,
        .sclk_io_num = CONFIG_EXAMPLE_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
        .flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(HOST, &cfg, 1));

    // Init device
    max31865_t dev =
    {
        .standard = RTD_STANDARD,
        .r_ref = CONFIG_EXAMPLE_RTD_REF,
        .rtd_nominal = CONFIG_EXAMPLE_RTD_NOMINAL,
    };
    
    ESP_ERROR_CHECK(max31865_init_desc(&dev, HOST, MAX31865_MAX_CLOCK_SPEED_HZ, GPIO_NUM_5));

    // Configure device
    ESP_ERROR_CHECK(max31865_set_config(&dev, &config));
   
    return dev;
}
/********************************Fin************************************ */

/************************Funciones RPC********************************* */
/// @brief Processes function for RPC call "example_json"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods
void processGetJson(const JsonVariantConst &data, JsonDocument &response) {
    ESP_LOGI("RPC Example", "Received the json RPC method");

    // Size of the response document needs to be configured to the size of the innerDoc + 1.
    StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
    innerDoc["string"] = "exampleResponseString";
    innerDoc["int"] = 5;
    innerDoc["float"] = 5.0f;
    innerDoc["bool"] = true;
    response["json_data"] = innerDoc;
    printf("Json : cinco C\n" );
}

/// @brief Processes function for RPC call "example_set_temperature"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods

void processTemperatureChange(const JsonVariantConst &data, JsonDocument &response) {
    const float example_temperature = data[RPC_TEMPERATURE_KEY];
    ESP_LOGI("RPC Example", "Received the set temperature RPC method: %f", example_temperature);

    // Ensure to only pass values do not store by copy, or if they do increase the MaxRPC template parameter accordingly to ensure that the value can be deserialized.
    // See https://arduinojson.org/v6/api/jsondocument/add/ for more information on which variables cause a copy to be created
    response["string"] = "exampleResponseString";
    response["int"] = 5;
    response["float"] = 5.0f;
    response["double"] = 10.0;
    response["bool"] = true;
    printf("temperatura : tres C\n" );
}

/// @brief Processes function for RPC call "example_set_switch"
/// JsonVariantConst is a JSON variant, that can be queried using operator[]
/// See https://arduinojson.org/v5/api/jsonvariant/subscript/ for more details
/// @param data Data containing the rpc data that was called and its current value
/// @param response Data containgin the response value, any number, string or json, that should be sent to the cloud. Useful for getMethods

//RPC Ventilador 1
void processSwitchChange(const JsonVariantConst &data, JsonDocument &response) {
    // Process data

    const bool switch_state = data; // Intenta acceder al valor directamente.
    ESP_LOGI("RPC Example", "Received the set switch method: %d", switch_state);
     
    response.set(22.02);
    if (data==1){
    gpio_set_level(GPIO_COIL_0, 1);
    }
    else {
    gpio_set_level(GPIO_COIL_0, 0);
    }
}

//RPC Ventilador 2
void processSwitchChange2(const JsonVariantConst &data, JsonDocument &response) {
    // Process data

    const bool switch_state = data; // Intenta acceder al valor directamente.
    ESP_LOGI("RPC Example", "Received the set switch method: %d", switch_state);
     
    response.set(22.02);
    if (data==1){
    gpio_set_level(GPIO_COIL_1, 1);
    }
    else {
    gpio_set_level(GPIO_COIL_1, 0);
    }
}
/*************************Fin Funciones RPC***************************** */

extern "C" void app_main() {
    ESP_LOGI("MAIN", "[APP] Startup..");
    ESP_LOGI("MAIN", "[APP] Free memory: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI("MAIN", "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    InitWiFi();
    sensor_detect();
     
    max31865_t dev = ConfigMax31865( ); // Configuracion de SPI y Max
#if ENCRYPTED
    mqttClient.set_server_certificate(ROOT_CERT);
#endif // ENCRYPTED
    gpio_reset_pin(GPIO_COIL_0); // Reset the pin first
    gpio_set_direction(GPIO_COIL_0, GPIO_MODE_OUTPUT); 
    gpio_reset_pin(GPIO_COIL_1); // Reset the pin first
    gpio_set_direction(GPIO_COIL_1, GPIO_MODE_OUTPUT); 
    for (;;) {
        // Wait until we connected to WiFi
 // Wait until we connected to WiFi
        if (!wifi_connected) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        if (!tb.connected()) {
            tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT);
             
        }


       // GENERAR una vez y usar múltiples veces

if (!subscribed) {
            const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
              // Requires additional memory in the JsonDocument for the JsonDocument that will be copied into the response
              RPC_Callback{ RPC_JSON_METHOD,           processGetJson },
              // Requires additional memory in the JsonDocument for 5 key-value pairs that do not copy their value into the JsonDocument itself
              //RPC_Callback{ RPC_TEMPERATURE_METHOD,    processTemperatureChange },
               // Internal size can be 0, because if we use the JsonDocument as a JsonVariant and then set the value we do not require additional memory
              RPC_Callback{ RPC_SWITCH_METHOD,         processSwitchChange },

              RPC_Callback{ RPC_SWITCH_DOS_METHOD,         processSwitchChange2 }
             
            };
            // Perform a subscription. All consequent data processing will happen in
            // processTemperatureChange() and processSwitchChange() functions,
            // as denoted by callbacks array.
            subscribed = rpc.RPC_Subscribe(callbacks.begin(), callbacks.end());
                }
    sensor_read();
	if (s_temperature < 100.0)//para evitar falsas lecturas
		{
        printf("Temperatura exterior : %0.1fC\n", s_temperature);
		}
    float temperature;
    esp_err_t res = max31865_measure(&dev, &temperature);
    printf("Temperatura interior : %0.2fC\n", temperature);
                          

// ENVIAR los valores
tb.sendTelemetryData(TEMPERATURE_KEY, s_temperature);
tb.sendTelemetryData(HUMIDITY_KEY, temperature);

tb.loop();

// IMPRIMIR los mismos valores que enviaste
printf("Temp: %f, Hum: %f\n", s_temperature, temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
