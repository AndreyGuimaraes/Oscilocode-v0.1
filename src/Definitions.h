#include <ADS1015_WE.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

//Debug Mode on or off
#define DEBUG
#ifdef DEBUG
#define debug(x) Serial.printf(x)
#else
#define debug(x)
#endif

//WIFI name
#define OSCILOBOY_NAME "Osciloboy-1"

// Main definitions for ADS1015
#define ADC_I2C_SDA_PIN 19
#define ADC_I2C_SCL_PIN 18
#define VOLTAGE_I2C_ADDR 0x48
#define CURRENT_I2C_ADDR 0x49
#define ADC_I2C_SPEED 800000U //800 kHz I2C
#define SAMPLING_TIME_US 85000 //Sample window size
#define VECTOR_SIZE 300 //Minimum vector size to fit each ADC data eg.: VECTOR_SIZE = 300 -> 300 voltage points, 300 current points.
#define SYNC_PIN 17
#define VOLTAGE_DRDY_PIN 23
#define CURRENT_DRDY_PIN 17

// Main variables for ADS1015
TwoWire AdcI2C = TwoWire(0);
ADS1015_WE ads_voltage = ADS1015_WE(&AdcI2C, VOLTAGE_I2C_ADDR);
ADS1015_WE ads_current = ADS1015_WE(&AdcI2C, CURRENT_I2C_ADDR);
float voltage_multiplier = 108.0F;
float current_multiplier = 1.0F;
bool voltage_data_ready = 0;
bool current_data_ready = 0;

//Wifi SSID variable
char ssid_name[63];
//Task handler for dual-core operation
TaskHandle_t ADC_handler;

//Client IDs for websocket connections
uint8_t main_client_id = 0;
uint8_t client_id[2];
uint8_t client_counter = 0;
uint8_t counter_support = 0;

//Websocket binary message
typedef struct
{
  int8_t chart_id=0;
  int8_t voltage_gain = 16;
  int8_t current_type = 16;
  int16_t voltage_value[VECTOR_SIZE];
  int16_t current_value[VECTOR_SIZE];
  int64_t voltage_time[VECTOR_SIZE];
  int64_t current_time[VECTOR_SIZE];
} Chart_data;
Chart_data localData, receivedData1, receivedData2;

// Message that comes from webserver (change voltage gain current type)
typedef struct
{
  int8_t voltage_gain;
  int8_t current_type;
} MC_income_message;

MC_income_message mcMessage;

// ESP-NOW variables
esp_now_peer_info_t peerInfo;
typedef struct {
  int8_t chart_id = 1;
  char ssid[63];
} esp_now_struct;
esp_now_struct esp_now_message;

// STATE MACHINE MAPPING and helper variables
enum State
{
  // Start State
  AWAITING_CONNECTION,
  // Server Mode
  ESP_SERVER_MODE,
  ESP_SERVER_PAIRING,
  ESP_SERVER_PAIRED,
  ESP_SERVER_SAMPLING,
  ESP_SERVER_SENDING,
  ESP_SERVER_RECEIVING_CLIENTS,
  // Client Mode
  ESP_CLIENT_MODE,
  ESP_CLIENT_AWAITING_SYNC,
  ESP_CLIENT_SAMPLING,
  ESP_CLIENT_SENDING,
};

State currentState = AWAITING_CONNECTION;
State lastState = AWAITING_CONNECTION;
bool firstRun = true;