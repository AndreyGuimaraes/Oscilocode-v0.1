#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ADS1015_WE.h>
#include <Definitions.h>

// HTTP and Websocket objects
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// CALLBACKS
void onHomepageRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

void onChartlibRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/chart.js", "text/javascript");
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_BIN:  //Only ESP-TO-ESP communication sends BINARIES in this implementation
    handleWebSocketMessage(payload, length);
    break;
  case WStype_TEXT:

  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED:
    Serial.printf("[%u] Connected\n", num);
    if(currentState == AWAITING_CONNECTION){
      main_client_id = num;
      currentState = ESP_SERVER_MODE;
    }
    else if (currentState == ESP_SERVER_PAIRED){
      client_id[client_counter] = num;
      
      currentState = ESP_SERVER_MODE;
      client_counter++;
    }
    break;
  default:
    break;
  }
}

// PROTOTYPES
void ads_read_values();
void ADC_Loop(void *pvParameters);
void handleWebSocketMessage(uint8_t *payload, size_t length);

void setup()
{
  // Begin Serial communication
  Serial.begin(115200);
  // Guarantee SYNC_PIN mode
  pinMode(SYNC_PIN, INPUT);

  // Configure ESP32 as an Access Point (AP)

  WiFi.softAP(OSCILOBOY_NAME);

  // Get the IP address of the AP
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  delay(500);

  // Begin HTML Server and callbacks
  server.on("/", HTTP_GET, onHomepageRequest);
  server.on("/chart.js", HTTP_GET, onChartlibRequest);
  server.begin();

  // Begin Websocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Begin ESP_NOW connection
  esp_now_init();

  // ADC initialization
  AdcI2C.begin(ADC_I2C_SDA_PIN, ADC_I2C_SCL_PIN, ADC_I2C_SPEED);

  if (!ads_voltage.init(true)) // true is necessary to configure lib for ADS1015 instead of ADS1115
  {
    Serial.println("ADS VOLTAGE initialization - FAILURE");
    while (1);
  } else {
    Serial.println("ADS VOLTAGE initialization - SUCCESS");
  }

  if (!ads_current.init(true))
  {
    Serial.println("ADS CURRENT initialization - FAILURE");
    while (1);
  } else {
    Serial.println("ADS CURRENT initialization - SUCCESS");
  }

  // Voltage reading initial config
  ads_voltage.setCompareChannels(ADS1015_COMP_0_1);
  ads_voltage.setConvRate(ADS1015_3300_SPS);
  ads_voltage.setMeasureMode(ADS1015_CONTINUOUS);
  ads_voltage.setVoltageRange_mV(ADS1015_RANGE_0256);

  // Current reading initial config
  ads_current.setCompareChannels(ADS1015_COMP_0_1);
  ads_current.setConvRate(ADS1015_3300_SPS);
  ads_current.setMeasureMode(ADS1015_CONTINUOUS);
  ads_current.setVoltageRange_mV(ADS1015_RANGE_0256);

  //Create a new task to handle ADC readings in a separated core
  xTaskCreatePinnedToCore(ADC_Loop, "ADC_Handler", 10000, NULL, 1, &ADC_handler, 0);
}

//Runs on Core_0
void ADC_Loop(void *pvParameters)
{
  while (true)
  {
    switch (currentState)
    {
    case ESP_SERVER_MODE:

      break;
    
    default:
      break;
    }
    
    vTaskDelay(1);
  }
}

//Runs on Core_1
void loop()
{
  webSocket.loop();
}

void ads_send_sync(){
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  delayMicroseconds(300);
  digitalWrite(SYNC_PIN, HIGH);
}
// FUNCTIONS
void ads_read_values()
{
  int16_t count = 0;

  int64_t read_start_time = esp_timer_get_time();

  while ((esp_timer_get_time() - read_start_time) < SAMPLING_TIME_US)
  {
    int16_t voltage_results = 0;
    int16_t current_results = 0;
    for (int i = 0; i < oversample_amount; i++)
    {
      // Don't need  to take into account the alert pin, it will only get the previous value
      delayMicroseconds(SAMPLE_DELAY_US);
      voltage_results += ads_voltage.getResultWithRange(-2048, 2048);
      current_results += ads_current.getResultWithRange(-2048, 2048);
    }
    voltage_results = voltage_results / oversample_amount;
    current_results = current_results / oversample_amount;

    localData.voltage_value[count] = voltage_results;
    localData.voltage_time[count] = esp_timer_get_time();
    localData.current_value[count] = current_results;
    localData.current_time[count] = esp_timer_get_time();
    count++;
  }
}

void handleWebSocketMessage(uint8_t *payload, size_t length)
{
  if (length == sizeof(receivedData1))
  {
    memcpy(&receivedData1, payload, length);
    // Process the complete data
    Serial.printf("Received data: %d\n", receivedData1.id);
  }
  else
  {
    Serial.println("Received data size mismatch");
  }
}