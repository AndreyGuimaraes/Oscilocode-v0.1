#include <Arduino.h>
#include <Definitions.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ADS1015_WE.h>
#include <math.h>
#include <stdlib.h>

// HTTP and Websocket objects
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Json helper object
//  StaticJsonDocument<200> doc;

uint64_t timeNow = 0;

// Voltage and current flags when data is ready to be read
void IRAM_ATTR onVoltageDrdy(){
  voltage_data_ready = true;
}
void IRAM_ATTR onCurrentDrdy(){
  current_data_ready = true;
}

// Http and chart lib callbacks for requests
void onHomepageRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/index.html", "text/html");
}

void onChartlibRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/chart.js", "text/javascript");
}

//Websocket event handler
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_TEXT:
    // Always receive JSON
    // deserializeJson(doc, payload, length);

    // if (doc.containsKey("start"))
    // {
    //   currentState = ESP_SERVER_SAMPLING;
    //   debug("ESP MODE SERVER SAMPLING\n");
    // }
    // // else if (doc.containsKey())

    // doc.clear();
    break;

  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  case WStype_CONNECTED:
    Serial.printf("[%u] Connected\n", num);
    if (currentState == AWAITING_CONNECTION)
    { // First connection possible is with cellphone
      main_client_id = num;
      currentState = ESP_SERVER_MODE;
      debug("ESP MODE SERVER MODE\n");
    }
    break;
  default:
    break;
  }
}

//Reset chart struct data
void resetChartData(Chart_data* data) {
    data->chart_id = 0;
    data->voltage_gain = 0;
    data->current_type = 0;
    memset(data->voltage_value, 0, sizeof(data->voltage_value));
    memset(data->current_value, 0, sizeof(data->current_value));
    memset(data->voltage_time, 0, sizeof(data->voltage_time));
    memset(data->current_time, 0, sizeof(data->current_time));
}

//Adc Reading function
void ads_read_values(Chart_data *chart)
{
  uint16_t voltage_count = 0;
  uint16_t current_count = 0;
  int64_t read_start_time = 0;

  //Guarantee that all data is 0 before populating
  resetChartData(chart);

  //Set the "zero" time reading
  read_start_time = esp_timer_get_time();
  
  // Make all the possible readings considering the maximum time given
  while (SAMPLING_TIME_US > (esp_timer_get_time() - read_start_time))
  {
    if (voltage_data_ready){ //voltage and current data ready are given by an interrupt
    voltage_data_ready = false;
    chart->voltage_value[voltage_count] = ads_voltage.getResultWithRange(-2048, 2048);
    chart->voltage_time[voltage_count] = esp_timer_get_time() - read_start_time;
    voltage_count++;
    }

    if (current_data_ready){
    current_data_ready = false;
    chart->current_value[current_count] = ads_current.getResultWithRange(-2048, 2048);
    chart->current_time[current_count] = esp_timer_get_time() - read_start_time;
    current_count++;
    }
  }
}

// Test function - To be Deleted
// void fill_data(Chart_data *chart)
// {
//   chart->chart_id = 1;
//   chart->voltage_gain = 2.5;
//   chart->current_type = 1;

//   float frequency = 60.0;
//   float sample_rate = VECTOR_SIZE / (5 / frequency); // 5 ciclos completos
//   float phase_offset = 30.0 * PI / 180.0; // 30 graus em radianos

//   // Deslocamento aleatório inicial
//   float random_offset = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2 * PI;

//   for (int i = 0; i < VECTOR_SIZE; i++)
//   {
//     float time = i / sample_rate;
//     chart->voltage_value[i] = 100 * sin(2 * PI * frequency * time + random_offset);
//     chart->current_value[i] = 20 * sin(2 * PI * frequency * time + random_offset + phase_offset);
//     chart->voltage_time[i] = 2000*i; // Tempo de tensão
//     chart->current_time[i] = 2000*i; // Tempo de corrente
//   }
// }

//Function responsible for sending data to the server
//Converts the readings into a Json file and then send via WebSocket
void send_data_to_MC(Chart_data *chart)
{
  StaticJsonDocument<8192> doc; //JSON size to accommodate the worst case of data (Full vectors)

  doc["chart_id"] = chart->chart_id;
  doc["voltage_gain"] = chart->voltage_gain;
  doc["current_type"] = chart->current_type;
  JsonArray voltageArray = doc["voltage_value"].to<JsonArray>();
  JsonArray currentArray = doc["current_value"].to<JsonArray>();
  JsonArray voltageTimeArray = doc["voltage_time"].to<JsonArray>();
  JsonArray currentTimeArray = doc["current_time"].to<JsonArray>();

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    voltageArray.add(chart->voltage_value[i]);
    currentArray.add(chart->current_value[i]);
    voltageTimeArray.add(chart->voltage_time[i]);
    currentTimeArray.add(chart->current_time[i]);
  }
  size_t jsonSize = measureJson(doc) + 1; // +1 for null terminator
  char *jsonString = (char *)malloc(jsonSize);

  if (jsonString != nullptr)
  {
    // Serialize JSON and append the last digit
    size_t n = serializeJson(doc, jsonString, jsonSize);
    jsonString[n] = '\0';
    webSocket.sendTXT(main_client_id, jsonString);
    free(jsonString); // Free allocated memory after sending
  }
  else
  {
    Serial.println("Falha ao alocar memória para jsonString");
  }
}

//Function to startup voltage and current in default configurations
void ads_initial_config(ADS1015_WE *ads){
  if (!ads->init(true)) // true is necessary to configure lib for ADS1015 instead of ADS1115
  {
    Serial.println("ADS initialization - FAILURE");
    while (1);
  } else {
    Serial.println("ADS initialization - SUCCESS");
  }

  ads->setCompareChannels(ADS1015_COMP_0_1);
  ads->setConvRate(ADS1015_3300_SPS);
  ads->setMeasureMode(ADS1015_CONTINUOUS);
  ads->setAlertPinMode(ADS1015_ASSERT_AFTER_1);
  ads->setAlertPinToConversionReady();
  ads->setAlertLatch(ADS1115_LATCH_ENABLED); //this prevents the code from being always interrupted when the adc gets another reading
}

void network_initialize(){
  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

  // Configure ESP32 as an Access Point (AP)
  WiFi.softAP(OSCILOBOY_NAME);

  //Set websocket callbacks
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  //Set http callbacks for homepage and chart.js lib
  server.on("/", HTTP_GET, onHomepageRequest);
  server.on("/chart.js", HTTP_GET, onChartlibRequest);
  server.begin();
}

void setup()
{
  //Initialize serial communication
  Serial.begin(115200);

  //Initialize all network communication
  network_initialize();

// ------------------ ADC initialization ----------------------
  AdcI2C.begin(ADC_I2C_SDA_PIN, ADC_I2C_SCL_PIN, ADC_I2C_SPEED);

// Voltage and current startup and initial config
  ads_initial_config(&ads_voltage);
  ads_initial_config(&ads_current);

  // -------- Voltage Ranges Definition --------------
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_0256);
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_0512);
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_1024);
  ads_voltage.setVoltageRange_mV(ADS1015_RANGE_2048);
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_4096);

  // -------- Current Ranges Definition -------------
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_0256);
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_0512);
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_1024);
  ads_current.setVoltageRange_mV(ADS1015_RANGE_2048);
  // ads_voltage.setVoltageRange_mV(ADS1015_RANGE_4096);

  //Pin definitions and interrupts for ADC
  pinMode(VOLTAGE_DRDY_PIN, INPUT_PULLUP);
  pinMode(CURRENT_DRDY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VOLTAGE_DRDY_PIN), onVoltageDrdy, FALLING);
  attachInterrupt(digitalPinToInterrupt(CURRENT_DRDY_PIN), onCurrentDrdy, FALLING);

  Serial.println("Setup Finalizado");
  delay(500);
}

void loop()
{
  ads_read_values(&localData);
  send_data_to_MC(&localData);
  webSocket.loop();
}

