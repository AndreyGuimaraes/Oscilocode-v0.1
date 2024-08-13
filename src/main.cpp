#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
#include <ADS1015_WE.h>
#include <Definitions.h>
#include <ArduinoJson.h>

// HTTP and Websocket objects
AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
WebSocketsClient client_ws;

//Json helper object
StaticJsonDocument<200> doc_aux;

// PROTOTYPES
void ads_read_values(Chart_data *chart);
void ads_send_sync();
void ADC_Loop(void *pvParameters);
void macStringToBytes(const char *macStr, uint8_t *macBytes);
void send_data_to_MC(Chart_data *chart);
void resetChartData(Chart_data* data);

// CALLBACKS
void onEspNowDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&esp_now_message, incomingData, sizeof(esp_now_message));
  if (firstRun){
    debug("ESP CLIENT MODE\n");
    currentState = ESP_CLIENT_MODE;
    firstRun = false;
  }
  else
  currentState = ESP_CLIENT_AWAITING_SYNC;
  debug("ESP CLIENT AWAITING SYNC\n");
}

void onHomepageRequest(AsyncWebServerRequest *request) {
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                  "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/test.html", "text/html");
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
    if (currentState == ESP_SERVER_RECEIVING_CLIENTS){
      // Uses counter support to count how many clients are still pending sending data
      if (counter_support == 0){
        counter_support = client_counter;
        currentState = ESP_SERVER_SENDING;
        debug("ESP MODE SERVER SENDING\n");
      }
      else if (client_id[0] == num){
        memcpy(&receivedData1, payload, length);
        counter_support--;
      } else if (client_id[1] == num){
        memcpy(&receivedData2, payload, length);
        counter_support--;
      }
    }
    break;
  case WStype_TEXT:
    // Always receive JSON 
    deserializeJson(doc_aux, payload, length);

      //If it is a new channel, receive mac address from browser
      if (doc_aux.containsKey("mac")) {
        //Converts the JSON mac to uint8_t mac
        const char *macStr = doc_aux["mac"];
        uint8_t macBytes[6];
        macStringToBytes(macStr, macBytes);

        //Populate ESP-NOW message
        esp_now_message.chart_id = client_counter + 1;
        strcpy(esp_now_message.ssid, ssid_name);

        //Define peer information
        memcpy(peerInfo.peer_addr, macBytes, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        esp_now_add_peer(&peerInfo);
        esp_err_t result = esp_now_send(macBytes, (uint8_t *) &esp_now_message, sizeof(esp_now_message));

        if (result == ESP_OK){
          currentState = ESP_SERVER_PAIRED;
          debug("ESP MODE SERVER PAIRED\n");
        } else {
          webSocket.sendTXT(num, "MAC NOK");
          currentState = ESP_SERVER_MODE;
          debug("ESP MODE SERVER MODE AFTER SERVER NOK\n");
        }
      }
      else if (doc_aux.containsKey("start")){
        currentState = ESP_SERVER_SAMPLING;
        debug("ESP MODE SERVER SAMPLING\n");
      }
      // else if (doc.containsKey())

    doc_aux.clear();
    break;

  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;

  case WStype_CONNECTED:
    Serial.printf("[%u] Connected\n", num);
    if(currentState == AWAITING_CONNECTION){ //First connection possible is with cellphone
      main_client_id = num;
      currentState = ESP_SERVER_SAMPLING;
      debug("ESP MODE SERVER MODE\n");
    }
    else if (currentState == ESP_SERVER_PAIRED){
      client_id[client_counter] = num;
      webSocket.sendTXT(main_client_id, "MAC OK");
      currentState = ESP_SERVER_MODE;
      client_counter++;
      counter_support = client_counter;
    }
    break;
  default:
    break;
  }
}

void IRAM_ATTR onVoltageDrdy(){
  voltage_data_ready = true;
}
void IRAM_ATTR onCurrentDrdy(){
  current_data_ready = true;
}

void setup()
{
  // Begin Serial communication
  Serial.begin(115200);
  // Guarantee SYNC_PIN mode
  pinMode(SYNC_PIN, INPUT);

  //Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

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
  esp_now_register_recv_cb(onEspNowDataRecv);
 
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
  ads_voltage.setAlertPinMode(ADS1015_ASSERT_AFTER_1);
  ads_voltage.setMeasureMode(ADS1015_CONTINUOUS);
  ads_voltage.setVoltageRange_mV(ADS1015_RANGE_4096);

  // Current reading initial config
  ads_current.setCompareChannels(ADS1015_COMP_0_1);
  ads_current.setConvRate(ADS1015_3300_SPS);
  ads_current.setMeasureMode(ADS1015_CONTINUOUS);
  ads_current.setVoltageRange_mV(ADS1015_RANGE_0512);

  //Create a new task to handle ADC readings in a separated core
  xTaskCreatePinnedToCore(ADC_Loop, "ADC_Handler", 10000, NULL, 1, &ADC_handler, 0);
}

//Runs on Core_0
void ADC_Loop(void *pvParameters)
{
  attachInterrupt(digitalPinToInterrupt(VOLTAGE_DRDY_PIN), onVoltageDrdy, FALLING);
  attachInterrupt(digitalPinToInterrupt(CURRENT_DRDY_PIN), onCurrentDrdy, FALLING);

  while (true)
  {
    switch (currentState)
    {
    case ESP_SERVER_SAMPLING:
        ads_send_sync();
        // currentState = ESP_SERVER_RECEIVING_CLIENTS;
        // debug("ESP MODE SERVER RECEIVING CLIENTS\n");
        ads_read_values(&localData);
        send_data_to_MC(&localData);
      break;
    
    case ESP_CLIENT_SAMPLING:
        ads_read_values(&localData);
        currentState = ESP_CLIENT_SENDING;
        debug("ESP MODE CLIENT SENDING\n");

    default:
      break;
    }
    vTaskDelay(1);
  }
}

//Runs on Core_1
void loop()
{
  switch (currentState)
  {
  case ESP_CLIENT_MODE:
    webSocket.close();
    WiFi.softAPdisconnect(true);
    WiFi.begin(esp_now_message.ssid);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting Wifi");
    }
    client_ws.begin("192.168.4.1", 81, "/");
    currentState = ESP_CLIENT_AWAITING_SYNC;
    break;

  case ESP_CLIENT_AWAITING_SYNC:
    client_ws.loop();
    break;
  
  case ESP_CLIENT_SAMPLING:
    client_ws.loop();
    break;
  
  case ESP_CLIENT_SENDING:
    
    break;

  case ESP_SERVER_SENDING:
      send_data_to_MC(&localData);
      if (client_counter == 2){
        send_data_to_MC(&receivedData2);
      }
      if (client_counter == 1){
        send_data_to_MC(&receivedData1);
      }
    break;
  
  default:
    webSocket.loop();
    break;
  }
}

//Functions
void macStringToBytes(const char *macStr, uint8_t *macBytes) {
    sscanf(macStr, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
           &macBytes[0], &macBytes[1], &macBytes[2],
           &macBytes[3], &macBytes[4], &macBytes[5]);
}

void ads_send_sync(){
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);
  delayMicroseconds(100);
  digitalWrite(SYNC_PIN, HIGH);
}

void ads_read_values(Chart_data *chart)
{
  uint8_t voltage_count = 0;
  uint8_t current_count = 0;

  //Guarantee that all data is 0 before populating
  resetChartData(chart);

  int64_t read_start_time = esp_timer_get_time();

  while ((esp_timer_get_time() - read_start_time) < SAMPLING_TIME_US)
  {
    
    if (voltage_data_ready){
    voltage_data_ready = false;
    chart->voltage_value[voltage_count] = ads_voltage.getResultWithRange(-2048, 2048);
    chart->voltage_time[voltage_count] = esp_timer_get_time() - read_start_time;
    }

    if (current_data_ready){
    current_data_ready = false;
    chart->current_time[current_count] = ads_current.getResultWithRange(-2048, 2048);
    chart->current_time[current_count] = esp_timer_get_time() - read_start_time;
    }
  }
}

void send_data_to_MC(Chart_data *chart)
{
  StaticJsonDocument<8192> doc;

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
  size_t jsonSize = measureJson(doc) + 1; // +1 para o terminador nulo
  char *jsonString = (char *)malloc(jsonSize);

  if (jsonString != nullptr)
  {
    // Serializa o JSON para a string alocada dinamicamente
    size_t n = serializeJson(doc, jsonString, jsonSize);
    jsonString[n] = '\0';
    webSocket.sendTXT(main_client_id, jsonString);
    free(jsonString); // Libera a memória alocada
  }
  else
  {
    Serial.println("Falha ao alocar memória para jsonString");
  }
}

void resetChartData(Chart_data* data) {
    data->chart_id = 0;
    data->voltage_gain = 0;
    data->current_type = 0;
    memset(data->voltage_value, 0, sizeof(data->voltage_value));
    memset(data->current_value, 0, sizeof(data->current_value));
    memset(data->voltage_time, 0, sizeof(data->voltage_time));
    memset(data->current_time, 0, sizeof(data->current_time));
}