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

bool adcType = 0;

// Voltage and current flags when data is ready to be read
void IRAM_ATTR onVoltageDrdy()
{
  voltage_data_ready = true;
}
void IRAM_ATTR onCurrentDrdy()
{
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

//Since there is a need to change the 
void change_voltage_range(ADS1015_WE *ads, ADS1115_RANGE range)
{
  ads->setVoltageRange_mV(range);
  ads->setAlertPinToConversionReady();
}

// Websocket event handler
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_TEXT:
    // Convert payload to a string and print it
    if (!strcmp("25V", (char *)payload))
    {
      change_voltage_range(&ads_voltage, ADS1015_RANGE_0256);
      debug("Voltage 25V\n");
    }
    else if (!strcmp("50V", (char *)payload))
    {
      change_voltage_range(&ads_voltage, ADS1015_RANGE_0512);
      debug("Voltage 50V\n");
    }
    else if (!strcmp("100V", (char *)payload))
    {
      change_voltage_range(&ads_voltage, ADS1015_RANGE_1024);
      debug("Voltage 100V\n");
    }
    else if (!strcmp("200V", (char *)payload))
    {
      change_voltage_range(&ads_voltage, ADS1015_RANGE_2048);
      debug("Voltage 200V\n");
    }
    else if (!strcmp("250V", (char *)payload))
    {
      change_voltage_range(&ads_voltage, ADS1015_RANGE_4096);
      debug("Voltage 250V\n");
    }
    else
    {
      // verify other cases
    }
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

// Reset chart struct data
void resetChartData(Chart_data *data)
{
  data->chart_id = 0;
  data->voltage_gain = 0;
  data->current_type = 0;
  memset(data->voltage_value, 0, sizeof(data->voltage_value));
  memset(data->current_value, 0, sizeof(data->current_value));
  memset(data->voltage_time, 0, sizeof(data->voltage_time));
  memset(data->current_time, 0, sizeof(data->current_time));
}

//Buzzer Setup and Control functions
void buzzer_setup(){
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
}

void play_buzzer(bool shouldPlay){
  if (shouldPlay) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// Adc Reading function
void ads_read_values(Chart_data *chart)
{
  uint16_t voltage_count = 0;
  uint16_t current_count = 0;
  int64_t read_start_time = 0;

  // Guarantee that all data is 0 before populating
  resetChartData(chart);

  // Set the "zero" time reading
  read_start_time = esp_timer_get_time();

  // Make all the possible readings considering the maximum time given
  while (SAMPLING_TIME_US > (esp_timer_get_time() - read_start_time))
  {
    if (voltage_data_ready)
    { // voltage and current data ready are given by an interrupt
      voltage_data_ready = false;
      chart->voltage_value[voltage_count] = ads_voltage.getResultWithRange(-2048, 2048);
      chart->voltage_time[voltage_count] = esp_timer_get_time() - read_start_time;
      voltage_count++;
    }

    if (current_data_ready)
    {
      current_data_ready = false;
      chart->current_value[current_count] = ads_current.getResultWithRange(-2048, 2048);
      chart->current_time[current_count] = esp_timer_get_time() - read_start_time;
      current_count++;

      //After current reading triggers the buzzer if value is close to 2048 ou -2048 to indicate current overflow
      if ((chart->current_value[current_count] >= 2045) || (chart->current_value[current_count] <= -2045)){
        play_buzzer(true);
      }else{
        play_buzzer(false);
      }
    }
  }
}

// Test only function
void fill_data(Chart_data *chart, int chart_id)
{
  chart->chart_id = chart_id;
  if (chart_id == 2)
  {
    chart->voltage_gain = 2.5;
    chart->current_type = 1;

    float frequency = 60.0;
    float sample_rate = VECTOR_SIZE / (5 / frequency); // 5 ciclos completos
    float phase_offset = 30.0 * PI / 180.0;            // 30 graus em radianos

    // Random initial offset
    float random_offset = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * PI;

    for (int i = 0; i < VECTOR_SIZE; i++)
    {
      float time = i / sample_rate;
      chart->voltage_value[i] = 100 * sin(2 * PI * frequency * time + random_offset);
      chart->current_value[i] = 20 * sin(2 * PI * frequency * time + random_offset + phase_offset);
      chart->voltage_time[i] = 2000 * i; // Tempo de tensão
      chart->current_time[i] = 2000 * i; // Tempo de corrente
    }
  }
  else if (chart_id == 1)
  {
    chart->voltage_gain = 5.0;
    chart->current_type = 2;

    float frequency = 50.0;
    float sample_rate = VECTOR_SIZE / (10 / frequency); // 10 ciclos completos
    float phase_offset = 45.0 * PI / 180.0;             // 45 graus em radianos

    // Different random initial offset
    float random_offset = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * PI;

    for (int i = 0; i < VECTOR_SIZE; i++)
    {
      float time = i / sample_rate;
      chart->voltage_value[i] = 150 * sin(2 * PI * frequency * time + random_offset);
      chart->current_value[i] = 25 * sin(2 * PI * frequency * time + random_offset + phase_offset);
      chart->voltage_time[i] = 1500 * i; // Tempo de tensão
      chart->current_time[i] = 1500 * i; // Tempo de corrente
    }
  }
}

// Sends data to the server
// Converts the readings into a Json file and then send via WebSocket
void send_data_to_MC(Chart_data charts[], int num_charts)
{
  DynamicJsonDocument doc(JSON_GRAPH_SIZE * num_charts); // Adjust size as needed based on the number of charts and data size
  JsonArray chartArray = doc.to<JsonArray>();

  for (int j = 0; j < num_charts; j++)
  {
    Chart_data *chart = &charts[j];

    JsonObject chartObject = chartArray.createNestedObject();
    chartObject["chart_id"] = chart->chart_id;
    chartObject["voltage_gain"] = chart->voltage_gain;
    chartObject["current_type"] = chart->current_type;

    JsonArray voltageArray = chartObject["voltage_value"].to<JsonArray>();
    JsonArray currentArray = chartObject["current_value"].to<JsonArray>();
    JsonArray voltageTimeArray = chartObject["voltage_time"].to<JsonArray>();
    JsonArray currentTimeArray = chartObject["current_time"].to<JsonArray>();

    for (int i = 0; i < VECTOR_SIZE; i++)
    {
      voltageArray.add(chart->voltage_value[i]);
      currentArray.add(chart->current_value[i]);
      voltageTimeArray.add(chart->voltage_time[i]);
      currentTimeArray.add(chart->current_time[i]);
    }
  }

  size_t jsonSize = measureJson(doc) + 1; // +1 for null terminator
  char *jsonString = (char *)malloc(jsonSize);

  if (jsonString != nullptr)
  {
    // Serialize JSON and send it
    size_t n = serializeJson(doc, jsonString, jsonSize);
    jsonString[n] = '\0';
    // WHEN MODULARITY IS IMPLEMENTED, THE PEERS NEED TO ADDRESS THE BROADCAST WAY
    // webSocket.sendTXT(main_client_id, jsonString);
    webSocket.broadcastTXT(jsonString);
    free(jsonString); // Free allocated memory after sending
  }
  else
  {
    Serial.println("Falha ao alocar memória para jsonString");
  }
}

// Function to startup ADC voltage and current in default configurations
void ads_initial_config(ADS1015_WE *ads)
{
  if (!ads->init(true)) // true is necessary to configure lib for ADS1015 instead of ADS1115
  {
    if (adcType == 0)
    Serial.println("Voltage ADS initialization - FAILURE");
    else
    Serial.println("Current ADS initialization - FAILURE");
    adcType = 1;
  }
  else
  {
    if (adcType == 0)
    Serial.println("Voltage ADS initialization - SUCCESS");
    else
    Serial.println("Current ADS initialization - SUCCESS");
    adcType = 1;
  }

  ads->setCompareChannels(ADS1015_COMP_0_1);
  ads->setConvRate(ADS1015_3300_SPS);
  ads->setMeasureMode(ADS1015_CONTINUOUS);
  ads->setAlertPinMode(ADS1015_ASSERT_AFTER_1);
  ads->setAlertLatch(ADS1015_LATCH_ENABLED); // this prevents the code from being always interrupted when the adc gets another reading
  ads->setAlertPinToConversionReady();
}

// Startup SPIFFS, Wifi AP, Websockets and - in the future - ESP_NOW
void network_initialize()
{
  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

  // Configure ESP32 as an Access Point (AP)
  WiFi.softAP(OSCILOBOY_NAME);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Set websocket callbacks
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  // Set http callbacks for homepage and chart.js lib
  server.on("/", HTTP_GET, onHomepageRequest);
  server.on("/chart.js", HTTP_GET, onChartlibRequest);
  server.begin();
}


// +++++++++++++++++++++++++ SETUP SETUP SETUP SETUP SETUP SETUP SETUP SETUP ++++++++++++++++++++++++
void setup()
{
  buzzer_setup();
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize all network communication
  network_initialize();

  // ------------------ ADC initialization ----------------------
  AdcI2C.begin(ADC_I2C_SDA_PIN, ADC_I2C_SCL_PIN, ADC_I2C_SPEED);

  // Set ESP32 pins to interrupt with conversion ready function of the adc
  pinMode(VOLTAGE_DRDY_PIN, INPUT_PULLUP);
  pinMode(CURRENT_DRDY_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VOLTAGE_DRDY_PIN), onVoltageDrdy, FALLING);
  attachInterrupt(digitalPinToInterrupt(CURRENT_DRDY_PIN), onCurrentDrdy, FALLING);
  
  // Voltage and current startup and initial config
  ads_initial_config(&ads_voltage);
  delay(300);
  ads_initial_config(&ads_current);


  // -------- Voltage Ranges _initial_ Definition --------------
  change_voltage_range(&ads_voltage, ADS1015_RANGE_4096);
  // change_voltage_range(&ads_voltage, ADS1015_RANGE_2048);
  // change_voltage_range(&ads_voltage, ADS1015_RANGE_1024);
  // change_voltage_range(&ads_voltage, ADS1015_RANGE_0512);
  // change_voltage_range(&ads_voltage, ADS1015_RANGE_0256);

  // -------- Current Ranges Definition (always remains the smallest one) -------------
  // change_voltage_range(&ads_current, ADS1015_RANGE_4096);
  // change_voltage_range(&ads_current, ADS1015_RANGE_2048);
  // change_voltage_range(&ads_current, ADS1015_RANGE_1024);
  // change_voltage_range(&ads_current, ADS1015_RANGE_0512);
  change_voltage_range(&ads_current, ADS1015_RANGE_0256);

  // Pin definitions and interrupts for ADC
  Serial.println("Setup Finalizado");
  delay(500);
}

// +++++++++++++++++++++++++ LOOP LOOP LOOP LOOP LOOP LOOP LOOP LOOP ++++++++++++++++++++++++
void loop()
{
  //--- Test only functions
  // fill_data(&charts[0], 1);
  // fill_data(&charts[1], 2);
  //--- Test only functions
  ads_read_values(&charts[0]);
  send_data_to_MC(charts, 1);
  webSocket.loop();
}