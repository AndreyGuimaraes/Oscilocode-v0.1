#include <Arduino.h>
#include <WiFi.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <Definitions.h>
#include <ArduinoJson.h>
#include <math.h>
#include <stdlib.h>

#define PI 3.14159265

AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Json helper object
//  StaticJsonDocument<200> doc;

uint64_t timeNow = 0;

void onHomepageRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/test.html", "text/html");
}

void onChartlibRequest(AsyncWebServerRequest *request)
{
  IPAddress remote_ip = request->client()->remoteIP();
  Serial.println("[" + remote_ip.toString() +
                 "] HTTP GET request of " + request->url());
  request->send(SPIFFS, "/chart.js", "text/javascript");
}

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

void fill_data(Chart_data *chart)
{
  chart->chart_id = 1;
  chart->voltage_gain = 2.5;
  chart->current_type = 1;

  float frequency = 60.0;
  float sample_rate = VECTOR_SIZE / (5 / frequency); // 5 ciclos completos
  float phase_offset = 30.0 * PI / 180.0; // 30 graus em radianos

  // Deslocamento aleatório inicial
  float random_offset = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) * 2 * PI;

  for (int i = 0; i < VECTOR_SIZE; i++)
  {
    float time = i / sample_rate;
    chart->voltage_value[i] = 100 * sin(2 * PI * frequency * time + random_offset);
    chart->current_value[i] = 20 * sin(2 * PI * frequency * time + random_offset + phase_offset);
    chart->voltage_time[i] = 2*i; // Tempo de tensão
    chart->current_time[i] = 2*i; // Tempo de corrente
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
    // Serialize JSON and append the last 
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

void setup()
{
  Serial.begin(115200);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An error has occurred while mounting SPIFFS");
    return;
  }

  // Configure ESP32 as an Access Point (AP)
  WiFi.softAP(OSCILOBOY_NAME);

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  server.on("/", HTTP_GET, onHomepageRequest);
  server.on("/chart.js", HTTP_GET, onChartlibRequest);
  server.begin();

  fill_data(&localData);
  Serial.println("Setup Finalizado");
}

void loop()
{
  while ((esp_timer_get_time() - timeNow) < 500)
  {
    webSocket.loop();
  }
  if (currentState == ESP_SERVER_MODE)
  {
    timeNow = esp_timer_get_time();
    send_data_to_MC(&localData);
    fill_data(&localData);
  }
  webSocket.loop();
}
