#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>

const char *ssid = "Osciloboy: B4:8A:0A:8D:38:08";
const char *websocket_server = "ws://192.168.4.1"; // Static IP address of the receiver

WebSocketsClient webSocket;
bool sync_status = 0;

#define VECTOR_SIZE 300

typedef struct
{
  int8_t id = 1;
  int8_t voltage_gain = 16;
  int8_t current_gain = 16;
  int16_t voltage_value[VECTOR_SIZE];
  int64_t voltage_time[VECTOR_SIZE];
  int16_t current_value[VECTOR_SIZE];
  int64_t current_time[VECTOR_SIZE];
} Chart_data;

Chart_data data;

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("WebSocket Disconnected");
    break;
  case WStype_CONNECTED:
    Serial.println("WebSocket Connected");
    break;
  case WStype_TEXT:
    Serial.printf("WebSocket message: %s\n", payload);
    break;
  case WStype_BIN:
    Serial.println("WebSocket binary message received");
    break;
  }
}
void IRAM_ATTR set_flag()
{
  sync_status = 1;
}

void setup()
{
  Serial.begin(115200);
  WiFi.begin(ssid);
  pinMode(33, INPUT);
  attachInterrupt(digitalPinToInterrupt(33), set_flag, RISING);
  // Connect to WiFi
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize WebSocket
  webSocket.begin("192.168.4.1", 81, "/"); // Replace with the static IP address of the receiver
  webSocket.onEvent(webSocketEvent);

  // Wait for WebSocket connection
  while (!webSocket.isConnected())
  {
    webSocket.loop();
    delay(100);
  }

  // Initialize arrays
  memset(data.voltage_value, 0, sizeof(data.voltage_value));
  memset(data.voltage_time, 0, sizeof(data.voltage_time));
  memset(data.current_value, 0, sizeof(data.current_value));
  memset(data.current_time, 0, sizeof(data.current_time));

  // Populate some of the data with example values
  for (int i = 0; i < VECTOR_SIZE; i++)
  { // Populate only first 50 elements for demonstration
    data.voltage_value[i] = i;
    data.current_value[i] = i + 1000;
    data.voltage_time[i] = i + 2000;
    data.current_time[i] = i + 3000;
  }
}

void loop()
{
  // Send the struct as a binary message
  if (sync_status)
  {
    webSocket.sendBIN((uint8_t *)&data, sizeof(data));
    sync_status = 0;
  }
  webSocket.loop();
}