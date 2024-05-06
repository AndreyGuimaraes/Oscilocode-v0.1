#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

// ADS1015 I2C parameters 
#define ADS_I2C_SDA 19
#define ADS_I2C_SCL 18
#define ADS_I2C_SPEED 400000
#define ADS_VOLTAGE_ADDR 0x48
#define ADS_CURRENT_ADDR 0x49

// ADS1015 data sampling and handling 
#define VECTOR_SIZE 130

Adafruit_ADS1015 ads_voltage;
Adafruit_ADS1015 ads_current;
TwoWire ADSwire = TwoWire(0);
int16_t oversample_amount = 1;
int16_t voltage_results = 0;
int32_t current_results = 0;

typedef struct GraphData
{
  int16_t output_array[VECTOR_SIZE];
  int64_t sample_time[VECTOR_SIZE]; //time in microseconds
} GraphData;

GraphData current_graph, voltage_graph;

float voltage_multiplier = 1.0F;
float current_multiplier = 1.0F;

// ESP-to-ESP communication
// TwoWire ESPWire = TwoWire(1);

void setup(void)
{
  /* -------------- ADS1015 GAIN REFERENCES ----------------------
    ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain  +/- 6.144V  1 bit = 3mV
    ads.setGain(GAIN_ONE);        // 1x gain    +/- 4.096V  1 bit = 2mV
    ads.setGain(GAIN_TWO);        // 2x gain    +/- 2.048V  1 bit = 1mV
    ads.setGain(GAIN_FOUR);       // 4x gain    +/- 1.024V  1 bit = 0.5mV
    ads.setGain(GAIN_EIGHT);      // 8x gain    +/- 0.512V  1 bit = 0.25mV
    ads.setGain(GAIN_SIXTEEN);    // 16x gain   +/- 0.256V  1 bit = 0.125mV
  */

  Serial.begin(115200);

  // ADS1015 SETUP
  ADSwire.setPins(ADS_I2C_SDA, ADS_I2C_SCL);
  ADSwire.setClock(ADS_I2C_SPEED);
  ads_voltage.setDataRate(RATE_ADS1015_3300SPS);
  ads_current.setDataRate(RATE_ADS1015_3300SPS);
  ads_voltage.setGain(GAIN_SIXTEEN);
  ads_current.setGain(GAIN_SIXTEEN);

  if (!ads_voltage.begin(ADS_VOLTAGE_ADDR, &ADSwire))
  {
    Serial.println("ADS VOLTAGE initialization - FAILURE");
    while (1);
  } else {
    Serial.println("ADS VOLTAGE initialization - SUCCESS");
  }

  if (!ads_current.begin(ADS_CURRENT_ADDR, &ADSwire))
  {
    Serial.println("ADS VOLTAGE initialization - FAILURE");
    while (1);
  } else {
    Serial.println("ADS CURRENT initialization - SUCCESS");
  }

  // ESP-TO-ESP SETUP
  /* -----------------------------
  --------------------------------
  */
}

void loop(void)
{
  // Initialize voltage and current reading
  Serial.print("VOLTAGE + CURRENT RAW READING START ");
  Serial.print("OVERSAMPLE = ");
  Serial.println(oversample_amount);
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    voltage_results = 0;
    current_results = 0;
    for (int i = 0; i < oversample_amount; i++)
    {
      voltage_results = voltage_results + ads_voltage.readADC_Differential_0_1();
      current_results = current_results + ads_current.readADC_Differential_0_1();
    }
    voltage_results = voltage_results / oversample_amount;
    current_results = current_results / oversample_amount;

    voltage_graph.output_array[j] = voltage_results;
    voltage_graph.sample_time[j] = esp_timer_get_time();
    current_graph.output_array[j] = current_results;
    current_graph.sample_time[j] = esp_timer_get_time();
  }
  Serial.println("VOLTAGE + CURRENT RAW READING FINISH ");
  Serial.print("OVERSAMPLE = ");
  Serial.println(oversample_amount);

  // Print Voltage and Current values to serial
  Serial.println("VOLTAGE READING PRINTING...");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    Serial.print(voltage_graph.output_array[j] * voltage_multiplier);
    Serial.print(";");
    Serial.print(voltage_graph.sample_time[j]);
  }
  Serial.println(" ");
  Serial.println("CURRENT READING PRINTING...");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    Serial.print(current_graph.output_array[j] * current_multiplier);
    Serial.print(";");
    Serial.print(current_graph.sample_time[j]);
  }
  Serial.println(" ");
  
}