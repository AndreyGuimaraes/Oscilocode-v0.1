#include <Arduino.h>
#include <Wire.h>
#include <ADS1015_WE.h> 

// ADS1015 I2C parameters 
#define ADS_I2C_SDA 19
#define ADS_I2C_SCL 18
#define ADS_I2C_SPEED 800000U //Use U in the end for Unsigned int, speed in Hz
#define ADS_VOLTAGE_ADDR 0x48
#define ADS_CURRENT_ADDR 0x49

// ADS1015 data sampling and handling 
#define VECTOR_SIZE 130

TwoWire ADSwire = TwoWire(0);
ADS1015_WE ads_voltage = ADS1015_WE(&ADSwire, ADS_VOLTAGE_ADDR);
ADS1015_WE ads_current = ADS1015_WE(&ADSwire, ADS_CURRENT_ADDR);
int16_t oversample_amount = 1;
int16_t voltage_results = 0;
int16_t current_results = 0;
float voltage_bit_resolution = 0.125;
float current_bit_resolution = 0.125;

typedef struct GraphData
{
  float output_array[VECTOR_SIZE];
  int64_t sample_time[VECTOR_SIZE]; //time in microseconds
} GraphData;

GraphData current_graph, voltage_graph;

float voltage_multiplier = 108.0F;
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
  ADSwire.begin(ADS_I2C_SDA, ADS_I2C_SCL, ADS_I2C_SPEED);
  ADSwire.setClock(ADS_I2C_SPEED);

  if (!ads_voltage.init(true)) // true is necessary to configure lib for ADS1015
  {
    Serial.println("ADS VOLTAGE initialization - FAILURE");
    while (1); } else {
    Serial.println("ADS VOLTAGE initialization - SUCCESS");
  }

  if (!ads_current.init(true))
  {
    Serial.println("ADS CURRENT initialization - FAILURE");
    while (1); } else {
    Serial.println("ADS CURRENT initialization - SUCCESS");
  }

  // Voltage reading initial config
  ads_voltage.setCompareChannels(ADS1015_COMP_0_1);
  ads_voltage.setConvRate(ADS1015_3300_SPS);
  ads_voltage.setMeasureMode(ADS1015_CONTINUOUS); 
  ads_voltage.setVoltageRange_mV(ADS1015_RANGE_0256);

  //Current reading initial config
  ads_current.setCompareChannels(ADS1015_COMP_0_1);
  ads_current.setConvRate(ADS1015_3300_SPS);
  ads_current.setMeasureMode(ADS1015_CONTINUOUS); 
  ads_current.setVoltageRange_mV(ADS1015_RANGE_0256);



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
    voltage_results = 0.0;
    current_results = 0.0;
    for (int i = 0; i < oversample_amount; i++)
    {
      // Dont need  to take into account the alert pin, it will only get the previous value
      // voltage_results += ads_voltage.getResult_mV();
      // current_results += ads_current.getResult_mV();
      delayMicroseconds(300);
      voltage_results += ads_voltage.getResultWithRange(-2048, 2048);
      current_results += ads_current.getResultWithRange(-2048, 2048);
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
    Serial.print(voltage_graph.output_array[j] * voltage_multiplier * voltage_bit_resolution);
    Serial.print(" ");
    Serial.print(voltage_graph.sample_time[j]);
    Serial.print(";");
  }
  Serial.println(" ");
  Serial.println("CURRENT READING PRINTING...");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    Serial.print(current_graph.output_array[j] * current_multiplier * current_bit_resolution);
    Serial.print(" ");
    Serial.print(current_graph.sample_time[j]);
    Serial.print(";");
  }
  Serial.println(" ");
  
  while(1);
}