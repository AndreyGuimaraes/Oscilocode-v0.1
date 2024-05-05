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
#define VECTOR_SIZE 300

Adafruit_ADS1015 ads_voltage;
Adafruit_ADS1015 ads_current;
TwoWire ADSwire = TwoWire(0);
int32_t oversample_amount = 5;
int32_t voltage_output_array[VECTOR_SIZE];
int32_t current_output_array[VECTOR_SIZE];
int32_t voltage_results = 0;
int32_t current_results = 0;

float voltage_multiplier = 99.8336F;
float current_multiplier = 100.0F;

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

  if (!ads_voltage.begin(ADS_VOLTAGE_ADDR, &ADSwire))
  {
    Serial.println("ADS CURRENT initialization - FAILURE");
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
}

void loop(void)
{
  // Initialize voltage reading
  Serial.println("VOLTAGE READING START");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    voltage_results = 0;
    for (int i = 0; i < oversample_amount; i++)
    {
      voltage_results = voltage_results + ads_voltage.readADC_Differential_0_1();
    }
    voltage_results = voltage_results / oversample_amount;
    voltage_output_array[j] = voltage_results;
  }
  Serial.println("VOLTAGE READING FINISH");

  // Initialize current reading
  Serial.println("CURRENT READING START");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    current_results = 0;
    for (int i = 0; i < oversample_amount; i++)
    {
      current_results = current_results + ads_current.readADC_Differential_0_1();
    }
    current_results = current_results / oversample_amount;
    current_output_array[j] = current_results;
  }
  Serial.println("CURRENT READING FINISH");

  // Print Voltage and Current values to serial
  Serial.println("VOLTAGE READING PRINT");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    Serial.print(voltage_output_array[j] * voltage_multiplier);
    Serial.print(";");
  }
  Serial.println(" ");
  
  Serial.println("CURRENT READING PRINT");
  for (int j = 0; j < VECTOR_SIZE; j++)
  {
    Serial.print(current_output_array[j] * current_multiplier);
    Serial.print(";");
  }
}