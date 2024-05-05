#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1015 ads; /* Use this for the 12-bit version */
float pre_multiplier = 1.0F;
float multiplier = 99.8336F;
int32_t results = 0;
int32_t oversample_amount = 5;
const int vector_size = 3300;
float output_array[3300];

void setup(void)
{
  Serial.begin(115200);
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain  +/- 6.144V  1 bit = 3mV
  //  ads.setGain(GAIN_ONE);           // 1x gain    +/- 4.096V  1 bit = 2mV
  ads.setGain(GAIN_TWO); // 2x gain    +/- 2.048V  1 bit = 1mV
  //   ads.setGain(GAIN_FOUR);       // 4x gain    +/- 1.024V  1 bit = 0.5mV
  //   ads.setGain(GAIN_EIGHT);      // 8x gain    +/- 0.512V  1 bit = 0.25mV
  //   ads.setGain(GAIN_SIXTEEN);    // 16x gain   +/- 0.256V  1 bit = 0.125mV
  ads.setDataRate(RATE_ADS1015_3300SPS);
  Wire.setPins(19, 18);
  if (!ads.begin(0x48))
  {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void loop(void)
{
  //  results = 0;
  //  for (int i=0; i<oversample_amount; i++){
  //      results = results + ads.readADC_Differential_0_1();
  //    }
  //    results = results / oversample_amount;
  //
  //    Serial.println(results*multiplier);

  for (int j = 0; j < vector_size; j++)
  {
    results = 0.0F;
    for (int i = 0; i < oversample_amount; i++)
    {
      results = results + ads.readADC_Differential_0_1();
    }
    results = results / oversample_amount;
    output_array[j] = results;
  }

  for (int j = 0; j < vector_size; j++)
  {
    Serial.print(output_array[j] * multiplier);
    Serial.print(";");
  }
}