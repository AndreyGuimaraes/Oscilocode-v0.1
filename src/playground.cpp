/***************************************************************************
* Example sketch for the ADS1115_WE library
*
* This sketch shows how you can use the alert pin as conversion ready alert pin.
* It works in continuous mode as well as in single shot mode. 
* Please note that you need to enable the alert pin with setAlertPinMode. Choose any 
* parameter except ADS1115_DISABLE_ALERT.
*  
* Further information can be found on:
* https://wolles-elektronikkiste.de/ads1115 (German)
* https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
* 
***************************************************************************/
#include <Arduino.h>
#include <ADS1115_WE.h> 
#include <Wire.h>
#include <WebSocketsClient.h>
#define I2C_ADDRESS 0x48
int interruptPin = 23;
int counter = 0;
volatile bool convReady = false;
bool useADS1015 = true;
typedef struct
{
  int8_t chart_id=0;
  int8_t voltage_gain = 16;
  int8_t current_type = 16;
  int16_t voltage_value[300];
  int16_t current_value[300];
  int64_t time[300];
} Chart_data;
Chart_data localData;

/* There are several ways to create your ADS1115_WE object:
 * ADS1115_WE adc = ADS1115_WE(); -> uses Wire / I2C Address = 0x48
 * ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
 * ADS1115_WE adc = ADS1115_WE(&Wire); -> you can pass any TwoWire object / I2C Address = 0x48
 * ADS1115_WE adc = ADS1115_WE(&Wire, I2C_ADDRESS); -> all together
 */
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
bool loop_once = false;

void setup() {
  Wire.begin(19,18,800000U);
  Serial.begin(115200);
  int i = 0;
  pinMode(interruptPin, INPUT_PULLUP);
  if(!adc.init(useADS1015)){
    Serial.println("ADS1015 not connected!");
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1015_RANGE_6144  ->  +/- 6144 mV
   * ADS1015_RANGE_4096  ->  +/- 4096 mV
   * ADS1015_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1015_RANGE_1024  ->  +/- 1024 mV
   * ADS1015_RANGE_0512  ->  +/- 512 mV
   * ADS1015_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1015_RANGE_2048); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1015_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1015_COMP_0_3    ->  compares 0 with 3
   *  ADS1015_COMP_1_3    ->  compares 1 with 3
   *  ADS1015_COMP_2_3    ->  compares 2 with 3
   *  ADS1015_COMP_0_GND  ->  compares 0 with GND
   *  ADS1015_COMP_1_GND  ->  compares 1 with GND
   *  ADS1015_COMP_2_GND  ->  compares 2 with GND
   *  ADS1015_COMP_3_GND  ->  compares 3 with GND
   */
  adc.setCompareChannels(ADS1015_COMP_0_1); //comment line/change parameter to change channel

  /* Set number of conversions after which the alert pin asserts
   * - or you can disable the alert 
   *  
   *  ADS1015_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1015_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1015_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1015_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  adc.setAlertPinMode(ADS1015_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1015_128_SPS 
   *  ADS1015_250_SPS  
   *  ADS1015_490_SPS 
   *  ADS1015_920_SPS  
   *  ADS1015_1600_SPS (default)
   *  ADS1015_2400_SPS 
   *  ADS1015_3300_SPS 
   */
  adc.setConvRate(ADS1015_3300_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1015_CONTINUOUS  ->  continuous mode
   *  ADS1015_SINGLE     ->  single shot mode (default)
   */
  adc.setMeasureMode(ADS1015_CONTINUOUS); //comment line/change parameter to change mode

   /* Choose maximum limit or maximum and minimum alert limit (window) in Volt - alert pin will 
   *  assert when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin assertion will be  
   *  cleared (if not latched)  
   * 
   *  ADS1015_MAX_LIMIT
   *  ADS1015_WINDOW
   * 
   */
  //adc.setAlertModeAndLimit_V(ADS1015_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alert pin will assert until the
   * conversion register is read (getResult functions). If disabled the alert pin assertion will be
   * cleared with next value within limits. 
   *  
   *  ADS1015_LATCH_DISABLED (default)
   *  ADS1015_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1015_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1015_ACT_LOW  ->  active low (default)   
   * ADS1015_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1015_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  adc.setAlertPinToConversionReady(); //uncomment if you want to change the default


  Serial.println("ADS1015 Example Sketch - Single Shot, Conversion Ready Alert Pin controlled");
  Serial.println();
  adc.startSingleMeasurement();
}

  /* In this example I measure 32 times before the result is output. This is only to slow down 
   * the output rate. I want to show that the output rate is controlled by the conversion ready  
   * signals and not by a delay.
   */
void loop() {
  if (!loop_once){
      for (int i = 0; i < 300; i++){
      localData.voltage_value[i] = adc.getResultWithRange(-2048, 2048); // alternative: getResult_mV for Millivolt
      localData.time[i] = esp_timer_get_time();
      delayMicroseconds(320);
      }
      for (int i = 0; i< 300; i++){
        Serial.print(localData.voltage_value[i]);
        Serial.print(",");
        Serial.print(localData.time[i]);
        Serial.print(";");
      }
      loop_once= true;
  }
  }
