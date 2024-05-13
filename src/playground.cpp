/***************************************************************************
* Example sketch for the ADS1015_WE library
*
* This sketch shows how to use the ADS1015 in continuous mode. 
*  
* Further information can be found on:
* https://wolles-elektronikkiste.de/ads1115 (German)
* https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
* 
***************************************************************************/

#include <ADS1015_WE.h> 
#include <Wire.h>
#define VOLTAGE_I2C_ADDR 0x48
#define CURRENT_I2C_ADDR 0x49
#define LOOP_AMOUNT 200
#define ADC_I2C_SPEED 800000U
int64_t old_time = 0;
int64_t voltage_time_now = 0, current_time_now = 0;
float voltage_read, current_read;
int i = 0;


/* There are several ways to create your ADS1015_WE object:
 * ADS1015_WE adc = ADS1015_WE(); -> uses Wire / I2C Address = 0x48
 * ADS1015_WE adc = ADS1015_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
 * ADS1015_WE adc = ADS1015_WE(&Wire); -> you can pass any TwoWire object / I2C Address = 0x48
 * ADS1015_WE adc = ADS1015_WE(&Wire, I2C_ADDRESS); -> all together
 */
TwoWire AdcI2C = TwoWire(0);
ADS1015_WE voltage_adc = ADS1015_WE(&AdcI2C, VOLTAGE_I2C_ADDR);
ADS1015_WE current_adc = ADS1015_WE(&AdcI2C, CURRENT_I2C_ADDR);

void setup() {
  bool useADS1015 = true;
  AdcI2C.begin(19, 18, ADC_I2C_SPEED);
  Serial.begin(115200);
  if(!voltage_adc.init(useADS1015)){ // passing true will tell the lib that an ADS1015 is used
    Serial.println("VOLTAGE - ADS1015 not connected!");
  }
  if(!current_adc.init(useADS1015)){ // passing true will tell the lib that an ADS1015 is used
    Serial.println("CURRENT - ADS1015 not connected!");
  }

  /* Set the voltage range of the voltage_adc to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1015_RANGE_6144  ->  +/- 6144 mV
   * ADS1015_RANGE_4096  ->  +/- 4096 mV
   * ADS1015_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1015_RANGE_1024  ->  +/- 1024 mV
   * ADS1015_RANGE_0512  ->  +/- 512 mV
   * ADS1015_RANGE_0256  ->  +/- 256 mV
   */
  voltage_adc.setVoltageRange_mV(ADS1015_RANGE_0256); //comment line/change parameter to change range
  current_adc.setVoltageRange_mV(ADS1015_RANGE_0256); //comment line/change parameter to change range

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
  voltage_adc.setCompareChannels(ADS1015_COMP_0_1); //comment line/change parameter to change channel
  current_adc.setCompareChannels(ADS1015_COMP_0_1); //comment line/change parameter to change channel

  /* Set number of conversions after which the alert pin asserts
   * - or you can disable the alert 
   *  
   *  ADS1015_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1015_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1015_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1015_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //voltage_adc.setAlertPinMode(ADS1015_ASSERT_AFTER_1); //uncomment if you want to change the default

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
  voltage_adc.setConvRate(ADS1015_3300_SPS); //uncomment if you want to change the default
  current_adc.setConvRate(ADS1015_3300_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1015_CONTINUOUS  ->  continuous mode
   *  ADS1015_SINGLE     ->  single shot mode (default)
   */
  voltage_adc.setMeasureMode(ADS1015_CONTINUOUS); //comment line/change parameter to change mode
  current_adc.setMeasureMode(ADS1015_CONTINUOUS); //comment line/change parameter to change mode
  
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
  //voltage_adc.setAlertModeAndLimit_V(ADS1015_MAX_LIMIT, 3.0, 1.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alert pin will assert until the
   * conversion register is read (getResult functions). If disabled the alert pin assertion will be
   * cleared with next value within limits. 
   *  
   *  ADS1015_LATCH_DISABLED (default)
   *  ADS1015_LATCH_ENABLED
   */
  //voltage_adc.setAlertLatch(ADS1015_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1015_ACT_LOW  ->  active low (default)   
   * ADS1015_ACT_HIGH ->  active high
   */
  //voltage_adc.setAlertPol(ADS1015_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //voltage_adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  Serial.println("ADS1015 CURRENT PRINT - Continuous Mode");
}

  /* If you change the compare channels you can immediately read values from the conversion 
   * register, although they might belong to the former channel if no precautions are taken. 
   * It takes about the time needed for two conversions to get the correct data. In single 
   * shot mode you can use the isBusy() function to wait for data from the new channel. This 
   * does not work in continuous mode. 
   * To solve this issue the library adds a delay after change of channels if you are in contunuous
   * mode. The length of the delay is adjusted to the conversion rate. But be aware that the output 
   * rate will be much lower that the conversion rate if you change channels frequently. 
   */

void loop() {
  voltage_read = 0;
  current_read = 0;
  if ((esp_timer_get_time() - old_time) > 600){
    voltage_read = voltage_adc.getResult_mV();
    voltage_time_now = esp_timer_get_time();
    current_read = current_adc.getResult_mV();
    current_time_now = esp_timer_get_time();
    if (i<LOOP_AMOUNT){
        Serial.print(current_read);
        Serial.print(",");
        Serial.print(current_time_now);
        Serial.print(" ");
        i++;
    }
  }

}

