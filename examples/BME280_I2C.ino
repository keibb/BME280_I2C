#include <BME280.h>
#define BME280_ADDRESS 0x76         //default I2C address
BME280 bme(BME280_ADDRESS);         //instantiate BME280 class
void setup() {
  // settings
    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 3;               //Normal mode
    uint8_t t_sb = 5;               //Tstandby 1000ms
    uint8_t filter = 0;             //Filter off 

    Serial.begin(9600);
    Wire.begin();
    bme.BME280_Init(
      osrs_t,
      osrs_p,
      osrs_h,
      mode,
      t_sb,
      filter
      );
}

void loop() {
  // put your main code here, to run repeatedly:
    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    
    bme.readData(temp_act,press_act,hum_act);
    
    Serial.print("TEMP : ");
    Serial.print(temp_act);
    Serial.print(" DegC  PRESS : ");
    Serial.print(press_act);
    Serial.print(" hPa  HUM : ");
    Serial.print(hum_act);
    Serial.println(" %");    
    
    delay(1000);

}
