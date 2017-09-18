//
//  BME280.h
//  
//
//  Created by Keiji on 2017/09/16.
//
//

#ifndef BME280_h
#define BME280_h

#include <Arduino.h>
#include <Wire.h>

class BME280
{
public:
    BME280(uint8_t bme280_address);
    void BME280_Init(uint8_t osrs_t,uint8_t osrs_p,uint8_t osrs_h,uint8_t mode,uint8_t t_sb,uint8_t filter);
    uint8_t osrs_t;
    uint8_t osrs_p;
    uint8_t osrs_h;
    uint8_t mode;
    uint8_t t_sb;
    uint8_t filter;
    
    void readData(double *temp_act,double *press_act,double *hum_act);
    
private:
    void readTrim();
    uint8_t _bme280_address;
    void writeReg(uint8_t reg_address,uint8_t data);
    uint8_t _ctrl_meas_reg;
    uint8_t _config_reg;
    uint8_t _ctrl_hum_reg;

    signed long int _t_fine;
    
    uint16_t _dig_T1;
    int16_t _dig_T2;
    int16_t _dig_T3;
    uint16_t _dig_P1;
    int16_t _dig_P2;
    int16_t _dig_P3;
    int16_t _dig_P4;
    int16_t _dig_P5;
    int16_t _dig_P6;
    int16_t _dig_P7;
    int16_t _dig_P8;
    int16_t _dig_P9;
    int8_t  _dig_H1;
    int16_t _dig_H2;
    int8_t  _dig_H3;
    int16_t _dig_H4;
    int16_t _dig_H5;
    int8_t  _dig_H6;
    
    signed long int calibration_T(signed long int _adc_T);
    unsigned long int calibration_P(signed long int _adc_P);
    unsigned long int calibration_H(signed long int _adc_H);
    
    unsigned long int _hum_raw;
    unsigned long int _press_raw;
    unsigned long int _temp_raw;

   
    
};


#endif /* BME280_h */
