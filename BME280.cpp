//
//  BME280.cpp
//  
//
//  Created by Keiji on 2017/09/16.
//
//

#include <BME280.h>

BME280::BME280(uint8_t bme280_address)
: _bme280_address(bme280_address)
{}

//  Initialising BME280
void BME280::BME280_Init(uint8_t osrs_t,uint8_t osrs_p,uint8_t osrs_h,uint8_t mode,uint8_t t_sb,uint8_t filter){
    
        uint8_t spi3w_en = 0;
    
        uint8_t _ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
        uint8_t _config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
        uint8_t _ctrl_hum_reg  = osrs_h;
    
        writeReg(0xF2,_ctrl_hum_reg);
        writeReg(0xF4,_ctrl_meas_reg);
        writeReg(0xF5,_config_reg);
        readTrim();
}
    
void BME280::writeReg(uint8_t reg_address,uint8_t data){
        Wire.beginTransmission(_bme280_address);
        Wire.write(reg_address);
        Wire.write(data);
        Wire.endTransmission();
}

void BME280::readTrim(){
        uint8_t data[32],i=0;
        Wire.beginTransmission(_bme280_address);
        Wire.write(0x88);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_bme280_address,(uint8_t)24);
        while(Wire.available()){
                data[i] = Wire.read();
                i++;
            }
            
        Wire.beginTransmission(_bme280_address);
        Wire.write(0xA1);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_bme280_address,(uint8_t)1);
        data[i] = Wire.read();
            i++;
            
        Wire.beginTransmission(_bme280_address);
        Wire.write(0xE1);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_bme280_address,(uint8_t)7);
        while(Wire.available()){
                data[i] = Wire.read();
                i++;
            }
        _dig_T1 = (data[1] << 8) | data[0];
        _dig_T2 = (data[3] << 8) | data[2];
        _dig_T3 = (data[5] << 8) | data[4];
        _dig_P1 = (data[7] << 8) | data[6];
        _dig_P2 = (data[9] << 8) | data[8];
        _dig_P3 = (data[11]<< 8) | data[10];
        _dig_P4 = (data[13]<< 8) | data[12];
        _dig_P5 = (data[15]<< 8) | data[14];
        _dig_P6 = (data[17]<< 8) | data[16];
        _dig_P7 = (data[19]<< 8) | data[18];
        _dig_P8 = (data[21]<< 8) | data[20];
        _dig_P9 = (data[23]<< 8) | data[22];
        _dig_H1 = data[24];
        _dig_H2 = (data[26]<< 8) | data[25];
        _dig_H3 = data[27];
        _dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
        _dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
        _dig_H6 = data[31];
}
void BME280::readData(double *temp_act,double *press_act,double *hum_act){
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(_bme280_address);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_bme280_address,(uint8_t)8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    unsigned long int _press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    unsigned long int _temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    unsigned long int _hum_raw = (data[6] <<8) | data[7];
    
    *temp_act = calibration_T(_temp_raw) / 100.0;
    *press_act = calibration_P(_press_raw) / 100.0;
    *hum_act = calibration_H(_hum_raw) / 1024.0;
}
signed long int BME280::calibration_T(signed long int _adc_T){
    signed long int var1, var2,T;
    var1 = ((((_adc_T >> 3) - ((signed long int)_dig_T1 << 1))) * ((signed long  int)_dig_T2)) >> 11;
    var2 = (((((_adc_T >> 4) - ((signed long int)_dig_T1)) * ((_adc_T >> 4) - ((signed long int)_dig_T1))) >> 12) * ((signed long int)_dig_T3)) >> 14;
    
    _t_fine = var1 + var2;
    T = (_t_fine * 5 + 128) >> 8;
    return T;
}
unsigned long int BME280::calibration_P(signed long int _adc_P){
    signed long int var1,var2;
    unsigned long int P;
    var1 = (((signed long int )_t_fine)>>1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)_dig_P6);
    var2 = var2 + ((var1 *((signed long int)_dig_P5)) << 1);
    var2 = (var2 >> 2) +(((signed long int )_dig_P4) << 16);
    var1 = (((_dig_P3 * (((var1 >> 2) * (var1 >> 2))  >> 13)) >> 3) + ((((signed long int)_dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)_dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576) - _adc_P) - (var2 >> 12))) * 3125;
    if (P < 0x80000000)
    {
        P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)_dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(P >> 2)) * ((signed long int)_dig_P8)) >> 13;
    P = (unsigned long int)((signed long int) P + ((var1 + var2 + _dig_P7) >> 4));
    return P;
}
unsigned long int BME280::calibration_H(signed long int _adc_H)
{
    signed long int v_x1;
    
    v_x1 = (_t_fine - ((signed long int)76800));
    v_x1 =(((((_adc_H << 14) - (((signed long int)_dig_H4) << 20) - (((signed long int)_dig_H5) * v_x1)) + ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)_dig_H6)) >> 10) * (((v_x1 * ((signed long int)_dig_H3)) >> 11) + ((signed long int)32768))) >> 10) + ((signed long int)2097152)) * ((signed long int)_dig_H2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)_dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 :v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (unsigned long int)(v_x1 >> 12);
}
