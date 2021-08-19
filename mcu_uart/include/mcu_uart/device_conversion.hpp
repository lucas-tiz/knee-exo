// Convert uint16_t sensor measurements to floats with units
#ifndef DEVICE_CONVERSION_HPP_
#define DEVICE_CONVERSION_HPP_


#include <map>
#include <stdint.h>


// devices 
#define PRES_ASDXAVX030PGAA5 0
#define PRES_ASDXAVX100PGAA5 1
#define PRES_SSCDRRN100PGAB5 4
#define ENC_AMT102V 2
#define OPT_SENS 3
#define ANALOG 3

// microcontroller parameters
#define ADC_RES10 1023.0
#define ADC_RES12 4095.0
#define ADC_RES14 16383.0


struct sensor_struct {
  float slope;
  float offset;
};


class Converter {
  private:
    float adc_res, v_supply, v_adc;
    std::map<int, sensor_struct> sensor_lib;

  public:
    Converter(float adc_resolution, float volt_supply, float volt_adc);
    float intToFloat(int msmt_int, int device);
    int floatToInt(float msmt_float, int device);
};


Converter::Converter(float adc_resolution, float volt_supply, float volt_adc) {
  adc_res = adc_resolution;
  v_supply = volt_supply;
  v_adc = volt_adc;    
    
  // tf_min, tf_max are transfer function limits (ex: 10% to 90%)
  // slope = (v_adc/adc_res)*((p_max-p_min))/((tf_max-tf_min)*v_supply)
  // offset = -(tf_min*(p_max-p_min))/(tf_max-tf_min) + p_min
  sensor_lib[PRES_ASDXAVX030PGAA5].slope = (v_adc/adc_res)*((30)/(0.8*v_supply));
  sensor_lib[PRES_ASDXAVX030PGAA5].offset = -(0.1*(30))/(0.8);
  sensor_lib[PRES_ASDXAVX100PGAA5].slope =  (v_adc/adc_res)*((100)/(0.8*v_supply));
  sensor_lib[PRES_ASDXAVX100PGAA5].offset = -(0.1*(100))/(0.8);
  sensor_lib[PRES_SSCDRRN100PGAB5].slope = (v_adc/adc_res)*((100)/(0.9*v_supply));
  sensor_lib[PRES_SSCDRRN100PGAB5].offset = -(0.05*(100))/(0.9);

  sensor_lib[ENC_AMT102V].slope = 360.0/8192.0; // (int to degrees)
  sensor_lib[ENC_AMT102V].offset = 0.0;

  sensor_lib[OPT_SENS].slope = v_adc/adc_res;
  sensor_lib[OPT_SENS].offset = 0.0;
}


float Converter::intToFloat(int msmt_int, int device) {
  return sensor_lib[device].slope*(float)msmt_int + sensor_lib[device].offset;
}

int Converter::floatToInt(float msmt_float, int device) {
  return (uint16_t)((msmt_float - sensor_lib[device].offset)/sensor_lib[device].slope);
}


#endif // DEVICE_CONVERSION_HPP_
