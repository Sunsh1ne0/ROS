#ifndef WHEELS_CONTROLLER_H_
#define WHEELS_CONTROLLER_H_

#include <ros/ros.h>
#include <cmath>
#include "agrobit_msg/can_bus.h"
#include "agrobit_msg/sensors.h"
#include "std_msgs/UInt16.h"

#define WHEELS_CONTROLLER_RATE_HZ 10
#define HUMIDITY_COEF 0.0265251989389
#define TEMPERATURE_COEF 0.02509915

#define T1 27385
#define T2 25589
#define T3 -1000

/*=========================================================================
	ME3_NH3
	-----------------------------------------------------------------------*/
#define ME3NH3_RFB 150000 // R_fb (Ohm)
#define ME3NH3_VCC 3.3 // Vcc (V)
#define ME3NH3_BIAS 0.0 // ppm
/*=========================================================================*/

/*=========================================================================
	ME3_H2S
	-----------------------------------------------------------------------*/
#define ME3H2S_RFB 33000 // R_fb (Ohm)
#define ME3H2S_VCC 3.3 // Vcc (V)
#define ME3H2S_BIAS 0.0 // ppm
/*=========================================================================*/

/*=========================================================================
	ME3_CO
	-----------------------------------------------------------------------*/
#define ME3CO_RFB 16000 // R_fb (Ohm)
#define ME3CO_VCC 3.3 // Vcc (V)
#define ME3CO_BIAS 0.0 // ppm
/*=========================================================================*/

struct data{
uint16_t NH3 = 0;
uint16_t CO = 0;
uint16_t H2S = 0;
uint32_t pressure = 0;
uint16_t lumen = 0;
uint16_t IR_temperature = 0;
uint16_t outside_humidity = 0;
uint16_t outside_temperature = 0;
uint16_t CO2 = 0;
uint16_t inside_temperature = 0;
};

class SensorsData {

public:
	SensorsData();
	void Run();
	void FirstPackageCallback(const agrobit_msg::can_bus& can_bus_msg);
	void SecondPackageCallback(const agrobit_msg::can_bus& can_bus_msg);
	void ThirdPackageCallback(const agrobit_msg::can_bus& can_bus_msg);
	float GetConcentrationMESensor(float (*m_array_of_data)[2], int m_rows, float m_vadc_in_mV, float m_r_fb);

	
private:
	void Init();
	void PublishSensorsDataPackage();

	float array_of_me3nh3_NH3_data[5][2] = {{0.0, 0.0}, {25.0, 3.2}, {50.0, 6.4}, {75.0, 10.1}, {100.0, 13.7}};

	float array_of_me3h2s_H2S_data[4][2] = {{0.0, 0.0}, {25.0, 14.8}, {50.0, 31.0}, {100.0, 64.0}};

	float array_of_me3co_CO_data[10][2] = {{0.0, 0.0}, {50.0, 4.0}, {100.0, 6.5}, {200.0, 14.5}, {300.0, 20.0}, {500.0, 33.3}, {750.0, 49.3}, {1000.0, 66.0}, {1500.0, 93.3}, {2000.0, 120.0}};


	ros::NodeHandle n_;
	ros::Subscriber first_package_subscriber_;
	ros::Subscriber second_package_subscriber_;
	ros::Subscriber third_package_subscriber_;
	ros::Publisher sensors_data_publisher_;

	data sensors_data;
};

#endif
