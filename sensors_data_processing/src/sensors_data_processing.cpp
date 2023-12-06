#include "sensors_data_processing/sensors_data_processing.h"

SensorsData::SensorsData() {

	first_package_subscriber_ = n_.subscribe("/status_first_can", 1, &SensorsData::FirstPackageCallback, this);
	second_package_subscriber_ = n_.subscribe("/status_second_can", 1, &SensorsData::SecondPackageCallback, this);
	third_package_subscriber_ = n_.subscribe("/status_third_can", 1, &SensorsData::ThirdPackageCallback, this);

	sensors_data_publisher_ = n_.advertise<agrobit_msg::sensors>("/status_data", 1);
	
}

void SensorsData::FirstPackageCallback(const agrobit_msg::can_bus& can_bus_msg) {
sensors_data.CO = can_bus_msg.data[0] | ((can_bus_msg.data[1] & 0xF0) << 4);
sensors_data.H2S = (can_bus_msg.data[2] << 4) | (can_bus_msg.data[1] & 0x0F);
sensors_data.NH3 = can_bus_msg.data[3] | ((can_bus_msg.data[4] & 0xF0) << 4);
sensors_data.outside_temperature = (can_bus_msg.data[5] << 4) | (can_bus_msg.data[4] & 0x0F);
sensors_data.lumen = can_bus_msg.data[6] | (can_bus_msg.data[7] << 8);
}

void SensorsData::SecondPackageCallback(const agrobit_msg::can_bus& can_bus_msg) {
sensors_data.IR_temperature = can_bus_msg.data[0] | (can_bus_msg.data[1] << 8);
sensors_data.outside_humidity = can_bus_msg.data[2] | ((can_bus_msg.data[3] & 0xF0) << 4);
sensors_data.pressure = (can_bus_msg.data[4] & 0xFF) | (can_bus_msg.data[5] << 8) | (can_bus_msg.data[6] << 16) | (can_bus_msg.data[7] << 24);
}

void SensorsData::ThirdPackageCallback(const agrobit_msg::can_bus& can_bus_msg) {
sensors_data.CO2 = can_bus_msg.data[0] | (can_bus_msg.data[1] << 8);
sensors_data.inside_temperature = can_bus_msg.data[3] | (can_bus_msg.data[2] << 8);
}

void SensorsData::PublishSensorsDataPackage() {
	int number_of_rows_array_me3nh3_NH3 = sizeof(array_of_me3nh3_NH3_data) / sizeof(array_of_me3nh3_NH3_data[0]);

	int number_of_rows_array_me3h2s_H2S = sizeof(array_of_me3h2s_H2S_data) / sizeof(array_of_me3h2s_H2S_data[0]);

	int number_of_rows_array_me3co_CO = sizeof(array_of_me3co_CO_data) / sizeof(array_of_me3co_CO_data[0]);

	agrobit_msg::sensors first_sensors_data_for_pub;
	first_sensors_data_for_pub.NH3 = GetConcentrationMESensor(array_of_me3nh3_NH3_data, number_of_rows_array_me3nh3_NH3, sensors_data.NH3 / 4096 * 3.2, ME3NH3_RFB) + ME3NH3_BIAS;
	if (first_sensors_data_for_pub.NH3 < 0) {
		first_sensors_data_for_pub.NH3 = 0.0;
	}
	first_sensors_data_for_pub.CO = GetConcentrationMESensor(array_of_me3co_CO_data, number_of_rows_array_me3co_CO, sensors_data.CO / 4096 * 3.2, ME3CO_RFB) + ME3CO_BIAS;
	if (first_sensors_data_for_pub.CO < 0) {
		first_sensors_data_for_pub.CO = 0.0;
	}
	first_sensors_data_for_pub.H2S = GetConcentrationMESensor(array_of_me3h2s_H2S_data, number_of_rows_array_me3h2s_H2S, sensors_data.H2S / 4096 * 3.2, ME3H2S_RFB) + ME3H2S_BIAS;
	if (first_sensors_data_for_pub.H2S < 0) {
		first_sensors_data_for_pub.H2S = 0.0;
	}
	first_sensors_data_for_pub.pressure = (double)sensors_data.pressure / 256.0;
	first_sensors_data_for_pub.lumen = sensors_data.lumen / 1.2;
	first_sensors_data_for_pub.IR_temperature = sensors_data.IR_temperature * 0.02 - 273.15;
	first_sensors_data_for_pub.outside_humidity = (float)sensors_data.outside_humidity * HUMIDITY_COEF;
	first_sensors_data_for_pub.outside_temperature = (float)sensors_data.outside_temperature * TEMPERATURE_COEF - 20;
	first_sensors_data_for_pub.CO2 = sensors_data.CO2;//
	first_sensors_data_for_pub.temperature_inside = (float)sensors_data.inside_temperature * 0.1;//
	sensors_data_publisher_.publish(first_sensors_data_for_pub);
}

float SensorsData::GetConcentrationMESensor(float (*m_array_of_data)[2], int m_rows, float m_vadc_in_mV, float m_r_fb) {
	float vadc = m_vadc_in_mV * 1e-3;
	float sensor_current = ((vadc) / (m_r_fb)) * 1e6;
	float concentration;
	float k = 0.0;
	float b = 0.0;
	if (m_array_of_data[1][1] > m_array_of_data[0][1]) {
		if (sensor_current <= 0.0) {
			concentration = 0.0;
		} else {
			if (sensor_current >= m_array_of_data[m_rows - 1][1]) {
				k = ((m_array_of_data[m_rows - 1][1]) - (m_array_of_data[m_rows - 2][1])) / ((m_array_of_data[m_rows - 1][0]) - (m_array_of_data[m_rows - 2][0]));
				b = (m_array_of_data[m_rows - 2][1]) - k * (m_array_of_data[m_rows - 2][0]);
				concentration = (sensor_current - b) / k;
			} else {
				int i = 0;
				while ((sensor_current >= m_array_of_data[i][1]) && (i < m_rows)) {
					i++;
				}
				k = ((m_array_of_data[i][1]) - (m_array_of_data[i - 1][1])) / ((m_array_of_data[i][0]) - (m_array_of_data[i - 1][0]));
				b = (m_array_of_data[i - 1][1]) - k * (m_array_of_data[i - 1][0]);
				concentration = (sensor_current - b) / k;
			}
		}
		return concentration;
	} else {
		if (sensor_current >= 0.0) {
			concentration = 0.0;
		} else {
			if (sensor_current <= m_array_of_data[m_rows - 1][1]) {
				k = ((m_array_of_data[m_rows - 1][1]) - (m_array_of_data[m_rows - 2][1])) / ((m_array_of_data[m_rows - 1][0]) - (m_array_of_data[m_rows - 2][0]));
				b = (m_array_of_data[m_rows - 2][1]) - k * (m_array_of_data[m_rows - 2][0]);
				concentration = (sensor_current - b) / k;
			} else {
				int i = 0;
				while ((sensor_current <= m_array_of_data[i][1]) && (i < m_rows)) {
					i++;
				}
				k = ((m_array_of_data[i][1]) - (m_array_of_data[i - 1][1])) / ((m_array_of_data[i][0]) - (m_array_of_data[i - 1][0]));
				b = (m_array_of_data[i - 1][1]) - k * (m_array_of_data[i - 1][0]);
				concentration = (sensor_current - b) / k;
			}
		}
		return concentration;
	}
}


void SensorsData::Run() {
	ros::Rate rate(WHEELS_CONTROLLER_RATE_HZ);
	while(ros::ok()) {
		PublishSensorsDataPackage();
		rate.sleep();
		ros::spinOnce();
	}
}





