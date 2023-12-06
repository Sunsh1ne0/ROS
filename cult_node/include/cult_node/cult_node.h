#ifndef CULTIVATOR_NODE_H_
#define CULTIVATOR_NODE_H_

#include <ros/ros.h>
#include "agrobit_msg/can_bus.h"
#include "agrobit_msg/target_values.h"
#include "agrobit_msg/kinematics_msg.h"
#include "agrobit_msg/status_msg.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include "time.h"

#define CULTIVATOR_CONTROLLER_RATE_HZ 2
#define MULTIPLIER_FOR_SENDING_PID_CONTROLLER_PARAMS (float)100.0

#define MATH_PI (float)3.14159265359

#define MAX_UINT16_VAL 65535
#define HALF_MAX_UINT16_VAL 32767

#define MAX_PWM_VAL (float)9140
#define ZERO_CURRENT_ADC_VAL 2048
#define MAX_ADC_VAL (float)4095
#define CURRENT_SENSOR_SENS_mV_A (float)26.5
#define ADC_REFERENCE_VOLTAGE (float)3.3

struct ActuatorsParameters {
	int kp_left;
	int kp_right;
    int extension;
};

struct CultivatorParameters {
	int pulses_per_rotation;
	int kp;
	int ki;
    int speed;
};

class CultivatorController {

public:
	CultivatorController();
	void Run();
	void CultivatorKinematicsCallback(const agrobit_msg::can_bus& can_bus_msg);
	void CultivatorStatusCallback(const agrobit_msg::can_bus& can_bus_msg);
	
	void TargetValuesCallback(const agrobit_msg::target_values& msg);

private:
	void Init();
	void UpdateParameters();
	void FillInCanMsgWithFourInt16(agrobit_msg::can_bus *m_can_bus_msg, int16_t *m_data);
	void PublishCultivatorData();
    void PublishKinematics();
    void PublishStatus(); 
	ros::NodeHandle n_;
	ros::Subscriber cultivator_kinematics_feedback_subscriber_;
	ros::Subscriber cultivator_status_feedback_subscriber_;
	ros::Publisher cultivator_parameters_publisher_;
    ros::Publisher cultivator_kinematics_publisher_;
    ros::Publisher cultivator_status_publisher_;

	ros::Subscriber target_values_subscriber_;

	ActuatorsParameters current_actuators_parameters;
    ActuatorsParameters target_actuators_parameters;
	CultivatorParameters current_cultivator_parameters;
    CultivatorParameters target_cultivator_parameters;

};

#endif
