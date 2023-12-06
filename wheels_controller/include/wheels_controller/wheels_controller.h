#ifndef WHEELS_CONTROLLER_H_
#define WHEELS_CONTROLLER_H_

#include <ros/ros.h>
#include <cmath>
#include "agrobit_msg/can_bus.h"
#include "agrobit_msg/wheels_electrical.h"
#include "agrobit_msg/wheels_kinematics.h"
#include "std_msgs/Float32.h"

#define WHEELS_CONTROLLER_RATE_HZ 10
#define MULTIPLIER_FOR_SENDING_PID_CONTROLLER_PARAMS (float)100.0

#define MATH_PI (float)3.14159265359
#define MM_PER_SEC_TO_KM_PER_HOUR (float)0.0036

#define MAX_UINT16_VAL 65535
#define HALF_MAX_UINT16_VAL 32767

#define MAX_PWM_VAL (float)9140
#define ZERO_CURRENT_ADC_VAL 2048
#define MAX_ADC_VAL (float)4095
#define CURRENT_SENSOR_SENS_mV_A (float)26.5
#define ADC_REFERENCE_VOLTAGE (float)3.3

enum WheelsSide {LEFT, RIGHT}; //0, 1
enum WheelsAxle {REAR, FRONT}; //0, 1

enum FeedbackData {REAR_ELECTRICAL, FRONT_ELECTRICAL}; //0, 1

struct WheelParameters {
	float wheel_diameter_in_mm;
	int number_of_ticks_per_turn;
};

struct ChassisParameters {
	float robot_track_in_mm;
	float robot_wheelbase_in_mm;
};

struct PIDControllerParameters {
	float kp;
	float kd;
	float ki;
};

struct WheelsControllerParameters {
	float k_steering;
	float command_actuality_time;
};

struct ChassisKinematicsVariables {
	float steering_system_angles[2];
	float wheels_speed[2][2];
	uint16_t previous_pulse_counter_val[2][2];
	int32_t encoders_pulse_counter[2][2];
};

struct ChassisElectricalVariables {
	float pwm_val[2][2];
	float current_val[2][2];
	bool hall_sensors_status[2][2];
	bool alm_status[2][2];
	bool brk_state[2][2];
	bool dir_state[2][2];
};

class WheelsController {

public:
	WheelsController();
	void Run();
	void RearWheelsKinematicsCallback(const agrobit_msg::can_bus& can_bus_msg);
	void RearWheelsElectricalCallback(const agrobit_msg::can_bus& can_bus_msg);
	void FrontWheelsKinematicsCallback(const agrobit_msg::can_bus& can_bus_msg);
	void FrontWheelsElectricalCallback(const agrobit_msg::can_bus& can_bus_msg);

	void TargetSpeedLeftCallback(const std_msgs::Float32& msg);
	void TargetSpeedRightCallback(const std_msgs::Float32& msg);

private:
	void Init();
	void UpdateParameters();
	void FillInCanMsgWithFourInt16(agrobit_msg::can_bus *m_can_bus_msg, int16_t *m_data);
	void PublishWheelsKinematicsData(uint8_t m_axle_number);
	void PublishWheelsElectricalData(uint8_t m_axle_number);
	void PublishWheelsControlData(uint8_t m_axle_number);
	ros::NodeHandle n_;
	ros::Subscriber wheels_kinematics_feedback_subscriber_[2];
	ros::Subscriber wheels_electrical_feedback_subscriber_[2];
	ros::Publisher wheels_pid_controller_params_publisher_[2];
	ros::Publisher wheels_control_publisher_[2];
	ros::Publisher wheels_kinematics_data_publisher_[2];
	ros::Publisher wheels_electrical_data_publisher_[2];

	ros::Subscriber target_speed_left_subscriber_;
	ros::Subscriber target_speed_right_subscriber_;

	ChassisKinematicsVariables current_chassis_kinematics_variables_;
	ChassisKinematicsVariables target_chassis_kinematics_variables_;
	ChassisElectricalVariables current_chassis_electrical_variables_;
	WheelParameters wheel_params_;
	ChassisParameters chassis_params_;
	PIDControllerParameters pid_controller_params_;
	WheelsControllerParameters wheels_controller_params_;

	float tick_length_in_mm_ = 0.0;
	float number_of_ticks_in_one_mm_ = 0.0;

	float conversion_factor_speed_in_ticks_to_speed_in_kmph_ = 0.0;
	float conversion_factor_speed_in_kmph_to_speed_in_ticks_ = 0.0;

	float conversion_factor_adc_val_to_current_ = 0.0;
	float conversion_factor_pwm_val_to_percentage_ = 0.0;

	bool flag_new_electrical_feedback_data_[2] = {false, false};

};

#endif
