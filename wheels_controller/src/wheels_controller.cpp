#include "wheels_controller/wheels_controller.h"

WheelsController::WheelsController() {
	Init();

	wheels_kinematics_feedback_subscriber_[WheelsAxle::REAR] = n_.subscribe("/wheels/can/rear/kinematics_feedback", 1, &WheelsController::RearWheelsKinematicsCallback, this);
	wheels_kinematics_feedback_subscriber_[WheelsAxle::FRONT] = n_.subscribe("/wheels/can/front/kinematics_feedback", 1, &WheelsController::FrontWheelsKinematicsCallback, this);
	wheels_electrical_feedback_subscriber_[WheelsAxle::REAR] = n_.subscribe("/wheels/can/rear/electrical_feedback", 1, &WheelsController::RearWheelsElectricalCallback, this);
	wheels_electrical_feedback_subscriber_[WheelsAxle::FRONT] = n_.subscribe("/wheels/can/front/electrical_feedback", 1, &WheelsController::FrontWheelsElectricalCallback, this);

	target_speed_left_subscriber_ = n_.subscribe("/target_speed_left", 1, &WheelsController::TargetSpeedLeftCallback, this);
	target_speed_right_subscriber_ = n_.subscribe("/target_speed_right", 1, &WheelsController::TargetSpeedRightCallback, this);

	wheels_pid_controller_params_publisher_[WheelsAxle::REAR] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/rear/pid_controller_params", 1);
	wheels_pid_controller_params_publisher_[WheelsAxle::FRONT] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/front/pid_controller_params", 1);
	wheels_control_publisher_[WheelsAxle::REAR] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/rear/control_data", 1);
	wheels_control_publisher_[WheelsAxle::FRONT] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/front/control_data", 1);
	wheels_kinematics_data_publisher_[WheelsAxle::REAR] = n_.advertise<agrobit_msg::wheels_kinematics>("/wheels/rear/kinematics_data", 1);
	wheels_kinematics_data_publisher_[WheelsAxle::FRONT] = n_.advertise<agrobit_msg::wheels_kinematics>("/wheels/front/kinematics_data", 1);
	wheels_electrical_data_publisher_[WheelsAxle::REAR] = n_.advertise<agrobit_msg::wheels_electrical>("/wheels/rear/electrical_data", 1);
	wheels_electrical_data_publisher_[WheelsAxle::FRONT] = n_.advertise<agrobit_msg::wheels_electrical>("/wheels/front/electrical_data", 1);

	UpdateParameters();
}

void WheelsController::TargetSpeedLeftCallback(const std_msgs::Float32& msg) {
	target_chassis_kinematics_variables_.wheels_speed[WheelsAxle::REAR][WheelsSide::LEFT] = msg.data;
}

void WheelsController::TargetSpeedRightCallback(const std_msgs::Float32& msg) {
	target_chassis_kinematics_variables_.wheels_speed[WheelsAxle::REAR][WheelsSide::RIGHT] = msg.data;
}

void WheelsController::Init() {
	for (int i = 0; i < 2; i++) {
		current_chassis_kinematics_variables_.steering_system_angles[i] = 0.0;
		target_chassis_kinematics_variables_.steering_system_angles[i] = 0.0;
		for (int j = 0; j < 2; j++) {
			current_chassis_kinematics_variables_.wheels_speed[j][i] = 0.0;
			current_chassis_kinematics_variables_.previous_pulse_counter_val[j][i] = 0;
			current_chassis_kinematics_variables_.encoders_pulse_counter[j][i] = 0;
			target_chassis_kinematics_variables_.wheels_speed[j][i] = 0.0;
			target_chassis_kinematics_variables_.previous_pulse_counter_val[j][i] = 0;
			target_chassis_kinematics_variables_.encoders_pulse_counter[j][i] = 0;

			current_chassis_electrical_variables_.pwm_val[j][i] = 0.0;
			current_chassis_electrical_variables_.current_val[j][i] = 0.0;
			current_chassis_electrical_variables_.hall_sensors_status[j][i] = false;
			current_chassis_electrical_variables_.alm_status[j][i] = false;
			current_chassis_electrical_variables_.brk_state[j][i] = false;
			current_chassis_electrical_variables_.dir_state[j][i] = false;
		}
	}
	conversion_factor_pwm_val_to_percentage_ = ((float)100.0) / MAX_PWM_VAL;
	conversion_factor_adc_val_to_current_ = (ADC_REFERENCE_VOLTAGE * float(1000.0)) / (MAX_ADC_VAL * CURRENT_SENSOR_SENS_mV_A);
}

void WheelsController::FillInCanMsgWithFourInt16(agrobit_msg::can_bus *m_can_bus_msg, int16_t *m_data) {
	m_can_bus_msg->timestamp_ms = ros::Time::now().toNSec() / 1e6;
	m_can_bus_msg->data.resize(8);
	for (int i = 0; i < 4; i++) {
		m_can_bus_msg->data[2 * i] = (uint8_t)((m_data[i] & 0xFF00) >> 8);
		m_can_bus_msg->data[2 * i + 1] = (uint8_t)(m_data[i] & 0x00FF);
	}
}

void WheelsController::UpdateParameters() {
	//wheel_parameters
	n_.param<float>("wheel_parameters/wheel_diameter_in_mm", wheel_params_.wheel_diameter_in_mm, 350.0);
	n_.param<int>("wheel_parameters/number_of_ticks_per_turn", wheel_params_.number_of_ticks_per_turn, 300);
	ROS_INFO("\n   wheel_parameters: \n\t%s: %.2f; \n\t%s: %d;", "wheel_diameter_in_mm", wheel_params_.wheel_diameter_in_mm, "number_of_ticks_per_turn", wheel_params_.number_of_ticks_per_turn);
	tick_length_in_mm_ = (MATH_PI * wheel_params_.wheel_diameter_in_mm) / ((float)wheel_params_.number_of_ticks_per_turn);
	number_of_ticks_in_one_mm_ = ((float)1.0) / tick_length_in_mm_;
	conversion_factor_speed_in_ticks_to_speed_in_kmph_ = tick_length_in_mm_ * MM_PER_SEC_TO_KM_PER_HOUR;
	conversion_factor_speed_in_kmph_to_speed_in_ticks_ = ((float)1.0) / conversion_factor_speed_in_ticks_to_speed_in_kmph_;
	ROS_INFO("\n   conversion_factors: \n\t%s: %.3f; \n\t%s: %.3f; \n\t%s: %.3f; \n\t%s: %.3f;",
	         "tick_length_in_mm", tick_length_in_mm_, "number_of_ticks_in_one_mm", number_of_ticks_in_one_mm_,
	         "conversion_factor_speed_in_ticks_to_speed_in_kmph", conversion_factor_speed_in_ticks_to_speed_in_kmph_,
	         "conversion_factor_speed_in_kmph_to_speed_in_ticks", conversion_factor_speed_in_kmph_to_speed_in_ticks_);
	//chassis_parameters
	n_.param<float>("chassis_parameters/robot_track_in_mm", chassis_params_.robot_track_in_mm, 323.0);
	n_.param<float>("chassis_parameters/robot_wheelbase_in_mm", chassis_params_.robot_wheelbase_in_mm, 357.0);
	ROS_INFO("\n   chassis_parameters: \n\t%s: %.2f; \n\t%s: %.2f;", "robot_track_in_mm", chassis_params_.robot_track_in_mm, "robot_wheelbase_in_mm", chassis_params_.robot_wheelbase_in_mm);
	//pid_controller_parameters
	n_.param<float>("pid_controller_parameters/kp", pid_controller_params_.kp, 3.0);
	n_.param<float>("pid_controller_parameters/kd", pid_controller_params_.kd, 9.0);
	n_.param<float>("pid_controller_parameters/ki", pid_controller_params_.ki, 0.0);
	ROS_INFO("\n   pid_controller_parameters: \n\t%s: %.2f; \n\t%s: %.2f; \n\t%s: %.2f;", "kp", pid_controller_params_.kp, "kd", pid_controller_params_.kd, "ki", pid_controller_params_.ki);
	//wheels_controller_parameters
	n_.param<float>("wheels_controller_parameters/k_steering", wheels_controller_params_.k_steering, 0.0025);
	n_.param<float>("wheels_controller_parameters/command_actuality_time", wheels_controller_params_.command_actuality_time, 1.0);
	ROS_INFO("\n   wheels_controller_parameters: \n\t%s: %.6f; \n\t%s: %.2f;", "k_steering", wheels_controller_params_.k_steering, "command_actuality_time", wheels_controller_params_.command_actuality_time);
	ros::Duration(2.0).sleep();
	int16_t pid_controller_params_for_sending[4];
	pid_controller_params_for_sending[0] = pid_controller_params_.kp * MULTIPLIER_FOR_SENDING_PID_CONTROLLER_PARAMS;
	pid_controller_params_for_sending[1] = pid_controller_params_.kd * MULTIPLIER_FOR_SENDING_PID_CONTROLLER_PARAMS;
	pid_controller_params_for_sending[2] = pid_controller_params_.ki * MULTIPLIER_FOR_SENDING_PID_CONTROLLER_PARAMS;
	pid_controller_params_for_sending[3] = 0;
	agrobit_msg::can_bus can_bus_msg_for_pub;
	FillInCanMsgWithFourInt16(&can_bus_msg_for_pub, pid_controller_params_for_sending);
	wheels_pid_controller_params_publisher_[WheelsAxle::REAR].publish(can_bus_msg_for_pub);
	wheels_pid_controller_params_publisher_[WheelsAxle::FRONT].publish(can_bus_msg_for_pub);
}

void WheelsController::RearWheelsKinematicsCallback(const agrobit_msg::can_bus& can_bus_msg) {
	int16_t speed_in_ticks = 0;
	int32_t delta_pulse_counter = 0;
	uint16_t current_pulse_counter_val = 0;
	for (int i = 0; i < 2; i++) {
		speed_in_ticks = (int16_t)(can_bus_msg.data[4 * i] << 8 | can_bus_msg.data[4 * i + 1]);
		current_chassis_kinematics_variables_.wheels_speed[WheelsAxle::REAR][i] = ((float)speed_in_ticks) * conversion_factor_speed_in_ticks_to_speed_in_kmph_;
		current_pulse_counter_val = (uint16_t)(can_bus_msg.data[4 * i + 2] << 8 | can_bus_msg.data[4 * i + 3]);
		delta_pulse_counter = current_pulse_counter_val - current_chassis_kinematics_variables_.previous_pulse_counter_val[WheelsAxle::REAR][i];
		if (abs(delta_pulse_counter) >= HALF_MAX_UINT16_VAL) {
			delta_pulse_counter = delta_pulse_counter - copysign(1, delta_pulse_counter) * (MAX_UINT16_VAL + 1);
		}
		current_chassis_kinematics_variables_.encoders_pulse_counter[WheelsAxle::REAR][i] = current_chassis_kinematics_variables_.encoders_pulse_counter[WheelsAxle::REAR][i] + delta_pulse_counter;
		current_chassis_kinematics_variables_.previous_pulse_counter_val[WheelsAxle::REAR][i] = current_pulse_counter_val;
	}
}

void WheelsController::FrontWheelsKinematicsCallback(const agrobit_msg::can_bus& can_bus_msg) {
	int16_t speed_in_ticks = 0;
	int32_t delta_pulse_counter = 0;
	uint16_t current_pulse_counter_val = 0;
	for (int i = 0; i < 2; i++) {
		speed_in_ticks = (int16_t)(can_bus_msg.data[4 * i] << 8 | can_bus_msg.data[4 * i + 1]);
		current_chassis_kinematics_variables_.wheels_speed[WheelsAxle::FRONT][i] = ((float)speed_in_ticks) * conversion_factor_speed_in_ticks_to_speed_in_kmph_;
		current_pulse_counter_val = (uint16_t)(can_bus_msg.data[4 * i + 2] << 8 | can_bus_msg.data[4 * i + 3]);
		delta_pulse_counter = current_pulse_counter_val - current_chassis_kinematics_variables_.previous_pulse_counter_val[WheelsAxle::FRONT][i];
		if (abs(delta_pulse_counter) >= HALF_MAX_UINT16_VAL) {
			delta_pulse_counter = delta_pulse_counter - copysign(1, delta_pulse_counter) * (MAX_UINT16_VAL + 1);
		}
		current_chassis_kinematics_variables_.encoders_pulse_counter[WheelsAxle::FRONT][i] = current_chassis_kinematics_variables_.encoders_pulse_counter[WheelsAxle::FRONT][i] + delta_pulse_counter;
		current_chassis_kinematics_variables_.previous_pulse_counter_val[WheelsAxle::FRONT][i] = current_pulse_counter_val;
	}
}

void WheelsController::RearWheelsElectricalCallback(const agrobit_msg::can_bus& can_bus_msg) {
	uint16_t uint16_pwm_val = 0;
	int16_t int16_current_val = 0;
	for (int i = 0; i < 2; i++) {
		uint16_pwm_val = ((can_bus_msg.data[4 * i] << 8) | can_bus_msg.data[4 * i + 1]);
		int16_current_val = ((can_bus_msg.data[4 * i + 3] & (0xF0)) << 4) | (can_bus_msg.data[4 * i + 2]);
		current_chassis_electrical_variables_.pwm_val[WheelsAxle::REAR][i] = (float)(uint16_pwm_val) * conversion_factor_pwm_val_to_percentage_;
		current_chassis_electrical_variables_.current_val[WheelsAxle::REAR][i] = (float)(int16_current_val - ZERO_CURRENT_ADC_VAL) * conversion_factor_adc_val_to_current_;
		if ((can_bus_msg.data[4 * i + 3] & 0x08) == 0x08) {
			current_chassis_electrical_variables_.hall_sensors_status[WheelsAxle::REAR][i] = true;
		} else {
			current_chassis_electrical_variables_.hall_sensors_status[WheelsAxle::REAR][i] = false;
		}
		if ((can_bus_msg.data[4 * i + 3] & 0x04) == 0x04) {
			current_chassis_electrical_variables_.alm_status[WheelsAxle::REAR][i] = true;
		} else {
			current_chassis_electrical_variables_.alm_status[WheelsAxle::REAR][i] = false;
		}
		if ((can_bus_msg.data[4 * i + 3] & 0x02) == 0x02) {
			current_chassis_electrical_variables_.brk_state[WheelsAxle::REAR][i] = true;
		} else {
			current_chassis_electrical_variables_.brk_state[WheelsAxle::REAR][i] = false;
		}
		if ((can_bus_msg.data[4 * i + 3] & 0x01) == 0x01) {
			current_chassis_electrical_variables_.dir_state[WheelsAxle::REAR][i] = true;
		} else {
			current_chassis_electrical_variables_.dir_state[WheelsAxle::REAR][i] = false;
		}
	}
	flag_new_electrical_feedback_data_[FeedbackData::REAR_ELECTRICAL] = true;
}

void WheelsController::FrontWheelsElectricalCallback(const agrobit_msg::can_bus& can_bus_msg) {
	uint16_t uint16_pwm_val = 0;
	int16_t int16_current_val = 0;
	for (int i = 0; i < 2; i++) {
		uint16_pwm_val = ((can_bus_msg.data[4 * i] << 8) | can_bus_msg.data[4 * i + 1]);
		int16_current_val = ((can_bus_msg.data[4 * i + 3] & (0xF0)) << 4) | (can_bus_msg.data[4 * i + 2]);
		current_chassis_electrical_variables_.pwm_val[WheelsAxle::FRONT][i] = (float)(uint16_pwm_val) * conversion_factor_pwm_val_to_percentage_;
		current_chassis_electrical_variables_.current_val[WheelsAxle::FRONT][i] = (float)(int16_current_val - ZERO_CURRENT_ADC_VAL) * conversion_factor_adc_val_to_current_;
		if ((can_bus_msg.data[4 * i + 3] & 0x08) == 0x08) {
			current_chassis_electrical_variables_.hall_sensors_status[WheelsAxle::FRONT][i] = true;
		} else {
			current_chassis_electrical_variables_.hall_sensors_status[WheelsAxle::FRONT][i] = false;
		}
		if ((can_bus_msg.data[4 * i + 3] & 0x04) == 0x04) {
			current_chassis_electrical_variables_.alm_status[WheelsAxle::FRONT][i] = true;
		} else {
			current_chassis_electrical_variables_.alm_status[WheelsAxle::FRONT][i] = false;
		}
		if ((can_bus_msg.data[4 * i + 3] & 0x02) == 0x02) {
			current_chassis_electrical_variables_.brk_state[WheelsAxle::FRONT][i] = true;
		} else {
			current_chassis_electrical_variables_.brk_state[WheelsAxle::FRONT][i] = false;
		}
		if ((can_bus_msg.data[4 * i + 3] & 0x01) == 0x01) {
			current_chassis_electrical_variables_.dir_state[WheelsAxle::FRONT][i] = true;
		} else {
			current_chassis_electrical_variables_.dir_state[WheelsAxle::FRONT][i] = false;
		}
	}
	flag_new_electrical_feedback_data_[FeedbackData::FRONT_ELECTRICAL] = true;
}

void WheelsController::PublishWheelsKinematicsData(uint8_t m_axle_number) {
	agrobit_msg::wheels_kinematics kinematics_data_for_pub;
	kinematics_data_for_pub.timestamp_ms = ros::Time::now().toNSec() / 1e6;
	kinematics_data_for_pub.left_target_speed = target_chassis_kinematics_variables_.wheels_speed[m_axle_number][WheelsSide::LEFT];
	kinematics_data_for_pub.left_current_speed = current_chassis_kinematics_variables_.wheels_speed[m_axle_number][WheelsSide::LEFT];
	kinematics_data_for_pub.left_encoder = current_chassis_kinematics_variables_.encoders_pulse_counter[m_axle_number][WheelsSide::LEFT];
	kinematics_data_for_pub.right_target_speed = target_chassis_kinematics_variables_.wheels_speed[m_axle_number][WheelsSide::RIGHT];
	kinematics_data_for_pub.rigth_current_speed = current_chassis_kinematics_variables_.wheels_speed[m_axle_number][WheelsSide::RIGHT];
	kinematics_data_for_pub.right_encoder = current_chassis_kinematics_variables_.encoders_pulse_counter[m_axle_number][WheelsSide::RIGHT];
	wheels_kinematics_data_publisher_[m_axle_number].publish(kinematics_data_for_pub);
}

void WheelsController::PublishWheelsElectricalData(uint8_t m_axle_number) {
	agrobit_msg::wheels_electrical electrical_data_for_pub;
	electrical_data_for_pub.timestamp_ms = ros::Time::now().toNSec() / 1e6;
	electrical_data_for_pub.pwm.resize(2);
	electrical_data_for_pub.current.resize(2);
	electrical_data_for_pub.hall_sensors_status.resize(2);
	electrical_data_for_pub.alm_status.resize(2);
	electrical_data_for_pub.brk_state.resize(2);
	electrical_data_for_pub.dir_state.resize(2);
	for (int i = 0; i < 2; i++) {
		electrical_data_for_pub.pwm[i] = current_chassis_electrical_variables_.pwm_val[m_axle_number][i];
		electrical_data_for_pub.current[i] = current_chassis_electrical_variables_.current_val[m_axle_number][i];
		electrical_data_for_pub.hall_sensors_status[i] = current_chassis_electrical_variables_.hall_sensors_status[m_axle_number][i];
		electrical_data_for_pub.alm_status[i] = current_chassis_electrical_variables_.alm_status[m_axle_number][i];
		electrical_data_for_pub.brk_state[i] = current_chassis_electrical_variables_.brk_state[m_axle_number][i];
		electrical_data_for_pub.dir_state[i] = current_chassis_electrical_variables_.dir_state[m_axle_number][i];
	}
	wheels_electrical_data_publisher_[m_axle_number].publish(electrical_data_for_pub);
}

void WheelsController::PublishWheelsControlData(uint8_t m_axle_number) {
	agrobit_msg::can_bus can_bus_msg_for_pub;
	int16_t target_speed_for_sending[4];
	target_speed_for_sending[WheelsSide::LEFT] = target_chassis_kinematics_variables_.wheels_speed[m_axle_number][WheelsSide::LEFT] * conversion_factor_speed_in_kmph_to_speed_in_ticks_;
	target_speed_for_sending[WheelsSide::RIGHT] = target_chassis_kinematics_variables_.wheels_speed[m_axle_number][WheelsSide::RIGHT] * conversion_factor_speed_in_kmph_to_speed_in_ticks_;
	target_speed_for_sending[2] = 0;
	target_speed_for_sending[3] = 0;
	FillInCanMsgWithFourInt16(&can_bus_msg_for_pub, target_speed_for_sending);
	wheels_control_publisher_[m_axle_number].publish(can_bus_msg_for_pub);
}

void WheelsController::Run() {
	ros::Rate rate(WHEELS_CONTROLLER_RATE_HZ);
	while(ros::ok()) {
		PublishWheelsKinematicsData(WheelsAxle::REAR);
		PublishWheelsControlData(WheelsAxle::REAR);
		for (int i = 0; i < 2; i++) {
			if (flag_new_electrical_feedback_data_[i]) {
				PublishWheelsElectricalData(i);
			}
		}

		rate.sleep();
		ros::spinOnce();
	}
}





