#include "cult_node/cult_node.h"


CultivatorController::CultivatorController(){
  Init();
  cultivator_kinematics_feedback_subscriber_ = n_.subscribe("/can_cultivator_kinematics", 1, &CultivatorController::CultivatorKinematicsCallback, this);
  cultivator_status_feedback_subscriber_ = n_.subscribe("/can_cultivator_status", 1, &CultivatorController::CultivatorStatusCallback, this);

  target_values_subscriber_ = n_.subscribe("/target_values", 1, &CultivatorController::TargetValuesCallback, this);
  
  cultivator_parameters_publisher_ = n_.advertise<agrobit_msg::can_bus>("/cultivator_can", 1);
  cultivator_kinematics_publisher_ = n_.advertise<agrobit_msg::kinematics_msg>("/cultivator_kinematics", 1);
  cultivator_status_publisher_ = n_.advertise<agrobit_msg::status_msg>("/cultivator_status", 1);

  UpdateParameters();
}

void CultivatorController::UpdateParameters() {
  n_.param<int>("actuator_parameters/kp_right", current_actuators_parameters.kp_right, 45);
  n_.param<int>("actuator_parameters/kp_left", current_actuators_parameters.kp_left, 100);
  n_.param<int>("actuator_parameters/kp_right", target_actuators_parameters.kp_right, 45);
  n_.param<int>("actuator_parameters/kp_left", target_actuators_parameters.kp_left, 100);
  n_.param<int>("cultivator_parameters/pulses_per_rotation", current_cultivator_parameters.pulses_per_rotation, 590);
  n_.param<int>("cultivator_parameters/kp", current_cultivator_parameters.kp, 3);
  n_.param<int>("cultivator_parameters/ki", current_cultivator_parameters.ki, 5);
  n_.param<int>("cultivator_parameters/pulses_per_rotation", target_cultivator_parameters.pulses_per_rotation, 590);
  n_.param<int>("cultivator_parameters/kp", target_cultivator_parameters.kp, 3);
  n_.param<int>("cultivator_parameters/ki", target_cultivator_parameters.ki, 5);
}

void CultivatorController::CultivatorKinematicsCallback(const agrobit_msg::can_bus& can_bus) {
  agrobit_msg::kinematics_msg kinematics_msg;
  kinematics_msg.actuator_right_extenshion = can_bus.data[0];
  kinematics_msg.actuator_left_extenshion = can_bus.data[1];
  kinematics_msg.actuator_mid_point = can_bus.data[2];
  kinematics_msg.actuator_right_pwm = can_bus.data[3];
  kinematics_msg.actuator_left_pwm = can_bus.data[4];
  kinematics_msg.cultivator_speed = (int16_t)((can_bus.data[5] << 8 | can_bus.data[6]) / 590.0 * 60);
  kinematics_msg.cultivator_pwm = can_bus.data[7];
  current_cultivator_parameters.speed = (int16_t)((can_bus.data[5] << 8 | can_bus.data[6]) / 590.0 * 60);
  current_actuators_parameters.extension = can_bus.data[2];

  cultivator_kinematics_publisher_.publish(kinematics_msg);
}

void CultivatorController::CultivatorStatusCallback(const agrobit_msg::can_bus& can_bus) {

  agrobit_msg::status_msg status_msg;
  status_msg.Hall_sensors = (can_bus.data[0] & 0x01);
  status_msg.alarm = (can_bus.data[0] & 0x02);
  status_msg.rotation_dir = (can_bus.data[0] & 0x04);
  status_msg.brake = (can_bus.data[0] & 0x08);
  status_msg.potentiometer_ref_voltage = (can_bus.data[0] & 0x10);
  status_msg.actuator_right_nc = (can_bus.data[0] & 0x20);
  status_msg.actuator_left_nc =  (can_bus.data[0] & 0x40);
  status_msg.max_error = (can_bus.data[0] & 0x80);
  status_msg.current_value = (float) (((uint16_t) (can_bus.data[1] << 8 | can_bus.data[2]) - 1960) * 3.3 / 4095 * 10);

  cultivator_status_publisher_.publish(status_msg);

}

void CultivatorController::TargetValuesCallback(const agrobit_msg::target_values& msg) {
	target_cultivator_parameters.speed = msg.speed;
  target_actuators_parameters.extension = msg.extension;
}


void CultivatorController::Init() {

	target_actuators_parameters.extension = 1;
	current_actuators_parameters.extension = 1;
  target_cultivator_parameters.speed = 0;
  current_cultivator_parameters.speed = 0;
}

void CultivatorController::PublishCultivatorData() {
  agrobit_msg::can_bus can_bus;
  int16_t speed_in_pps = 0;
  speed_in_pps = target_cultivator_parameters.speed * 590 / 60;
	can_bus.timestamp_ms = ros::Time::now().toNSec() / 1e6;
  can_bus.data.resize(8);
  can_bus.data[0] = (uint8_t)(target_actuators_parameters.extension);
  can_bus.data[1] = (uint8_t)((speed_in_pps & 0xFF00) >> 8);
  can_bus.data[2] = (uint8_t)(speed_in_pps & 0x00FF);
  can_bus.data[3] = (uint8_t)(target_cultivator_parameters.kp);
  can_bus.data[4] = (uint8_t)(target_cultivator_parameters.ki);
  can_bus.data[5] = (uint8_t)(target_actuators_parameters.kp_left);
  can_bus.data[6] = (uint8_t)(target_actuators_parameters.kp_right);
  cultivator_parameters_publisher_.publish(can_bus);
}


void CultivatorController::Run() {
	ros::Rate rate(CULTIVATOR_CONTROLLER_RATE_HZ);
	ros::Time time1;
	ros::Time time2;
	while(ros::ok()) {
		time1 = ros::Time::now();
		printf("START: %.6f; cycle = %.6f\n", time1.toNSec() / 1e6, (time1 - time2).toNSec() / 1e6);
		printf("---\n");
		printf("Cultivator: spt = %i; spc = %i; Actuators: ext = %i; exc = %i \n", target_cultivator_parameters.speed,
		        current_cultivator_parameters.speed, target_actuators_parameters.extension, current_actuators_parameters.extension);
		printf("---\n");
		time2 = ros::Time::now();
		printf("END: %.6f; read_time = %.6f\n", time2.toNSec() / 1e6, (time2 - time1).toNSec() / 1e6);
		PublishCultivatorData();

		rate.sleep();
		ros::spinOnce();
	}
}
