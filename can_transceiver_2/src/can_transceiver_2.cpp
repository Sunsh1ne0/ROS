#include "can_transceiver_2/can_transceiver_2.h"

CanTransceiver::CanTransceiver() {
	InitECAN();

	ecan_status_publisher_ = n_.advertise<agrobit_msg::ecan_e01s_status>("/ecan_e01s_status", 1);
	wheels_kinematics_feedback_publisher_[WheelsAxle::REAR] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/rear/kinematics_feedback", 1);
	wheels_kinematics_feedback_publisher_[WheelsAxle::FRONT] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/front/kinematics_feedback", 1);
	wheels_electrical_feedback_publisher_[WheelsAxle::REAR] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/rear/electrical_feedback", 1);
	wheels_electrical_feedback_publisher_[WheelsAxle::FRONT] = n_.advertise<agrobit_msg::can_bus>("/wheels/can/front/electrical_feedback", 1);
	first_sensors_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/status_first_can", 1);
	second_sensors_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/status_second_can", 1);
	third_sensors_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/status_third_can", 1);
	first_IMU_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/IMU_first", 1);
	second_IMU_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/IMU_second", 1);
	cult_kinematic_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/can_cultivator_kinematics", 1);
	cult_status_data_publisher_ = n_.advertise<agrobit_msg::can_bus>("/can_cultivator_status", 1);
	
	wheels_pid_controller_params_subscriber_[WheelsAxle::REAR] = n_.subscribe("/wheels/can/rear/pid_controller_params", 1, &CanTransceiver::RearWheelsSettingsCallback, this);
	wheels_pid_controller_params_subscriber_[WheelsAxle::FRONT] = n_.subscribe("/wheels/can/front/pid_controller_params", 1, &CanTransceiver::FrontWheelsSettingsCallback, this);
	wheels_control_data_subscriber_[WheelsAxle::REAR] = n_.subscribe("/wheels/can/rear/control_data", 1, &CanTransceiver::RearWheelsControlCallback, this);
	wheels_control_data_subscriber_[WheelsAxle::FRONT] = n_.subscribe("/wheels/can/front/control_data", 1, &CanTransceiver::FrontWheelsControlCallback, this);
	cultivator_can_subscriber_ = n_.subscribe("/cultivator_can", 1, &CanTransceiver::CultivatorCallback, this);

}

void CanTransceiver::RearWheelsControlCallback(const agrobit_msg::can_bus& can_bus_msg) {
	for (int i = 0; i < 8; i++) {
		new_data_for_sending_[CanBusIdentifiers::RearWheelsControl][i] = can_bus_msg.data[i];
	}
	flag_of_new_message_for_sending_[CanBusIdentifiers::RearWheelsControl] = true;
}

void CanTransceiver::FrontWheelsControlCallback(const agrobit_msg::can_bus& can_bus_msg) {
	for (int i = 0; i < 8; i++) {
		new_data_for_sending_[CanBusIdentifiers::FrontWheelsControl][i] = can_bus_msg.data[i];
	}
	flag_of_new_message_for_sending_[CanBusIdentifiers::FrontWheelsControl] = true;
}

void CanTransceiver::CultivatorCallback(const agrobit_msg::can_bus& can_bus_msg) {
	int index = 0;
	for (int j = 0; j < NUMBER_OF_CAN_BUS_IDENTIFIERS; j++)
	{
		if (GetEnumValue(j) == CanBusIdentifiers::CultControlSignals)
			index = j;
	}
	for (int i = 0; i < 8; i++) {
		// new_data_for_sending_[CanBusIdentifiers::CultControlSignals] [i] = can_bus_msg.data[i];
		new_data_for_sending_[index] [i] = can_bus_msg.data[i];
	}
	// flag_of_new_message_for_sending_[CanBusIdentifiers::CultControlSignals] = true;
	flag_of_new_message_for_sending_[index] = true;
}

void CanTransceiver::RearWheelsSettingsCallback(const agrobit_msg::can_bus& can_bus_msg) {
	for (int i = 0; i < 8; i++) {
		new_data_for_sending_[CanBusIdentifiers::RearWheelsSettings][i] = can_bus_msg.data[i];
	}
	flag_of_new_message_for_sending_[CanBusIdentifiers::RearWheelsSettings] = true;
}

void CanTransceiver::FrontWheelsSettingsCallback(const agrobit_msg::can_bus& can_bus_msg) {
	for (int i = 0; i < 8; i++) {
		new_data_for_sending_[CanBusIdentifiers::FrontWheelsSettings][i] = can_bus_msg.data[i];
	}
	flag_of_new_message_for_sending_[CanBusIdentifiers::FrontWheelsSettings] = true;
}

void CanTransceiver::InitECAN() {
	tv_.tv_sec = 0;
	tv_.tv_usec = ECAN_TIMEOUT_IN_USEC;
	serv_addr_.sin_family = AF_INET;
	serv_addr_.sin_port = htons(ECAN_PORT_NUMBER);
	if ((client_fd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		ROS_ERROR("ECAN: Socket creation error!");
		socket_creation_status_ = false;
	} else {
		socket_creation_status_ = true;
	}
	setsockopt(client_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv_, sizeof(struct timeval));
	setsockopt(client_fd_, SOL_SOCKET, SO_SNDTIMEO, &tv_, sizeof(struct timeval));
	if (inet_pton(AF_INET, ECAN_HOST_IP_ADDRESS, &serv_addr_.sin_addr) <= 0) {
		ROS_ERROR("ECAN: Invalid address / Address not supported!");
		network_address_status_ = false;
	} else {
		network_address_status_ = true;
	}
	
	if ((ecan_status_ = connect(client_fd_, (struct sockaddr*)&serv_addr_, sizeof(serv_addr_))) < 0) {
		ROS_ERROR("ECAN: Connection Failed!");
		ecan_connection_status_ = false;
	} else {
		ecan_connection_status_ = true;
	}
}

void CanTransceiver::Run() {
	ros::Rate rate(CAN_TRANSCEIVER_RATE_HZ);
	uint64_t global_counter = 0;
	while(ros::ok()) {
		global_counter++;
		ReadData();
		for (int i = 0; i < NUMBER_OF_CAN_BUS_IDENTIFIERS; i++) {
			if (flag_of_new_message_for_sending_[i]) {
				SendData(GetEnumValue(i), new_data_for_sending_[i]);
				flag_of_new_message_for_sending_[i] = false;
			}
		}
		if ((global_counter % CAN_TRANSCEIVER_RATE_HZ) == 0) {
			PublishCanBusStatus();
		}
		rate.sleep();
		ros::spinOnce();
	}
}

void CanTransceiver::PublishData(uint8_t *m_can_data) {
	agrobit_msg::can_bus can_bus_msg_for_pub;
	can_bus_msg_for_pub.timestamp_ms = ros::Time::now().toNSec() / 1e6;
	can_bus_msg_for_pub.data.resize(8);
	for (int i = 0; i < 8; i++) {
		can_bus_msg_for_pub.data[i] = m_can_data[5 + i];
	}
	switch (m_can_data[4]) {
	case (CanBusIdentifiers::RearWheelsKinematicsFeedback):
		wheels_kinematics_feedback_publisher_[WheelsAxle::REAR].publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::FrontWheelsKinematicsFeedback):
		wheels_kinematics_feedback_publisher_[WheelsAxle::FRONT].publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::RearWheelsElectricalFeedback):
		wheels_electrical_feedback_publisher_[WheelsAxle::REAR].publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::FrontWheelsElectricalFeedback):
		wheels_electrical_feedback_publisher_[WheelsAxle::FRONT].publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::FirstSensorsFeedback):
		first_sensors_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::SecondSensorsFeedback):
		second_sensors_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::ThirdSensorsFeedback):
		third_sensors_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::FirstIMUFeedback):
		first_IMU_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::SecondIMUFeedback):
		second_IMU_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::CultKinematicData):
		cult_kinematic_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	case (CanBusIdentifiers::CultStatusData):
		cult_status_data_publisher_.publish(can_bus_msg_for_pub);
		break;
	default:
		ROS_WARN("Unknown Can Bus Identifier!");
		break;
	}
}

void CanTransceiver::ReadData() {
	uint8_t input_data[13] = {0, };
	bytes_number_for_reading_ = read(client_fd_, input_data, sizeof(input_data));
	while (bytes_number_for_reading_ == sizeof(input_data)) {
		PublishData(input_data);
		number_of_cycles_without_data_ = 0;
		bytes_number_for_reading_ = read(client_fd_, input_data, sizeof(input_data));
	}
	number_of_cycles_without_data_++;
	if (number_of_cycles_without_data_ > MAX_NUMBER_OF_CYCLES_WITHOUT_DATA) {
		error_reading_counter_++;
		number_of_cycles_without_data_ = 0;
	}
}

void CanTransceiver::SendData(uint8_t m_identifier, uint8_t *m_can_data) {
	output_data_[4] = m_identifier;
	for (int i = 0; i < 8; i++) {
		output_data_[5 + i] = m_can_data[i];
	}
	bytes_number_for_sending_ = send(client_fd_, output_data_, sizeof(output_data_), 0);
	if ((bytes_number_for_sending_ < sizeof(output_data_)) || (bytes_number_for_sending_ == -1)) {
		error_sending_counter_++;
	}
}

void CanTransceiver::PublishCanBusStatus() {
	agrobit_msg::ecan_e01s_status ecan_status_for_pub;
	ecan_status_for_pub.timestamp_ms = ros::Time::now().toNSec() / 1e6;
	ecan_status_for_pub.socket_descriptor = client_fd_;
	ecan_status_for_pub.socket_creation_status = socket_creation_status_;
	ecan_status_for_pub.network_address_status = network_address_status_;
	ecan_status_for_pub.ecan_connection_status = ecan_connection_status_;
	ecan_status_for_pub.error_reading_counter = error_reading_counter_;
	ecan_status_for_pub.error_sending_counter = error_sending_counter_;
	ecan_status_publisher_.publish(ecan_status_for_pub);
}

CanBusIdentifiers CanTransceiver::GetEnumValue(int index){
	switch (index) {
		case 0:
			return CanBusIdentifiers::RearWheelsSettings;
		case 1:
			return CanBusIdentifiers::RearWheelsControl;
		case 2:
			return CanBusIdentifiers::RearWheelsKinematicsFeedback;
		case 3:
			return CanBusIdentifiers::RearWheelsElectricalFeedback;
		case 4:
			return CanBusIdentifiers::FrontWheelsSettings;
		case 5:
			return CanBusIdentifiers::FrontWheelsControl;
		case 6:
			return CanBusIdentifiers::FrontWheelsKinematicsFeedback;
		case 7:
			return CanBusIdentifiers::FrontWheelsElectricalFeedback;
		case 8:
			return CanBusIdentifiers::SteeringSystemControl;
		case 9:
			return CanBusIdentifiers::SteeringSystemFeedback;
		case 10:
			return CanBusIdentifiers::FirstSensorsFeedback;
		case 11:
			return CanBusIdentifiers::SecondSensorsFeedback;
		case 12:
			return CanBusIdentifiers::ThirdSensorsFeedback;
		case 13:
			return CanBusIdentifiers::CultKinematicData;
		case 14:
			return CanBusIdentifiers::CultStatusData;
		case 15:
			return CanBusIdentifiers::CultControlSignals;
		case 16:
			return CanBusIdentifiers::FirstIMUFeedback;
		case 17:
			return CanBusIdentifiers::SecondIMUFeedback;
		default:
			NULL;
	}
}



