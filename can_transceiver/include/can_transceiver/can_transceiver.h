#ifndef CAN_TRANSCEIVER_H_
#define CAN_TRANSCEIVER_H_

#include <ros/ros.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include "agrobit_msg/can_bus.h"
#include "agrobit_msg/ecan_e01s_status.h"

#define CAN_TRANSCEIVER_RATE_HZ 100000

#define ECAN_PORT_NUMBER 8881
#define ECAN_HOST_IP_ADDRESS "192.168.3.35"
// #define ECAN_HOST_IP_ADDRESS "192.168.4.101"
#define ECAN_TIMEOUT_IN_USEC 10

#define NUMBER_OF_CAN_BUS_IDENTIFIERS 18

#define MAX_NUMBER_OF_CYCLES_WITHOUT_DATA 100

enum CanBusIdentifiers {RearWheelsSettings, RearWheelsControl, RearWheelsKinematicsFeedback, RearWheelsElectricalFeedback,		//0, 1, 2, 3
	                    FrontWheelsSettings, FrontWheelsControl, FrontWheelsKinematicsFeedback, FrontWheelsElectricalFeedback,	//4, 5, 6, 7
	                    SteeringSystemControl, SteeringSystemFeedback, FirstSensorsFeedback = 0x30,								//8, 9, 30,
						SecondSensorsFeedback, ThirdSensorsFeedback, CultKinematicData, 										//31, 32, 33,
						CultStatusData, CultControlSignals, FirstIMUFeedback, SecondIMUFeedback,								//34, 35, 36, 37
						};
enum WheelsSide {LEFT, RIGHT};	 //0, 1
enum WheelsAxle {REAR, FRONT}; //0, 1

class CanTransceiver {

public:
	CanTransceiver();
	void Run();
	void RearWheelsSettingsCallback(const agrobit_msg::can_bus& can_bus_msg);
	void FrontWheelsSettingsCallback(const agrobit_msg::can_bus& can_bus_msg);
	void RearWheelsControlCallback(const agrobit_msg::can_bus& can_bus_msg);
	void FrontWheelsControlCallback(const agrobit_msg::can_bus& can_bus_msg);
	void CultivatorCallback(const agrobit_msg::can_bus& can_bus_msg);
	CanBusIdentifiers GetEnumValue(int index);

private:
	void InitECAN();
	void ReadData();
	void SendData(uint8_t m_identifier, uint8_t *m_can_data);
	void PublishData(uint8_t *m_can_data);
	void PublishCanBusStatus();

	ros::NodeHandle n_;
	ros::Publisher ecan_status_publisher_;
	ros::Publisher wheels_kinematics_feedback_publisher_[2];
	ros::Publisher wheels_electrical_feedback_publisher_[2];
	ros::Subscriber wheels_pid_controller_params_subscriber_[2];
	ros::Subscriber wheels_control_data_subscriber_[2];
	ros::Subscriber cultivator_can_subscriber_;
	ros::Publisher first_sensors_data_publisher_;
	ros::Publisher second_sensors_data_publisher_;
	ros::Publisher third_sensors_data_publisher_;
	ros::Publisher first_IMU_data_publisher_;
	ros::Publisher second_IMU_data_publisher_;
	ros::Publisher cult_kinematic_data_publisher_;
	ros::Publisher cult_status_data_publisher_;


	int ecan_status_ = 0;
	int bytes_number_for_reading_ = 0;
	int bytes_number_for_sending_ = 0;
	int client_fd_ = 0;
	struct sockaddr_in serv_addr_;
	struct timeval tv_;
	bool socket_creation_status_ = false;
	bool network_address_status_ = false;
	bool ecan_connection_status_ = false;
	bool flag_of_new_message_for_sending_[NUMBER_OF_CAN_BUS_IDENTIFIERS] = {false, };
	uint8_t new_data_for_sending_[NUMBER_OF_CAN_BUS_IDENTIFIERS][8] = {{0, }, };
	uint8_t output_data_[13] = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint16_t error_sending_counter_ = 0;
	uint16_t error_reading_counter_ = 0;
	uint16_t number_of_cycles_without_data_ = 0;

};

#endif



