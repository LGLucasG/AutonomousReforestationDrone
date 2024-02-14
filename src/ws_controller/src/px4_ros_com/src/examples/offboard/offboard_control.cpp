/****************************************************************************
 *
 * Modified 2024 by the Kariboo Project Team for drone reforestation purposes.
 * Original copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Enhancement: Extended the offboard control capabilities to allow the drone to
 * reach multiple predefined positions, beyond the initial takeoff position capability.
 * This modification supports the Kariboo project's goal of reforesting fields using drones.
 * Modification conducted by engineering students from ENSEIRB-MATMECA as part of a
 * collaborative effort to improve drone position control for environmental restoration tasks.
 *
 ****************************************************************************/

/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <GeographicLib/LocalCartesian.hpp>

#include <stdint.h>
#include <proj.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <array>

using namespace std;
using namespace chrono;
using namespace chrono_literals;
using namespace px4_msgs::msg;

#define MSG_QUEUE_SIZE 10
#define ACCURACY 0.2f
#define HEIGHT 5.0f

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", MSG_QUEUE_SIZE);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", MSG_QUEUE_SIZE);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", MSG_QUEUE_SIZE);
		// subscribe to the vehicle local position to compare with the target position
		vehicle_local_pos_listener_ = this->create_subscription<VehicleLocalPosition>("/fmu/out/vehicle_local_position", rclcpp::QoS(MSG_QUEUE_SIZE).best_effort().transient_local(), [this](const VehicleLocalPosition::UniquePtr msg)
																					  { this->current_local_pos = {msg->x, msg->y, msg->z}; });
		// subscribe to the vehicle global position to initialize the GPS to local converter
		vehicle_global_pos_listener_ = this->create_subscription<VehicleGlobalPosition>("/fmu/out/vehicle_global_position", rclcpp::QoS(MSG_QUEUE_SIZE).best_effort().transient_local(), [this](const VehicleGlobalPosition::UniquePtr msg)
																						{ this->current_gps = {msg->lat, msg->lon, msg->alt}; });
		offboard_setpoint_counter_ = 0;
		// read the csv file
		const char *homeDir = getenv("HOME");
		// find the ws_controller directory in homeDir recursively
		if (homeDir)
		{
			string command = "find " + string(homeDir) + " -type d -name 'ws_controller'";
			// store the result of the command in a string
			string result = "";
			FILE *pipe = popen(command.c_str(), "r");
			if (!pipe)
			{
				RCLCPP_ERROR(this->get_logger(), "popen failed");
			}
			char buffer[128];
			while (!feof(pipe))
			{
				if (fgets(buffer, 128, pipe) != NULL)
				{
					result += buffer;
				}
			}
			pclose(pipe);
			// remove the last character of the string
			result.pop_back();
			// store the path of the csv file
			string filePath = result + "/src/px4_ros_com/src/examples/offboard/bfp_gps.csv";
			// read bfp_gps.csv file
			ifstream file(filePath);
			// check if file is not empyt/not found
			if (!file)
			{
				RCLCPP_ERROR(this->get_logger(), "File not found");
			}
			string line;
			// skip the first line
			getline(file, line);
			while (getline(file, line))
			{
				istringstream ss(line);
				string token;
				array<double, 3> coords;
				int i = 0;
				while (getline(ss, token, ','))
				{
					coords[i] = stod(token);
					i++;
				}
				this->targets_from_csv.push_back(coords);
			}
			for (long unsigned int i = 0; i < this->targets_from_csv.size(); i++)
			{
				poses.push_back({targets_from_csv[i][0], this->targets_from_csv[i][1], -HEIGHT});
				RCLCPP_INFO(this->get_logger(), "UTM: %f, %f, %f", poses[i][0], poses[i][1], poses[i][2]);
			}
			RCLCPP_INFO(this->get_logger(), "___________________________________________________________________________");
			RCLCPP_INFO(this->get_logger(), "CSV File read");
			RCLCPP_INFO(this->get_logger(), "___________________________________________________________________________");
		}
		else
		{
			RCLCPP_ERROR(this->get_logger(), "HOME environment variable not found");
		}
		auto timer_callback = [this]() -> void
		{
			if (offboard_setpoint_counter_ == 10)
			{
				// Change to Offboard mode after 10 setpoints
				this->PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->Arm();
			}
			// offboard_control_mode needs to be paired with trajectory_setpoint
			PublishOffboardControlMode();
			this->FollowTrajectory(poses, ACCURACY);
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11)
			{
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}
	GeographicLib::LocalCartesian gpsToLocalConverter;
	long unsigned int index_pos = 0;
	array<float, 3> current_local_pos = {0, 0, 0};
	array<float, 3> current_global_pos = {0, 0, 0};
	array<double, 3> current_gps = {0, 0, 0};
	array<float, 3> current_target_pos = {0, 0, 0};
	array<double, 3> init_global_pos = {0, 0, 0};
	array<double, 3> orientation = {0, 0, 0};
	vector<array<double, 3>> poses = {{0, 0, -HEIGHT}};
	vector<array<double, 3>> targets_from_csv;
	void Arm();
	void FollowTrajectory(vector<array<double, 3>> poses_to_reach, float accuracy);
	void Disarm();
	void Land();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_pos_listener_;
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_pos_listener_;

	atomic<uint64_t> timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	void PublishOffboardControlMode();
	void PublishTrajectorySetpoint(array<float, 3> position);
	void PublishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	array<float, 3> convertDoubleToFloat(const array<double, 3> &input);
	float euclidian_distance(array<float, 3> array1, array<float, 3> array2);
	array<float, 3> matrixMultiply(const array<array<float, 3>, 3> &A, const array<float, 3> &B);
	tuple<double, double, double> convertGPSToLocal(double latitude, double longitude, double altitude);
	void initializeGPSToLocalConverter(double initialLatitude, double initialLongitude, double initialAltitude = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::Arm()
{
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::Disarm()
{
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

// land the drone
void OffboardControl::Land()
{
	PublishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0);
	RCLCPP_INFO(this->get_logger(), "Land command send");
}

// convert a double array to a float array
array<float, 3> OffboardControl::convertDoubleToFloat(const array<double, 3> &input)
{
	array<float, 3> output = {0, 0, 0};
	for (int i = 0; i < 3; ++i)
	{
		output[i] = static_cast<float>(input[i]);
	}
	return output;
}

float OffboardControl::euclidian_distance(array<float, 3> array1, array<float, 3> array2)
{
	array<float, 3> diff = {array1[0] - array2[0], array1[1] - array2[1], array1[2] - array2[2]};
	return std::sqrt(diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2]);
}

// initialize the GPS to local converter
void OffboardControl::initializeGPSToLocalConverter(double initialLatitude, double initialLongitude, double initialAltitude)
{
	this->gpsToLocalConverter.Reset(initialLatitude, initialLongitude, initialAltitude);
}

// convert GPS coordinates to local coordinates
tuple<double, double, double> OffboardControl::convertGPSToLocal(double latitude, double longitude, double altitude)
{
	double localX, localY, localZ;
	this->gpsToLocalConverter.Forward(latitude, longitude, altitude, localX, localY, localZ);
	return {localX, localY, localZ};
}

// multiply a 3*3 matrix by a 3*1 matrix
array<float, 3> OffboardControl::matrixMultiply(const array<array<float, 3>, 3> &A, const array<float, 3> &B)
{
	int rowsA = A.size();
	int colsA = A[0].size();

	// Initialise the result matrix with zeros.
	array<float, 3> C = {0, 0, 0};

	// compute the matrix multiplication
	for (int i = 0; i < rowsA; ++i)
	{
		for (int k = 0; k < colsA; ++k)
		{
			C[i] += A[i][k] * B[k];
		}
	}
	return C;
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::PublishOffboardControlMode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::PublishTrajectorySetpoint(array<float, 3> position)
{
	TrajectorySetpoint msg{};
	msg.position = position;
	msg.yaw = 0; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::PublishVehicleCommand(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

// Publish multiple setpoints to make the drone follow a trajectory
void OffboardControl::FollowTrajectory(vector<array<double, 3>> poses_to_reach, float accuracy)
{
	// first step
	if (this->index_pos == 0)
	{
		this->current_target_pos = {0, 0, -HEIGHT};
		if (this->init_global_pos[0] == 0 && this->init_global_pos[1] == 0 && abs(this->current_local_pos[2]) > HEIGHT/3)
		{
			RCLCPP_INFO(this->get_logger(), "-------------------------------------------------------------------------------------------------------------");
			RCLCPP_INFO(this->get_logger(), "INITIALISING POSITION");
			this->init_global_pos = {this->current_gps[0], this->current_gps[1], this->current_gps[2]};
			RCLCPP_INFO(this->get_logger(), "init_global_pos: %f, %f, %f", this->init_global_pos[0], this->init_global_pos[1], this->init_global_pos[2]);
			initializeGPSToLocalConverter(this->current_gps[0], this->current_gps[1], this->current_gps[2]);
		}
	}
	// last steps
	else if (this->index_pos == poses_to_reach.size())
	{
		this->current_target_pos = {0, 0, -HEIGHT};
	}
	else if (this->index_pos == poses_to_reach.size() + 1)
	{
		this->Land();
		this->index_pos++;
	}
	// all other steps
	else
	{
		tuple<double, double, double> local = convertGPSToLocal(poses_to_reach[this->index_pos][0], poses_to_reach[this->index_pos][1], poses_to_reach[this->index_pos][2]);
		// define a transform matrix 3*3 : -90 on Z and +180 on Y
		array<array<float, 3>, 3> transform = {{{0, 1, 0}, {1, 0, 0}, {0, 0, -1}}};
		// apply the transform matrix to the local position
		this->current_target_pos = {static_cast<float>(get<0>(local)), static_cast<float>(get<1>(local)), HEIGHT};
		this->current_target_pos = matrixMultiply(transform, this->current_target_pos);
	}
	if (this->index_pos <= poses_to_reach.size())
	{
		if (euclidian_distance(this->current_local_pos,this->current_target_pos) < accuracy)
		{
			this->index_pos++;
		}
		else
		{
			PublishOffboardControlMode();
			PublishTrajectorySetpoint(this->current_target_pos);
			RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------------------------------");
			RCLCPP_INFO(this->get_logger(), "index_pos  : %lu / %lu", this->index_pos, poses_to_reach.size());
			RCLCPP_INFO(this->get_logger(), "------------");
			RCLCPP_INFO(this->get_logger(), "gps_pos    : %f, %f, %f", this->current_gps[0], this->current_gps[1], this->current_gps[2]);
			RCLCPP_INFO(this->get_logger(), "gps_target : %f, %f, %f", this->targets_from_csv[this->index_pos][0], this->targets_from_csv[this->index_pos][1], this->targets_from_csv[this->index_pos][2]);
			RCLCPP_INFO(this->get_logger(), "------------");
			RCLCPP_INFO(this->get_logger(), "current_local_pos  : %f, %f, %f", this->current_local_pos[0], this->current_local_pos[1], this->current_local_pos[2]);
			RCLCPP_INFO(this->get_logger(), "current_target_pos : %f, %f, %f", this->current_target_pos[0], this->current_target_pos[1], this->current_target_pos[2]);
		}
	}
}

int main(int argc, char *argv[])
{
	cout << "Starting offboard control node..." << endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}