#pragma once
#include <iostream>
#include <string>

enum Station {
	STATION_BASE     = 100,
	STATION_RING     = 200,
	STATION_CAP      = 300,
	STATION_DELIVERY = 400,
	STATION_STORAGE  = 500,
};

enum BaseColor {
	BASE_COLOR_RED    = 1,
	BASE_COLOR_BLACK  = 3,
	BASE_COLOR_SILVER = 2,
};

enum Operation {
	OPERATION_GET_BASE       = 1,
	OPERATION_WAIT_FOR_BASES = 1,
	OPERATION_MOUNT_RING     = 3,
	OPERATION_CAP_ACTION     = 1,
	OPERATION_CAP_RETRIEVE   = 1,
	OPERATION_CAP_MOUNT      = 2,
	OPERATION_DELIVER        = 1,
	OPERATION_GET_F_PRODUCT  = 1,
	OPERATION_RETRIEVE       = 30,
	OPERATION_STORE          = 40,
	OPERATION_RELOCATE       = 50,
	OPERATION_MOVE_CONVEYOR  = 2,
};

enum Command {
	COMMAND_NOTHING       = 0,
	COMMAND_SET_TYPE      = 10,
	COMMAND_RESET         = 0,
	COMMAND_MOVE_CONVEYOR = 2,
};

struct MachineTypeException : std::exception
{
	const char *
	what() const throw()
	{
		return "Unknown machine type";
	}
};

class OpcUaConfig
{
public:
	static std::string
	get_endpoint(Station station)
	{
		std::string p = "opc.tcp://localhost:";
		switch (station) {
		case STATION_BASE: return p + "4840/";
		case STATION_RING: return p + "4841/";
		case STATION_CAP: return p + "4842/";
		case STATION_DELIVERY: return p + "4843/";
		case STATION_STORAGE: return p + "4844/";
		default: throw MachineTypeException();
		}
	};

	static std::string
	get_URI(Station station)
	{
		std::string p = "urn://ll.robocup.org/gazebo/";
		switch (station) {
		case STATION_BASE: return p + "base";
		case STATION_RING: return p + "ring";
		case STATION_CAP: return p + "cap";
		case STATION_DELIVERY: return p + "delivery";
		case STATION_STORAGE: return p + "storage";
		default: throw MachineTypeException();
		}
	}
};
