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
	get_endpoint(std::string name)
	{
		std::string p = "opc.tcp://localhost:";
		if (name == "C-BS")
			return p + "4840/";
		if (name == "C-CS1")
			return p + "4841/";
		if (name == "C-CS2")
			return p + "4842/";
		if (name == "C-RS1")
			return p + "4843/";
		if (name == "C-RS2")
			return p + "4844/";
		if (name == "C-DS")
			return p + "4845/";
		if (name == "M-BS")
			return p + "4850/";
		if (name == "M-CS1")
			return p + "4851/";
		if (name == "M-CS2")
			return p + "4852/";
		if (name == "M-RS1")
			return p + "4853/";
		if (name == "M-RS2")
			return p + "4854/";
		if (name == "M-DS")
			return p + "4855/";
		throw MachineTypeException();
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
