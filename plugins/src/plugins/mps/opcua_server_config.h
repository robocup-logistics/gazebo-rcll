#pragma once
#include <iostream>
#include <string>

enum Station {
	STATION_BASE     = 0,
	STATION_RING     = 1,
	STATION_CAP      = 2,
	STATION_DELIVERY = 3,
	STATION_STORAGE  = 4,
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
