#pragma once

#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>

namespace gazebo {
class Mps;
}

class SubscriptionClient : public OpcUa::SubscriptionHandler
{
public:
	SubscriptionClient(gazebo::Mps *station_,
	                   void (gazebo::Mps::*callback_funk_)(void),
	                   std::shared_ptr<spdlog::logger> logger_)
	: station(station_), callback_funk(callback_funk_), logger(logger_)
	{
	}

	SubscriptionClient(gazebo::Mps *station_, void (gazebo::Mps::*callback_funk_)(void))
	: SubscriptionClient(station_, callback_funk_, nullptr)
	{
	}

	SubscriptionClient(gazebo::Mps *station_) : SubscriptionClient(station_, nullptr, nullptr)
	{
	}

	SubscriptionClient()
	{
	}

	~SubscriptionClient()
	{
		//delete mpsValue;
	}
	void
	set_callback_funk(void (gazebo::Mps::*callback_funk_)(void))
	{
		callback_funk = callback_funk_;
	}

	void
	set_station(gazebo::Mps *station_)
	{
		station = station_;
	}

protected:
	gazebo::Mps *station;
	void (gazebo::Mps::*callback_funk)(void);
	std::shared_ptr<spdlog::logger> logger;

	void
	DataChange(uint32_t              handle,
	           const OpcUa::Node &   node,
	           const OpcUa::Variant &val,
	           OpcUa::AttributeId    attr) override
	{
		if (logger != nullptr) {
			logger->info("Received DataChange event for Node {}", node);
			print_node_value(&node, val, logger);
		} else {
			std::cout << "Received DataChange event for Node " << node << std::endl;
			print_node_value(&node, val);
		}
		(station->*callback_funk)();
	};
	void
	print_node_value(const OpcUa::Node *             n,
	                 const OpcUa::Variant &          val,
	                 std::shared_ptr<spdlog::logger> logger = NULL)
	{
		if (n == NULL)
			return;
		OpcUa::QualifiedName qn = n->GetBrowseName();
		if (val.IsNul()) {
			if (logger == NULL)
				std::cout << "Name=" << qn.Name << ", Value=NULL" << std::endl;
			else
				logger->info("Name={0}, Value=NULL", qn.Name, *n);
		} else {
			std::string typeS;
			switch (val.Type()) {
			case OpcUa::VariantType::BOOLEAN: typeS = "BOOLEAN"; break;
			case OpcUa::VariantType::STRING: typeS = "STRING"; break;
			case OpcUa::VariantType::QUALIFIED_NAME: typeS = "QUALIFIED_NAME"; break;
			case OpcUa::VariantType::LOCALIZED_TEXT: typeS = "LOCALIZED_TEXT"; break;
			case OpcUa::VariantType::BYTE: typeS = "BYTE"; break;
			case OpcUa::VariantType::UINT16: typeS = "UINT16"; break;
			case OpcUa::VariantType::UINT32: typeS = "UINT32"; break;
			case OpcUa::VariantType::UINT64: typeS = "UINT64"; break;
			case OpcUa::VariantType::INT16: typeS = "INT16"; break;
			case OpcUa::VariantType::INT32: typeS = "INT32"; break;
			case OpcUa::VariantType::INT64: typeS = "INT64"; break;
			case OpcUa::VariantType::DOUBLE: typeS = "DOUBLE"; break;
			default: typeS = "Other";
			}

			if (logger == NULL)
				std::cout << "Name=" << qn.Name << ", Value=" << val.ToString() << ", Type=" << typeS
				          << std::endl;
			else
				logger->info("Name={0}, Value is {1} ({2}),", qn.Name, val.ToString(), typeS);
		}
	}
};
