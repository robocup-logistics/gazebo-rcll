#pragma once
#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>

class SubscriptionClient : public OpcUa::SubscriptionHandler
{
public:
	SubscriptionClient(std::shared_ptr<spdlog::logger> logger_) : logger(logger_)
	{
	}

	SubscriptionClient() : SubscriptionClient(nullptr)
	{
	}

	~SubscriptionClient()
	{
		//delete mpsValue;
	}

protected:
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
