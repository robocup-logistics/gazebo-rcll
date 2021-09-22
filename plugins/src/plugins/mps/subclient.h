#pragma once
#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>


class SubscriptionClient : public OpcUa::SubscriptionHandler
{
public:
  /*
  typedef std::function<void(OpcUtils::ReturnValue *)>           ReturnValueCallback;
  typedef std::pair<OpcUtils::MPSRegister, SubscriptionClient *> pair;
  typedef std::map<OpcUtils::MPSRegister, SubscriptionClient *>  map;
  OpcUa::Subscription::SharedPtr                                 subscription;
  OpcUtils::ReturnValue *                                        mpsValue = nullptr;
  uint32_t                                                       handle;
  OpcUtils::MPSRegister                                          reg;  
  OpcUa::Node                                                    node;

  SubscriptionClient(std::shared_ptr<spdlog::logger> logger_, OpcUtils::ReturnValue *mpsValue_)
  : mpsValue(mpsValue_), logger(logger_)
  {
  }
  */
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
  /*
  void	add_callback(ReturnValueCallback callback)
  {
    callbacks.push_back(callback);
  }
  */
protected:
  std::shared_ptr<spdlog::logger>  logger;
  //std::vector<ReturnValueCallback> callbacks;
  void	DataChange(uint32_t              handle,
	           const OpcUa::Node &   node,
	           const OpcUa::Variant &val,
	           OpcUa::AttributeId    attr) override
  {
    if( logger != nullptr)
      logger->info("Received DataChange event for Node {}", node);
    else 
      std::cout << "Received DataChange event for Node " << node << std::endl;
  }
};
