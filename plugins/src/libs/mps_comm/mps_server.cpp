/// @brief OPC UA Server main.
/// @license GNU LGPL
///
/// Distributed under the GNU LGPL License
/// (See accompanying file LICENSE or copy at
/// http://www.gnu.org/licenses/lgpl.html)
///
#include "mps_server.h"

using namespace OpcUa;

using namespace mps_comm;

OPCServer::OPCServer(const std::string &name, const std::string &type,
                     const std::string &ip, unsigned int port) {
  // First setup our server
  mps_name_ = name;
  std::string endpoint = std::string("opc.tcp://") + ip + std::string(":") +
                         std::to_string(port) + std::string("/");
  printf("starting OPC Server");
  printf(endpoint.c_str());
  logger_ = spdlog::stderr_color_mt("OPCserver" + name);
  server_ = std::make_shared<OpcUa::UaServer>(logger_);
  server_->SetEndpoint(endpoint);
  server_->SetServerURI(("urn://" + name + ".opcserver").c_str());
}

OPCServer::~OPCServer() {
  if (server_started_)
    server_->Stop();
  // delete server_;
}

void OPCServer::run_server() {
  try {
    server_->Start();
    // then register our server namespace and get its index in server
    // uint32_t idx =
    server_->RegisterNamespace("http://" + mps_name_);

    Node n_g = server_->GetObjectsNode()
                   .AddObject(2, "DeviceSet")
                   .AddObject(4, "CPX-E-CEC-C1-PN")
                   .AddObject(4, "Resources")
                   .AddObject(4, "Application")
                   .AddObject(3, "GlobalVars")
                   .AddObject(4, "G");

    Node n_basic = n_g.AddObject(4, "Basic");
    Node n_in = n_g.AddObject(4, "In");

    Node n_p;
    Node n_data;
    Node n_status;

    // populate BASIC node objects
    n_p = n_basic.AddObject(4, "p");
    n_p.AddVariable(4, "ActionId", Variant((uint16_t)0));
    n_p.AddVariable(4, "BarCode", Variant((uint32_t)0));
    n_p.AddVariable(4, "Error", Variant((uint8_t)0));
    n_p.AddVariable(4, "SlideCnt", Variant((uint16_t)0));

    n_data = n_p.AddObject(4, "Data");
    n_data.AddVariable(4, "payload1", Variant((uint16_t)0));
    n_data.AddVariable(4, "payload2", Variant((uint16_t)0));

    n_status = n_p.AddObject(4, "Status");
    n_status.AddVariable(4, "Busy", Variant(false));
    n_status.AddVariable(4, "Enable", Variant(false));
    n_status.AddVariable(4, "Error", Variant((uint8_t)0));
    n_status.AddVariable(4, "Ready", Variant(false));

    // populate IN node objects
    n_p = n_basic.AddObject(4, "p");
    n_p.AddVariable(4, "ActionId", Variant((uint16_t)0));
    n_p.AddVariable(4, "BarCode", Variant((uint32_t)0));
    n_p.AddVariable(4, "Error", Variant((uint8_t)0));
    n_p.AddVariable(4, "SlideCnt", Variant((uint16_t)0));

    n_data = n_p.AddObject(4, "Data");
    n_data.AddVariable(4, "payload1", Variant((uint16_t)0));
    n_data.AddVariable(4, "payload2", Variant((uint16_t)0));

    n_status = n_p.AddObject(4, "Status");
    n_status.AddVariable(4, "Busy", Variant(false));
    n_status.AddVariable(4, "Enable", Variant(false));
    n_status.AddVariable(4, "Error", Variant((uint8_t)0));
    n_status.AddVariable(4, "Ready", Variant(false));

    server_started_ = true;
  }

  catch (const std::exception &exc) {
    std::cout << "Starting Server Failed!" << exc.what() << std::endl;
  }
}

void OPCServer::print(Node start_node, std::string level) {
  for (Node node : start_node.GetChildren()) {
    std::string print_s = level + " {}";
    std::string i_s = level + "--";

    logger_->info(print_s.c_str(), node);
    logger_->info("Qualified name: {}", node.GetBrowseName());
    logger_->info("ValueType: {}", node.GetValue().Type());
    logger_->info("is Array: {}", node.GetValue().IsArray());
    print(node, i_s);
  }
}

class SubClient : public SubscriptionHandler {
  void DataChange(uint32_t handle, const Node &node, const Variant &val,
                  AttributeId attr) override {
    std::cout << "Received DataChange event for Node " << node << std::endl;
  }
};

// struct MPSDataNode {
// idx: registered server namespace index in server
//  MPSDataNode( Node parent, uint32_t idx ){
//    NodeId n_G("4:p",idx);
//    QualifiedName qn_G("4:p", idx);

//  }
//}

// void subscribe()
//{

//   SubClient clt1;
//  std::shared_ptr<Subscription> sub1 = server.CreateSubscription(100, clt1);
//  sub1->SubscribeDataChange(Ac1);

//  SubClient clt2;
//  std::shared_ptr<Subscription> sub2 = server.CreateSubscription(100, clt2);
//  sub2->SubscribeDataChange(Ac2);
//}
