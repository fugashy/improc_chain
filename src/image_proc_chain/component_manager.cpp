#include <algorithm>
#include <vector>
#include <string>
#include <functional>
#include <memory>

#include "image_proc_chain/component_manager.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


namespace image_proc_chain {

ComponentManager::ComponentManager(
      std::weak_ptr<rclcpp::Executor> executor,
      std::string node_name,
      const rclcpp::NodeOptions& options)
    : rclcpp_components::ComponentManager(executor, std::move(node_name), options) {
  srv_ = this->create_service<image_proc_chain_msgs::srv::ChangeChainNum>(
      "~/change_length",
      std::bind(&ComponentManager::ChangeChainNum, this, _1, _2, _3));
  RCLCPP_INFO(this->get_logger(), "Manager has initialized");
}

void ComponentManager::ChangeChainNum(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Request> request,
    std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Response> response) {
  (void)request_header;
  RCLCPP_INFO(this->get_logger(), "Service has called with num: %d", request->num);

  // 管理しているchainの数を得て，仕事する必要があるかどうかを判断する
  std::vector<std::string> node_names;
  std::vector<uint64_t> unique_ids;
  uint32_t component_num = 0;
  std::shared_ptr<rmw_request_id_t> lh;
  std::shared_ptr<ListNodes::Request> lreq(new ListNodes::Request());
  std::shared_ptr<ListNodes::Response> lres(new ListNodes::Response());
  this->on_list_nodes(lh, lreq, lres);
  if (lres == nullptr) {
    RCLCPP_INFO(this->get_logger(), "No components are working");
    component_num = 0;
  } else {
    node_names = lres->full_node_names;
    unique_ids = lres->unique_ids;
    component_num = static_cast<int>(node_names.size());
  }

  // 同じ数なら何もしなくて良い
  if (request->num == component_num) {
    RCLCPP_INFO(
        this->get_logger(),
        "Requested number is the same as current configuration. We ignore this request.");
    response->successful = true;
    return;
  }

  for (uint32_t i = 0; i < component_num; ++i) {
    RCLCPP_DEBUG(this->get_logger(), "[%d]: %s 0x%lx", i, node_names[i].c_str(), unique_ids[i]);
  }

  // 足りないなら増やすし，多い場合は減らす
  const uint32_t& base_num = component_num;
  const int difference_num = request->num - component_num;
  if (difference_num > 0) {
    const uint32_t try_num = difference_num;
    for (uint32_t i = 0; i < try_num; ++i) {
      std::shared_ptr<rmw_request_id_t> h;
      std::shared_ptr<LoadNode::Request> req(new LoadNode::Request());
      std::shared_ptr<LoadNode::Response> res(new LoadNode::Response());
      req->package_name = "image_proc_chain";
      req->plugin_name = "image_proc_chain::ChainPiece";
      req->node_namespace = "/image_proc_chain/pieces";
      const uint32_t abs_idx = base_num + i;
      req->node_name = std::string("no_") + std::to_string(abs_idx);
      // req->log_level = rcl_interfaces::msg::Log::INFO;
      req->log_level = static_cast<uint8_t>(20);  // INFO
      this->on_load_node(h, req, res);
      if (!res->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load %s", req->node_name.c_str());
        response->successful = res->success;
        return;
      }
    }
  } else if (request->num < component_num) {
    const uint32_t try_num = std::abs(difference_num);
    uint32_t tried_num = 0;
    std::reverse(unique_ids.begin(), unique_ids.end());
    for (const auto& e : unique_ids) {
      std::shared_ptr<rmw_request_id_t> h;
      std::shared_ptr<UnloadNode::Request> req(new UnloadNode::Request());
      req->unique_id = e;
      std::shared_ptr<UnloadNode::Response> res(new UnloadNode::Response());
      this->on_unload_node(h, req, res);
      if (!res->success) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to unload id: 0x%lx: %s",
            req->unique_id,
            res->error_message.c_str());
        response->successful = res->success;
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Unload 0x%lx", req->unique_id);
      if (++tried_num == try_num) {
        break;
      }
    }
  }

  // validate the consitency of message chain

  response->successful = true;
}

}  // end of namespace image_proc_chain
