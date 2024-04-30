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

  // instantiate IO node
  using IoParamType = std::map<std::string, std::map<std::string, std::string>>;
  const IoParamType io_param = {
    {
      "input", {
        {"package", "topic_tools"},
        {"plugin_name", "topic_tools::RelayNode"},
        {"namespace", "/image_proc_chain/io"},
        {"name", "input"},
        {"input_topic", "/image_proc_chain/io/image_in"},
        {"output_topic", "/image_proc_chain/pieces/no_0/image_in"}
      }
    },
    {
      "output", {
        {"package", "topic_tools"},
        {"plugin_name", "topic_tools::RelayNode"},
        {"namespace", "/image_proc_chain/io"},
        {"name", "output"},
        {"input_topic", "/image_proc_chain/pieces/no_0/image_in"},
        {"output_topic", "/image_proc_chain/io/image_out"}
      }
    }
  };
  for (IoParamType::const_iterator it = io_param.begin(); it != io_param.end(); ++it) {
    std::shared_ptr<rmw_request_id_t> h;
    std::shared_ptr<LoadNode::Request> req(new LoadNode::Request());
    std::shared_ptr<LoadNode::Response> res(new LoadNode::Response());
    req->package_name = it->second.at("package");
    req->plugin_name = it->second.at("plugin_name");
    req->node_namespace = it->second.at("namespace");
    req->node_name = it->second.at("name");;
    for (const auto& e : {"input_topic", "output_topic"}) {
      auto p = rcl_interfaces::msg::Parameter();
      p.name = e;
      p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      p.value.string_value = it->second.at(e);
      req->parameters.push_back(p);
    }
    req->log_level = static_cast<uint8_t>(20);  // INFO
    this->on_load_node(h, req, res);
    if (!res->success) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load %s", req->node_name.c_str());
      throw std::runtime_error("Failed instantiate IO");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Manager has initialized");
}

void ComponentManager::ChangeChainNum(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Request> request,
    std::shared_ptr<image_proc_chain_msgs::srv::ChangeChainNum::Response> response) {
  (void)request_header;
  RCLCPP_INFO(this->get_logger(), "Service has called with num: %d", request->num);

  // 管理しているchainの数を得て，仕事する必要があるかどうかを判断する
  std::shared_ptr<rmw_request_id_t> lh;
  std::shared_ptr<ListNodes::Request> lreq(new ListNodes::Request());
  std::shared_ptr<ListNodes::Response> lres(new ListNodes::Response());
  this->on_list_nodes(lh, lreq, lres);
  const uint32_t all_component_num = static_cast<int>(lres->full_node_names.size());
  using ComponentInfo = std::map<std::string, uint64_t>;
  ComponentInfo all_component_info;
  for (size_t i = 0; i < all_component_num; ++i) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "[%ld]: %s 0x%lx",
        i,
        lres->full_node_names[i].c_str(),
        lres->unique_ids[i]);
    all_component_info.insert({lres->full_node_names[i], lres->unique_ids[i]});
  }

  // IOに関する部分は無視するためにフィルター
  ComponentInfo component_info_without_io;
  std::copy_if(
      all_component_info.begin(),
      all_component_info.end(),
      std::inserter(component_info_without_io, component_info_without_io.end()),
      [](const std::pair<std::string, uint64_t> pair) {
        const bool not_input = pair.first != "/image_proc_chain/io/input";
        const bool not_output = pair.first != "/image_proc_chain/io/output";
        return  not_input && not_output; });
  for (
      ComponentInfo::const_iterator it = component_info_without_io.begin();
      it != component_info_without_io.end();
      ++it) {
    RCLCPP_INFO(
        this->get_logger(),
        "target component: %s 0x%lx",
        it->first.c_str(),
        it->second);
  }

  const uint32_t component_num = component_info_without_io.size();

  // 同じ数なら何もしなくて良い
  if (request->num == component_num) {
    RCLCPP_INFO(
        this->get_logger(),
        "Requested number is the same as current configuration. We ignore this request.");
    response->successful = true;
    return;
  }

  // 足りないなら増やすし，多い場合は減らす
  const uint32_t& base_num = component_num;
  RCLCPP_INFO(this->get_logger(), "base num: %d", base_num);
  const int difference_num = request->num - component_num;
  RCLCPP_INFO(this->get_logger(), "diff num: %d", difference_num);
  if (difference_num > 0) {
    const uint32_t try_num = difference_num;
    for (uint32_t i = 0; i < try_num; ++i) {
      std::shared_ptr<rmw_request_id_t> h;
      std::shared_ptr<LoadNode::Request> req(new LoadNode::Request());
      std::shared_ptr<LoadNode::Response> res(new LoadNode::Response());
      req->package_name = "image_proc_chain";
      req->plugin_name = "image_proc_chain::ChainPiece";
      req->node_namespace = "/image_proc_chain/pieces";
      req->log_level = static_cast<uint8_t>(20);  // INFO
      const uint32_t abs_idx = base_num + i;
      const std::string node_name = std::string("no_") + std::to_string(abs_idx);
      req->node_name = node_name;
      // 二個目以降の実体化の際は，以前の出力を入力とするようにトピックをリマップする
      if (abs_idx != 0) {
        const std::string prev_node_name = std::string("no_") + std::to_string(abs_idx-1);
        const std::string full_node_name = req->node_namespace + std::string("/") + node_name;
        const std::string full_prev_node_name = req->node_namespace + std::string("/") + prev_node_name;
        const std::string topic_name = full_node_name + "/image_in";
        const std::string prev_topic_name = full_prev_node_name + "/image_out";
        const std::string remap_rule = topic_name + std::string(":=") + prev_topic_name;
        RCLCPP_INFO(this->get_logger(), "remap rule-> %s", remap_rule.c_str());
        req->remap_rules.push_back(remap_rule);
      }
      // req->log_level = rcl_interfaces::msg::Log::INFO;
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
    for (
        auto it = component_info_without_io.rbegin();
        it != component_info_without_io.rend();
        ++it) {
      std::shared_ptr<rmw_request_id_t> h;
      std::shared_ptr<UnloadNode::Request> req(new UnloadNode::Request());
      req->unique_id = it->second;
      std::shared_ptr<UnloadNode::Response> res(new UnloadNode::Response());
      this->on_unload_node(h, req, res);
      if (!res->success) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Failed to unload %s: 0x%lx: %s",
            it->first.c_str(),
            req->unique_id,
            res->error_message.c_str());
        response->successful = res->success;
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Unload %s 0x%lx", it->first.c_str(), req->unique_id);
      if (++tried_num == try_num) {
        break;
      }
    }
  }

  // このまで処理が進んでいる場合は，処理器の数が変化しているので，
  // 最終的な出力ノードに接続するトピックも変わっている
  // それを合わせる必要がある

  response->successful = true;
}

}  // end of namespace image_proc_chain
