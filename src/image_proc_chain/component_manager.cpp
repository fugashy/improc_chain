#include <algorithm>
#include <vector>
#include <string>
#include <functional>
#include <memory>

#include "image_proc_chain/component_manager.hpp"


using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


namespace {
using StringKeyStringValue = std::map<std::string, std::string>;
using IoParamType = std::map<std::string, StringKeyStringValue>;
IoParamType GetIoParameter() {
  return {
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
}

using composition_interfaces::srv::LoadNode;
using composition_interfaces::srv::UnloadNode;

struct LoadIoNodeQuery {
 public:
  std::shared_ptr<rmw_request_id_t> header;
  std::shared_ptr<LoadNode::Request> request;

  explicit LoadIoNodeQuery(const StringKeyStringValue& param)
      : header(new rmw_request_id_t()),
        request(new LoadNode::Request()) {
    request->package_name = param.at("package");
    request->plugin_name = param.at("plugin_name");
    request->node_namespace = param.at("namespace");
    request->node_name = param.at("name");;

    for (const auto& e : {"input_topic", "output_topic"}) {
      auto p = rcl_interfaces::msg::Parameter();
      p.name = e;
      p.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
      p.value.string_value = param.at(e);
      request->parameters.push_back(p);
    }

    request->log_level = static_cast<uint8_t>(20);  // INFO
  }
};


struct UnloadNodeQuery {
 public:
  explicit UnloadNodeQuery(const uint64_t id)
      : header(new rmw_request_id_t()),
        request(new UnloadNode::Request()) {
    request->unique_id = id;
  }

  std::shared_ptr<rmw_request_id_t> header;
  std::shared_ptr<UnloadNode::Request> request;
};


StringKeyStringValue GetIoFullNodeName() {
  const auto io_param = GetIoParameter();
  std::map<std::string, std::string> out;
  for (auto it = io_param.begin(); it != io_param.end(); ++it) {
    out.insert({
        it->first,
        it->second.at("namespace") + std::string("/") + it->second.at("name")
        });
  }
  return out;
}


ComponentIdByName Filter(const ComponentIdByName& in, const std::function<bool(const std::pair<std::string, uint64_t>&)>& filter_func) {
  ComponentIdByName out;
  std::copy_if(in.begin(), in.end(), std::inserter(out, out.end()), filter_func);
  return out;
}

}  // end of anonymous namespace


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
  const auto io_param = GetIoParameter();
  for (IoParamType::const_iterator it = io_param.begin(); it != io_param.end(); ++it) {
    const auto query = LoadIoNodeQuery(it->second);
    std::shared_ptr<LoadNode::Response> res(new LoadNode::Response());
    this->on_load_node(query.header, query.request, res);
    if (!res->success) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load %s", query.request->node_name.c_str());
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

  // 管理しているchainの数を得て，仕事する必要があるかどうかを判断していく
  const auto all_component_info = GetCurrentComponentInfo();;

  // IOに関する部分は無視するためにフィルター
  const ComponentIdByName component_info_without_io = Filter(
      all_component_info,
      [](const std::pair<std::string, uint64_t> pair) {
        const bool not_input = pair.first != GetIoFullNodeName().at("input");
        const bool not_output = pair.first != GetIoFullNodeName().at("output");
        return  not_input && not_output;
        });

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
  const uint32_t& base_idx = component_num;
  RCLCPP_DEBUG(this->get_logger(), "base num: %d", base_idx);
  const int difference_num = request->num - component_num;
  RCLCPP_DEBUG(this->get_logger(), "diff num: %d", difference_num);
  if (difference_num > 0) {
    const uint32_t try_num = difference_num;
    if (!this->ExtendLength(base_idx, try_num)) {
      response->successful = false;
      return;
    }
  } else if (difference_num < 0) {
    const uint32_t try_num = std::abs(difference_num);
    if (!this->ReduceLength(component_info_without_io, try_num)) {
      response->successful = false;
      return;
    }
  }

  // このまで処理が進んでいる場合は，処理器の数が変化しているので，
  // 最終的な出力ノードに接続するトピックも変わっている
  // それを合わせる必要がある
  const auto current_components = GetCurrentComponentInfo();
  // 出力のみを取り出し
  const auto output_component = Filter(
      current_components,
      [](const std::pair<std::string, uint64_t> pair) {
        return pair.first == GetIoFullNodeName().at("output");
        });
  // ここでエラーになられると，少なくとも出力トピックにメッセージが流れなくなるので対応が必要
  if (output_component.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find output node");
    response->successful = false;
    return;
  }
  // 出力ノードを一旦unloadして，入力トピックをリマップしたものをloadする
  const uint64_t& output_component_id = output_component.begin()->second;
  const auto unload_query = UnloadNodeQuery(output_component_id);
  std::shared_ptr<UnloadNode::Response> unload_response(new UnloadNode::Response());;
  this->on_unload_node(unload_query.header, unload_query.request, unload_response);
  StringKeyStringValue output_param = GetIoParameter().at("output");

  // following code block to be more cooler
  const int current_node_num = current_components.size() - 3;
  if (current_node_num >= 0) {
    output_param["input_topic"] = std::string("/image_proc_chain/pieces/no_") + std::to_string(current_node_num) + std::string("/image_out");
  } else {
    output_param["input_topic"] = std::string("/image_proc_chain/io/image_in");
  }
  const auto load_query = LoadIoNodeQuery(output_param);
  std::shared_ptr<LoadNode::Response> load_response(new LoadNode::Response());
  this->on_load_node(load_query.header, load_query.request, load_response);
  if (!load_response->success) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load %s", load_query.request->node_name.c_str());
    response->successful = false;
    throw std::runtime_error("Failed instantiate output");
  }

  response->successful = true;
}


ComponentIdByName ComponentManager::GetCurrentComponentInfo() {
  std::shared_ptr<rmw_request_id_t> lh;
  std::shared_ptr<ListNodes::Request> lreq(new ListNodes::Request());
  std::shared_ptr<ListNodes::Response> lres(new ListNodes::Response());
  this->on_list_nodes(lh, lreq, lres);
  const uint32_t all_component_num = static_cast<int>(lres->full_node_names.size());
  ComponentIdByName all_component_info;
  for (size_t i = 0; i < all_component_num; ++i) {
    RCLCPP_DEBUG(
        this->get_logger(),
        "[%ld]: %s 0x%lx",
        i,
        lres->full_node_names[i].c_str(),
        lres->unique_ids[i]);
    all_component_info.insert({lres->full_node_names[i], lres->unique_ids[i]});
  }

  return all_component_info;
}

bool ComponentManager::ExtendLength(const uint32_t base_idx, const uint32_t additional_length) {
  for (uint32_t i = 0; i < additional_length; ++i) {
    std::shared_ptr<rmw_request_id_t> h;
    std::shared_ptr<LoadNode::Request> req(new LoadNode::Request());
    std::shared_ptr<LoadNode::Response> res(new LoadNode::Response());
    req->package_name = "image_proc_chain";
    req->plugin_name = "image_proc_chain::ChainPiece";
    req->node_namespace = "/image_proc_chain/pieces";
    req->log_level = static_cast<uint8_t>(20);  // INFO
    // req->log_level = rcl_interfaces::msg::Log::INFO;
    const uint32_t abs_idx = base_idx + i;
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
    this->on_load_node(h, req, res);
    if (!res->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load %s", req->node_name.c_str());
      return false;
    }
  }
  return true;
}


bool ComponentManager::ReduceLength(
    const ComponentIdByName& component_info, const uint32_t reduced_length) {
  uint32_t tried_num = 0;
  for (
      auto it = component_info.rbegin();
      it != component_info.rend();
      ++it) {
    std::shared_ptr<UnloadNode::Response> res(new UnloadNode::Response());
    const auto query = UnloadNodeQuery(it->second);
    this->on_unload_node(query.header, query.request, res);
    if (!res->success) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to unload %s: 0x%lx: %s",
          it->first.c_str(),
          query.request->unique_id,
          res->error_message.c_str());
      return false;
    }
    RCLCPP_INFO(
        this->get_logger(),
        "Unload %s 0x%lx",
        it->first.c_str(),
        query.request->unique_id);
    if (++tried_num == reduced_length) {
      break;
    }
  }
  return true;
}

}  // end of namespace image_proc_chain
