#include "orca_decision/behavior_tree_nodes.hpp"
#include <cctype>
#include <vector>
#include <unordered_map>

WaitForFlareOrder::WaitForFlareOrder(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList WaitForFlareOrder::providedPorts() {
    return {};
}

namespace {

std::vector<std::string> ParseFlareOrder(
    const std::string& order, rclcpp::Logger logger, const char* source_name)
{
    static const std::unordered_map<char, std::string> flare_map = {
        {'r', "red_flare"},
        {'b', "blue_flare"},
        {'y', "yellow_flare"}
    };

    std::vector<std::string> flare_order;
    flare_order.reserve(order.size());

    for (char c : order) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        const auto it = flare_map.find(c);
        if (it != flare_map.end()) {
            flare_order.push_back(it->second);
        } else {
            RCLCPP_WARN(
                logger,
                "WaitForFlareOrder: unknown flare char '%c' in %s order '%s'",
                c, source_name, order.c_str());
        }
    }

    return flare_order;
}

void SetFlareOrderBlackboard(
    const BT::NodeConfiguration& config, const std::vector<std::string>& flare_order)
{
    for (size_t i = 0; i < flare_order.size() && i < 3; ++i) {
        const std::string key = "flare_" + std::to_string(i + 1);
        config.blackboard->set(key, flare_order[i]);
    }

    for (size_t i = flare_order.size(); i < 3; ++i) {
        const std::string key = "flare_" + std::to_string(i + 1);
        config.blackboard->set(key, std::string(""));
    }
}

}  // namespace

BT::NodeStatus WaitForFlareOrder::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label = "";

    const rclcpp::Time now = ctx_->node->now();
    if (!waiting_started_) {
        waiting_started_ = true;
        wait_start_time_ = now;
    }

    const double timeout_sec =
        ctx_->node->get_parameter("wait_for_flare_order_timeout_sec").as_double();
    const std::string default_order =
        ctx_->node->get_parameter("wait_for_flare_order_default_order").as_string();

    std::string order = ctx_->current_flare_order;
    bool using_default_order = false;

    if (order.empty()) {
        const double elapsed = (now - wait_start_time_).seconds();
        if (timeout_sec > 0.0 && elapsed >= timeout_sec) {
            order = default_order;
            using_default_order = true;
        } else {
            ctx_->debug_msg = "Waiting for flare order...";
            return BT::NodeStatus::RUNNING;
        }
    }

    const char* order_source = using_default_order ? "default" : "received";
    const std::vector<std::string> flare_order =
        ParseFlareOrder(order, ctx_->node->get_logger(), order_source);

    if (flare_order.empty()) {
        ctx_->debug_msg = using_default_order
            ? "WaitForFlareOrder: invalid default flare order"
            : "WaitForFlareOrder: invalid flare order";
        return BT::NodeStatus::FAILURE;
    }

    SetFlareOrderBlackboard(config(), flare_order);

    waiting_started_ = false;

    if (using_default_order) {
        ctx_->debug_msg = "Flare order timeout reached, using default: " + order;
        RCLCPP_WARN(ctx_->node->get_logger(),
            "WaitForFlareOrder: timeout after %.2f s, using default order '%s' -> [%s, %s, %s]",
            timeout_sec,
            order.c_str(),
            flare_order.size() > 0 ? flare_order[0].c_str() : "",
            flare_order.size() > 1 ? flare_order[1].c_str() : "",
            flare_order.size() > 2 ? flare_order[2].c_str() : "");
        return BT::NodeStatus::SUCCESS;
    }

    ctx_->debug_msg = "Flare order received: " + order;
    RCLCPP_INFO(ctx_->node->get_logger(),
        "WaitForFlareOrder: parsed %s order '%s' -> [%s, %s, %s]",
        order_source,
        order.c_str(),
        flare_order.size() > 0 ? flare_order[0].c_str() : "",
        flare_order.size() > 1 ? flare_order[1].c_str() : "",
        flare_order.size() > 2 ? flare_order[2].c_str() : "");

    return BT::NodeStatus::SUCCESS;
}

void WaitForFlareOrder::halt() {
    waiting_started_ = false;
}
