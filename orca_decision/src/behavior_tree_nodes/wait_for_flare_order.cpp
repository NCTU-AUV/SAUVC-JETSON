#include "orca_decision/behavior_tree_nodes.hpp"
#include <cctype>
#include <unordered_map>

WaitForFlareOrder::WaitForFlareOrder(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
{
}

BT::PortsList WaitForFlareOrder::providedPorts() {
    return {};
}

BT::NodeStatus WaitForFlareOrder::tick() {
    if (!ctx_) {
        config().blackboard->get("ctx", ctx_);
    }

    ctx_->current_action = name();
    ctx_->target_label = "";

    const std::string& order = ctx_->current_flare_order;
    if (order.empty()) {
        ctx_->debug_msg = "Waiting for flare order...";
        return BT::NodeStatus::RUNNING;
    }

    // Parse flare order string (e.g., "rby", "ryb", "byr")
    static const std::unordered_map<char, std::string> flare_map = {
        {'r', "red_flare"},
        {'b', "blue_flare"},
        {'y', "yellow_flare"}
    };

    std::vector<std::string> flare_order;
    for (char c : order) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        auto it = flare_map.find(c);
        if (it != flare_map.end()) {
            flare_order.push_back(it->second);
        } else {
            RCLCPP_WARN(ctx_->node->get_logger(),
                "WaitForFlareOrder: unknown flare char '%c'", c);
        }
    }

    // Set blackboard keys for each flare (up to 3)
    for (size_t i = 0; i < flare_order.size() && i < 3; ++i) {
        std::string key = "flare_" + std::to_string(i + 1);
        config().blackboard->set(key, flare_order[i]);
    }
    // Fill remaining with empty string
    for (size_t i = flare_order.size(); i < 3; ++i) {
        std::string key = "flare_" + std::to_string(i + 1);
        config().blackboard->set(key, std::string(""));
    }

    if (flare_order.empty()) {
        ctx_->debug_msg = "WaitForFlareOrder: invalid flare order";
        return BT::NodeStatus::FAILURE;
    }

    ctx_->debug_msg = "Flare order received: " + order;
    RCLCPP_INFO(ctx_->node->get_logger(),
        "WaitForFlareOrder: parsed order '%s' → [%s, %s, %s]",
        order.c_str(),
        flare_order.size() > 0 ? flare_order[0].c_str() : "",
        flare_order.size() > 1 ? flare_order[1].c_str() : "",
        flare_order.size() > 2 ? flare_order[2].c_str() : "");

    return BT::NodeStatus::SUCCESS;
}

void WaitForFlareOrder::halt() {
    // Nothing to clean up
}
