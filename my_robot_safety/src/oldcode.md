// file: limit_switch_watcher_node.cpp
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

using namespace std::chrono_literals;

class LimitSwitchWatcher : public rclcpp::Node
{
public:
  LimitSwitchWatcher()
  : Node("limit_switch_watcher")
  {
    // ---- Parameters ----
    // List of Bool topics to subscribe to; one per DI channel.
    input_topics_ = this->declare_parameter<std::vector<std::string>>(
      "input_topics",
      {
        // Example defaults for 10 channels; replace with your actual Phidgets DI topics.
        "/phidgets/di/0", "/phidgets/di/1",
        "/phidgets/di/2", "/phidgets/di/3",
        "/phidgets/di/4", "/phidgets/di/11",
        "/phidgets/di/12", "/phidgets/di/13",
        "/phidgets/di/14", "/phidgets/di/15"
      });

    normally_closed_  = this->declare_parameter<bool>("normally_closed", false); // NC recommended for safety
    log_each_message_ = this->declare_parameter<bool>("log_each_message", false);

    //std::vector<bool> tripped_now_;   // current logical state per channel (after NO/NC mapping)

    // controller_manager service & controller lists
    cm_switch_service_name_ = this->declare_parameter<std::string>(
      "controller_manager_switch_service",
      "/controller_manager/switch_controller");

    controllers_to_stop_ = this->declare_parameter<std::vector<std::string>>(
      "controllers_to_stop",
      std::vector<std::string>{"joint_trajectory_controller"});

    controllers_to_start_ = this->declare_parameter<std::vector<std::string>>(
      "controllers_to_start",
      std::vector<std::string>{/* e.g., "hold_position_controller" */});

    switch_strictness_ = this->declare_parameter<std::string>(
      "switch_strictness", "BEST_EFFORT"); // or "STRICT"

    // QoS (keep last few, reliable)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // ---- Publishers & services ----
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop", 1);

    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
  "reset_estop",
  [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
         std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!latched_estop_) {
      resp->success = true;
      resp->message = "E-stop already cleared.";
      return;
    }

    // RECOMPUTE actual state from per-channel values
    const bool any_tripped_now = std::any_of(
      tripped_now_.begin(), tripped_now_.end(), [](bool b){ return b; });

    if (any_tripped_now) {
      // Build a tiny list to help you see which one blocks reset
      std::string list;
      for (size_t i = 0; i < tripped_now_.size(); ++i)
        if (tripped_now_[i]) { if (!list.empty()) list += ", "; list += std::to_string(i); }
      resp->success = false;
      resp->message = "Cannot reset: one or more inputs still tripped. [" + list + "]";
      return;
    }

    // Clear latch and publish estop=false
    latched_estop_ = false;
    any_input_tripped_ = false;
    publishEstop(false);
    RCLCPP_WARN(this->get_logger(), "E-stop reset by service call.");

    // (optional) switch controllers back here if you added that earlier

    resp->success = true;
    resp->message = "E-stop reset.";
    });


    // controller_manager switch client
    cm_switch_client_ =
      this->create_client<controller_manager_msgs::srv::SwitchController>(cm_switch_service_name_);

    // Prepare per-channel state
    const int n = static_cast<int>(input_topics_.size());
    tripped_now_.assign(n, false);
    any_input_tripped_ = false;
    latched_estop_ = false;


    // Subscribe all inputs
    subscriptions_.reserve(n);
    for (int i = 0; i < n; ++i) {
      const std::string topic = input_topics_[i];
      // Capture index i for this subscription
      auto sub = this->create_subscription<std_msgs::msg::Bool>(
        topic, qos,
        [this, i, topic](std_msgs::msg::Bool::ConstSharedPtr msg)
        {
          this->handleInputMessage(i, topic, msg->data);
        });
      subscriptions_.push_back(sub);
      RCLCPP_INFO(get_logger(), "Watching input[%d]: %s", i, topic.c_str());
    }

    // Periodic heartbeat (and to log which channels are tripped)
    timer_ = this->create_wall_timer(500ms, [this]() { this->heartbeat(); });

    RCLCPP_INFO(get_logger(), "LimitSwitchWatcher node ready. Waiting for input...");
  }

private:

// Handle each Bool message; apply NC/NO logic, debounce, latch, and possibly halt controllers.
void handleInputMessage(int idx, const std::string & topic, bool raw_state)
{
  if (idx < 0 || static_cast<size_t>(idx) >= tripped_now_.size()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "handleInputMessage index %d OOB (size=%zu)",
                          idx, tripped_now_.size());
    return;
  }

  std::lock_guard<std::mutex> lk(mutex_);

  // Map raw -> logical-tripped per NO/NC
  const bool tripped = normally_closed_ ? (!raw_state) : raw_state;
  const bool was = tripped_now_[idx];

  tripped_now_[idx] = tripped;

  if (log_each_message_ && was != tripped) {
    RCLCPP_INFO(get_logger(), "Channel %d (%s) %s.",
                idx, topic.c_str(), tripped ? "TRIPPED" : "CLEARED");
  }

  // Recompute overall
  const bool any_now = std::any_of(tripped_now_.begin(), tripped_now_.end(),
                                   [](bool b){ return b; });

  // Latch on first transition to "any true"
  if (any_now && !latched_estop_) {
    latched_estop_ = true;
    any_input_tripped_ = true;
    publishEstop(true);
    RCLCPP_ERROR(this->get_logger(),
                 "Limit switch TRIPPED. Latching E-STOP and halting controllers.");
    haltControllers();  // async
  } else {
    any_input_tripped_ = any_now;  // keep this updated for reset logic
  }
}

  void publishEstop(bool state)
  {
    auto msg = std_msgs::msg::Bool();
    msg.data = state;
    estop_pub_->publish(msg);
  }

  // Call controller_manager to stop and/or start controllers
 void haltControllers()
{
  using controller_manager_msgs::srv::SwitchController;

  if (!cm_switch_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(get_logger(),
      "controller_manager switch service '%s' not available.",
      cm_switch_service_name_.c_str());
    return;
  }

  auto req = std::make_shared<SwitchController::Request>();
  req->deactivate_controllers = controllers_to_stop_;
  req->activate_controllers   = controllers_to_start_;
  req->strictness             = (switch_strictness_ == "STRICT") ? 2 : 1; // Jazzy enums
  req->activate_asap          = true;
  req->timeout.sec            = 0;
  req->timeout.nanosec        = 0;

  // Non-blocking: handle result when it arrives
  cm_switch_client_->async_send_request(
    req,
    [this](rclcpp::Client<SwitchController>::SharedFuture resp) {
      try {
        if (resp.get()->ok) {
          RCLCPP_WARN(this->get_logger(), "Controllers switched successfully after E-STOP.");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Controller switch failed during E-STOP: %s",
                       resp.get()->message.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "SwitchController response error: %s", e.what());
      }
    });
}

void heartbeat()
{
  std::lock_guard<std::mutex> lk(mutex_);
  if (latched_estop_) {
    std::string list;
    for (size_t i = 0; i < tripped_now_.size(); ++i) {
      if (tripped_now_[i]) {
        if (!list.empty()) list += ", ";
        list += std::to_string(i);
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "E-STOP latched. Tripped channels: [%s]", list.c_str());
  }
}

  // --- Params/state ---
  std::vector<std::string> input_topics_;

  bool normally_closed_{true};
  bool log_each_message_{false};
 
  // Current logical state per channel after NO/NC mapping
  std::vector<bool> tripped_now_;

  std::string cm_switch_service_name_;
  std::vector<std::string> controllers_to_stop_;
  std::vector<std::string> controllers_to_start_;
  std::string switch_strictness_;

  // Global state
  bool latched_estop_{false};
  bool any_input_tripped_{false};

  // ROS handles
  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr cm_switch_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Thread-safety
  std::mutex mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LimitSwitchWatcher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
