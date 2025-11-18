// file: limit_switch_watcher_node.cpp
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include <stdexcept>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <phidget22.h>

using namespace std::chrono_literals;

class LimitSwitchWatcher : public rclcpp::Node
{
public:
  LimitSwitchWatcher()
  : rclcpp::Node("limit_switch_watcher")
  {
    // --- before opening Phidget channels ---

    // Safer to read as int64 and cast
    int64_t hub_serial_i64 = this->declare_parameter<int64_t>("hub_serial", 0);
    int64_t daq_port_i64   = this->declare_parameter<int64_t>("hub_port", 0);

    // If you prefer keeping your members as int:
    hub_serial_ = static_cast<int>(hub_serial_i64);
    daq_port_   = static_cast<int>(daq_port_i64);

    // Read channels as int64_t vector, then convert
    std::vector<int64_t> channels_i64 = this->declare_parameter<std::vector<int64_t>>(
      "channels", std::vector<int64_t>{0,1,2,3,4,11,12,13,14,15});

    channels_.resize(channels_i64.size());
    std::transform(channels_i64.begin(), channels_i64.end(), channels_.begin(),
                  [](int64_t v){ return static_cast<int>(v); });

    // other params
    normally_closed_  = this->declare_parameter<bool>("normally_closed", false);
    log_each_message_ = this->declare_parameter<bool>("log_each_message", false);

    cm_switch_service_name_ = this->declare_parameter<std::string>(
      "controller_manager_switch_service", "/controller_manager/switch_controller");

    controllers_to_stop_ = this->declare_parameter<std::vector<std::string>>(
      "controllers_to_stop", std::vector<std::string>{"arm_joints_controller", "gripper_controller"});

    controllers_to_start_ = this->declare_parameter<std::vector<std::string>>(
      "controllers_to_start", std::vector<std::string>{"hold_position_controller"});

    switch_strictness_ = this->declare_parameter<std::string>("switch_strictness", "BEST_EFFORT");
    auto_restore_on_reset_ = this->declare_parameter<bool>("auto_restore_on_reset", true);

    // allocate per-channel state **before** creating subscriptions / opening devices
    tripped_now_.assign(channels_.size(), false);
    di_.resize(channels_.size(), nullptr);
    ctx_.resize(channels_.size(), nullptr);

    // ---------- Publishers / services / clients ----------
    estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop", rclcpp::QoS(1).reliable());

    reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "reset_estop",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        onReset(resp);
      });

    cm_switch_client_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      cm_switch_service_name_);

    // ---------- Open Phidget DI channels ----------
    const size_t n = channels_.size();
    tripped_now_.assign(n, false);
    di_.resize(n, nullptr);
    ctx_.resize(n, nullptr);

    for (size_t i = 0; i < n; ++i)
    {
      const int ch = channels_[i];
      PhidgetDigitalInputHandle h = nullptr;
      PTHROW(PhidgetDigitalInput_create(&h));

      PTHROW(Phidget_setDeviceSerialNumber((PhidgetHandle)h, hub_serial_));
      PTHROW(Phidget_setHubPort((PhidgetHandle)h, daq_port_));
      PTHROW(Phidget_setIsHubPortDevice((PhidgetHandle)h, 0)); // smart VINT device
      PTHROW(Phidget_setChannel((PhidgetHandle)h, ch));

      // allocate a tiny context (this + index)
      ctx_[i] = new Ctx{this, i};

      PTHROW(PhidgetDigitalInput_setOnStateChangeHandler(
        h,
        [](PhidgetDigitalInputHandle /*unused*/, void* ctx, int state){
          auto *c = static_cast<Ctx*>(ctx);
          c->self->onEdge(c->index, state);
        },
        ctx_[i]));

      // attach
      {
        const int timeout_ms = 5000;
        const auto rc = Phidget_openWaitForAttachment((PhidgetHandle)h, timeout_ms);
        if (rc != EPHIDGET_OK) {
          const char* err = nullptr; Phidget_getErrorDescription(rc, &err);
          RCLCPP_FATAL(get_logger_static(), "Failed to attach DAQ1301 DI channel %d: %s",
                       ch, err ? err : "(unknown)");
          throw std::runtime_error("Phidget attach failed");
        }
      }

      di_[i] = h;
      RCLCPP_INFO(get_logger(), "Attached DAQ1301 DI channel %d (idx %zu).", ch, i);
    }

    // ---------- Heartbeat ----------
    heartbeat_timer_ = this->create_wall_timer(5s, [this]() { heartbeat(); });

    RCLCPP_INFO(get_logger(), "LimitSwitchWatcher ready. Watching %zu DI channels on hub %d port %d.",
                channels_.size(), hub_serial_, daq_port_);
  }

  ~LimitSwitchWatcher() override
  {
    // Cleanly detach all handlers & handles
    for (size_t i = 0; i < di_.size(); ++i)
    {
      if (di_[i]) {
        PhidgetDigitalInput_setOnStateChangeHandler(di_[i], nullptr, nullptr);
        Phidget_close((PhidgetHandle)di_[i]);
        PhidgetDigitalInput_delete(&di_[i]);
      }
      if (ctx_[i]) { delete ctx_[i]; ctx_[i] = nullptr; }
    }
  }

private:
  // Context passed to Phidget callbacks
  struct Ctx { LimitSwitchWatcher* self; size_t index; };

  // Helper to translate Phidget return codes to exceptions for brevity
  static void PTHROW(PhidgetReturnCode rc)
  {
    if (rc != EPHIDGET_OK) {
      const char* err = nullptr; Phidget_getErrorDescription(rc, &err);
      throw std::runtime_error(err ? err : "Phidget error");
    }
  }

  // Static logger getter used in destructor error messages (avoid use-after-destroy)
  static rclcpp::Logger get_logger_static()
  {
    return rclcpp::get_logger("limit_switch_watcher");
  }

  // Publish estop Bool
  void publishEstop(bool state)
  {
    std_msgs::msg::Bool msg; msg.data = state;
    estop_pub_->publish(msg);
  }

  // Called from the Phidget edge callback (in this process)
  void onEdge(size_t idx, int raw_state)
  {
    if (idx >= tripped_now_.size()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "Edge index %zu OOB (size=%zu)", idx, tripped_now_.size());
      return;
    }

    std::lock_guard<std::mutex> lk(mutex_);

    const bool logical = normally_closed_ ? (raw_state == 0) : (raw_state != 0);
    const bool was = tripped_now_[idx];
    tripped_now_[idx] = logical;

    if (log_each_message_ && was != logical) {
      RCLCPP_INFO(get_logger(), "Channel %zu (DI %d) %s.",
                  idx, channels_[idx], logical ? "TRIPPED" : "CLEARED");
    }

    const bool any_now = std::any_of(tripped_now_.begin(), tripped_now_.end(),
                                     [](bool b){ return b; });

    if (any_now && !latched_estop_) {
      latched_estop_ = true;
      any_input_tripped_ = true;
      publishEstop(true);
      RCLCPP_ERROR(get_logger(), "Limit switch TRIPPED. Latching E-STOP and halting controllers.");
      haltControllers();  // async
    } else {
      any_input_tripped_ = any_now;
    }
  }

  // Asynchronous controller switch (no temporary executor)
  void haltControllers()
  {
    using Switch = controller_manager_msgs::srv::SwitchController;

    if (!cm_switch_client_->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "Controller manager service '%s' not available.",
                   cm_switch_service_name_.c_str());
      return;
    }

    auto req = std::make_shared<Switch::Request>();
    req->deactivate_controllers = controllers_to_stop_;
    req->activate_controllers   = controllers_to_start_;
    req->strictness   = (switch_strictness_ == "STRICT") ? 2 : 1; // STRICT=2, BEST_EFFORT=1
    req->activate_asap = true;
    req->timeout.sec = 0; req->timeout.nanosec = 0;

    cm_switch_client_->async_send_request(req,
      [this](rclcpp::Client<Switch>::SharedFuture f){
        try {
          auto resp = f.get();
          if (resp->ok) {
            RCLCPP_WARN(this->get_logger(), "Controllers switched successfully after E-STOP.");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Controller switch failed during E-STOP: %s",
                         resp->message.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_ERROR(this->get_logger(), "SwitchController response error: %s", e.what());
        }
      });
  }

  // Reset handler: **poll** true device state to avoid stale latch
  void onReset(const std::shared_ptr<std_srvs::srv::Trigger::Response>& resp)
  {
    std::lock_guard<std::mutex> lk(mutex_);

    if (!latched_estop_) {
      resp->success = true;
      resp->message = "E-stop already cleared.";
      return;
    }

    // Poll each channel now
    bool any_tripped_now = false;
    std::string list;
    for (size_t i = 0; i < di_.size(); ++i) {
      int raw = 0;
      if (di_[i]) {
        PhidgetDigitalInput_getState(di_[i], &raw);
      }
      const bool logical = normally_closed_ ? (raw == 0) : (raw != 0);
      tripped_now_[i] = logical;
      if (logical) {
        if (!list.empty()) list += ", ";
        list += std::to_string(channels_[i]);
        any_tripped_now = true;
      }
    }

    if (any_tripped_now) {
      resp->success = false;
      resp->message = "Cannot reset: inputs still tripped [DI " + list + "]";
      return;
    }

    // Clear latch, publish estop=false
    latched_estop_ = false;
    any_input_tripped_ = false;
    publishEstop(false);
    RCLCPP_WARN(get_logger(), "E-stop reset; inputs verified via direct poll.");

    // Optionally restore controllers automatically
    if (auto_restore_on_reset_) {
      using Switch = controller_manager_msgs::srv::SwitchController;
      if (cm_switch_client_->wait_for_service(2s)) {
        auto req = std::make_shared<Switch::Request>();
        req->activate_controllers   = controllers_to_stop_;   // arm + gripper back on
        req->deactivate_controllers = controllers_to_start_;  // hold off
        req->strictness   = (switch_strictness_ == "STRICT") ? 2 : 1;
        req->activate_asap = true;
        cm_switch_client_->async_send_request(req);
      }
    }

    resp->success = true;
    resp->message = "E-stop reset.";
  }

  void heartbeat()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (!latched_estop_) return;

    std::string list;
    for (size_t i = 0; i < tripped_now_.size(); ++i) {
      if (tripped_now_[i]) {
        if (!list.empty()) list += ", ";
        list += std::to_string(channels_[i]); // report physical DI number
      }
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "E-STOP latched. Tripped channels: [DI %s]", list.c_str());
  }

  // ---------- Params ----------
  int hub_serial_{0};
  int daq_port_{0};
  std::vector<int> channels_;
  bool normally_closed_{false};
  bool log_each_message_{false};
  bool auto_restore_on_reset_{true};

  std::string cm_switch_service_name_;
  std::vector<std::string> controllers_to_stop_;
  std::vector<std::string> controllers_to_start_;
  std::string switch_strictness_;

  // ---------- State ----------
  std::vector<PhidgetDigitalInputHandle> di_;
  std::vector<Ctx*> ctx_;                 // one ctx per channel
  std::vector<bool> tripped_now_;         // logical state per channel (after NO/NC mapping)

  bool latched_estop_{false};
  bool any_input_tripped_{false};

  // ---------- ROS handles ----------
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr cm_switch_client_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // ---------- Thread-safety ----------
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
