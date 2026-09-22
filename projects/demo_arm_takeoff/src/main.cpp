// ============================================================================
// demo_arm_takeoff (simple version)
//
// What it does, all by itself:
//   1. wait until the wrapper starts sending telemetry
//   2. ask for control authority
//   3. take off   (the takeoff command arms the drone by itself -- we never
//                  call turn_on_motors, so nothing arms on the ground)
//   4. CLIMB: fly ourselves up or down to the altitude we actually wanted
//   5. hold there for 60 seconds
//   6. land, then give control authority back
//
// WHY STEP 4 EXISTS
// -----------------
// The `takeoff` service takes no arguments, and on this setup the automatic
// takeoff always stops at about 1.18 m -- it is not a setting we can change.
// So the automatic takeoff only gets the drone off the ground; every metre
// after that is ours to fly. Asking for 5 m means WE climb roughly 3.8 m of it.
//
// We do that by sending velocity commands on
//     /wrapper/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate
// which is a sensor_msgs/Joy carrying
//     axes = [speed east, speed north, speed up, yaw rate]   (m/s, m/s, m/s, deg/s)
// in the GROUND frame. We only ever use the third axis -- go up, go down --
// and leave the other three at zero, so the drone stays where it is and only
// changes height.
//
// IMPORTANT: those commands only do something when PX4 is in OFFBOARD mode.
// The bridge switches to OFFBOARD by itself about 1.5 s after we start sending
// commands (when it is in project testing mode). We know it worked because
// display_mode becomes SDK_CTRL. If it never becomes SDK_CTRL, we give up on
// the altitude correction and just hold wherever the takeoff left us.
//
// WHY THE TIMER IS 20 Hz NOW
// --------------------------
// PX4 drops out of OFFBOARD if velocity commands stop arriving. 20 per second
// is the normal rate. So instead of counting seconds directly we count ticks,
// and secondsInStep() turns ticks back into seconds.
// ============================================================================

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <psdk_interfaces/msg/display_mode.hpp>
#include <psdk_interfaces/msg/flight_status.hpp>

using namespace std::chrono_literals;
using psdk_interfaces::msg::DisplayMode;
using psdk_interfaces::msg::FlightStatus;
using std_srvs::srv::Trigger;

// How many times per second tick() runs.
const double TICK_HZ = 20.0;

// The steps of the flight, in the order they happen.
enum Step
{
  WAIT_FOR_DATA,
  GET_AUTHORITY,
  TAKEOFF,
  CLIMB,
  HOLD,
  LAND,
  DONE
};

class SimpleArmTakeoff : public rclcpp::Node
{
public:
  SimpleArmTakeoff()
  : Node("demo_arm_takeoff")
  {
    // ---- settings you can change from the command line ----
    takeoff_altitude_   = declare_parameter<double>("takeoff_altitude", 1.8);
    hold_seconds_       = declare_parameter<double>("hold_seconds", 60.0);
    altitude_tolerance_ = declare_parameter<double>("altitude_tolerance", 0.3);
    min_takeoff_height_ = declare_parameter<double>("min_takeoff_height", 0.5);
    max_climb_speed_    = declare_parameter<double>("max_climb_speed", 1.0);
    climb_gain_         = declare_parameter<double>("climb_gain", 0.8);
    takeoff_timeout_    = declare_parameter<double>("takeoff_timeout", 60.0);
    offboard_timeout_   = declare_parameter<double>("offboard_timeout", 15.0);
    climb_timeout_      = declare_parameter<double>("climb_timeout", 60.0);

    // ---- listen to the drone ----
    // best_effort matches every publisher, reliable or not. A reliable
    // subscriber would silently receive nothing from a best-effort publisher.
    rclcpp::QoS qos(10);
    qos.best_effort();

    flight_status_sub_ = create_subscription<FlightStatus>(
      "/wrapper/psdk_ros2/flight_status", qos,
      [this](const FlightStatus::SharedPtr msg) {
        flight_status_ = msg->flight_status;
        got_telemetry_ = true;
      });

    display_mode_sub_ = create_subscription<DisplayMode>(
      "/wrapper/psdk_ros2/display_mode", qos,
      [this](const DisplayMode::SharedPtr msg) {
        display_mode_ = msg->display_mode;
      });

    height_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/wrapper/psdk_ros2/height_above_ground", qos,
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        height_ = msg->data;
      });

    // ---- how we steer the drone ----
    rclcpp::QoS command_qos(10);
    command_qos.reliable();
    velocity_pub_ = create_publisher<sensor_msgs::msg::Joy>(
      "/wrapper/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate", command_qos);

    // ---- commands we can send to the drone ----
    authority_client_ = create_client<Trigger>("/wrapper/psdk_ros2/obtain_ctrl_authority");
    release_client_   = create_client<Trigger>("/wrapper/psdk_ros2/release_ctrl_authority");
    takeoff_client_   = create_client<Trigger>("/wrapper/psdk_ros2/takeoff");
    land_client_      = create_client<Trigger>("/wrapper/psdk_ros2/land");

    // tick() runs 20 times a second and drives the whole sequence
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / TICK_HZ),
      std::bind(&SimpleArmTakeoff::tick, this));

    RCLCPP_INFO(
      get_logger(), "started: will climb to %.1f m, hold %.0f s, then land",
      takeoff_altitude_, hold_seconds_);
  }

private:
  // ------------------------------------------------------------------
  // small helpers
  // ------------------------------------------------------------------

  bool isArmed()  { return flight_status_ != FlightStatus::FLIGHT_STATUS_STOPED; }
  bool isFlying() { return flight_status_ == FlightStatus::FLIGHT_STATUS_ON_AIR; }

  // True when PX4 is in OFFBOARD, i.e. when our velocity commands are obeyed.
  bool weAreSteering()
  {
    return display_mode_ == DisplayMode::DISPLAY_MODE_NAVI_SDK_CTRL;
  }

  // True while PX4 is bringing the drone down on its own.
  bool isComingDown()
  {
    return display_mode_ == DisplayMode::DISPLAY_MODE_AUTO_LANDING ||
           display_mode_ == DisplayMode::DISPLAY_MODE_FORCE_AUTO_LANDING ||
           display_mode_ == DisplayMode::DISPLAY_MODE_NAVI_GO_HOME;
  }

  double secondsInStep() { return ticks_in_step_ / TICK_HZ; }

  const char * statusName()
  {
    if (flight_status_ == FlightStatus::FLIGHT_STATUS_STOPED)    return "STOPPED";
    if (flight_status_ == FlightStatus::FLIGHT_STATUS_ON_GROUND) return "ON_GROUND";
    if (flight_status_ == FlightStatus::FLIGHT_STATUS_ON_AIR)    return "ON_AIR";
    return "UNKNOWN";
  }

  const char * stepName()
  {
    switch (step_) {
      case WAIT_FOR_DATA:  return "WAIT_FOR_DATA";
      case GET_AUTHORITY:  return "GET_AUTHORITY";
      case TAKEOFF:        return "TAKEOFF";
      case CLIMB:          return "CLIMB";
      case HOLD:           return "HOLD";
      case LAND:           return "LAND";
      case DONE:           return "DONE";
    }
    return "?";
  }

  // Move to the next step and reset the step timer.
  void goToStep(Step next)
  {
    step_ = next;
    ticks_in_step_ = 0;
    settled_ticks_ = 0;
  }

  // Send one velocity command: east, north and up, in metres per second.
  // We keep east and north at zero, so the drone only goes up or down.
  void publishVerticalSpeed(double speed_up)
  {
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.axes = {0.0f, 0.0f, static_cast<float>(speed_up), 0.0f};
    velocity_pub_->publish(joy);
  }

  // Send a command and print whatever comes back.
  // We do NOT wait for the answer here -- waiting inside a timer callback
  // would freeze the node forever, because the answer arrives on this same
  // thread that would be stuck waiting.
  void sendCommand(rclcpp::Client<Trigger>::SharedPtr client, std::string name)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s is not available -- is the PSDK bridge running?",
                  name.c_str());
      return;
    }

    client->async_send_request(
      std::make_shared<Trigger::Request>(),
      [this, name](rclcpp::Client<Trigger>::SharedFuture future) {
        auto reply = future.get();
        if (reply->success) {
          RCLCPP_INFO(get_logger(), "%s: ok", name.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "%s: refused (%s)", name.c_str(), reply->message.c_str());
        }
      });
  }

  // ------------------------------------------------------------------
  // the flight sequence -- runs 20 times per second
  // ------------------------------------------------------------------
  void tick()
  {
    ticks_in_step_++;

    // print one status line per second, not twenty
    if (ticks_in_step_ % (int)TICK_HZ == 0) {
      RCLCPP_INFO(get_logger(), "[%s %.0fs] status=%s mode=%s height=%.1f m",
                  stepName(), secondsInStep(), statusName(),
                  weAreSteering() ? "SDK_CTRL" : "not-offboard", height_);
    }

    switch (step_) {

      // Wait until the wrapper actually publishes something.
      case WAIT_FOR_DATA:
        if (got_telemetry_) {
          RCLCPP_INFO(get_logger(), "telemetry is flowing, starting");
          goToStep(GET_AUTHORITY);
        } else if (ticks_in_step_ % 100 == 0) {
          RCLCPP_WARN(get_logger(), "still waiting for /wrapper/psdk_ros2/flight_status");
        }
        break;

      // A payload app must own control authority before it can command anything.
      case GET_AUTHORITY:
        if (ticks_in_step_ == 1) {
          sendCommand(authority_client_, "obtain_ctrl_authority");
        }
        if (secondsInStep() >= 2.0) {
          goToStep(TAKEOFF);
          sendCommand(takeoff_client_, "takeoff");
        }
        break;

      // Let PX4 do its own automatic takeoff. We send NO velocity commands
      // here on purpose: sending them would switch PX4 to OFFBOARD in the
      // middle of the climb and cut the takeoff short.
      //
      // The automatic takeoff stops at about 1.18 m, so do NOT wait for the
      // altitude we actually want -- that would never arrive. We only wait for
      // the drone to be safely off the ground and for PX4 to leave
      // AUTO_TAKEOFF, which is PX4 saying "the climb is finished".
      case TAKEOFF:
        if (isFlying() && height_ >= min_takeoff_height_ &&
            display_mode_ != DisplayMode::DISPLAY_MODE_AUTO_TAKEOFF &&
            secondsInStep() > 5.0)
        {
          RCLCPP_INFO(get_logger(),
                      "automatic takeoff finished at %.2f m, taking over to reach %.1f m",
                      height_, takeoff_altitude_);
          goToStep(CLIMB);
        } else if (secondsInStep() >= takeoff_timeout_) {
          RCLCPP_ERROR(get_logger(),
                       "only %.2f m after %.0f s, landing. Check the bridge console's "
                       "DASHBOARD preflight row for PX4's reason.",
                       height_, takeoff_timeout_);
          goToStep(LAND);
        } else if (!isArmed() && ticks_in_step_ % 100 == 0) {
          sendCommand(takeoff_client_, "takeoff");   // ask again, nothing is happening
        }
        break;

      // THE NEW PART. Send our own up/down speed until we are at the altitude
      // we actually asked for, then move on to HOLD.
      case CLIMB: {
        double error = takeoff_altitude_ - height_;   // + means we are too low

        // Speed is proportional to the error, so it slows down as it arrives,
        // and is capped so it never rushes.
        double speed_up = climb_gain_ * error;
        if (speed_up >  max_climb_speed_) { speed_up =  max_climb_speed_; }
        if (speed_up < -max_climb_speed_) { speed_up = -max_climb_speed_; }

        publishVerticalSpeed(speed_up);

        // Count how long we have been close enough. We want it to STAY there,
        // not just pass through, so we need 2 steady seconds.
        if (std::fabs(error) <= altitude_tolerance_) {
          settled_ticks_++;
        } else {
          settled_ticks_ = 0;
        }

        if (weAreSteering() && settled_ticks_ >= 2 * TICK_HZ) {
          RCLCPP_INFO(get_logger(), "reached %.1f m, holding", height_);
          goToStep(HOLD);
        } else if (!weAreSteering() && secondsInStep() >= offboard_timeout_) {
          // Our commands are being sent but PX4 never switched to OFFBOARD,
          // so nothing we send can change the altitude. Hold where we are.
          RCLCPP_WARN(get_logger(),
                      "PX4 never switched to SDK_CTRL after %.0f s, so the altitude "
                      "cannot be corrected. Turn on project testing mode on the bridge "
                      "console (key P). Holding at %.1f m instead.",
                      offboard_timeout_, height_);
          goToStep(HOLD);
        } else if (secondsInStep() >= climb_timeout_) {
          RCLCPP_WARN(get_logger(),
                      "could not settle at %.1f m after %.0f s (now %.1f m), holding anyway",
                      takeoff_altitude_, climb_timeout_, height_);
          goToStep(HOLD);
        }
        break;
      }

      // Sit still in the air, then land automatically. Zero speed on every
      // axis keeps PX4 in OFFBOARD -- if we stopped sending, it would drop out.
      case HOLD:
        publishVerticalSpeed(0.0);

        if (ticks_in_step_ == 1) {
          RCLCPP_INFO(get_logger(), "holding at %.1f m, landing in %.0f s",
                      height_, hold_seconds_);
        }
        if (secondsInStep() >= hold_seconds_) {
          RCLCPP_INFO(get_logger(), "hold time is up, landing now");
          goToStep(LAND);
        }
        break;

      // Ask to land, and keep asking every 5 s until the motors stop.
      // Notice we stop sending velocity commands here: if we kept sending
      // them, OFFBOARD would stay alive and fight the landing.
      case LAND:
        if (!isArmed() && secondsInStep() > 2.0) {
          RCLCPP_INFO(get_logger(), "landed and disarmed");
          sendCommand(release_client_, "release_ctrl_authority");
          goToStep(DONE);
        } else if (!isComingDown() && ticks_in_step_ % 100 == 1) {
          sendCommand(land_client_, "land");
        }
        break;

      case DONE:
        if (ticks_in_step_ == 1) {
          RCLCPP_INFO(get_logger(), "flight finished, nothing left to do");
          rclcpp::shutdown();
        }
        break;
    }
  }

  // ---- settings ----
  double takeoff_altitude_   = 1.8;
  double hold_seconds_       = 60.0;
  double altitude_tolerance_ = 0.3;
  double min_takeoff_height_ = 0.5;
  double max_climb_speed_    = 1.0;
  double climb_gain_         = 0.8;
  double takeoff_timeout_    = 60.0;
  double offboard_timeout_   = 15.0;
  double climb_timeout_      = 60.0;

  // ---- ROS stuff ----
  rclcpp::Subscription<FlightStatus>::SharedPtr flight_status_sub_;
  rclcpp::Subscription<DisplayMode>::SharedPtr display_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_pub_;
  rclcpp::Client<Trigger>::SharedPtr authority_client_;
  rclcpp::Client<Trigger>::SharedPtr release_client_;
  rclcpp::Client<Trigger>::SharedPtr takeoff_client_;
  rclcpp::Client<Trigger>::SharedPtr land_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---- what the drone is doing ----
  uint8_t flight_status_ = FlightStatus::FLIGHT_STATUS_STOPED;
  uint8_t display_mode_  = DisplayMode::DISPLAY_MODE_MANUAL_CTRL;
  float height_ = 0.0f;
  bool got_telemetry_ = false;

  // ---- where we are in the sequence ----
  Step step_ = WAIT_FOR_DATA;
  int ticks_in_step_ = 0;
  int settled_ticks_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleArmTakeoff>());
  rclcpp::shutdown();
  return 0;
}