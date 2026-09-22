// ============================================================================
// demo_orbit_mission (simple version)
//
// What it does, all by itself:
//   1. wait until the wrapper starts sending telemetry
//   2. ask for control authority
//   3. take off   (the takeoff command arms the drone by itself -- we never
//                  call turn_on_motors, so nothing arms on the ground)
//   4. CLIMB: fly ourselves up to the altitude we actually wanted
//   5. ORBIT: fly a circle at that altitude
//   6. land, then give control authority back
//
// Steps 1 to 4 are copied from demo_arm_takeoff, on purpose and word for word.
// Read that one first; the only new part here is step 5.
//
// WHY STEP 4 EXISTS
// -----------------
// The `takeoff` service takes no arguments, and on this setup the automatic
// takeoff always stops at about 1.18 m -- it is not a setting we can change.
// So the automatic takeoff only gets the drone off the ground; every metre
// after that is ours to fly.
//
// HOW WE STEER
// ------------
// Everything after the automatic takeoff is sent on
//     /wrapper/psdk_ros2/flight_control_setpoint_ENUvelocity_yawrate
// which is a sensor_msgs/Joy carrying
//     axes = [speed east, speed north, speed up, yaw rate]   (m/s, m/s, m/s, deg/s)
// in the GROUND frame. East is +x, North is +y, Up is +z.
//
// The yaw rate really is in DEGREES per second, not radians. The psdk_ros2
// documentation table says radians; the bridge converter that actually reads
// these messages takes degrees (see psdk_enu_velocity_to_setpoint in
// gui/simty/converters.py). Believe the code.
//
// HOW THE CIRCLE IS FLOWN
// -----------------------
// Open loop, because it is easy to follow. We decide how long one lap should
// take, then every tick we work out which way "around the circle" points right
// now and fly in that direction at a constant speed:
//
//     one lap takes            lap_seconds
//     angle around the circle  a = 360 * (seconds so far / lap_seconds)
//     speed along the circle   v = 2 * pi * radius / lap_seconds
//     speed east               v * -sin(a)
//     speed north              v *  cos(a)
//
// The height is NOT open loop -- drifting up or down is not acceptable -- so
// the up axis keeps using the same climb controller as step 4.
//
// Being open loop sideways, wind or drift means the circle will not close
// perfectly. That is fine for a demo and it keeps the code short. Closing the
// loop would mean reading position_fused and correcting, which is a good next
// exercise.
//
// IMPORTANT: velocity commands only do something when PX4 is in OFFBOARD mode.
// The bridge switches to OFFBOARD about 1.5 s after we start sending commands.
// We know it worked because display_mode becomes SDK_CTRL. SDK_CTRL happens
// AFTER we start commanding, so it is not something to wait for before taking
// off.
//
// WHY THE TIMER IS 20 Hz
// ----------------------
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

const double TICK_HZ = 20.0;

enum Step
{
  WAIT_FOR_DATA,
  GET_AUTHORITY,
  TAKEOFF,
  CLIMB,
  ORBIT,
  LAND,
  DONE
};

class SimpleOrbitMission : public rclcpp::Node
{
public:
  SimpleOrbitMission()
  : Node("demo_orbit_mission")
  {
    // ---- settings you can change from the command line ----
    takeoff_altitude_   = declare_parameter<double>("takeoff_altitude", 10);
    radius_             = declare_parameter<double>("radius", 12.0);
    lap_seconds_        = declare_parameter<double>("lap_seconds", 20.0);
    laps_               = declare_parameter<double>("laps", 2.0);
    altitude_tolerance_ = declare_parameter<double>("altitude_tolerance", 0.3);
    min_takeoff_height_ = declare_parameter<double>("min_takeoff_height", 0.5);
    max_climb_speed_    = declare_parameter<double>("max_climb_speed", 1.0);
    climb_gain_         = declare_parameter<double>("climb_gain", 0.8);
    takeoff_timeout_    = declare_parameter<double>("takeoff_timeout", 60.0);
    offboard_timeout_   = declare_parameter<double>("offboard_timeout", 15.0);
    climb_timeout_      = declare_parameter<double>("climb_timeout", 60.0);

    // ---- listen to the drone ----
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

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / TICK_HZ),
      std::bind(&SimpleOrbitMission::tick, this));

    RCLCPP_INFO(
      get_logger(), "started: climb to %.1f m, then %.0f lap(s) of a %.1f m circle, %.0f s each",
      takeoff_altitude_, laps_, radius_, lap_seconds_);
  }

private:
  // ------------------------------------------------------------------
  // small helpers -- the same ones demo_arm_takeoff uses
  // ------------------------------------------------------------------

  bool isArmed()  { return flight_status_ != FlightStatus::FLIGHT_STATUS_STOPED; }
  bool isFlying() { return flight_status_ == FlightStatus::FLIGHT_STATUS_ON_AIR; }

  bool weAreSteering()
  {
    return display_mode_ == DisplayMode::DISPLAY_MODE_NAVI_SDK_CTRL;
  }

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
      case ORBIT:          return "ORBIT";
      case LAND:           return "LAND";
      case DONE:           return "DONE";
    }
    return "?";
  }

  void goToStep(Step next)
  {
    step_ = next;
    ticks_in_step_ = 0;
    settled_ticks_ = 0;
  }

  // Send one velocity command. East, north and up are metres per second;
  // yaw rate is DEGREES per second.
  void publishVelocity(double east, double north, double up, double yaw_rate_deg)
  {
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.axes = {static_cast<float>(east), static_cast<float>(north),
                static_cast<float>(up), static_cast<float>(yaw_rate_deg)};
    velocity_pub_->publish(joy);
  }

  // How fast to go up or down to reach the altitude we want. Proportional to
  // how far off we are, so it slows down as it arrives, and capped so it never
  // rushes. Used in CLIMB and again all the way round the circle.
  double climbSpeed()
  {
    double speed_up = climb_gain_ * (takeoff_altitude_ - height_);
    if (speed_up >  max_climb_speed_) { speed_up =  max_climb_speed_; }
    if (speed_up < -max_climb_speed_) { speed_up = -max_climb_speed_; }
    return speed_up;
  }

  // Send a command without waiting for the answer. Waiting inside a timer
  // callback would freeze the node forever, because the answer arrives on the
  // same thread that would be stuck waiting.
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

    if (ticks_in_step_ % (int)TICK_HZ == 0) {
      RCLCPP_INFO(get_logger(), "[%s %.0fs] status=%s mode=%s height=%.1f m",
                  stepName(), secondsInStep(), statusName(),
                  weAreSteering() ? "SDK_CTRL" : "not-offboard", height_);
    }

    switch (step_) {

      case WAIT_FOR_DATA:
        if (got_telemetry_) {
          RCLCPP_INFO(get_logger(), "telemetry is flowing, starting");
          goToStep(GET_AUTHORITY);
        } else if (ticks_in_step_ % 100 == 0) {
          RCLCPP_WARN(get_logger(), "still waiting for /wrapper/psdk_ros2/flight_status");
        }
        break;

      case GET_AUTHORITY:
        if (ticks_in_step_ == 1) {
          sendCommand(authority_client_, "obtain_ctrl_authority");
        }
        if (secondsInStep() >= 2.0) {
          goToStep(TAKEOFF);
          sendCommand(takeoff_client_, "takeoff");
        }
        break;

      // Send NO velocity commands here: they would switch PX4 to OFFBOARD in
      // the middle of the automatic climb and cut it short.
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
          sendCommand(takeoff_client_, "takeoff");
        }
        break;

      // Go straight up or down until we are at the altitude we asked for.
      // Nothing sideways yet.
      case CLIMB: {
        publishVelocity(0.0, 0.0, climbSpeed(), 0.0);

        if (std::fabs(takeoff_altitude_ - height_) <= altitude_tolerance_) {
          settled_ticks_++;
        } else {
          settled_ticks_ = 0;
        }

        if (weAreSteering() && settled_ticks_ >= 2 * TICK_HZ) {
          RCLCPP_INFO(get_logger(), "reached %.1f m, starting the circle", height_);
          goToStep(ORBIT);
        } else if (!weAreSteering() && secondsInStep() >= offboard_timeout_) {
          RCLCPP_ERROR(get_logger(),
                       "PX4 never switched to SDK_CTRL after %.0f s, so we cannot fly "
                       "the circle either. Turn on project testing mode on the bridge "
                       "console (key P). Landing.",
                       offboard_timeout_);
          goToStep(LAND);
        } else if (secondsInStep() >= climb_timeout_) {
          RCLCPP_WARN(get_logger(),
                      "could not settle at %.1f m after %.0f s (now %.1f m), "
                      "starting the circle anyway",
                      takeoff_altitude_, climb_timeout_, height_);
          goToStep(ORBIT);
        }
        break;
      }

      // THE NEW PART. Fly round a circle while holding the altitude.
      case ORBIT: {
        double seconds = secondsInStep();

        // One lap = four legs. Which leg are we on, and how far into it?
        double leg_seconds = lap_seconds_ / 4.0;
        int    leg         = static_cast<int>(seconds / leg_seconds) % 4;
        double into_leg    = std::fmod(seconds, leg_seconds);

        // Side length: same side-to-side extent the circle had.
        double side  = 2.0 * radius_;
        double speed = side / leg_seconds;   // cover one side per leg

        // Leg 0 north, 1 west, 2 south, 3 east -- counter-clockwise,
        // same sense and same starting direction as the circle.
        double east = 0.0, north = 0.0;
        switch (leg) {
          case 0: north =  speed; break;
          case 1: east  = -speed; break;
          case 2: north = -speed; break;
          case 3: east  =  speed; break;
        }

        // Corners: spend the last turn_seconds of each leg rotating 90 degrees,
        // so the nose ends each leg already pointing down the next one.
        double turn_seconds  = std::min(2.0, leg_seconds * 0.25);
        double yaw_rate_rad  = 0.0;
        if (into_leg >= leg_seconds - turn_seconds) {
          yaw_rate_rad = (M_PI / 2.0) / turn_seconds;   // +90 deg, CCW
        }

        publishVelocity(east, north, climbSpeed(), yaw_rate_rad);

        if (seconds >= lap_seconds_ * laps_) {
          RCLCPP_INFO(get_logger(), "square finished after %.0f s, landing", seconds);
          goToStep(LAND);
        }
        break;
      }

      // Stop moving, then land. We keep sending zero for a moment so the drone
      // settles before PX4 takes over -- landing while still sliding sideways
      // is how an aircraft tips over on touchdown. After that we stop sending
      // entirely, because OFFBOARD would fight the landing.
      case LAND:
        if (secondsInStep() < 2.0) {
          publishVelocity(0.0, 0.0, 0.0, 0.0);
        } else if (!isArmed() && secondsInStep() > 4.0) {
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
  double radius_             = 6.0;
  double lap_seconds_        = 40.0;
  double laps_               = 1.0;
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
  rclcpp::spin(std::make_shared<SimpleOrbitMission>());
  rclcpp::shutdown();
  return 0;
}
