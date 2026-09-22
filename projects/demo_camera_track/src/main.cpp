// ============================================================================
// demo_camera_track (simple version)
//
// What it does, all by itself:
//   1. wait until the wrapper starts sending telemetry
//   2. ask for control authority
//   3. take off   (the takeoff command arms the drone by itself -- we never
//                  call turn_on_motors, so nothing arms on the ground)
//   4. CLIMB: fly ourselves up to the altitude we actually wanted
//   5. TRACK: turn to face a tower and hold station in front of it, using
//             nothing but the picture from the camera
//   6. land, then give control authority back
//
// Steps 1 to 4 are copied from demo_arm_takeoff, on purpose and word for word.
// Read that one first; the only new part here is step 5.
//
// WHY STEP 4 EXISTS
// -----------------
// The `takeoff` service takes no arguments, and on this setup the automatic
// takeoff always stops at about 1.18 m -- it is not a setting we can change.
// Every metre after that is ours to fly.
//
// HOW WE STEER, AND WHY IT IS A DIFFERENT TOPIC FROM THE OTHER DEMOS
// ------------------------------------------------------------------
// Chasing something you can see is naturally a body-frame job: "the tower is
// off to my left, so turn left". So this demo uses
//     /wrapper/psdk_ros2/flight_control_setpoint_FLUvelocity_yawrate
// where FLU means Forward, Left, Up -- directions relative to the nose, not to
// the compass. It is a sensor_msgs/Joy carrying
//     axes = [speed forward, speed left, speed up, yaw rate]  (m/s, m/s, m/s, deg/s)
//
// The other demos use the ENU (East North Up) topic instead. Only ONE of these
// setpoint topics can be in charge at a time -- the bridge allows a single
// writer -- which is why demo.conf next to this file says SETPOINT=body. If
// you copy this demo and forget that line, your commands will go nowhere.
//
// The yaw rate really is in DEGREES per second, not radians. The psdk_ros2
// documentation table says radians; the bridge converter that actually reads
// these messages takes degrees (see gui/simty/converters.py). Believe the code.
//
// HOW THE TOWER IS FOUND
// ----------------------
// No machine learning, no OpenCV. A galvanised steel lattice tower is GREY:
// its red, green and blue values are all close together. Grass and sky are not
// -- grass is far greener than it is red or blue, sky is far bluer. So:
//
//     a pixel is "tower" if  (brightest colour - darkest colour) is small
//                       and  it is not nearly black and not nearly white
//
// Average the positions of all such pixels and that is roughly the middle of
// the tower. Count them and that count says roughly how big it looks, which
// tells us whether to move closer or back off.
//
// This is deliberately crude, and it works here because the ground in
// worlds/powerline.sdf is a strongly coloured green. On a grey overcast day
// against concrete it would fail completely. That is the honest limit of a
// colour rule.
//
// IMPORTANT: velocity commands only do something when PX4 is in OFFBOARD mode.
// The bridge switches to OFFBOARD about 1.5 s after we start sending commands,
// and display_mode becomes SDK_CTRL. That happens AFTER we start commanding,
// so it is not something to wait for before taking off.
//
// WHY THE TIMER IS 20 Hz
// ----------------------
// PX4 drops out of OFFBOARD if velocity commands stop arriving. 20 per second
// is the normal rate. We count ticks instead of seconds, and secondsInStep()
// turns ticks back into seconds.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
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
  TRACK,
  LAND,
  DONE
};

class SimpleCameraTrack : public rclcpp::Node
{
public:
  SimpleCameraTrack()
  : Node("demo_camera_track")
  {
    // ---- settings you can change from the command line ----
    takeoff_altitude_   = declare_parameter<double>("takeoff_altitude", 1.8);
    track_seconds_      = declare_parameter<double>("track_seconds", 120.0);
    altitude_tolerance_ = declare_parameter<double>("altitude_tolerance", 0.3);
    min_takeoff_height_ = declare_parameter<double>("min_takeoff_height", 0.5);
    max_climb_speed_    = declare_parameter<double>("max_climb_speed", 1.0);
    climb_gain_         = declare_parameter<double>("climb_gain", 0.8);
    takeoff_timeout_    = declare_parameter<double>("takeoff_timeout", 60.0);
    offboard_timeout_   = declare_parameter<double>("offboard_timeout", 15.0);
    climb_timeout_      = declare_parameter<double>("climb_timeout", 60.0);

    // how grey a pixel must be to count as tower, 0 to 255
    greyness_           = declare_parameter<int>("greyness", 40);
    // how big the tower should look; bigger number means we fly closer
    wanted_size_        = declare_parameter<double>("wanted_size", 0.06);
    turn_gain_          = declare_parameter<double>("turn_gain", 60.0);   // deg/s per unit
    approach_gain_      = declare_parameter<double>("approach_gain", 8.0);
    max_approach_speed_ = declare_parameter<double>("max_approach_speed", 1.5);
    search_turn_rate_   = declare_parameter<double>("search_turn_rate", 20.0);  // deg/s

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

    rclcpp::QoS image_qos(1);
    image_qos.best_effort();
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/wrapper/psdk_ros2/main_camera_stream", image_qos,
      std::bind(&SimpleCameraTrack::onImage, this, std::placeholders::_1));

    // ---- how we steer the drone ----
    rclcpp::QoS command_qos(10);
    command_qos.reliable();
    velocity_pub_ = create_publisher<sensor_msgs::msg::Joy>(
      "/wrapper/psdk_ros2/flight_control_setpoint_FLUvelocity_yawrate", command_qos);

    // ---- our own outputs, for looking at in RViz ----
    detection_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/detection", 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>("~/image", image_qos);

    // ---- commands we can send to the drone ----
    authority_client_ = create_client<Trigger>("/wrapper/psdk_ros2/obtain_ctrl_authority");
    release_client_   = create_client<Trigger>("/wrapper/psdk_ros2/release_ctrl_authority");
    takeoff_client_   = create_client<Trigger>("/wrapper/psdk_ros2/takeoff");
    land_client_      = create_client<Trigger>("/wrapper/psdk_ros2/land");

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / TICK_HZ),
      std::bind(&SimpleCameraTrack::tick, this));

    RCLCPP_INFO(get_logger(), "started: climb to %.1f m, then track for %.0f s",
                takeoff_altitude_, track_seconds_);
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
      case TRACK:          return "TRACK";
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

  // Send one velocity command. Forward, left and up are metres per second,
  // relative to the nose; yaw rate is DEGREES per second.
  void publishVelocity(double forward, double left, double up, double yaw_rate_deg)
  {
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = now();
    joy.axes = {static_cast<float>(forward), static_cast<float>(left),
                static_cast<float>(up), static_cast<float>(yaw_rate_deg)};
    velocity_pub_->publish(joy);
  }

  double climbSpeed()
  {
    double speed_up = climb_gain_ * (takeoff_altitude_ - height_);
    if (speed_up >  max_climb_speed_) { speed_up =  max_climb_speed_; }
    if (speed_up < -max_climb_speed_) { speed_up = -max_climb_speed_; }
    return speed_up;
  }

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
  // looking at the picture
  // ------------------------------------------------------------------
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    images_seen_++;

    // The wrapper sends rgb8 for the main camera. bgr8 works too: we only ever
    // compare the three colour channels to each other, and swapping two of
    // them does not change which is biggest or smallest.
    if (msg->encoding != "rgb8" && msg->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "camera sends '%s'; this demo understands rgb8 and bgr8",
                           msg->encoding.c_str());
      return;
    }
    if (msg->width == 0 || msg->height == 0 ||
        msg->data.size() < (size_t)msg->step * msg->height)
    {
      return;
    }

    // Look at every 4th pixel across and down. That is 16 times less work and
    // makes no real difference to where the middle of a big object is.
    const int skip = 4;

    long sum_x = 0, sum_y = 0, count = 0;
    for (uint32_t row = 0; row < msg->height; row += skip) {
      const uint8_t * line = msg->data.data() + (size_t)row * msg->step;
      for (uint32_t col = 0; col < msg->width; col += skip) {
        const uint8_t * pixel = line + (size_t)col * 3;
        int a = pixel[0], b = pixel[1], c = pixel[2];

        int brightest = std::max(a, std::max(b, c));
        int darkest   = std::min(a, std::min(b, c));

        bool is_grey   = (brightest - darkest) < greyness_;
        bool is_usable = brightest > 45 && brightest < 235;  // not shadow, not blown-out sky

        if (is_grey && is_usable) {
          sum_x += col;
          sum_y += row;
          count++;
        }
      }
    }

    long looked_at = ((long)msg->width / skip) * ((long)msg->height / skip);

    // A handful of stray pixels is noise, not a tower.
    if (count > 40 && looked_at > 0) {
      double centre_x = (double)sum_x / (double)count;
      double centre_y = (double)sum_y / (double)count;

      // Turn "pixel 812 of 1920" into "0.15 of the way right of the middle",
      // so the numbers mean the same thing whatever the picture size is.
      target_x_ = (centre_x - msg->width * 0.5) / (msg->width * 0.5);
      target_y_ = (centre_y - msg->height * 0.5) / (msg->height * 0.5);
      target_size_ = (double)count / (double)looked_at;
      target_seen_ = true;
      ticks_since_seen_ = 0;
    } else {
      target_seen_ = false;
    }

    publishDetection(msg->header);
    publishDebugImage(*msg);
  }

  void publishDetection(const std_msgs::msg::Header & header)
  {
    if (detection_pub_->get_subscription_count() == 0) { return; }
    geometry_msgs::msg::PointStamped point;
    point.header = header;
    // Not a position in space: x and y are how far off-centre the target is,
    // and z is how big it looks. Packed into a standard message so that
    // `ros2 topic echo` can read it without a custom type.
    point.point.x = target_x_;
    point.point.y = target_y_;
    point.point.z = target_size_;
    detection_pub_->publish(point);
  }

  // Copy the picture and draw a red cross on the target, so a person can see
  // what the demo thinks it is looking at.
  void publishDebugImage(const sensor_msgs::msg::Image & source)
  {
    if (debug_image_pub_->get_subscription_count() == 0 || !target_seen_) { return; }

    auto out = std::make_unique<sensor_msgs::msg::Image>(source);
    int centre_col = (int)((target_x_ + 1.0) * 0.5 * source.width);
    int centre_row = (int)((target_y_ + 1.0) * 0.5 * source.height);
    const int arm_length = 30;

    for (int col = centre_col - arm_length; col <= centre_col + arm_length; col++) {
      paintRed(*out, centre_row, col);
    }
    for (int row = centre_row - arm_length; row <= centre_row + arm_length; row++) {
      paintRed(*out, row, centre_col);
    }
    debug_image_pub_->publish(std::move(out));
  }

  void paintRed(sensor_msgs::msg::Image & image, int row, int col)
  {
    if (row < 0 || col < 0 || row >= (int)image.height || col >= (int)image.width) { return; }
    uint8_t * pixel = image.data.data() + (size_t)row * image.step + (size_t)col * 3;
    pixel[0] = 255; pixel[1] = 30; pixel[2] = 30;
  }

  // ------------------------------------------------------------------
  // the flight sequence -- runs 20 times per second
  // ------------------------------------------------------------------
  void tick()
  {
    ticks_in_step_++;
    ticks_since_seen_++;

    if (ticks_in_step_ % (int)TICK_HZ == 0) {
      RCLCPP_INFO(get_logger(), "[%s %.0fs] status=%s mode=%s height=%.1f m %s",
                  stepName(), secondsInStep(), statusName(),
                  weAreSteering() ? "SDK_CTRL" : "not-offboard", height_,
                  target_seen_ ? "target in sight" : "no target");
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

      case CLIMB: {
        publishVelocity(0.0, 0.0, climbSpeed(), 0.0);

        if (std::fabs(takeoff_altitude_ - height_) <= altitude_tolerance_) {
          settled_ticks_++;
        } else {
          settled_ticks_ = 0;
        }

        if (weAreSteering() && settled_ticks_ >= 2 * TICK_HZ) {
          RCLCPP_INFO(get_logger(), "reached %.1f m, starting to look around", height_);
          goToStep(TRACK);
        } else if (!weAreSteering() && secondsInStep() >= offboard_timeout_) {
          RCLCPP_ERROR(get_logger(),
                       "PX4 never switched to SDK_CTRL after %.0f s, so we cannot steer "
                       "towards anything. Turn on project testing mode on the bridge "
                       "console (key P), and check that demo.conf says SETPOINT=body. "
                       "Landing.",
                       offboard_timeout_);
          goToStep(LAND);
        } else if (secondsInStep() >= climb_timeout_) {
          RCLCPP_WARN(get_logger(),
                      "could not settle at %.1f m after %.0f s (now %.1f m), "
                      "tracking anyway",
                      takeoff_altitude_, climb_timeout_, height_);
          goToStep(TRACK);
        }
        break;
      }

      // THE NEW PART. Turn towards the tower and hold station in front of it.
      case TRACK: {
        bool seen_recently = target_seen_ || ticks_since_seen_ < TICK_HZ;

        if (!seen_recently) {
          // Nothing in sight. Turn slowly on the spot and keep looking.
          publishVelocity(0.0, 0.0, climbSpeed(), search_turn_rate_);
          break;
        }

        // Turn towards it. If the tower is right of centre (target_x_ > 0) we
        // must turn RIGHT, and turning left is the positive direction, so the
        // sign flips.
        double turn = -turn_gain_ * target_x_;

        // Move closer or back off so it fills the frame by about wanted_size_.
        // Looking too small means we are too far away, so fly forward.
        double forward = approach_gain_ * (wanted_size_ - target_size_);
        if (forward >  max_approach_speed_) { forward =  max_approach_speed_; }
        if (forward < -max_approach_speed_) { forward = -max_approach_speed_; }

        publishVelocity(forward, 0.0, climbSpeed(), turn);

        if (secondsInStep() >= track_seconds_) {
          RCLCPP_INFO(get_logger(), "tracked for %.0f s, landing", secondsInStep());
          goToStep(LAND);
        }
        break;
      }

      // Stop moving, then land. We keep sending zero for a moment so the drone
      // settles first, then stop sending entirely -- OFFBOARD would fight the
      // landing.
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
          RCLCPP_INFO(get_logger(), "flight finished, %zu pictures looked at", images_seen_);
          rclcpp::shutdown();
        }
        break;
    }
  }

  // ---- settings ----
  double takeoff_altitude_   = 1.8;
  double track_seconds_      = 120.0;
  double altitude_tolerance_ = 0.3;
  double min_takeoff_height_ = 0.5;
  double max_climb_speed_    = 1.0;
  double climb_gain_         = 0.8;
  double takeoff_timeout_    = 60.0;
  double offboard_timeout_   = 15.0;
  double climb_timeout_      = 60.0;
  int    greyness_           = 40;
  double wanted_size_        = 0.06;
  double turn_gain_          = 60.0;
  double approach_gain_      = 8.0;
  double max_approach_speed_ = 1.5;
  double search_turn_rate_   = 20.0;

  // ---- ROS stuff ----
  rclcpp::Subscription<FlightStatus>::SharedPtr flight_status_sub_;
  rclcpp::Subscription<DisplayMode>::SharedPtr display_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr height_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
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

  // ---- what the camera sees ----
  bool   target_seen_ = false;
  double target_x_ = 0.0;      // -1 far left of frame, +1 far right
  double target_y_ = 0.0;      // -1 top, +1 bottom
  double target_size_ = 0.0;   // fraction of the picture covered
  int    ticks_since_seen_ = 0;
  size_t images_seen_ = 0;

  // ---- where we are in the sequence ----
  Step step_ = WAIT_FOR_DATA;
  int ticks_in_step_ = 0;
  int settled_ticks_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleCameraTrack>());
  rclcpp::shutdown();
  return 0;
}
