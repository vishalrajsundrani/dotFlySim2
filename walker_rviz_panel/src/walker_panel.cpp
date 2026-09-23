// ============================================================================
// WalkerPanel — the whole operator surface RViz needs, in one dock.
//
// WHY THIS REPLACES V1's PANELS
// -----------------------------
// Version 1 shipped several RViz panels. Each held its own subscriptions to raw
// wrapper topics, parsed them itself, and repainted from the ROS callback --
// which, on topics arriving at 50 Hz, meant repainting a Qt widget tree fifty
// times a second to show numbers a person reads about twice. They also dragged
// a Qt5 toolchain into the image for their own sake.
//
// This one is built on a deliberately cheap contract:
//
//   ONE subscription   /walker/state   std_msgs/String, one JSON doc at 5 Hz,
//                                      already digested by walkerd
//   ONE publisher      /walker/control std_msgs/String, one verb per message
//
// The ROS callback does nothing but copy a string under a mutex. A QTimer
// repaints at 5 Hz from that cached copy, on the Qt thread where it is legal to
// touch widgets. The widget tree is built once and only ever has its text
// replaced -- no widgets are created or destroyed while it runs.
//
// The panel owns no service clients and no policy. A button sends a verb;
// walkerd decides what "land" means, exactly as it does for walker's own keys.
// One place to change behaviour, one place to get it wrong.
//
// SCOPE IS FIXED AT FOUR ACTIONS. Takeoff, land, E-STOP and a camera toggle are
// the things worth having without leaving the 3D view. Anything more belongs in
// walker, which is where the keyboard already is -- and growing this panel into
// a second control surface is exactly how V1's became unmaintainable.
// ============================================================================

#include <memory>
#include <mutex>
#include <string>

#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <std_msgs/msg/string.hpp>

namespace walker_rviz_panel
{

// A tiny JSON reader. Deliberately not a dependency: the document is written by
// walkerd, its shape is fixed, and pulling in a JSON library to read six flat
// values would be the heaviest thing in this package.
static std::string jsonStr(const std::string & doc, const std::string & key,
                           const std::string & fallback = "?")
{
  const std::string needle = "\"" + key + "\":";
  auto at = doc.find(needle);
  if (at == std::string::npos) { return fallback; }
  at += needle.size();
  if (at >= doc.size()) { return fallback; }
  if (doc[at] == '"') {
    auto end = doc.find('"', at + 1);
    return end == std::string::npos ? fallback : doc.substr(at + 1, end - at - 1);
  }
  auto end = doc.find_first_of(",}", at);
  return end == std::string::npos ? fallback : doc.substr(at, end - at);
}

class WalkerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WalkerPanel(QWidget * parent = nullptr)
  : rviz_common::Panel(parent)
  {
    auto * root = new QVBoxLayout;

    aircraft_ = new QLabel("—");
    state_    = new QLabel("waiting for walkerd…");
    height_   = new QLabel("—");
    lock_     = new QLabel("");
    cameras_  = new QLabel("—");

    QFont bold = state_->font();
    bold.setBold(true);
    state_->setFont(bold);

    root->addWidget(aircraft_);
    root->addWidget(state_);
    root->addWidget(height_);
    root->addWidget(cameras_);
    root->addWidget(lock_);

    auto * buttons = new QHBoxLayout;
    takeoff_ = new QPushButton("Takeoff");
    land_    = new QPushButton("Land");
    estop_   = new QPushButton("E-STOP");
    // The one button that must never be pressed by accident, and must always
    // be findable at a glance.
    estop_->setStyleSheet("QPushButton { color: white; background: #b3261e; "
                          "font-weight: bold; }");
    buttons->addWidget(takeoff_);
    buttons->addWidget(land_);
    buttons->addWidget(estop_);
    root->addLayout(buttons);

    auto * cams = new QHBoxLayout;
    for (const auto & profile : {"none", "fisheye", "payload", "all"}) {
      auto * b = new QPushButton(QString::fromStdString(profile));
      const std::string verb = std::string("cameras ") + profile;
      connect(b, &QPushButton::clicked, this, [this, verb]() { send(verb); });
      cams->addWidget(b);
    }
    root->addLayout(cams);
    root->addStretch();
    setLayout(root);

    connect(takeoff_, &QPushButton::clicked, this, [this]() { send("takeoff"); });
    connect(land_,    &QPushButton::clicked, this, [this]() { send("land"); });
    connect(estop_,   &QPushButton::clicked, this, [this]() { send("estop"); });

    // 5 Hz, on the Qt thread. Touching widgets from the ROS executor thread is
    // undefined behaviour that usually looks like it works.
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &WalkerPanel::refresh);
    timer_->start(200);
  }

  void onInitialize() override
  {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    rclcpp::QoS latched(1);
    latched.transient_local().reliable();
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/walker/state", latched,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        // The ONLY work done on the ROS thread: copy a string.
        std::lock_guard<std::mutex> guard(mutex_);
        doc_ = msg->data;
      });

    pub_ = node_->create_publisher<std_msgs::msg::String>("/walker/control", 10);
  }

private Q_SLOTS:
  void refresh()
  {
    std::string doc;
    {
      std::lock_guard<std::mutex> guard(mutex_);
      doc = doc_;
    }
    if (doc.empty()) { return; }

    aircraft_->setText(QString("%1 in %2")
        .arg(QString::fromStdString(jsonStr(doc, "drone")))
        .arg(QString::fromStdString(jsonStr(doc, "world"))));

    const std::string mode = jsonStr(doc, "mode", "");
    const std::string hz   = jsonStr(doc, "telemetry", "0");
    state_->setText(mode.empty() || mode == "?"
      ? QString("no telemetry")
      : QString("%1   (%2 Hz)").arg(QString::fromStdString(mode))
                               .arg(QString::fromStdString(hz)));

    height_->setText(QString("height  %1 m")
        .arg(QString::fromStdString(jsonStr(doc, "height", "0"))));
    cameras_->setText(QString("cameras  %1")
        .arg(QString::fromStdString(jsonStr(doc, "cameras", "none"))));

    const std::string holder = jsonStr(doc, "flight_lock", "");
    lock_->setText(holder.empty()
      ? QString("")
      : QString("flight lock held by %1 — QGC is an observer")
          .arg(QString::fromStdString(holder)));
    lock_->setStyleSheet(holder.empty() ? "" : "color: #e2a03f;");

    // A mission owns the aircraft while it flies; offering Takeoff then is an
    // invitation to fight it.
    const bool free = holder.empty();
    takeoff_->setEnabled(free);
    land_->setEnabled(free);
  }

private:
  void send(const std::string & verb)
  {
    if (!pub_) { return; }
    std_msgs::msg::String msg;
    msg.data = verb;
    pub_->publish(msg);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;

  QLabel * aircraft_; QLabel * state_; QLabel * height_;
  QLabel * lock_; QLabel * cameras_;
  QPushButton * takeoff_; QPushButton * land_; QPushButton * estop_;
  QTimer * timer_;

  std::mutex mutex_;
  std::string doc_;
};

}  // namespace walker_rviz_panel

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(walker_rviz_panel::WalkerPanel, rviz_common::Panel)

#include "walker_panel.moc"
