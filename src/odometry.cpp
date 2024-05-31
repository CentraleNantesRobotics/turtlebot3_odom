#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

using sensor_msgs::msg::JointState;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;

struct WheelVelocity
{
  double dt = 0;
  double wl;
  double wr;
};

struct Wheel
{
  double rad;
  double separation;
  double vmax;

  inline void scale(double &v, double &w)
  {
    if(vmax == 0.)
       return;

     const auto vl{std::abs(v + separation*w/2)};
     const auto vr{std::abs(v - separation*w/2)};

     const auto scale{std::max(vl, vr)/vmax};
     if(scale > 1)
     {
       v /= scale;
       w /= scale;
     }
  }

};

class TurtleOdometry : public rclcpp::Node
{
public:
  explicit TurtleOdometry() : Node("odometry")
  {
    wheel.rad = declare_parameter("wheels.radius", 0.033);
    wheel.separation = declare_parameter("wheels.separation", 0.287);
    wheel.vmax = declare_parameter("wheels.max_vel", 0.);

    odom.header.frame_id = tf.header.frame_id = declare_parameter<std::string>("odom.frame_id", "odom");
    odom.child_frame_id = tf.child_frame_id = declare_parameter<std::string>("odom.child_frame_id", "base_footprint");

    const auto publish_tf{declare_parameter("publish_tf", true)};
    if(publish_tf)
      br = tf2_ros::TransformBroadcaster(this);

    js_sub = create_subscription<JointState>("joint_states", 10,
                                             [&](JointState::ConstSharedPtr msg)
    {update(msg);});

    odom_pub = create_publisher<Odometry>("odom", 10);

    odom.pose.covariance[0] = 0.05;
    odom.pose.covariance[7] = 0.05;
    odom.pose.covariance[14] = 1.0e-9;
    odom.pose.covariance[21] = 1.0e-9;
    odom.pose.covariance[28] = 1.0e-9;
    odom.pose.covariance[35] = 0.0872665;

    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[7] = 1.0e-9;
    odom.twist.covariance[14] = 1.0e-9;
    odom.twist.covariance[21] = 1.0e-9;
    odom.twist.covariance[28] = 1.0e-9;
    odom.twist.covariance[35] = 0.01;

    if(wheel.vmax > 0)
    {
      static auto cmd_pub = create_publisher<Twist>("cmd_vel_scaled", 1);
      static auto cmd_sub = create_subscription<Twist>("cmd_vel", 1, [&](Twist::ConstSharedPtr msg)
      {
        static Twist cmd = *msg;
        wheel.scale(cmd.linear.x, cmd.angular.z);
        cmd_pub->publish(cmd);
      });
    }
  }

private:

  double vmax{declare_parameter("max_vel", 0.26)};


  std::optional<tf2_ros::TransformBroadcaster> br;

  rclcpp::SubscriptionBase::SharedPtr js_sub;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub;
  Odometry odom;
  geometry_msgs::msg::TransformStamped tf;
  double theta{0};

  // cmd_vel
  Wheel wheel;


  void update(JointState::ConstSharedPtr msg)
  {
    // get current velocity
    const auto wheel_joint{extractJointVelocity(msg)};

    if(wheel_joint.dt == 0.)
      return;

    const auto wl{wheel_joint.wl};
    const auto wr{wheel_joint.wr};
    const auto dt{wheel_joint.dt};

    auto &v{odom.twist.twist.linear.x};
    auto &w{odom.twist.twist.angular.z};

    v = .5*(wl+wr)*wheel.rad;
    w = (wr-wl)*wheel.rad/wheel.separation;

    // update pose
    theta += w*dt/2;
    auto &x{odom.pose.pose.position.x};
    auto &y{odom.pose.pose.position.y};

    x += v*cos(theta)*dt;
    y += v*sin(theta)*dt;
    theta += w*dt/2;

    odom.pose.pose.orientation.z = tf.transform.rotation.z = sin(theta/2);
    odom.pose.pose.orientation.w = tf.transform.rotation.w = cos(theta/2);
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;

    odom.header.stamp = tf.header.stamp = msg->header.stamp;

    odom_pub->publish(odom);

    if(br.has_value())
      br->sendTransform(tf);
  }

  inline WheelVelocity extractJointVelocity(JointState::ConstSharedPtr js) const
  {
    static std::array<double,2> prev{0.,0.};
    static rclcpp::Time t_prev{js->header.stamp};

    const size_t left_idx = js->name[0] == "wheel_left_joint" ? 0 : 1;
    const size_t right_idx = 1-left_idx;

    const rclcpp::Time t{js->header.stamp};
    const auto duration{t-t_prev};
    WheelVelocity result;

    if(duration.seconds() > 0.)
    {
      const auto dt{duration.seconds()};
      const auto dl{js->position[left_idx]-prev[0]};
      const auto dr{js->position[right_idx]-prev[1]};
    }

    prev = {js->position[left_idx], js->position[right_idx]};
    t_prev = t;
    return result;
  }


};




int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto robot{std::make_shared<TurtleOdometry>()};

  rclcpp::spin(robot);
}
