/**
 * BSD 3-Clause License
 * @file walker_node.hpp
 * @brief Contains state class and walker node implementation to avoid obstacle
 * and move forward
 * @version 1.0
 * @date 2024-11-22
 * @author Amogha Sunil
 */

#include "walker/walker_node.hpp"

#include <limits>
#include <memory>

namespace walker {
/*WalkerNode constructor*/
WalkerNode::WalkerNode(rclcpp::Node::SharedPtr node) : walkerNode(node) {
  cmdVelPub =
      walkerNode->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  laserScanSub = walkerNode->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&WalkerNode::LaserScanCb, this, std::placeholders::_1));
  /*currState initialized to MoveForward*/
  currState = std::make_shared<MoveForward>();
}

void WalkerNode::LaserScanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  /*minimum distance*/
  float minDistance = std::numeric_limits<float>::infinity();
  /*iterate through laser scan ranges*/
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    double angle = msg->angle_min + i * msg->angle_increment;
    /*check if angle is within current range*/
    if (angle >= -M_PI / 5 && angle <= M_PI / 5) {
      float range = msg->ranges[i];
      /*update minimumm distance*/
      if (std::isfinite(range) && range < minDistance) {
        minDistance = range;
      }
    }
  }

  /*log for minimum distancec*/
  if (std::isfinite(minDistance)) {
    RCLCPP_INFO(walkerNode->get_logger(),
                "Minimum distance from /scan : %.2f meters", minDistance);
  } else {
    RCLCPP_WARN(walkerNode->get_logger(), "No valid ranges detected");
  }

  /*update obstacle detected flag*/
  constexpr float OBSTACLE_THRESHOLD = 0.5;  // Meters
  mObstacleDetected = (minDistance < OBSTACLE_THRESHOLD);

  /*log for obstacle detection*/
  if (mObstacleDetected) {
    RCLCPP_WARN(walkerNode->get_logger(), "Obstacle detected at %.2f meters",
                minDistance);
  } else {
    RCLCPP_INFO(walkerNode->get_logger(), "No obstacle detected");
  }
}

void WalkerNode::publishMethod(double linX, double angZ) {
  /*twist message type*/
  geometry_msgs::msg::Twist cmdVel;
  cmdVel.linear.x = linX;
  cmdVel.angular.z = angZ;
  /*publish to cmd_vel*/
  cmdVelPub->publish(cmdVel);
}

/*returns true if obstacle is detected*/
bool WalkerNode::isObstacleDetected() { return mObstacleDetected; }

/*Sets currentstate of the robot- MoveForward or Rotate*/
void WalkerNode::setState(std::shared_ptr<WalkerState> state) {
  currState = state;
}

/*walker node process*/
void WalkerNode::process() {
  if (currState) currState->process(*this);
}

/*Toggles robot's direction*/
void WalkerNode::setDirection() { mRotateClockwise = !mRotateClockwise; }

/*returns robot's current direction*/
bool WalkerNode::getDirection() { return mRotateClockwise; }

/*Moveforward state process method*/
void MoveForward::process(class WalkerNode &currNode) {
  /*if an obstacle is detected*/
  if (currNode.isObstacleDetected()) {
    currNode.publishMethod(0.0, 0.0);
    /*Change state to Rotate*/
    currNode.setState(std::make_shared<Rotate>());
  } else {
    /*publish forward velocity*/
    currNode.publishMethod(0.18, 0.0);
  }
}

void Rotate::process(class WalkerNode &currNode) {
  /*rotation direction*/
  double angular = currNode.getDirection() ? -0.5 : 0.5;

  /*if the obstacle is cleared*/
  if (!currNode.isObstacleDetected()) {
    /*stop*/
    currNode.publishMethod(0.0, 0.0);
    currNode.setDirection();

    /*Change state to MoveForward state*/
    currNode.setState(std::make_shared<MoveForward>());
  }

  /*Continue to Rotate to avoid the obstacle*/
  currNode.publishMethod(0.0, angular);
}
}  // namespace walker
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  /*walker node*/
  auto node = std::make_shared<rclcpp::Node>("walker_node");
  /*walker node class instance*/
  auto walker = std::make_shared<walker::WalkerNode>(node);
  rclcpp::Rate rate(10);
  /*shutdown robot should stop*/
  rclcpp::on_shutdown([walker]() { walker->publishMethod(0.0, 0.0); });

  while (rclcpp::ok()) {
    /*State logic process call*/
    walker->process();
    /*spin the node*/
    rclcpp::spin_some(node);
    rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}