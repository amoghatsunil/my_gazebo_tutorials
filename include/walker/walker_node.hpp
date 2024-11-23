/**
 * BSD 3-Clause License
 * @file walker_node.hpp
 * @brief Contains state class and walker node implementation to avoid obstacle
 * and move forward
 * @version 1.0
 * @date 2024-11-22
 * @author Amogha Sunil
 */

#ifndef WALKER_NODE_H_
#define WALKER_NODE_H_

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace walker {

/**
 * @class WalkerState
 * @brief WalkerState base class holds all possible states of the robot movement
 */
class WalkerState {
 public:
  /*process method virtual*/
  virtual void process(class WalkerNode &currNode) = 0;
  /*WalkerState destructor*/
  virtual ~WalkerState() = default;
};

/**
 * @class MoveForward
 * @brief MoveForward State class declaration
 */
class MoveForward : public WalkerState {
 public:
  /*process method overriden for the state*/
  void process(class WalkerNode &currNode) override;
};

/**
 * @class Rotate
 * @brief Rotate State class declaration
 */
class Rotate : public WalkerState {
 public:
  /*process method overriden for the state*/
  void process(class WalkerNode &currNode) override;
};

/**
 * @class WalkerNode
 * @brief WalkerNode class holds the implementation of state design
 * implementation to switch between forward and rotation states based on the
 * obstacles
 */
class WalkerNode {
 public:
  /*constructor*/
  WalkerNode(rclcpp::Node::SharedPtr node);

  /*walker node process function*/
  void process();

  /*change state method to switch states*/
  void setState(std::shared_ptr<WalkerState> state);

  /*laserscan callback function to update obstacle distance*/
  void LaserScanCb(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /*this method returns positive if there is an obstacle detected*/
  bool isObstacleDetected();

  /*method to publish to robot's cmd_vel topic*/
  void publishMethod(double linX, double angZ);

  /*set current direction of the robot*/
  void setDirection();

  /*get the current direction of the robot*/
  bool getDirection();

 private:
  /*walker node as sharedpointer to switch states*/
  rclcpp::Node::SharedPtr walkerNode;

  /*stores the current state data*/
  std::shared_ptr<WalkerState> currState;

  /*cmd velocity publisher*/
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub;

  /*laserscan subcriber*/
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScanSub;

  /*flag value for obstacle detected*/
  bool mObstacleDetected{false};

  /*flag value for rotation direction*/
  bool mRotateClockwise{false};
};
}  // namespace walker
#endif