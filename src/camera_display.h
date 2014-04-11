#ifndef CAMERA_DISPLAY_H
#define CAMERA_DISPLAY_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/debug.h>
#include <ros/subscriber.h>
#include <rviz/display.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/visualization_manager.h>
#include <view_controller_msgs/CameraPlacement.h>

namespace rviz_camera_publisher {

class CameraDisplay : public rviz::Display {
Q_OBJECT

 public:
  CameraDisplay();
  virtual ~CameraDisplay();

 protected:
  virtual void onInitialize();
  virtual void update(float wall_dt, float ros_dt);

 private Q_SLOTS:
  void UpdateInputTopic();

 private:
  ros::NodeHandle node_handle_;
  rviz::RosTopicProperty* input_topic_;
  ros::Subscriber camera_pose_subscriber_;
  ros::Publisher camera_placement_publisher_;
  rviz::VisualizationManager* visualization_manager_;
  geometry_msgs::Point position_;
  geometry_msgs::Quaternion orientation_;

  void CameraPoseCallback(const geometry_msgs::Pose& pose);
  void GetCameraPose(geometry_msgs::Pose* pose);
  void SetCameraPlacement(
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement);
  void QuaternionToFocus(const geometry_msgs::Quaternion& quaternion,
                         const geometry_msgs::Point& position,
                         geometry_msgs::Point* point);
};

}

#endif
