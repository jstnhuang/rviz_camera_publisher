#ifndef CAMERA_PUBLISHER_PANEL_H
#define CAMERA_PUBLISHER_PANEL_H

#include <geometry_msgs/Pose.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <QLineEdit>
#include <QPushButton>

namespace rviz_camera_publisher {

static const std::string kDefaultTopic = "rviz_camera_publisher/camera_pose";
static const std::string kButtonPublish = "Publish camera pose";
static const std::string kButtonStop = "Stop publishing camera pose";

class CameraPublisherPanel : public rviz::Panel {
Q_OBJECT

 public:
  CameraPublisherPanel();
  virtual ~CameraPublisherPanel();

 private Q_SLOTS:
  void UpdateOutputTopic();
  void UpdatePublishButton();
  void Update();

 private:
  ros::NodeHandle node_handle_;
  ros::Publisher camera_pose_publisher_;
  QLineEdit* output_topic_editor_;
  QPushButton* publish_button_;
  bool IsPublishing();
  void GetCameraPose(geometry_msgs::Pose* pose);
};

}

#endif
