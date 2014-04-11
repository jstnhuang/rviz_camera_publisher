/*
 * A display that moves the Rviz camera to match a pose topic.
 */

#include <rviz/render_panel.h>
#include <view_controller_msgs/CameraPlacement.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include "camera_display.h"

namespace rviz_camera_publisher {

/*
 * Constructor.
 */
CameraDisplay::CameraDisplay()
    : node_handle_(""),
      position_(),
      orientation_() {
  input_topic_ = new rviz::RosTopicProperty(
      "Camera pose topic",
      "rviz_camera_publisher/camera_pose",
      QString::fromStdString(
          ros::message_traits::datatype<geometry_msgs::Pose>()),
      "Topic with the camera pose messages to read from.", this,
      SLOT(UpdateInputTopic()));
  UpdateInputTopic();

  camera_placement_publisher_ = node_handle_
      .advertise<view_controller_msgs::CameraPlacement>(
      "/rviz/camera_placement", 5);
}

/*
 * Destructor.
 */
CameraDisplay::~CameraDisplay() {
}

/*
 * Does initialization that takes place after the visualization manager is
 * initialized.
 */
void CameraDisplay::onInitialize() {
  Display::onInitialize();
  visualization_manager_ = static_cast<rviz::VisualizationManager*>(context_);
  geometry_msgs::Pose initial_pose;
  GetCameraPose(&initial_pose);
  position_ = initial_pose.position;
  orientation_ = initial_pose.orientation;
}

/*
 * Main loop that gets the camera placement.
 *
 * Input:
 *   wall_dt: The time delta, in seconds, in wall time.
 *   ros_dt: The time delta in ROS time.
 */
void CameraDisplay::update(float wall_dt, float ros_dt) {
  view_controller_msgs::CameraPlacement camera_placement;
  SetCameraPlacement(ros::Duration(wall_dt), &camera_placement);
  camera_placement_publisher_.publish(camera_placement);
}

/*
 * Handle changes in the input topic.
 */
void CameraDisplay::UpdateInputTopic() {
  camera_pose_subscriber_ = node_handle_.subscribe(
      input_topic_->getStdString(), 5, &CameraDisplay::CameraPoseCallback,
      this);
}

/*
 * Handles camera pose messages from the topic we subscribed to.
 *
 * Input:
 *   pose: The camera pose from the topic we subscribed to.
 */
void CameraDisplay::CameraPoseCallback(const geometry_msgs::Pose& pose) {
  position_ = pose.position;
  orientation_ = pose.orientation;
}

/*
 * Gets the camera pose.
 *
 * Output:
 *   pose: The pose message to populate with the camera pose.
 */
void CameraDisplay::GetCameraPose(geometry_msgs::Pose* pose) {
  auto camera = visualization_manager_->getRenderPanel()->getCamera();
  auto position = camera->getPosition();
  auto orientation = camera->getOrientation();
  pose->position.x = position.x;
  pose->position.y = position.y;
  pose->position.z = position.z;
  pose->orientation.w = orientation.w;
  pose->orientation.x = orientation.x;
  pose->orientation.y = orientation.y;
  pose->orientation.z = orientation.z;
}

/*
 * Convenience method to fill in the fields of a camera placement message.
 *
 * Input:
 *   time_from_start: Time to achieve the camera placement, in seconds.
 *
 * Output:
 *   camera_placement: The message to generate.
 */
void CameraDisplay::SetCameraPlacement(
    const ros::Duration& time_from_start,
    view_controller_msgs::CameraPlacement* camera_placement) {
  camera_placement->target_frame = "<Fixed Frame>";

  camera_placement->time_from_start = time_from_start;

  camera_placement->eye.header.stamp = ros::Time::now();
  camera_placement->eye.header.frame_id = "<Fixed Frame>";
  camera_placement->focus.header.stamp = ros::Time::now();
  camera_placement->focus.header.frame_id = "<Fixed Frame>";
  camera_placement->up.header.stamp = ros::Time::now();
  camera_placement->up.header.frame_id = "<Fixed Frame>";

  camera_placement->eye.point.x = position_.x;
  camera_placement->eye.point.y = position_.y;
  camera_placement->eye.point.z = position_.z;
  geometry_msgs::Point focus_point;
  QuaternionToFocus(orientation_, position_, &focus_point);
  camera_placement->focus.point.x = focus_point.x;
  camera_placement->focus.point.y = focus_point.y;
  camera_placement->focus.point.z = focus_point.z;

  camera_placement->up.vector.x = 0.0;
  camera_placement->up.vector.y = 0.0;
  camera_placement->up.vector.z = 1.0;
}

/*
 * Computes a "look-at" point given the current position and a quaternion.
 *
 * Input:
 *   quaternion: The orientation of the camera.
 *   position: The position of the camera.
 *
 * Output:
 *   point: A "look-at" point represented by the input.
 */
void CameraDisplay::QuaternionToFocus(
    const geometry_msgs::Quaternion& quaternion,
    const geometry_msgs::Point& position, geometry_msgs::Point* point) {
  Ogre::Vector3 ogre_position(position.x, position.y, position.z);
  Ogre::Quaternion ogre_quaternion(quaternion.w, quaternion.x, quaternion.y,
                                   quaternion.z);
  Ogre::Vector3 result = ogre_quaternion * -Ogre::Vector3::UNIT_Z;
  point->x = position.x + result.x;
  point->y = position.y + result.y;
  point->z = position.z + result.z;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_camera_publisher::CameraDisplay, rviz::Display)
