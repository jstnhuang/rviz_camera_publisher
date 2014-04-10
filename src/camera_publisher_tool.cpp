/*
 * Panel that publishes the camera pose to a user-specified topic.
 */

#include <geometry_msgs/Pose.h>
#include <rviz/render_panel.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreCamera.h>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>

#include "camera_publisher_tool.h"

namespace rviz_camera_publisher {

/*
 * Constructor. Starts the publisher on the default topic and creates the panel
 * layout.
 */
CameraPublisherTool::CameraPublisherTool()
    : node_handle_("") {
  camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::Pose>(
      kDefaultTopic, 5);

  QHBoxLayout* output_topic_layout = new QHBoxLayout();
  output_topic_editor_ = new QLineEdit(QString::fromStdString(kDefaultTopic));
  connect(output_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(UpdateOutputTopic()));
  output_topic_layout->addWidget(new QLabel("Output topic:"));
  output_topic_layout->addWidget(output_topic_editor_);

  publish_button_ = new QPushButton(QString::fromStdString(kButtonPublish));
  connect(publish_button_, SIGNAL(clicked()), this,
          SLOT(UpdatePublishButton()));
  QVBoxLayout* layout = new QVBoxLayout();
  layout->addLayout(output_topic_layout);
  layout->addWidget(publish_button_);

  QTimer* output_timer = new QTimer(this);
  connect(output_timer, SIGNAL(timeout()), this, SLOT(Update()));
  output_timer->start(33);

  setLayout(layout);
}

/*
 * Destructor.
 */
CameraPublisherTool::~CameraPublisherTool() {
}

/*
 * Updates the publisher's topic when the user input changes.
 */
void CameraPublisherTool::UpdateOutputTopic() {
  camera_pose_publisher_ = node_handle_.advertise<geometry_msgs::Pose>(
      output_topic_editor_->text().toStdString(), 5);
}

/*
 * Toggles the publish button when it's clicked.
 */
void CameraPublisherTool::UpdatePublishButton() {
  if (publish_button_->text().toStdString() == kButtonPublish) {
    publish_button_->setText(QString::fromStdString(kButtonStop));
  } else {
    publish_button_->setText(QString::fromStdString(kButtonPublish));
  }
}

/*
 * Gets the camera pose and publishes it. Called 30 times per second.
 */
void CameraPublisherTool::Update() {
  if (IsPublishing()) {
    geometry_msgs::Pose pose;
    GetCameraPose(&pose);
    camera_pose_publisher_.publish(pose);
  }
}

/*
 * Returns whether to publish the camera pose or not.
 */
bool CameraPublisherTool::IsPublishing() {
  return publish_button_->text().toStdString() == kButtonStop;
}

/*
 * Gets the camera pose.
 *
 * Output:
 *   pose: The pose message to populate with the camera pose.
 */
void CameraPublisherTool::GetCameraPose(geometry_msgs::Pose* pose) {
  auto camera = vis_manager_->getRenderPanel()->getCamera();
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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_camera_publisher::CameraPublisherTool, rviz::Panel)
