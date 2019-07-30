#include "rviz/selection/selection_manager.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/display_context.h"
#include "rviz/selection/forwards.h"
#include "rviz/properties/property_tree_model.h"
#include "rviz/properties/property.h"
#include "rviz/properties/vector_property.h"


#include "publish_selected_patch.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/PointCloud2.h>
#include <QVariant>

// since I don't konw where the constant 'Render' is defined
// I copy the whole include part of https://github.com/ros-visualization/rviz/blob/melodic-devel/src/rviz/default_plugin/tools/selection_tool.cpp
#include <QKeyEvent>

#include <OgreRay.h>
#include <OgreSceneManager.h>
#include <OgreCamera.h>
#include <OgreMovableObject.h>
#include <OgreRectangle2D.h>
#include <OgreSceneNode.h>
#include <OgreViewport.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>

#include "rviz/ogre_helpers/camera_base.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"

/*********************************/

#include <iostream>
#include <string>
using namespace std;

namespace publish_selected_patch
{
PublishSelectedPatch::PublishSelectedPatch():
cnt(0),
cloud_topic_("/selected_patch")
{
  updateTopic();
}

PublishSelectedPatch::~PublishSelectedPatch()
{
}

void PublishSelectedPatch::updateTopic()
{
  // nh_.param("frame_id", tf_frame_, std::string("/base_link"));
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>( cloud_topic_.c_str(), 1 );
  ROS_INFO("Selected point cloud patch will be published to topic %s.", nh_.resolveName(cloud_topic_).c_str());
  // ROS_INFO( "Publishing data on topic %s with frame_id %s.",
  //           nh_.resolveName (cloud_topic_).c_str (),
  //           tf_frame_.c_str() );
}

int PublishSelectedPatch::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
{
  // printf("key_event is called!\n");
  rviz::SelectionTool::processKeyEvent(event, panel);
  rviz::SelectionManager* sel_manager = context_->getSelectionManager();

  // printf("event address: %x\n", event);
  // printf("key: %x\n", event->key());
  // printf("key repeat print: %x\n", event->key());
  // printf("F: %x\n", Qt::Key_F);
  if( event->key() == Qt::Key_F )
  {
    sel_manager->focusOnSelection();
  }

  if(  event->key() == Qt::Key_P )
  {
    // printf("P is pressd\n");
    rviz::M_Picked selection = sel_manager->getSelection();
    rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
    int num_points = model->rowCount();

    if( selection.empty() || num_points <= 0 )
    {
      ROS_INFO("No selected points to publish, skip...");
      return Render;
    }
  
    sensor_msgs::PointCloud2 msg_pc;
    msg_pc.header.frame_id = context_->getFixedFrame().toStdString();
    msg_pc.height = 1;
    msg_pc.width = num_points;
    msg_pc.point_step = 3 * 4;
    msg_pc.row_step = num_points * msg_pc.point_step;
    msg_pc.is_dense = false;
    msg_pc.is_bigendian = false;

    msg_pc.data.resize( msg_pc.row_step );
    msg_pc.fields.resize( 3 );

    msg_pc.fields[0].name = "x";
    msg_pc.fields[0].offset = 0;
    msg_pc.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    msg_pc.fields[0].count = 1;

    msg_pc.fields[1].name = "y";
    msg_pc.fields[1].offset = 4;
    msg_pc.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    msg_pc.fields[1].count = 1;

    msg_pc.fields[2].name = "z";
    msg_pc.fields[2].offset = 8;
    msg_pc.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    msg_pc.fields[2].count = 1;

    cnt++;
    ROS_INFO("Publishing new selected patch with %d points, index: %d", num_points, this->cnt);
    ofstream pcd_csv;
    string fname = "selected_pcd_patch_"+to_string(this->cnt)+".csv";
    pcd_csv.open(fname, ios::out);
    pcd_csv<<'x'<<','<<'y'<<','<<'z'<<endl;
    //TODO: publish pcd with intensity information
    for( int i = 0; i < num_points; i++ )
    {
      QModelIndex child_index = model->index( i, 0 );
      rviz::Property* child = model->getProp( child_index );
      rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
      Ogre::Vector3 vec = subchild->getVector();

      uint8_t* ptr = &msg_pc.data[0] + i * msg_pc.point_step;
      *(float*)ptr = vec.x;
      ptr += 4;
      *(float*)ptr = vec.y;
      ptr += 4;
      *(float*)ptr = vec.z;
      ptr += 4;
      pcd_csv<< vec.x << ',' << vec.y << ',' << vec.z << endl;
    }
    ROS_INFO("Selected patch has been written to file: %s.",  fname.c_str());

    msg_pc.header.stamp = ros::Time::now();
    pub_.publish( msg_pc );
  }
  // printf("key_event finished\n");
  return Render;
}
} // end namespace publish_selected_patch

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( publish_selected_patch::PublishSelectedPatch, rviz::Tool )
