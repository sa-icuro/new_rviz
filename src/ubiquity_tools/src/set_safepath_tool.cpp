#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/display_context.h>
#include <rviz/selection/forwards.h>
#include <rviz/properties/property_tree_model.h>
#include <rviz/properties/property.h>
#include <rviz/properties/vector_property.h>

#include "set_safepath_tool.h"

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <QVariant>

Ogre::Vector3 my_data;
std_msgs::String msg;

namespace ubiquity_tools
{
  SetSafepathTool::SetSafepathTool()
  {
    updateTopic();
  }

  SetSafepathTool::~SetSafepathTool()
  {
  }

  void SetSafepathTool::onInitialize()
    {
      setName( "Set Safepaths" );
    }

  void SetSafepathTool::updateTopic()
  {
    //nh_.param("frame_id", tf_frame_, std::string("/base_link"));
    pub_ = mnh.advertise<std_msgs::String>( "safepath_data", 1 );
  }

  int SetSafepathTool::processMouseEvent( rviz::ViewportMouseEvent& event )
  {
    int flags = rviz::SelectionTool::processMouseEvent( event );

    // determine current selection mode
    if( event.alt() )
    {
      selecting_ = false;
    }
    else
    {
      if( event.leftDown() )
      {
        selecting_ = true;
      }
    }

    if( selecting_ )
    {
      if( event.leftUp() )
      {
        rviz::SelectionManager* sel_manager = context_->getSelectionManager();
        rviz::M_Picked selection = sel_manager->getSelection();
        rviz::PropertyTreeModel *model = sel_manager->getPropertyModel();
        int num_points = model->rowCount();
        if( selection.empty() || num_points <= 0 )
        {
          return flags;
        }

        for( int i = 0; i < num_points; i++ )
        {
          QModelIndex child_index = model->index( i, 0 );
          rviz::Property* child = model->getProp( child_index );
          rviz::VectorProperty* subchild = (rviz::VectorProperty*) child->childAt( 0 );
          Ogre::Vector3 vec = subchild->getVector();
          my_data = vec;
          std::stringstream stream;
          stream << vec.x << " " << vec.y << " " << vec.z;
          msg.data = stream.str();
        }
      }
    }
    return flags;
  }

  int SetSafepathTool::processKeyEvent( QKeyEvent* event, rviz::RenderPanel* panel )
  {
    int render = rviz::SelectionTool::processKeyEvent(event, panel);
    if( event->key() == Qt::Key_A )
    {
      std::cout<<"Publishing Safepath!!" << std::endl;
      pub_.publish(msg);
    }
  }

} // end namespace rviz_plugin_selected_points_topic

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( ubiquity_tools::SetSafepathTool, rviz::Tool )
