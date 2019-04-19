/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
#include <iostream>
#include <ros/time.h>

#include "global.h"
#include "move_tool.h"

#include "rviz/ogre_helpers/camera_base.h"
#include "rviz/ogre_helpers/qt_ogre_render_window.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"

#include "set_safepath_tool.h"


namespace rviz
{
  int counting_path = 0;
  SetSafepathTool::SetSafepathTool()
    : Tool()
    , move_tool_( new MoveTool() )
    , selecting_( false )
    , sel_start_x_( 0 )
    , sel_start_y_( 0 )
    , moving_( false )
  {
    shortcut_key_ = 'p';
    access_all_keys_ = true;
  }

  SetSafepathTool::~SetSafepathTool()
  {
    delete move_tool_;
  }

  void SetSafepathTool::onInitialize()
  {
    move_tool_->initialize( context_ );
  }
    
  void SetSafepathTool::activate()
  {
    setStatus( "Select safepoints to build a safepath. Safepaths connect safepoints and can be traversed. Select safepoints (or even a Fiducial!) and Press 'A' to add it to the safepath. Must build in parent-child pairs!");

    context_->getSelectionManager()->setTextureSize(512);
    selecting_ = false;
    moving_ = false;
    //context_->getSelectionManager()->enableInteraction(true);
    //std::cout << "Here @ 2" <<  ".\n";
  }

  void SetSafepathTool::deactivate()
  {
    context_->getSelectionManager()->removeHighlight();
    //system("sh ~/catkin_ws/src/rviz/src/rviz/default_plugin/tools/kill_routeBuilder.sh");

  }

  void SetSafepathTool::update(float wall_dt, float ros_dt)
  {
    SelectionManager* sel_manager = context_->getSelectionManager();
    if (!selecting_)
    {
      sel_manager->removeHighlight();
    }
  }

  int SetSafepathTool::processMouseEvent( ViewportMouseEvent& event )
  {
    SelectionManager* sel_manager = context_->getSelectionManager();
    //std::cout << "Here @ 5" << ".\n";
    int flags = 0;

    if( event.alt() )
    {
      moving_ = true;
      selecting_ = false;
    }
    else
    {
      moving_ = false;

      if( event.leftDown() )
      {
        selecting_ = true;

        sel_start_x_ = event.x;
        sel_start_y_ = event.y;
        //std::cout << "Here @ 6" << ".\n";
      }
    }

    if( selecting_ )
    {
      sel_manager->highlight( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y );
      //std::cout << "Here @ 7" << ".\n";

      if( event.leftUp() )
      {
        SelectionManager::SelectType type = SelectionManager::Replace;

        M_Picked selection;
        //std::cout << "Here @ 8" <<  ".\n";

        /*
        for ( auto local_it = selection.begin(); local_it!= selection.end(); ++local_it )
        {
            std::cout << " " << local_it->first << ":" << local_it->second;
            std::cout << std::endl;
        }
        */
        
        if( event.shift() )
        {
          type = SelectionManager::Add;

        }
        sel_manager->select( event.viewport, sel_start_x_, sel_start_y_, event.x, event.y, type );

        selecting_ = false;
      }

      flags |= Render;
    }
    else if( moving_ )
    {
      sel_manager->removeHighlight();

      flags = move_tool_->processMouseEvent( event );

      if( event.type == QEvent::MouseButtonRelease )
      {
        moving_ = false;
      }
    }
    else
    {
      sel_manager->highlight(event.viewport, event.x, event.y, event.x, event.y);
    }

    return flags;
  }

  int SetSafepathTool::processKeyEvent( QKeyEvent* event, RenderPanel* panel )
  {
    SelectionManager* sel_manager = context_->getSelectionManager();
    

    if( event->key() == Qt::Key_F )
    {
      sel_manager->focusOnSelection();
    }

    if( event->key() == Qt::Key_A )
    {
      //ros::Duration(1.0).sleep();
      my_pub2.publish(goal2);
    }

    return Render;
  }

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SetSafepathTool, rviz::Tool )
