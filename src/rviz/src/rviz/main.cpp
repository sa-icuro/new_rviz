/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#include <QApplication>
#include <string>
#include <fstream>
#include "rviz/visualizer_app.h"

int main( int argc, char** argv )
{
  QApplication qapp( argc, argv );

  rviz::VisualizerApp vapp;
  vapp.setApp( &qapp );
  
  if( vapp.init( argc, argv ))
  {
    std::ofstream ofs;
    ofs.open("/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals.yaml", std::ofstream::out | std::ofstream::trunc);
    ofs << "goals: \n";
    ofs.close();

    /*
    int i = 0;
    std::string filename = "/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals_" + std::to_string(i) + ".yaml";
    std::ifstream ifile(filename.c_str());
    
    std::ifstream ifile(filename.c_str()); 
    while(true) 
    {  

       check if final goals does not exist
        if exist: create a final_goals file and break

       start checking for final_goals_i
       if it does not exist, move it to final_goals_i
       create final goals file

      std::string filename = "/home/centaur/catkin_ws/src/magni_goal_sender/param/final_goals_" + std::to_string(i) + ".yaml";
      std::ifstream ifile(filename.c_str());
      if(ifile.is_open()){
          std::cout<<filename<<"-----exists"<<std::endl;
          mv existing goal to goali
      }
      i++
    
       i++;

       continue;
     
    } 
    */
    return qapp.exec();
  }
  else
  {
    return 1;
  }
}
