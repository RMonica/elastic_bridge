/*
 * Copyright (c) 2017-2019, Andrea Pagani, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <elastic_bridge/downloadPointcloud2Action.h>
#include <std_msgs/Empty.h>

#include <fstream>
#include <stdint.h>

//Pcl
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "save_pcl_client");

    typedef uint64_t uint64;
    
    std::string path = "./";
    std::string fileName = "pointcloud.pcd";
    std::string fileNameComplete = path+fileName;

    std::string fileName2 = "pointcloud.corresp";
    std::string fileNameComplete2 = path+fileName2;

    actionlib::SimpleActionClient<elastic_bridge::downloadPointcloud2Action> ac("save_pcl", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    elastic_bridge::downloadPointcloud2Goal goal;
    goal.stop = 1;
    ac.sendGoal(goal);
    
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (!finished_before_timeout)
    {
      ROS_INFO("Action did not finish before the time out.");
      return 1;
    }

    actionlib::SimpleClientGoalState state = ac.getState();

    elastic_bridge::downloadPointcloud2Result result = *ac.getResult();

    const sensor_msgs::PointCloud2 pc2 = result.pointcloud;
    pcl::PointCloud<pcl::PointSurfel> pcl_cloud;
    pcl::fromROSMsg(pc2, pcl_cloud);

    pcl::io::savePCDFile(fileNameComplete, pcl_cloud, true);

    ROS_INFO("Action finished: %s", state.toString().c_str());
    ROS_INFO("Saved point cloud in: %s",fileNameComplete.c_str());

    std::ofstream ofile(fileNameComplete2.c_str(),std::ios_base::binary);
    for (uint64 i = 0; i < result.guids.size(); i++)
    {
        const uint64 guid = result.guids[i];
        ofile.write((char *)&guid,sizeof(guid));
    }

    ROS_INFO("Saved GUIDs in: %s", fileNameComplete2.c_str());

    return 0;
}
