/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include "Converter.h"
#include"System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
    bool bReuseMap = false;
    
    string VocabularyFile = "empty";
    string SettingFile = "empty";
    string ImageTopic = "empty";

    ros::NodeHandle nodeHandler;

    nodeHandler.getParam("VocabularyFile", VocabularyFile);
    nodeHandler.getParam("SettingFile", SettingFile);
    nodeHandler.getParam("ImageTopic", ImageTopic);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
   	ORB_SLAM2::System SLAM(VocabularyFile.c_str(), SettingFile.c_str(), ORB_SLAM2::System::MONOCULAR,true, bReuseMap);
    if (bReuseMap)
		SLAM.LoadMap("Slam_Map.bin");
	ImageGrabber igb(&SLAM);
    ros::Subscriber sub = nodeHandler.subscribe(ImageTopic.c_str(), 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher PosPub = nodeHandler.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);
    igb.pPosPub = &(PosPub);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    // Save map
    SLAM.SaveMap("Slam_latest_Map.bin");
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw= mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    PublishPose(Tcw);
}

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        
        poseMSG.pose.position.x = -twc.at<float>(2);
        poseMSG.pose.position.y = twc.at<float>(0);
        poseMSG.pose.position.z = -twc.at<float>(1);
        poseMSG.pose.orientation.x = -q[3];
        poseMSG.pose.orientation.y =  q[1];
        poseMSG.pose.orientation.z = -q[2];
        poseMSG.pose.orientation.w = q[0];
        poseMSG.header.frame_id = "VSLAM";
        poseMSG.header.stamp = ros::Time::now();

        (pPosPub)->publish(poseMSG);
    }
}
