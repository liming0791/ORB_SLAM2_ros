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

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Converter.h"
#include "orb_slam2/GetIMUPoseWorldService.h"

using namespace std;


int SkipFrameNum = 0;
int FrameIndex = 0, FrameCount = 0;
ros::ServiceClient *pClient;
int status = 0;     // Status of ORB_SLAM2 tracking
                    // SYSTEM_NOT_READY=-1,
                    // NO_IMAGES_YET=0,
                    // NOT_INITIALIZED=1,
                    // OK=2,
                    // LOST=3

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher *pPosPub, *pTransPub;

};

void GetIMUPoseWorld(double *Q)
{
    orb_slam2::GetIMUPoseWorldService srv;
    if (pClient->call(srv)) {
        ROS_INFO("Get Q: %f %f %f %f\n", 
                srv.response.w, srv.response.x, srv.response.y, srv.response.z);
        // Get Q, and convert Axis
        Q[0] = srv.response.w;
        Q[1] = -srv.response.y;
        Q[2] = srv.response.z;
        Q[3] = srv.response.x;
    } else {
        ROS_INFO("Get Q Failed !");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    ros::NodeHandle nh;

    ROS_INFO("run init");

    // Get params
    string VocabularyFile = "empty";
    string SettingFile = "empty";
    nh.getParam("VocabularyFile", VocabularyFile);
    nh.getParam("SettingFile", SettingFile);
    nh.getParam("SkipFrameNum", SkipFrameNum);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(VocabularyFile.c_str() ,SettingFile.c_str(), ORB_SLAM2::System::RGBD, true);

    ImageGrabber igb(&SLAM);

    // Subscriber for images
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    // Publisher for pose
    ros::Publisher TransPub = nh.advertise<geometry_msgs::TransformStamped>("orb_slam/trans", 5);
    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("orb_slam/pose", 5);
    igb.pTransPub = &(TransPub);
    igb.pPosPub = &(PosPub);

    // Client for IMU Pose World Service
    ros::ServiceClient client = 
            nh.serviceClient<orb_slam2::GetIMUPoseWorldService>("GetIMUPoseWorldService");
    pClient = &client;

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save map
//    if (SLAM.SaveMap("SlamMap.bin")) {
//        ROS_INFO("Save Map Success!");
//    } else {
//        ROS_INFO("Save Map Failed!");
//    }

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{

    // Skip Frame to limit FPS
    FrameIndex++;
    if (FrameIndex <= SkipFrameNum) {
        return;
    } else {
        FrameIndex = 0;
    }

    // Test Load Map During Slam
//    FrameCount++;
//    if (FrameCount == 100) {
//        if (mpSLAM->LoadMap("SlamMap.bin")) {
//            ROS_INFO("LoadMap Done!");
//        } else {
//            ROS_INFO("LoadMap Failed!");
//        }
//        FrameCount = 0;
//    }

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try{
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try{
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (status <= 1) {
        double q[4];
        GetIMUPoseWorld(q);
        cv::Mat RMat(4, 4, CV_32F);
        float* R = RMat.ptr<float>(0);
        float q02 = q[0]*q[0], q12 = q[1]*q[1], q22 = q[2]*q[2], q32 = q[3]*q[3];
        float dq0q1 = 2*q[0]*q[1], dq1q2 = 2*q[1]*q[2], dq2q3 = 2*q[2]*q[3], dq3q0 = 2*q[3]*q[0];
        float dq0q2 = 2*q[0]*q[2], dq1q3 = 2*q[1]*q[3];
        R[0] = q02+q12-q22-q32;   R[4] = dq1q2 + dq3q0;    R[8] = dq1q3 - dq0q2;    R[12] = 0;
        R[1] = dq1q2 - dq3q0;     R[5] = q02-q12+q22-q32;  R[9] = dq2q3 + dq0q1;    R[13] = 0;
        R[2] = dq1q3 + dq0q2;     R[6] = dq2q3 - dq0q1;    R[10] = q02-q12-q22+q32; R[14] = 0;
        R[3] = 0;                 R[7] = 0;                R[11] = 0;               R[15] = 1;
        mpSLAM->SetInitR(RMat);
    }
    cv::Mat Tcw;
    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),
            &Tcw, &status);
    PublishPose(Tcw);
}

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    // Pose edition
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
        poseMSG.header.frame_id = "world";
        poseMSG.header.stamp = ros::Time::now();

        (pPosPub)->publish(poseMSG);
    }

    // Transform edition
    geometry_msgs::TransformStamped transformMSG;
    if(!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        
        transformMSG.child_frame_id = "/orb_slam/trans";

        transformMSG.transform.translation.x = -twc.at<float>(2);
        transformMSG.transform.translation.y = twc.at<float>(0);
        transformMSG.transform.translation.z = -twc.at<float>(1);
        
        transformMSG.transform.rotation.x = -q[3];
        transformMSG.transform.rotation.y =  q[1];
        transformMSG.transform.rotation.z = -q[2];
        transformMSG.transform.rotation.w = q[0];

        transformMSG.header.frame_id = "world";
        transformMSG.header.stamp = ros::Time::now();

        (pTransPub)->publish(transformMSG);
    }
}
