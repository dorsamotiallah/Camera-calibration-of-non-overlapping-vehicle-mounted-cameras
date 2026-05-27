/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>

#include<opencv2/core/core.hpp>

#include "System.h"
#include "calib.hpp"

using namespace std;

class ImageGrabber
{
private:
    ORB_SLAM3::System* mpSLAM;
    string mName;
    unsigned long mnFrames;
    bool mbPublishAck;
    int mnAckLogEvery;
    ros::Publisher mAckPub;
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, const string& name, bool publishAck=false, int ackLogEvery=100)
        : mpSLAM(pSLAM), mName(name), mnFrames(0), mbPublishAck(publishAck),
          mnAckLogEvery(ackLogEvery > 0 ? ackLogEvery : 100){}

    void SetAckPublisher(const ros::Publisher& ackPub)
    {
        mAckPub = ackPub;
    }

    void PublishAck(const ros::Time& stamp)
    {
        if(!mbPublishAck || !mAckPub)
            return;

        std_msgs::Header ack;
        ack.seq = mnFrames;
        ack.stamp = stamp;
        ack.frame_id = mName;
        mAckPub.publish(ack);

        if(mnFrames == 1 || mnFrames % mnAckLogEvery == 0)
        {
            ROS_INFO("%s ACK frame %lu stamp %.6f",
                     mName.c_str(), mnFrames, stamp.toSec());
        }
    }

    void GrabMono(const sensor_msgs::ImageConstPtr& msgRGB)
    {
        const auto start = chrono::steady_clock::now();
        mnFrames++;
        if(mnFrames == 1 || mnFrames % 500 == 0)
        {
            ROS_INFO("%s processed image %lu stamp %.6f",
                     mName.c_str(), mnFrames, msgRGB->header.stamp.toSec());
        }

        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        mpSLAM->TrackMonocular(cv_ptrRGB->image, cv_ptrRGB->header.stamp.toSec());
        const auto end = chrono::steady_clock::now();
        const double elapsedMs = chrono::duration_cast<chrono::duration<double, milli>>(end - start).count();
        if(mbPublishAck && (mnFrames == 1 || mnFrames % mnAckLogEvery == 0))
        {
            ROS_INFO("%s TrackMonocular frame %lu took %.3f ms",
                     mName.c_str(), mnFrames, elapsedMs);
        }
        PublishAck(cv_ptrRGB->header.stamp);
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB1,const sensor_msgs::ImageConstPtr& msgD1)
    {
        const auto start = chrono::steady_clock::now();
        mnFrames++;
        if(mnFrames == 1 || mnFrames % 500 == 0)
        {
            ROS_INFO("%s processed RGB-D image %lu stamp %.6f",
                     mName.c_str(), mnFrames, msgRGB1->header.stamp.toSec());
        }

        // Copy the ros image message to cv::Mat.
        cv_bridge::CvImageConstPtr cv_ptrRGB1;
        try
        {
            cv_ptrRGB1 = cv_bridge::toCvShare(msgRGB1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD1;
        try
        {
            cv_ptrD1 = cv_bridge::toCvShare(msgD1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        mpSLAM->TrackRGBD(cv_ptrRGB1->image, cv_ptrD1->image, cv_ptrRGB1->header.stamp.toSec());
        const auto end = chrono::steady_clock::now();
        const double elapsedMs = chrono::duration_cast<chrono::duration<double, milli>>(end - start).count();
        if(mbPublishAck && (mnFrames == 1 || mnFrames % mnAckLogEvery == 0))
        {
            ROS_INFO("%s TrackRGBD frame %lu took %.3f ms",
                     mName.c_str(), mnFrames, elapsedMs);
        }
        PublishAck(cv_ptrRGB1->header.stamp);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calib_node");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "./build/calib path_to_vocabulary calib_settings camera1_setttings camera2_settings" << endl;
        ros::shutdown();
        return 1;
    }    
    
    // read config
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    string mode = (string)fsSettings["Mode"];
    bool use_viewer = (mode == "slam");
    cv::FileNode useViewerNode = fsSettings["UseViewer"];
    if(!useViewerNode.empty())
        use_viewer = ((int)useViewerNode) != 0;
    int camera1type = (int)fsSettings["Camera1.Type"];
    int camera2type = (int)fsSettings["Camera2.Type"];

    string camera1image = (string)fsSettings["Camera1.Image"];
    string camera2image = (string)fsSettings["Camera2.Image"];
    string camera1depth = (string)fsSettings["Camera1.Depth"];
    string camera2depth = (string)fsSettings["Camera2.Depth"];    

    bool publishAck = false;
    cv::FileNode ackEnabledNode = fsSettings["PlaybackAck.Enabled"];
    if(!ackEnabledNode.empty())
        publishAck = ((int)ackEnabledNode) != 0;

    string ackTopicPrefix = "/orbcalib";
    cv::FileNode ackTopicPrefixNode = fsSettings["PlaybackAck.TopicPrefix"];
    if(!ackTopicPrefixNode.empty())
        ackTopicPrefix = (string)ackTopicPrefixNode;
    if(!ackTopicPrefix.empty() && ackTopicPrefix.back() == '/')
        ackTopicPrefix.pop_back();

    int ackLogEvery = 100;
    cv::FileNode ackLogEveryNode = fsSettings["PlaybackAck.LogEvery"];
    if(!ackLogEveryNode.empty())
        ackLogEvery = (int)ackLogEveryNode;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM1(argv[1], argv[3], static_cast<ORB_SLAM3::System::eSensor>(camera1type), use_viewer, 0, "Camera 1");
    ORB_SLAM3::System SLAM2(argv[1], argv[4], static_cast<ORB_SLAM3::System::eSensor>(camera2type), use_viewer, 0, "Camera 2");

    if(mode == "calib"){
        cout << "atlas are loaded!!" << endl;
        cout << "start to calib..." << endl;
        CalibC2C c2c(&SLAM1, &SLAM2);
        c2c.RunCalib();        
        cout << "calib finish, exit" << endl;
        return 0;
    }

    ImageGrabber igb1(&SLAM1, "Camera 1", publishAck, ackLogEvery);
    ImageGrabber igb2(&SLAM2, "Camera 2", publishAck, ackLogEvery);
    ros::NodeHandle nh;

    ros::Publisher ackPub1;
    ros::Publisher ackPub2;
    if(publishAck)
    {
        ackPub1 = nh.advertise<std_msgs::Header>(ackTopicPrefix + "/camera1/processed", 100);
        ackPub2 = nh.advertise<std_msgs::Header>(ackTopicPrefix + "/camera2/processed", 100);
        igb1.SetAckPublisher(ackPub1);
        igb2.SetAckPublisher(ackPub2);
        ROS_INFO("Playback ACK enabled: %s/camera1/processed and %s/camera2/processed",
                 ackTopicPrefix.c_str(), ackTopicPrefix.c_str());
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber rgb_sub1, rgb_sub2;
    message_filters::Subscriber<sensor_msgs::Image> sync_rgb1, sync_depth1;
    message_filters::Subscriber<sensor_msgs::Image> sync_rgb2, sync_depth2;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync1(sync_pol(10), sync_rgb1, sync_depth1);
    message_filters::Synchronizer<sync_pol> sync2(sync_pol(10), sync_rgb2, sync_depth2);

    if(camera1type == 0)
    {
        rgb_sub1 = it.subscribe(camera1image, 1000, &ImageGrabber::GrabMono, &igb1);
    }else
    {
        sync_rgb1.subscribe(nh, camera1image, 1000);
        sync_depth1.subscribe(nh, camera1depth, 1000);
        sync1.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb1, _1, _2));
    }

    if(camera2type == 0)
    {
       rgb_sub2 = it.subscribe(camera2image, 1000, &ImageGrabber::GrabMono, &igb2);
    }else{
        sync_rgb2.subscribe(nh, camera2image, 1000);
        sync_depth2.subscribe(nh, camera2depth, 1000);
        sync2.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb2, _1, _2));
    }

    ros::spin();
    // Stop all threads
    SLAM1.Shutdown();
    SLAM2.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();
    cout << "SLAM are shutdown" << endl;

    return 0;
}
