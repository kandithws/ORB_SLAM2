//
// Created by kandithws on 15/8/2562.
//

#ifndef VIORB_STEREOMSGSYNCHRONIZER_H
#define VIORB_STEREOMSGSYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <mutex>

#include <message_filters/time_sequencer.h>
#include <message_filters/subscriber.h>
#include <message_filters/pass_through.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>

/**
*   Idea use "Time Sequencer filter" (instead of subscriber) to sync timestamp btw 2 images, then push treat 2 ima
*   Then treat them as a single message then copy a logic from MsgSynchornizer to get past accumulated messages
*/


namespace ORBVIO
{

class StereoMsgSynchronizer {
  public:
    enum Status{
        NOTINIT = 0,
        INIT,
        NORMAL
    };

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;

    StereoMsgSynchronizer(const double &imagedelay1 = 0.,
            const double &imagedelay2 = 0.);

    StereoMsgSynchronizer(const std::string &image1_topic,
            const std::string &image2_topic,
            const std::string &imu_topic,
            const double& imagedelay1 = 0.);

    ~StereoMsgSynchronizer() = default;


    void addImage1Msg(const sensor_msgs::ImageConstPtr &imgmsg); // RGB or Left
    void addImage2Msg(const sensor_msgs::ImageConstPtr &imgmsg); // Depth or Right


    void addImuMsg(const sensor_msgs::ImuConstPtr &imumsg);

    void stereoSyncCallback(const sensor_msgs::ImageConstPtr& img1_msg,
            const sensor_msgs::ImageConstPtr& img2_msg);

    // loop in main function to handle all messages
    bool getRecentMsgs(
            sensor_msgs::ImageConstPtr &img1_msg,
            sensor_msgs::ImageConstPtr &img2_msg,
            std::vector<sensor_msgs::ImuConstPtr> &vimumsgs
            );

    void clearMsgs(void);

    // Work around for now
    std::shared_ptr<message_filters::Synchronizer<sync_policy> > _syncFilter;

  private:
    std::shared_ptr<message_filters::PassThrough< sensor_msgs::Image> > _img1MsgFilter;
    std::shared_ptr<message_filters::PassThrough< sensor_msgs::Image> > _img2MsgFilter;


    std::shared_ptr<message_filters::Subscriber< sensor_msgs::Image> > _img1MsgSubFilter;
    std::shared_ptr<message_filters::Subscriber< sensor_msgs::Image> > _img2MsgSubFilter;
    ros::Subscriber _imu_sub;

    // TODO -- Do we need Time Sequencer to chain from passthrough ??
    std::mutex _mutexImageQueue;
    std::queue< std::pair< sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> > _imageMsgQueue;
    std::mutex _mutexIMUQueue;
    std::queue< sensor_msgs::ImuConstPtr> _imuMsgQueue;

    double _img1Delay = 0.0;
    double _img2Delay = 0.0;
    bool _realTimeFlag = false;

    ros::Time _imuMsgTimeStart;
    double _imageMsgDelaySec;
    int _dataUnsyncCnt;

    Status _status;
};
}
#endif //VIORB_STEREOMSGSYNCHRONIZER_H
