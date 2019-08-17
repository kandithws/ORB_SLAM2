//
// Created by kandithws on 15/8/2562.
//
#include "StereoMsgSynchronizer.h"

namespace ORBVIO
{

StereoMsgSynchronizer::StereoMsgSynchronizer(const double &imagedelay1,
        const double &imagedelay2, bool bRealtime)
        : _img1Delay(imagedelay1), _img2Delay(imagedelay2), _realTimeFlag(bRealtime) {

    // Assumption, use everything based on img1 because ORBSLAM2 mostly rely on left image
    // (we just need the second image for scale computation)
    _imageMsgDelaySec = _img1Delay;

    if (!_realTimeFlag){
        _img1MsgFilter = std::shared_ptr<message_filters::PassThrough<sensor_msgs::Image> >();
        _img2MsgFilter = std::shared_ptr<message_filters::PassThrough<sensor_msgs::Image> >();

        // message_filters::Synchronizer
        _syncFilter = std::make_shared<message_filters::Synchronizer<sync_policy> >(
                sync_policy(10),
                *_img1MsgFilter,
                *_img2MsgFilter );

    }
    else{
        ROS_ASSERT_MSG(!_bRealtime, "NOT IMPLEMENTED");
        // TODO use messsage_filters::Subscriber instead
    }

    _syncFilter->registerCallback(boost::bind(&StereoMsgSynchronizer::stereoSyncCallback, this, _1, _2));
}


void StereoMsgSynchronizer::addImuMsg(const sensor_msgs::ImuConstPtr &imumsg)
{
    std::unique_lock<std::mutex> lock(_mutexIMUQueue);


    if(_imageMsgDelaySec>=0) {
        _imuMsgQueue.push(imumsg);
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = imumsg->header.stamp;
            _status = INIT;
        }
    }
    else {
        // if there's no image messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if(imumsg->header.stamp.toSec() + _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            {
                _imuMsgQueue.push(imumsg);
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _imuMsgQueue.push(imumsg);
        }
    }



}

void StereoMsgSynchronizer::addImage1Msg(const sensor_msgs::ImageConstPtr &imgmsg){
    _img1MsgFilter->add(imgmsg);
}

void StereoMsgSynchronizer::addImage2Msg(const sensor_msgs::ImageConstPtr &imgmsg){
    _img2MsgFilter->add(imgmsg);
}

void StereoMsgSynchronizer::stereoSyncCallback(const sensor_msgs::ImageConstPtr& img1_msg_const,
        const sensor_msgs::ImageConstPtr& img2_msg_const){

    std::unique_lock<std::mutex> lock(_mutexImageQueue);

    sensor_msgs::ImageConstPtr img1_msg = boost::make_shared<sensor_msgs::Image const>(*img1_msg_const);
    sensor_msgs::ImageConstPtr img2_msg = boost::make_shared<sensor_msgs::Image const>(*img2_msg_const);

    if(_imageMsgDelaySec >= 0) {
        // if there's no imu messages, don't add image
        if(_status == NOTINIT)
            return;
        else if(_status == INIT)
        {
            // ignore all image messages with no imu messages between them
            // only add below images
            if(img1_msg->header.stamp.toSec() - _imageMsgDelaySec > _imuMsgTimeStart.toSec())
            {
                _imageMsgQueue.push(std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>(img1_msg,
                                                                                                      img2_msg));
                _status = NORMAL;
            }
        }
        else
        {
            // push message into queue
            _imageMsgQueue.push(std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>(img1_msg,
                    img2_msg));
        }
    }
    else {  // start by image message
        if(_status == NOTINIT)
        {
            _imuMsgTimeStart = img1_msg->header.stamp;
            _status = INIT;
        }
        else
        {   // no image data if there's no imu message
            _imageMsgQueue.push(
                    std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>(img1_msg, img2_msg));
        }

    }

    if(_realTimeFlag)
    {
        // Ignore earlier frames
        if(_imageMsgQueue.size()>2)
            _imageMsgQueue.pop();
    }
}

bool StereoMsgSynchronizer::getRecentMsgs(sensor_msgs::ImageConstPtr &img1_msg,
                                          sensor_msgs::ImageConstPtr &img2_msg,
                                          std::vector<sensor_msgs::ImuConstPtr> &vimumsgs)
{

    //unique_lock<mutex> lock2(_mutexIMUQueue);
    std::unique_lock<std::mutex> lock1(_mutexImageQueue);

    if(_status == NOTINIT || _status == INIT)
    {
        //ROS_INFO("synchronizer not inited");
        return false;
    }

    if(_imageMsgQueue.empty())
    {
        //ROS_INFO("no image stored in queue currently.");
        return false;
    }

    if(_imuMsgQueue.empty())
    {
        //ROS_WARN("no imu message stored, shouldn't");
        return false;
    }

    {
        //
        auto imsg = _imageMsgQueue.back();
        auto bmsg = _imuMsgQueue.front();

        // Check dis-continuity, tolerance 3 seconds
        if(imsg.first->header.stamp.toSec()-_imageMsgDelaySec + 3.0 < bmsg->header.stamp.toSec() )
        {
            ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
            clearMsgs();
            return false;
        }

        //
        imsg = _imageMsgQueue.front();
        bmsg = _imuMsgQueue.back();

        // Wait imu messages in case communication block
        if(imsg.first->header.stamp.toSec()-_imageMsgDelaySec > bmsg->header.stamp.toSec())
        {
            return false;
        }

        // Check dis-continuity, tolerance 3 seconds
        if(imsg.first->header.stamp.toSec()-_imageMsgDelaySec > bmsg->header.stamp.toSec() + 3.0)
        {
            ROS_ERROR("Data dis-continuity, > 3 seconds. Buffer cleared");
            clearMsgs();
            return false;
        }

        // Wait until the imu packages totolly com
        if(_imageMsgQueue.size()<10 && _imuMsgQueue.size()<15
           && imsg.first->header.stamp.toSec()-_imageMsgDelaySec>bmsg->header.stamp.toSec() )
        {
            //ROS_WARN_STREAM("here return, last imu time "<<);
            return false;

        }

    }

    // get image message
    auto imgmsg_pair = _imageMsgQueue.front();
    // img1_msg = imgmsg_pair.first;
    // img2_msg = imgmsg_pair.second;
    _imageMsgQueue.pop();

    // clear imu message vector, and push all imu messages whose timestamp is earlier than image message
    vimumsgs.clear();
    while(true)
    {
        // if no more imu messages, stop loop
        if(_imuMsgQueue.empty())
            break;

        // consider delay between image and imu serial
        sensor_msgs::ImuConstPtr tmpimumsg = _imuMsgQueue.front();
        if(tmpimumsg->header.stamp.toSec() < imgmsg_pair.first->header.stamp.toSec() - _imageMsgDelaySec)
        {
            // add to imu message vector
            vimumsgs.push_back(tmpimumsg);
            {
                std::unique_lock<std::mutex> lock(_mutexIMUQueue);
                _imuMsgQueue.pop();
            }

            _dataUnsyncCnt = 0;
        }
        else
        {
            if(_dataUnsyncCnt++ >10)
            {
                _dataUnsyncCnt = 0;
                //_imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
                clearMsgs();
                ROS_ERROR("data unsynced many times, reset sync");
                return false;
            }
            // stop loop
            break;
        }
    }

    // the camera fps 20Hz, imu message 100Hz. so there should be not more than 5 imu messages between images
    if(vimumsgs.size()>10)
        ROS_WARN("%lu imu messages between images, note",vimumsgs.size());
    if(vimumsgs.size()==0)
        ROS_ERROR("no imu message between images!");

    return true;
}

void StereoMsgSynchronizer::clearMsgs(void)
{
    _imuMsgQueue = std::queue<sensor_msgs::ImuConstPtr>();
    _imageMsgQueue = std::queue< std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> >();
}

}