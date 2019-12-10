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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include "utils/smart_ptr_make_macro.h"

#include "dnn/CVObjectDetector.h"
#include "dnn/GrpcObjectDetector.h"
#include "dnn/GrpcObjectDetectorV2.h"

#include "utils/Config.h"
#include <boost/filesystem.hpp>
#include <pcl/io/ply_io.h>

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

bool System::bLocalMapAcceptKF()
{
    return (mpLocalMapper->AcceptKeyFrames() && !mpLocalMapper->isStopped());
}


void System::SaveKeyFrameTrajectoryNavState(const string &filename)
{
    cout << endl << "Saving keyframe NavState to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        Eigen::Vector3d P = pKF->GetNavState().Get_P();
        Eigen::Vector3d V = pKF->GetNavState().Get_V();
        Eigen::Quaterniond q = pKF->GetNavState().Get_R().unit_quaternion();
        Eigen::Vector3d bg = pKF->GetNavState().Get_BiasGyr();
        Eigen::Vector3d ba = pKF->GetNavState().Get_BiasAcc();
        Eigen::Vector3d dbg = pKF->GetNavState().Get_dBias_Gyr();
        Eigen::Vector3d dba = pKF->GetNavState().Get_dBias_Acc();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " ";
        f << P(0) << " " << P(1) << " " << P(2) << " ";
        f << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " ";
        f << V(0) << " " << V(1) << " " << V(2) << " ";
        f << bg(0)+dbg(0) << " " << bg(1)+dbg(1) << " " << bg(2)+dbg(2) << " ";
        f << ba(0)+dba(0) << " " << ba(1)+dba(1) << " " << ba(2)+dba(2) << " ";
//        f << bg(0) << " " << bg(1) << " " << bg(2) << " ";
//        f << ba(0) << " " << ba(1) << " " << ba(2) << " ";
//        f << dbg(0) << " " << dbg(1) << " " << dbg(2) << " ";
//        f << dba(0) << " " << dba(1) << " " << dba(2) << " ";
        f << endl;
    }

    f.close();
    cout << endl << "NavState trajectory saved!" << endl;
}


cv::Mat System::TrackMonoVI(const cv::Mat &im, const utils::eigen_aligned_vector<ORB_SLAM2::IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    if (!Config::getInstance().SystemParams().use_imu){
        cerr << "Track MonoVI must set flag use_imu" << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    return mpTracker->GrabImageMonoVI(im,vimu,timestamp);
}

cv::Mat System::TrackStereoVI(const cv::Mat &imLeft, const cv::Mat &imRight,
                            const utils::eigen_aligned_vector<IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereoVI(imLeft,imRight, vimu,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
//    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
//    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBDVI(const cv::Mat &im, const cv::Mat &depthmap,
                          const utils::eigen_aligned_vector<IMUData> &vimu, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBDVI(im,depthmap, vimu, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
//    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
//    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}


//----------------------------

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    InitLogger();
    // Output welcome message
    cerr << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    // Read global configuration
    Config::getInstance().readConfig(strSettingsFile);

    std::string sensor_str;
    if(mSensor==MONOCULAR)
        sensor_str = "Monocular";
    else if(mSensor==STEREO)
        sensor_str = "Stereo";
    else if(mSensor==RGBD)
        sensor_str = "RGB-D";

    mbUseObject = Config::getInstance().SystemParams().use_object;
    SPDLOG_INFO("Input sensor was set to: {0}", sensor_str);
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       //cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        SPDLOG_CRITICAL("Failed to open settings file at: {0}", strSettingsFile);
        exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    //bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        SPDLOG_CRITICAL("Wrong path to vocabulary. Failed to open file at: {0}", strVocFile);
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    // Init Object Detector
    // TODO -- Implement proper object detector factory
    if (mbUseObject){
        mpObjectDetector = BuildObjectDetector(Config::getInstance().ObjectDetectionParams().type);
        auto lbmap = mpObjectDetector->getLabelMap();
        SPDLOG_INFO("Label map size {}", lbmap.size());
        mpFrameDrawer->SetLabelMap(mpObjectDetector->getLabelMap());
    }

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor,
                             mpObjectDetector);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer, mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    if (Config::getInstance().SystemParams().use_imu && !Config::getInstance().IMUParams().fast_init && Config::getInstance().SystemParams().real_time){
        mptLocalMappingVIOInit = std::make_shared<std::thread>(
                &ORB_SLAM2::LocalMapping::VINSInitThread,
                mpLocalMapper
        );
    }
}

System::~System() {
    Shutdown(true);
}

void System::InitLogger() {
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("%^[%E.%F][%l][%!:%@] %v%$");
    /*
     *  Multisink example to both screen and file
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::warn);
    console_sink->set_pattern("[multi_sink_example] [%^%l%$] %v");

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>("logs/multisink.txt", true);
    file_sink->set_level(spdlog::level::trace);

    std::shared_ptr<spdlog::logger> logger =
        std::shared_ptr<spdlog::logger>(new spdlog::logger( "multi_sink", {console_sink, file_sink}));
    spdlog::set_default_logger(logger);
*/
}

std::shared_ptr<BaseObjectDetector> System::BuildObjectDetector(string type) {
    std::shared_ptr<BaseObjectDetector> pBaseDetector;
    auto objectDetectorParam = Config::getInstance().ObjectDetectionParams();
    if(type == "CV"){
        std::shared_ptr<CVObjectDetector> pObjDetector
                = std::make_shared<CVObjectDetector>(
                        objectDetectorParam.model_path,
                        objectDetectorParam.config_path,
                        CV_DNN_FRAMEWORK_DARKNET
                );
        pObjDetector->setInputSize(objectDetectorParam.input_size);
        pObjDetector->setLabelMap(objectDetectorParam.label_map);
        pObjDetector->setConfidenceThreshold(objectDetectorParam.min_confidence);
        pObjDetector->setApplyNMS(objectDetectorParam.apply_nms);
        pObjDetector->setNMSThreshold(objectDetectorParam.nms_threshold);

        pBaseDetector = std::static_pointer_cast<BaseObjectDetector>(pObjDetector);
    }
    else if (type == "GRPC"){
        // std::shared_ptr<>
        std::shared_ptr<GrpcObjectDetector> pObjDetector =
                std::make_shared<GrpcObjectDetector>(objectDetectorParam.grpc_url);

        pObjDetector->setLabelMap(objectDetectorParam.label_map);

        pBaseDetector = std::static_pointer_cast<BaseObjectDetector>(pObjDetector);
    }
    else if (type == "GRPCV2"){
        std::shared_ptr<GrpcObjectDetectorV2 > pObjDetector =
                std::make_shared<GrpcObjectDetectorV2 >(objectDetectorParam.grpc_url);

        pObjDetector->setLabelMap(objectDetectorParam.label_map);

        pBaseDetector = std::static_pointer_cast<BaseObjectDetector>(pObjDetector);
    }
    else {
        SPDLOG_CRITICAL("Detector Type {} is not implemented!", type);
        exit(-1);
    }

    return pBaseDetector;
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown(bool bShutDownViewer)
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }
    mpMap->ShutDown();
    if(bShutDownViewer){
        if(mpViewer) {
            mpViewer->RequestFinish();
            while(!mpViewer->isFinished())
                usleep(5000);
        }

        if(mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

    if (mptLocalMappingVIOInit){
        if (mptLocalMappingVIOInit->joinable())
            mptLocalMappingVIOInit->join();
    }
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUMWithObjects(const string &outdir_str)
{
    auto logp = boost::filesystem::path(outdir_str);
    std::string outdir = outdir_str;
    cout << endl << "Saving ObjectSLAM Result to " << outdir << " ..." << endl;
    if (boost::filesystem::exists(logp) && boost::filesystem::is_directory(logp)){
        boost::filesystem::remove_all(logp);
    }
    else {
        boost::filesystem::create_directory(logp);
    }
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);



    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.


    ofstream f, ftrack, fobj, f_with_id;

    if (outdir.back() != '/'){
        outdir += '/';
    }

    // Save All Frames trajectory
    if(mSensor!=MONOCULAR)
    {
        ftrack.open(outdir + "full_trajectory.txt");
        ftrack << fixed;

        cv::Mat Two = vpKFs[0]->GetPoseInverse();
        list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
                    lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
        {
            if(*lbL)
                continue;

            KeyFrame* pKF = *lRit;

            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while(pKF->isBad())
            {
                Trw = Trw*pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw*pKF->GetPose()*Two;

            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            ftrack << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }


        ftrack.close();
    }
    // ----------------- Save Keyframe trajectory only ---------------------

    f.open(outdir + "keyframes.txt");
    f_with_id.open(outdir + "keyframes_with_id.txt");
    f << fixed;
    f_with_id << "#id stamp x y z qx qy qz qw" << std::endl;
    f_with_id << fixed;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        f_with_id << pKF->mnId << " " << setprecision(6)
                  << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
                  << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }

    f.close();
    f_with_id.close();

    auto vpMOs = mpMap->GetAllMapObjects();
    SPDLOG_INFO("Trajectory saved!, saving object pointclouds .. ");

    if (!boost::filesystem::exists(logp)){
        boost::filesystem::create_directory(logp);
    }

    auto pcddir = logp / boost::filesystem::path("object_pointclouds/");
    if (boost::filesystem::exists(pcddir) && boost::filesystem::is_directory(pcddir)){
        boost::filesystem::remove_all(pcddir);
    }

    boost::filesystem::create_directory(pcddir);

    std::string pcddirstr = pcddir.string();
    std::cout << "Saving pointcloud to folder: " << pcddir << std::endl;

    fobj.open(outdir + "objects.txt");
    fobj << "#id label x y z qx qy qz qw sx/2 sy/2 sz/2" << std::endl;
    fobj << fixed;

    pcl::PLYWriter writer;

    for (auto& pMO : vpMOs){
        if (! pMO->IsReady())
            continue;

        Eigen::Vector10d c = pMO->GetCuboidPtr()->toVector();

        fobj << pMO->mnId << " " << pMO->mLabel << setprecision(6);
        for (int i=0; i < 10; i++)
            fobj << " " << c[i];

        fobj << std::endl;

        // Pointclouds

        auto cloud = PCLConverter::toPointCloud(pMO->GetMapPoints());
        std::stringstream ss;
        ss << pcddirstr << '/' << pMO->mnId << ".ply";
        writer.write(ss.str(), *cloud);

    }

    fobj.close();


    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
