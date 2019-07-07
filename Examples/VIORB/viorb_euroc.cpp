//
// Created by rasp on 17/4/2561., modified by kandithws
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cstdio>//to pause console screen
#include <typeinfo>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../include/System.h"

#include "../../include/imu/IMUData.h"
#include "../../include/utils/Config.h"
#include "../../include/utils/vector_utils.h"

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;

int main(int argc, char **argv) {

    if(argc != 3)
    {
        cerr << endl << "Usage: ./VIORB vocabulary_file path_to_settings" << endl;
        return 1;
    }
    std::cout << "Reading MAV config file...\n";
    //MAV::ConfigParam configmav(argv[1]);
    std::cout << "Starting SLAM...\n";
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);

    std::cout << "Finish reading SLAM config file...\n";
    std::cout << "Checking for dataset...\n";
    //Checking files

    auto runtime_cfg = ORB_SLAM2::Config::getInstance().RuntimeParams();
    auto imu_cfg = ORB_SLAM2::Config::getInstance().IMUParams();
    if (!(boost::filesystem::is_directory(boost::filesystem::path(runtime_cfg.bagfile)))){
        throw std::runtime_error("Dataset for EuRoC should be in a directory. (Not ROSBAG)");
    }

    std::string img_path = runtime_cfg.bagfile + runtime_cfg.image_topic + + "/data";
    boost::filesystem::path dir(img_path);
    if (!(boost::filesystem::exists(dir))) {
        throw std::runtime_error("Images do not exist or in wrong format");
    }
    std::ifstream imu(runtime_cfg.bagfile + runtime_cfg.imu_topic + "/data.csv");
    if (!imu.is_open()) {
        std::cout << "ERROR: Cannot Open IMU File" << std::endl;
        throw 1;
    }

    //create result file
    std::ofstream visionpose, processing_time;
    boost::filesystem::path log_file_p(runtime_cfg.log_file_path);
    if (!(boost::filesystem::exists(log_file_p))) {
        boost::filesystem::create_directory(log_file_p);
    }
    visionpose.open(runtime_cfg.log_file_path + "/slam_pose.txt");
    processing_time.open(runtime_cfg.log_file_path + "/processing_time.csv");
    processing_time << "t1" << "," << "t2" << "," << "dt" << "," << "state" << std::endl;
    // Read all images from folder order by name(default)
    std::vector<cv::String> fn;
    cv::glob(img_path + "/*.png", fn, false);

    cv::Mat image;
    double xgyro, ygyro, zgyro, ax, ay, az, timestamp;
    std::string getval;
    cv::Mat vision_estimated_pose;
    static double startT=-1;

    /**
    * @brief added data sync
    */
    double imageMsgDelaySec = runtime_cfg.image_delay_to_imu;
    bool bAccMultiply98 = runtime_cfg.multiply_g;
    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = imu_cfg.g;
    bool isFirstFrame = false;

    // load each frame with timestamp from name along with IMU data
    size_t count = fn.size(); //number of png files in images folder
    for (size_t i=0; i<count; i++) {
        std::string stimestamp = fn[i].substr(fn[i].find_last_of("/") + 1, 19);
        double timestamp_camera = std::stod(stimestamp) / 1e9;
        image = cv::imread(fn[i]);

        ORB_SLAM2::utils::eigen_aligned_vector<ORB_SLAM2::IMUData> vimuData;

        while (imu.good()) {
            getline(imu, getval, ',');
            timestamp = atof(getval.c_str()) / 1e9; //cout << " timestamp : " << timestamp << endl;
            getline(imu, getval, ',');
            xgyro = atof(getval.c_str()); // cout << " xgyro : " << xgyro << endl;
            getline(imu, getval, ',');
            ygyro = atof(getval.c_str()); // cout << " ygyro : " << ygyro << endl;
            getline(imu, getval, ',');
            zgyro = atof(getval.c_str()); // cout << " zgyro : " << zgyro << endl;
            getline(imu, getval, ',');
            ax = atof(getval.c_str());// cout << " xacc : " << xacc << endl;
            getline(imu, getval, ',');
            ay = atof(getval.c_str()); // cout << " yacc : " << yacc << endl;
            getline(imu, getval, '\n');
            az = atof(getval.c_str()); // cout << " zacc : " << zacc << endl;
            if (bAccMultiply98) {
                ax *= g3dm;
                ay *= g3dm;
                az *= g3dm;
            }

            if (timestamp >= timestamp_camera) {
                if(vimuData.size()==0) {
                    cout << "no imu message between images! \n";
                    break;
                }
                // Consider delay of image message

                if(startT<0)
                    startT = timestamp_camera;
                // Below to test relocalizaiton
                //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                if(timestamp_camera < startT+runtime_cfg.discard_time)
                    image = cv::Mat::zeros(image.rows,image.cols,image.type());

//                std::cout << "-------------------" << '\n';
//                std::cout << std::setprecision(19) << "Lastest IMU timestamp: " << timestamp << '\n';
//                std::cout << "Total Number of IMU: " << vimuData.size() << '\n';
//                std::cout << "-------------------" << '\n';

                //skip first frame
                if (!isFirstFrame) {
                    vimuData.clear();
                    isFirstFrame = true;
                } else {

                    // Check dis-continuity, tolerance 3 seconds
                    if(timestamp_camera - imageMsgDelaySec + 3.0 < vimuData.front()._t)
                    {
                        cout << "Data dis-continuity, > 3 seconds. Buffer cleared \n";
                        vimuData.clear();
                        continue;
                    }

                    uint64_t  cp_time1 = (boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count()));

                    // Pass the image to the SLAM system
                    vision_estimated_pose = SLAM.TrackMonoVI(image, vimuData, timestamp_camera - imageMsgDelaySec);
//                    cout << "visionpose : " << vision_estimated_pose << endl;

                    uint64_t  cp_time2 = (boost::lexical_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch()).count()));

                    vimuData.clear();
                    // angular_velocity.x, angular_velocity.y, angular_velocity.z, linear_acceleration ax, ay, az, timestamp
                    ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, ax, ay, az, timestamp);
                    vimuData.push_back(imudata);
                }
                break;

            } else {
                ORB_SLAM2::IMUData imudata(xgyro, ygyro, zgyro, ax, ay, az, timestamp);
                vimuData.push_back(imudata);
            }
        }
    }

    bool bstop = false;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM(runtime_cfg.log_file_path + "/KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryNavState(runtime_cfg.log_file_path + "/KeyFrameNavStateTrajectory.txt");

    cout<<endl<<endl<<"press any key to shutdown"<<endl;
    getchar();

    // Stop all threads
    cout << "shuting down...";
    SLAM.Shutdown();

    return 0;
}