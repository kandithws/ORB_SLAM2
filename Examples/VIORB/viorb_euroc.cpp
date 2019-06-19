#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <cstdio>//to pause console screen
#include <typeinfo>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <System.h>
#include <utils/Config.h>
#include <imu/IMUData.h>

//TODO -- when free, implement this!
int main(int argc, char **argv){
    if(argc != 2)
    {
        cerr << endl << "Usage: Usage: ./viorb_euroc path_to_vocabulary"
                        " path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }
}