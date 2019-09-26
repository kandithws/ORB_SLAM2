
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>


// Just to prove that C++ API unlike python, it will sort publish time for us
int main(int argc, char **argv) {
    rosbag::Bag bag;
    bag.open(argv[1]);
    int num_msgs = 100;
    for (rosbag::MessageInstance const m: rosbag::View(bag)){
        if (num_msgs < 0)
            break;

        if ((m.getTopic() != "/imu0") && (m.getTopic() != "/imu1")){
            std::cout << m.getTopic() << ": " << m.getTime() << std::endl;
            num_msgs -= 1;
        }
    }

}