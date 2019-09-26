#!/usr/bin/env python2
import rosbag
import argparse
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped
import os.path as osp
import os
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import sys


def progressbar_iterator(it, prefix="", size=40, file=sys.stdout):
    count = len(it)
    def show(j):
        x = int(size*j/count)
        file.write("%s[%s%s] %i/%i\r" % (prefix, "#"*x, "."*(size-x), j, count))
        file.flush()
    show(0)
    for i, item in enumerate(it):
        yield item
        show(i+1)
    file.write("\n")
    file.flush()

# ETH3D
DS_MAP = {
    '/cam0': 'rgb',
    '/cam1': 'rgb2',
    '/depth': 'depth',
    '/imu0': 'imu.txt',
    '/imu1': 'imu2.txt',
    '/gt': 'groundtruth.txt'
}

bridge = CvBridge()


def _get_timestamp_from_filename(fname):
    return rospy.Time.from_seconds(float(osp.splitext(fname)[0]))


def _process_images(bag, topic, input_dir):
    image_folder_path = osp.join(input_dir, DS_MAP[topic])
    image_files = [f for f in os.listdir(image_folder_path) if osp.splitext(f)[-1] == '.png']
    image_files = sorted(image_files)
    for image_file in progressbar_iterator(image_files, 'Processed Messages {}: '.format(topic)):
        cv_img = cv2.imread(osp.join(image_folder_path, image_file), -1) # CV_LOAD_IMAGE_UNCHANGED
        try:
            if len(cv_img.shape) == 3:
                img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            else:
                img_msg = bridge.cv2_to_imgmsg(cv_img, encoding='mono16') # ORBSLAM depth PNG format

            stamp = _get_timestamp_from_filename(image_file)
            img_msg.header.frame_id = topic
            img_msg.header.stamp = stamp
            bag.write(topic + '/image_raw', img_msg, t=stamp)
        except CvBridgeError as e:
            print(e)


def _process_imu_file(bag, topic, input_dir):

    imu_data = np.genfromtxt(osp.join(input_dir, DS_MAP[topic]), delimiter=' ', dtype=float)

    assert(len(imu_data.shape) == 2)
    assert(imu_data.shape[1] == 7)
    msg = Imu()
    msg.header.frame_id = topic
    ## TODO -- set cov!
    for d in progressbar_iterator(imu_data, 'Processed Messages {}: '.format(topic)):
        msg.header.stamp = rospy.Time.from_seconds(d[0])
        msg.angular_velocity.x = d[1]
        msg.angular_velocity.y = d[2]
        msg.angular_velocity.z = d[3]
        msg.linear_acceleration.x = d[4]
        msg.linear_acceleration.y = d[5]
        msg.linear_acceleration.z = d[6]
        bag.write(topic, msg, t=msg.header.stamp)


def _process_gt_file(bag, topic, input_dir):

    gt_data = np.genfromtxt(osp.join(input_dir, DS_MAP[topic]), delimiter=' ', dtype=float)

    assert (len(gt_data.shape) == 2)
    assert (gt_data.shape[1] == 8)
    msg = PoseStamped()
    msg.header.frame_id = '/cam0'
    ## TODO -- set cov!
    for d in progressbar_iterator(gt_data, 'Processed Messages {}: '.format(topic)):
        msg.header.stamp = rospy.Time.from_seconds(d[0])

        msg.pose.position.x = d[1]
        msg.pose.position.y = d[2]
        msg.pose.position.z = d[3]

        msg.pose.orientation.x = d[4]
        msg.pose.orientation.y = d[5]
        msg.pose.orientation.z = d[6]
        msg.pose.orientation.w = d[7]
        bag.write(topic, msg, t=msg.header.stamp)


def main():
    parser = argparse.ArgumentParser(description='Convert ETH3D to minimal (EuRoC-like) rosbag stereo and RGB-D VIO')
    parser.add_argument('input_dir', help='Input ETH3D dataset. Should include rgb rgb2 and imu.txt.'
                                          'Will add depth and groundtruth if provided')
    parser.add_argument('--output', default='output.bag')
    args = parser.parse_args()
    print('Converting ETH3D dataset from dir {}, to {}'.format(args.input_dir, args.output))
    assert(osp.isdir(osp.join(args.input_dir, DS_MAP['/cam0'])))
    assert(osp.isdir(osp.join(args.input_dir, DS_MAP['/cam1'])))
    assert(osp.isfile(osp.join(args.input_dir, DS_MAP['/imu0'])))
    bag = rosbag.Bag(args.output, 'w')
    try:

        _process_images(bag, '/cam0', args.input_dir)

        _process_images(bag, '/cam1', args.input_dir)

        _process_imu_file(bag, '/imu0', args.input_dir)

        # For depth images in ORB-SLAM2, it will treat depth as grayscale image, no float!
        if osp.isdir(osp.join(args.input_dir, DS_MAP['/depth'])):
            _process_images(bag, '/depth', args.input_dir)

        if osp.isfile(osp.join(args.input_dir, DS_MAP['/imu1'])):
            _process_imu_file(bag, '/imu1', args.input_dir)

        if osp.isfile(osp.join(args.input_dir, DS_MAP['/gt'])):
            _process_gt_file(bag, '/gt', args.input_dir)

        print('---------- DONE ! ---------')

    finally:
        bag.close()



if __name__ == '__main__':
    main()
