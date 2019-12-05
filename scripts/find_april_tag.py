import apriltag
from PIL import Image
import argparse
import os.path as osp
import glob
import numpy as np
import transformations as tf
import csv
import os


# LAZY TO PARSE FROM ARGS
# From projection matrix
CAM_FX = 761.8414660933026
CAM_FY = 761.8414660933026
CAM_CX = 430.68267822265625
CAM_CY = 233.37753677368164

def main():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument('input_dir', help='input directory')
    # parser.add_argument('--cam', required=True, help='Camera parameter for pose (ROS.yaml)')
    parser.add_argument('--family', default='tag16h5', help='Apriltag family to detect!')
    parser.add_argument('--tagsize', type=float, default=0.135, help='Apriltag family to detect!')
    parser.add_argument('--imgtype', default='png', help='image type in a folder')
    args = parser.parse_args()
    detector_options = apriltag.DetectorOptions(
        families=args.family,
        refine_decode=True,
        refine_pose=True
    )

    fx, fy, cx, cy = CAM_FX, CAM_FY, CAM_CX, CAM_CY

    cam_params = (fx, fy, cx, cy)
    detector = apriltag.Detector(options=detector_options)
    results = []
    os.mkdir('outimages')
    for img_file in glob.glob(osp.join(args.input_dir, '*.{}'.format(args.imgtype))):
        
        img = np.array(Image.open(img_file).convert('L'))
        detections, img_result = detector.detect(img, True)
        # get keyframe id
        parts = osp.basename(img_file).split('-')
        assert(len(parts) == 3)
        keyframe_id = int(parts[1])
        
        for d in detections:
            # find pose
            pose, _, _ = detector.detection_pose(d, cam_params, tag_size=args.tagsize) 
            # Note, check order: qx,qy,qz,qw
            # assume return = qw, qx, qy, qz
            q = tf.quaternion_from_matrix(pose)
            t = pose[:3, 3]
            results.append( (keyframe_id, t[0], t[1], t[2], q[1], q[2], q[3], q[0]) )
        
        Image.fromarray(img_result).save(osp.join('outimages', 'out_{}.png'.format(keyframe_id)))
            


    with open('result.csv', 'w', newline='\n') as f:
        wr = csv.writer(f)
        wr.writerow('# kf_id,x,y,z,qx,qy,qz,qw'.split(','))
        wr.writerows(results)
        
if __name__ == '__main__':
    main()
