""" Publishes pointclouds to use in ROS

    This script converts .ply files (obtained from launch_ego_vehicle.py) to a 
    ROS PointCloud2 message type and publishes the same. 

    REQUIREMENTS:
        Requires rospkg, netifaces, defusedxml for Python3. 
        Installation: 
            pip3 install rospkg netifaces defusedxml 

            OR

            python3 -m pip install rospkg netifaces defusedxml 
            
    USAGE:
        [FOR ALOAM]
        python3 pcd_publisher.py --data_dir=lidar_output/dynamic2 --topic=velodyne_points --frame=camera_init

        [FOR LEGO_LOAM]
        python3 pcd_publisher.py --data_dir=lidar_output/dynamic2 --topic=velodyne_points --frame=velodyne

"""
import open3d as o3d
import numpy as np
import rospy
import sensor_msgs
import std_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import glob
import argparse
from roslib import message
from rosgraph_msgs.msg import Clock


# def point_cloud(points, parent_frame):
#     ros_dtype = sensor_msgs.msg.PointField.FLOAT32

#     dtype = np.float32

#     itemsize = np.dtype(dtype).itemsize

#     data = points.astype(dtype).tobytes()

#     fields = [sensor_msgs.msg.PointField(
#         name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
#         for i, n in enumerate('xyz')]

#     header = std_msgs.msg.Header(frame_id=parent_frame, stamp=rospy.Time.now())

#     return sensor_msgs.msg.PointCloud2(
#         header=header,
#         height=1,
#         width=points.shape[0],
#         is_dense=False,
#         is_bigendian=False,
#         fields=fields,
#         point_step=(itemsize * 3),
#         row_step=(itemsize * 3 * points.shape[0]),
#         data=data
#     )

# def open3d_ply(filename):
#     pcd = o3d.io.read_point_cloud(filename)

#     pcd = np.asarray(pcd.points)
#     pcd = pcd.astype('float32')
#     pcd[:, 1] = -pcd[:, 1]
#     return pcd

def pcd_2_point_cloud(points, parent_frame, frametime):
    ros_dtype = sensor_msgs.msg.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize
    data = points.astype(dtype).tobytes()
    fields = [
        sensor_msgs.msg.PointField(
            name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate(['x', 'y', 'z', 'intensity', 'ring'])
    ]
    header = std_msgs.msg.Header(frame_id=parent_frame, stamp=frametime)

    return sensor_msgs.msg.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 5),
        row_step=(itemsize * 5 * points.shape[0]),
        data=data
    )


def read_pcd(filename):
    with open(filename, 'r') as f:
        data = [x.strip() for x in f.readlines()]

    data = data[9:]
    data = [[float(x) for x in line.split()] for line in data]
    data = np.array(data, dtype='float32')
    data[:, 1] = -data[:, 1]

    # get ring channel
    depth = np.linalg.norm(data, 2, axis=1)
    pitch = np.arcsin(data[:, 2] / depth)  # arcsin(z, depth)
    fov_down = -24.8 / 180.0 * np.pi
    fov = (abs(-24.8) + abs(2.0)) / 180.0 * np.pi
    proj_y = (pitch + abs(fov_down)) / fov  # in [0.0, 1.0]
    proj_y *= 64  # in [0.0, H]
    proj_y = np.floor(proj_y)
    proj_y = np.minimum(64 - 1, proj_y)
    proj_y = np.maximum(0, proj_y).astype(np.int32)  # in [0,H-1]
    proj_y = proj_y.reshape(-1, 1)
    data = np.concatenate((data, proj_y), axis=1)

    print('[INFO] PCD Shape: ', data.shape)
    # data[np.isnan(data)] = 0.0

    return data


def main(args):
    rospy.init_node(args.node)
    pub_points = rospy.Publisher(args.topic, PointCloud2, queue_size=1)
    rate = rospy.Rate(30)  # hz

    for datafile in sorted(glob.glob(args.data_dir + '/*'))[args.skip:]:
        frametime = rospy.Time.now()

        pcd = read_pcd(datafile)
        pcl2data = pcd_2_point_cloud(pcd, args.frame, frametime)

        pub_points.publish(pcl2data)

        if rospy.is_shutdown():
            print('shutdown')
            break
        rate.sleep()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--data_dir',
        metavar='D',
        help='Path where lidar files are stored')
    argparser.add_argument(
        '--node',
        metavar='N',
        default='pcd_publisher',
        help='Name of the node that will publish data')
    argparser.add_argument(
        '--frame',
        metavar='F',
        help='Frame of the points being published')
    argparser.add_argument(
        '--topic',
        metavar='T',
        help='Topic at which the data will be published')
    argparser.add_argument(
        '--skip',
        metavar='S',
        type=int,
        default=100,
        help='Skip first few frames')

    args = argparser.parse_args()

    main(args)
