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
        python3 pcd_publisher.py
"""
import open3d as o3d
import numpy as np
import rospy
import sensor_msgs, std_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import glob
import sys

def point_cloud(points, parent_frame):
    ros_dtype = sensor_msgs.msg.PointField.FLOAT32
    
    dtype = np.float32
    
    itemsize = np.dtype(dtype).itemsize
    
    data = points.astype(dtype).tobytes()
    
    fields = [sensor_msgs.msg.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.msg.Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return sensor_msgs.msg.PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3),
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )


def open3d_ply(filename):
    pcd = o3d.io.read_point_cloud(filename)

    pcd = np.asarray(pcd.points) 
    pcd = pcd.astype('float32')
    return pcd


def main(lidar_folder):
    rospy.init_node('maaromujhe')
    pub_points = rospy.Publisher('velodyne_points', PointCloud2, queue_size=1)
    rate = rospy.Rate(30)  #hz

    while not rospy.is_shutdown():
        for datafile in sorted(glob.glob(lidar_folder + '/*')):
            pcd = open3d_ply(datafile)
            pointcloud2 = point_cloud(pcd,'camera_init')
            pub_points.publish(pointcloud2)
            rate.sleep()
        break


if __name__ == '__main__':
    lidar_folder = sys.argv[1]
    main(lidar_folder)
