import open3d as o3d
import numpy as np
import rospy
import sensor_msgs, std_msgs
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import glob


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


def main(args=None):
    rospy.init_node('maaromujhe')
    pub_points = rospy.Publisher('velodyne_points', PointCloud2, queue_size=1)
    rate = rospy.Rate(30)  #hz

    while not rospy.is_shutdown():
        for datafile in sorted(glob.glob('lidar_output/*')):
            pcd = open3d_ply(datafile)
            pointcloud2 = point_cloud(pcd,'camera_init')
            pub_points.publish(pointcloud2)
            rate.sleep()
        break


if __name__ == '__main__':
    main()