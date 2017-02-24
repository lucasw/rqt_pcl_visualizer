#!/usr/bin/env python
# Sample code to publish a pcl2 with python

import math
import numpy
import rospy
import sensor_msgs.point_cloud2 as pcl2
import sys

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, UInt32


class GenPC2:
    def __init__(self):
        self.pcl_pub = rospy.Publisher("/rqt_pcl_visualizer/point_cloud2", PointCloud2,
                                       queue_size=5)
        self.buffer_size_pub = rospy.Publisher("/rqt_pcl_visualizer/buffer_size", UInt32,
                                               queue_size=2)

        rospy.sleep(1.0)
        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('rgba', 12, PointField.UINT32, 1),
                      ]

        self.header = Header()
        self.header.frame_id = 'map'

        # while not rospy.is_shutdown():
        if True:
            dt = 1.0
            rospy.loginfo("small buffer mode")
            self.buffer_size_pub.publish(UInt32(10))
            rospy.sleep(0.4)
            self.pub_sphere(dt)
            self.pub_sphere(dt, 0.02)
            rospy.sleep(1.3)
            rospy.loginfo("unlimited buffer mode")
            self.buffer_size_pub.publish(UInt32(0))
            rospy.sleep(0.4)
            self.pub_sphere(dt, 0.03)
            rospy.sleep(1.3)
            rospy.loginfo("buffer size 1 mode")
            self.buffer_size_pub.publish(UInt32(1))
            self.pub_sphere(dt, 0.04)
            # break;

    def pub_sphere(self, dt, xo=0):
        for theta in numpy.arange(0, math.pi * 2.0, 0.1):
            if rospy.is_shutdown():
                return
            cloud_points = []
            for phi in numpy.arange(0, math.pi, 0.02):
                if rospy.is_shutdown():
                    return
                r = math.sin(phi)
                z = math.cos(phi)
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                r = int(255 * phi / math.pi)
                g = int(255 * theta / (2.0 * math.pi))
                b = 0xff
                rgba = 0xffffffff & (0xff000000 + (r << 16) + (g << 8) + (b))
                cloud_points.append([x, y, z, rgba])

            self.header.stamp = rospy.Time.now()
            pc2 = pcl2.create_cloud(self.header, self.fields, cloud_points)
            self.pcl_pub.publish(pc2)
            rospy.sleep(dt)

if __name__ == '__main__':
    rospy.init_node('gen_pc2')
    gen_pc2 = GenPC2()
    # rospy.spin()

