#!/usr/bin/env python
# Sample code to publish a pcl2 with python

import math
import numpy
import rospy
import sensor_msgs.point_cloud2 as pcl2
import std_msgs.msg
import sys

from sensor_msgs.msg import PointCloud2, PointField


if __name__ == '__main__':
    rospy.init_node('gen_pc2')
    pcl_pub = rospy.Publisher("/rqt_pcl_visualizer/point_cloud2", PointCloud2,
                              queue_size=2)
    rospy.sleep(1.0)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]

    header = std_msgs.msg.Header()

    count = 0
    while not rospy.is_shutdown():
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'

        cloud_points = []
        for theta in numpy.arange(0, math.pi * 2.0, 0.1):
            for phi in numpy.arange(0, math.pi, 0.02):
                r = math.sin(phi)
                z = math.cos(phi)
                x = r * math.cos(theta) + count * 0.1
                y = r * math.sin(theta)
                r = int(255 * phi / math.pi)
                g = int(255 * theta / (2.0 * math.pi))
                b = 0xff & int(count * 5)
                rgba = 0xffffffff & (0xff000000 + (r << 16) + (g << 8) + (b))
                cloud_points.append([x, y, z, rgba])

        pc2 = pcl2.create_cloud(header, fields, cloud_points)
        pcl_pub.publish(pc2)
        rospy.sleep(0.4)
        count += 1
        count = count
