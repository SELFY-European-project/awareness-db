#!/usr/bin/env python
# Software License Agreement (BSD License)
#

import rospy
import numpy as np
from selfy_msgs.msg import ObjectsListing
from selfy_msgs.msg import Classification
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def main():
    global _markers_pub
    global _marker_color
    global _label_scale
    global _is_raw_source
    global _only_contour
    global _edge_width

    rospy.init_node('objects_drawer', anonymous=False)
    node_name = rospy.get_name()[1:]

    # ----- Get parameters

    _is_raw_source = rospy.get_param('/'+node_name+ '/IsRawSource', True)

    _only_contour = rospy.get_param('/'+node_name+ '/Contour', False)

    # Carla Ground Truth data on vehicles and walkers
    ground_truth_topic = rospy.get_param('/'+node_name+ '/ObjectsTopic', '')
    if ground_truth_topic == '':
        rospy.logerr("Params %s ObjectsTopic not defined. Aborting drawing node.", node_name)
        return

    ground_truth_sub = rospy.Subscriber(ground_truth_topic, ObjectsListing, callback)

    _marker_color = rospy.get_param('/'+node_name+ '/Color', [0.9, 0.9, 0.9, 0.4])

    _label_scale = rospy.get_param('/'+node_name+ '/LabelScale', 0.8)

    _edge_width = rospy.get_param('/'+node_name+ '/EdgeWidth', 0.2)

    # Rviz marker publisher
    _markers_pub = rospy.Publisher('/'+node_name, MarkerArray, queue_size=10)

    rospy.spin()



# def fill_oriented_bb(obj):
#     obj.oriented_bb.clear()

#     tf::Quaternion q(
#         obj.pose_with_covariance.pose.orientation.x,
#         obj.pose_with_covariance.pose.orientation.y,
#         obj.pose_with_covariance.pose.orientation.z,
#         obj.pose_with_covariance.pose.orientation.w)
#     tf::Matrix3x3 m(q)
#     double roll, pitch, yaw
#     m.getRPY(roll, pitch, yaw)

    
#     c = cos(yaw)
#     s = sin(yaw)
#     r1x = -obj.extent.y * c - obj.extent.x * s
#     r1y = -obj.extent.y * s + obj.extent.x * c
#     r2x =  obj.extent.y * c - obj.extent.x * s
#     r2y =  obj.extent.y * s + obj.extent.x * c

#     center = obj.pose_with_covariance.pose.position
#     p.x = center.x + r1x; p.y = center.y + r1y
#     obj.oriented_bb.push_back(p)
#     p.x = center.x + r2x; p.y = center.y + r2y
#     obj.oriented_bb.push_back(p)
#     p.x = center.x - r1x; p.y = center.y - r1y
#     obj.oriented_bb.push_back(p)
#     p.x = center.x - r2x; p.y = center.y - r2y
#     obj.oriented_bb.push_back(p)


def callback(objects):
    global _markers_pub
    global _markers_pub
    global _marker_color
    global _label_scale
    global _only_contour
    global _edge_width

    marker_array = MarkerArray()

    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    delete_marker.header.frame_id = objects.header.frame_id
    marker_array.markers.append(delete_marker)

    if _is_raw_source:
        obj_list = objects.raw_objects
    else:
        obj_list = objects.fused_objects

    id = 0
    for obj in obj_list:
        # Trick to handle post fusion object
        if not _is_raw_source:
            obj = obj.object

        # ----- Draw velocity vector

        arrow = Marker()
        arrow.ns = "Velocity"
        arrow.id = id
        arrow.header.frame_id = objects.header.frame_id
        arrow.header.stamp = rospy.Time.now()

        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD

        arrow.scale.x = 0.2
        arrow.scale.y = 0.5
        arrow.scale.z = 0.3
        arrow.color.a = 1.0
        arrow.color.r = _marker_color[0]
        arrow.color.g = _marker_color[1]
        arrow.color.b = _marker_color[2]

        p = obj.pose_with_covariance.pose.position
        v = obj.twist_with_covariance.twist.linear
        arrow.points.append(p)
        arrow.points.append(Point(p.x+v.x, p.y+v.y, p.z+v.z))
        
        marker_array.markers.append(arrow)

        # ----- Draw Box vector

        box_marker = Marker()
        box_marker.ns = "Box"
        box_marker.id = id+1
        box_marker.header.frame_id = objects.header.frame_id
        box_marker.header.stamp = rospy.Time.now()
        box_marker.action = Marker.ADD

        if _only_contour:
            box_marker.type = Marker.LINE_LIST
            box_marker.pose.orientation.w = 1
            box_marker.scale.x = _edge_width

            if len(obj.oriented_bb) == 8:
                # 3D bounding box
                edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]
            elif len(obj.oriented_bb) == 4:
                # 2D bounding box
                edges = [[0,1], [1,2], [2,3], [3,0]]
            else:
                edges = []

            for edge in edges:
                box_marker.points.append(obj.oriented_bb[edge[0]])
                box_marker.points.append(obj.oriented_bb[edge[1]])

        else:
            box_marker.type = Marker.CUBE
            box_marker.scale = obj.extent
            if box_marker.scale.z == 0.0:
                # 2D bounding box
                box_marker.scale.z = 0.01
            box_marker.pose = obj.pose_with_covariance.pose

        box_marker.color.a = _marker_color[3]
        box_marker.color.r = _marker_color[0]
        box_marker.color.g = _marker_color[1]
        box_marker.color.b = _marker_color[2]

        marker_array.markers.append(box_marker)

        # ----- Draw Label

        if obj.classification.type != Classification.UNDEFINED:

            label_marker = Marker()
            label_marker.ns = "Label"
            label_marker.header.frame_id = objects.header.frame_id
            label_marker.header.stamp = rospy.Time.now()
            label_marker.id = id+2

            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD

            label_marker.pose.position.x = obj.pose_with_covariance.pose.position.x
            label_marker.pose.position.y = obj.pose_with_covariance.pose.position.y
            label_marker.pose.position.z = obj.pose_with_covariance.pose.position.z + obj.extent.z
            label_marker.pose.orientation.w = 1.0

            label_marker.scale.x = _label_scale
            label_marker.scale.y = _label_scale
            label_marker.scale.z = _label_scale

            label_marker.color.r = _marker_color[0]
            label_marker.color.g = _marker_color[1]
            label_marker.color.b = _marker_color[2]
            label_marker.color.a = 1.0
        
            if obj.classification.type == Classification.PEDESTRIAN:
                label_marker.text = "Walker"
            elif obj.classification.type == Classification.VEHICLE:
                label_marker.text = "Vehicle"
            elif obj.classification.type == Classification.BIKE:
                label_marker.text = "Bike"
            else:
                label_marker.text = "?"

            marker_array.markers.append(label_marker)

        id += 3

    _markers_pub.publish(marker_array)

if __name__ == '__main__':
    main()

