#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from scipy.spatial import Voronoi, voronoi_plot_2d
from shapely.geometry import LineString, Point
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math

min_dis=0
sec_x=0
sec_y=0
points_min_x=0.0
points_min_y=0.0
points_max_x=0.0
points_max_y=0.0
points=[]
traffic_cone_pose=[]
intersections=[]
closest_goal_point=[]

def cal_dis(x, y):
	global closest_goal_point, min_dis, sec_x, sec_y

	#print(x, y)
	
	dis=math.sqrt(x**2 + y**2)
	if dis<min_dis:
		closest_goal_point=[]
		closest_goal_point.append([x, y])
		closest_goal_point.append([sec_x, sec_y])
		sec_x=x
		sec_y=y
	min_dis=dis
	
def cal_steering():
	global min_dis, closest_goal_point
	wheel_base = 1.04	
      	
	if len(closest_goal_point)>1:
	        #cal_steering = np.arctan2(2*closest_goal_point[0][1]*wheel_base, min_dis*min_dis)
		#print("---------",cal_steering)
		if min_dis!=0:
			return math.atan((2*closest_goal_point[0][1]*wheel_base) / (min_dis*min_dis))*(180/math.pi)
		else:
			return math.atan((2*closest_goal_point[0][1]*wheel_base) / 1)*(180/math.pi)
def plot_voronoi(points):
	points_array=np.array(points, dtype=np.float64)
	#print(len(points_array))
	vor=Voronoi(points_array)
	lines=[LineString(vor.vertices[line]) for line in vor.ridge_vertices if -1 not in line]
	intersection_points=[lines[i].intersection(lines[j]) for i in range(len(lines)) for j in range(i+1, len(lines))]

	intersection_points=[point for point in intersection_points if isinstance(point, Point)]

	unique_intersection_points=set()
	for point in intersection_points:
		if isinstance(point, Point):
			point_tuple=(point.x, point.y)
			unique_intersection_points.add(point_tuple)

	return unique_intersection_points

def BoundingBox_callback(msg):
	global points_min_x, points_max_x, points_min_y, points_max_y, intersections
	print(len(msg.markers))	
	for traffic_cone in msg.markers:
		traffic_cone_pose.append((traffic_cone.pose.position.x, traffic_cone.pose.position.y))
	
	points_min_x=min(traffic_cone_pose[i][0] for i in range(len(msg.markers)))
	points_min_y=min(traffic_cone_pose[i][1] for i in range(len(msg.markers)))
	points_max_x=max(traffic_cone_pose[i][0] for i in range(len(msg.markers)))
	points_max_y=max(traffic_cone_pose[i][1] for i in range(len(msg.markers)))

	intersections=plot_voronoi(traffic_cone_pose)

	#print(traffic_cone_pose)
	#print(intersections)
	#print("------------------------------------")

rospy.init_node("traffic_cone_drive", anonymous=False)

pub_way_point=rospy.Publisher("/way_point", MarkerArray, queue_size=1000)
rospy.Subscriber("/object_bounding_boxes", MarkerArray, BoundingBox_callback)

rate=rospy.Rate(5)

way_point_markerarray=MarkerArray()

while not rospy.is_shutdown():
	way_point_markerarray.markers = []
	print("-----Intersection-----")
	for i, intersection in enumerate(intersections):
		if((points_min_x<=intersection[0]<=points_max_x) and (points_min_y/2<=intersection[1]<=points_max_y/2)):
			print(intersection[0], intersection[1])
			#print(type(intersection))

			cal_dis(intersection[0], intersection[1])

			way_point_marker=Marker()
			way_point_marker.header.frame_id="velodyne"
			way_point_marker.header.stamp=rospy.Time.now()
			way_point_marker.ns="cylinder"
			way_point_marker.id=i
			way_point_marker.type=Marker.CYLINDER
			way_point_marker.action=Marker.ADD
			way_point_marker.pose.position.x=intersection[0]
			way_point_marker.pose.position.y=intersection[1]
			way_point_marker.pose.position.z=-0.25
			way_point_marker.pose.orientation.x=0.0
			way_point_marker.pose.orientation.y=0.0
			way_point_marker.pose.orientation.z=0.0
			way_point_marker.pose.orientation.w=1.0
			way_point_marker.scale.x=0.2
			way_point_marker.scale.y=0.2
			way_point_marker.scale.z=0.2
			way_point_marker.color.a=1.0
			way_point_marker.color.r=0.0
			way_point_marker.color.g=1.0
			way_point_marker.color.b=0.0
			way_point_marker.lifetime = rospy.Duration(0.5)

			way_point_markerarray.markers.append(way_point_marker)
		else:
			continue

	sec_x=0
	sec_y=0
	
	#print("Closest Goal Point")
	print("-----Closest Goal Point-----")
	print(closest_goal_point)
	
	"""
	if len(closest_goal_point)>0:
		#print(closest_goal_point);
		goal_point_marker=Marker()
		goal_point_marker.header.frame_id="velodyne"
		goal_point_marker.header.stamp=rospy.Time.now()
		goal_point_marker.ns="cylinder"
		goal_point_marker.id=0
		goal_point_marker.type=Marker.CYLINDER
		goal_point_marker.action=Marker.ADD
		goal_point_marker.pose.position.x=closest_goal_point[0][0]
		goal_point_marker.pose.position.y=closest_goal_point[0][1]
		goal_point_marker.pose.position.z=-0.25
		goal_point_marker.pose.orientation.x=0.0
		goal_point_marker.pose.orientation.y=0.0
		goal_point_marker.pose.orientation.z=0.0
		goal_point_marker.pose.orientation.w=1.0
		goal_point_marker.scale.x=0.2
		goal_point_marker.scale.y=0.2
		goal_point_marker.scale.z=0.2
		goal_point_marker.color.a=1.0
		goal_point_marker.color.r=0.0
		goal_point_marker.color.g=0.0
		goal_point_marker.color.b=1.0
		way_point_markerarray.markers.append(goal_point_marker)
	"""
	steering=cal_steering()
	#min_dis=0
	print("-----Steering-----")
	print(steering)

	print("==============================")

	pub_way_point.publish(way_point_markerarray)

	intersections=[]
	traffic_cone_pose=[]
	rate.sleep()
