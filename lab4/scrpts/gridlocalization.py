#!/usr/bin/env python
import roslib
roslib.load_manifest('lab4')
import rospy
import rosbag
import numpy as np
import os
import math
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

landmarks = np.array([[125, 525],[125, 325],[125, 125],[425, 125],[425, 325],[425, 525]])
cell_size = 20
angle_discretize = 40
grid = np.zeros((35, 35, 36))
grid_temp = np.zeros((35, 35, 36))
cell_number = int(200.52 / angle_discretize + 1)
grid[12,28,cell_number] = 1
threshold = 0.1
counter = 0
marker = Marker()

def readbag():
    global landmarks
    global grid
    global grid_temp

    rate=rospy.Rate(10)
    module_path = os.path.dirname(__file__)
    file_path = module_path + '/grid.bag'
    bag = rosbag.Bag(file_path)
    print(file_path)
    for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
        grid_temp = grid
        grid = np.copy(grid_temp)
        if topic == 'Movements':
            rot1 = msg.rotation1
            rot2 = msg.rotation2
            trans = msg.translation
            rot1 = euler_from_quaternion([rot1.x,rot1.y,rot1.z,rot1.w])[2]
            rot2 = euler_from_quaternion([rot2.x,rot2.y,rot2.z,rot2.w])[2]
            update(rot1,trans*100,rot2)
        else:
            landmarks_rviz()
            dist = msg.range * 100
            rot = msg.bearing
            tagnum = msg.tagNum
            rot = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
            perception(dist, rot, tagnum)

def update(rot1,trans,rot2):
    global grid
    global grid_temp
    global threshold
    total_prob = 0

    for a in range(35):
        for b in range(35):
            for c in range(cell_number + 1):
                if grid_temp[a, b, c] < threshold:
                       continue
                trans1x = a * cell_size + cell_size / 2.0
            	trans1y = b * cell_size + cell_size / 2.0
            	rot11 = c * angle_discretize + angle_discretize / 2.0 -180
                for i in range(35):
            		for j in range(35):
            			for k in range(cell_number + 1):
                                    	trans2x = i * cell_size + cell_size / 2.0
                                	trans2y = j * cell_size + cell_size / 2.0
                                	rot21 = k * angle_discretize + angle_discretize / 2.0 -180
                            		trans_tmp = np.sqrt((trans2x - trans1x) ** 2 + (trans2y - trans1y) ** 2)
                            		temp_angle = np.arctan2((trans2y-trans1y), (trans2x - trans1x))
                            		rot2_tmp = temp_angle - rot11
                           		rot1_tmp = rot21 - temp_angle

					prob1 = gaussian_noise(rot1, angle_discretize/2.0, rot1_tmp)
					prob2 = gaussian_noise(trans, cell_size/2.0, trans_tmp)
					prob3 = gaussian_noise(rot2, angle_discretize/2.0, rot2_tmp)
					prob = prob2 * prob1 * prob3
                    			prob = prob * grid_temp[a, b, c]
					grid[i, j, k] += prob
					total_prob += prob
    grid = grid / total_prob
    localization_rviz(1)

def perception(trans, rot, tagnum):
    global grid
    global grid_temp
    global landmarks
    total_prob = 0

    for i in range(35):
        for j in range(35):
            for k in range(cell_number + 1):
                x = i * cell_size + cell_size / 2.0
        	y = j * cell_size + cell_size / 2.0
        	rot = k * angle_discretize + angle_discretize / 2.0 -180
        	trans_tmp = np.sqrt((x - landmarks[tagnum,0]) ** 2 + (y - landmarks[tagnum,1]) ** 2)
            	tag_angle = np.arctan2(landmarks[tagnum,1]-y, landmarks[tagnum,0] - x)
        	rot_tmp = tag_angle - rot

                prob1 = gaussian_noise(rot, angle_discretize/2.0, rot_tmp)
                prob2 = gaussian_noise(trans, cell_size/2.0, trans_tmp)
                prob = prob2 * prob1
                prob = prob * grid_temp[i, j, k]
                grid[i, j, k] = prob
                total_prob += prob
    grid = grid / total_prob
    localization_rviz(2)

def gaussian_noise( average, variance, temp):
    noise = 1.0 / (np.sqrt(2 * np.pi) * variance)
    noise = noise * np.power(np.e, -1.0 * (((temp - average)**2)/(2.0 * (variance ** 2))))
    return noise

def get_index():
    global grid
    index = np.argmax(grid)
    ang_ind = index % grid.shape[2]
    index = index / grid.shape[2]
    y_ind = index % grid.shape[1]
    index = index / grid.shape[1]
    x_ind = index % grid.shape[0]
    return x_ind,y_ind,ang_ind

def landmarks_rviz():
    pub = rospy.Publisher('visualization_marker1', Marker, queue_size=10)
    landmarker = Marker()
    landmarker.header.frame_id = "/grid_localization"
    landmarker.header.stamp = rospy.Time.now()
    landmarker.lifetime = rospy.Time(0)
    landmarker.ns = "landmarks"
    landmarker.id = 1
    landmarker.type = Marker.SPHERE_LIST
    landmarker.action = Marker.ADD

    landmarker.scale.x = 0.2;
    landmarker.scale.y = 0.2;
    landmarker.scale.z = 0.2;
    landmarker.color.a = 1.0;
    landmarker.color.r = 0.0;
    landmarker.color.g = 1.0;
    landmarker.color.b = 1.0;

    for i in range(6):
        p = Point()
        x = landmarks[i, 0]
        y = landmarks[i, 1]
        p.x = x / 100
        p.y = y / 100
        p.z = 0
        landmarker.points.append(p)
    pub.publish(landmarker)

def localization_rviz(type):
    	global marker
        global counter
    	i, j, k = get_index()
    	x_ind = i * cell_size + cell_size / 2.0
	y_ind = j * cell_size + cell_size / 2.0
	ang_ind = k * angle_discretize + angle_discretize / 2.0 - 180
    	if type == 2:
        	counter += 1
        	print(counter)
            	if counter == 1:
                	with open('trajectory.txt', 'w') as f:
                        	f.write(str(counter) + ': ')
                		f.write('(' + str(x_ind/100.0) +', ' + str(y_ind/100.0) + ')\n')
        	else:
                	with open('trajectory.txt', 'a') as f:
                    		f.write(str(counter) + ': ')
            			f.write('(' + str(x_ind/100.0) +', ' + str(y_ind/100.0) + ')\n')
        	f.close()
	pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	marker.header.frame_id = "/grid_localization"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "trajectory"
	marker.id = 0
	marker.type = Marker.LINE_STRIP
    	marker.action = Marker.ADD

    	marker.scale.x = 0.05
    	marker.scale.y = 0.0
	marker.scale.z = 0.0
    	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 1.0
	marker.color.a = 1.0

	p = Point()
	p.x = x_ind/100.0
	p.y = y_ind/100.0
	p.z = 0
	marker.points.append(p)
	pub.publish(marker)

if __name__ == '__main__':
	try:
		rospy.init_node('grid_localization_bayes_filter')
		readbag()
	except rospy.ROSInterruptException:
		pass
