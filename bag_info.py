#! /usr/bin/python3

import rosbag
from  anytree import Node, RenderTree
import texttable
import re

def extract_topics(bag_file):
	return bag_file.get_type_and_topic_info().topics

def extract_frames(bag_file):
	first_message = True
	begin_time = 0
	frames_list = {}
	frames_roots = []
	for topic, msg, t in bag_file.read_messages(topics=["/tf"]):
		if first_message == True:
			begin_time = t.secs
			first_message = False
		if abs(t.secs - begin_time) > 2:
			break
		for transform in msg.transforms:
			if transform.header.frame_id not in frames_list:
				frames_list[transform.header.frame_id] = Node(transform.header.frame_id)
			if transform.child_frame_id not in frames_list:
				frames_list[transform.child_frame_id] = Node(transform.child_frame_id,
				                                             parent=frames_list[transform.header.frame_id])
			
			else:
				frames_list[transform.child_frame_id].parent=frames_list[transform.header.frame_id]
	for node_name, node in frames_list.items():
		if node.root not in frames_roots:
			frames_roots.append(node.root)
	return frames_roots

def find_frame(node,frame,node_list):
	if frame in node.name:
		node_list.append(node)
	for child in node.children:
		find_frame(child,frame,node_list)
			

def extract_parrent_and_child_frames(frame_roots,parent_frames,child_frames):
	d = {}
	for root in frame_roots:		
		for possible_world_frame in parent_frames:
			world_frames = []
			find_frame(root,possible_world_frame,world_frames)
			for world_frame in world_frames:
				if world_frame not in d:
					d[world_frame] = []
				for possible_child_frame in child_frames:
					current_frames = []
					find_frame(world_frame,possible_child_frame,current_frames)
					for current_frame in current_frames:
						if current_frame not in d[world_frame]:
							d[world_frame].append(current_frame)
	return d

def match_topic_types(items):
	topicmatches = {'camera':[], 'camera_left':[], 'camera_right':[], 'camera_depth':[], 'laser_scan':[], 'imu':[], 'odometry':[]} # to hold topics with matching message types

	# dictionary of score weights
	# structure {sensor:(message_type, [(keyword, weight), (keyword, weight)])}
	topictypes = {'camera':('sensor_msgs/Image', [('0',1), ('stereo',1),('cam',2)]), 
	'camera_left':('sensor_msgs/Image', [('left',5), ('_l_',1),('cam',2)]), 
	'camera_right':('sensor_msgs/Image',[('right',5), ('_r_',1),('cam',2)]), 
	'camera_depth':('sensor_msgs/Image',[('depth',5), ('_d_',1),('cam',2)]), 
	'laser_scan':('sensor_msgs/LaserScan', [('base',1), ('scan',2), ('laser', 1), ('robot', 1)]), 
	'imu':('sensor_msgs/Imu',[('0',1)]), 
	'odometry':('nav_msgs/Odometry',[('',1)])} 

	for topic, info in items: # loop over topics
		for match in topicmatches:
			if info.msg_type in topictypes[match][0]: # if topic has correct message type
				topicmatches[match].append(topic) # append it to matches

	topicscores = {'camera':[], 'camera_left':[], 'camera_right':[], 'camera_depth':[], 'laser_scan':[], 'imu':[], 'odometry':[]} # to hold scores
	
	for score in topicscores: # loop over scores
		n = len(topicmatches[score])
		topicscores[score] = [0] * n
		s = sum([j[1] for j in topictypes[score][1]])
		for i in range(len(topicmatches[score])):
			ad = False
			for m in topictypes[score][1]: # add some to score if there's a keyword match
				if m[0] in topicmatches[score][i]:
					if m[0] != '':
						topicscores[score][i] += 1/s * m[1]
						ad = True

	#print(topicscores) # positive scores

	for score in topicscores: # loop over scores
		for i in range(len(topicscores[score])):
			s = topicscores[score][i]
			t = topicmatches[score][i]

			for topic in topicmatches: # check to see if another sensor has higher score for this topic
				if score == topic: continue
				if t in topicmatches[topic]:
					oidx = topicmatches[topic].index(t)
					if topicscores[topic][oidx] > s:
						topicscores[score][i] -= 1 # if so, subtract from score
						break

	#print(topicscores) # includes positivee and negative scores

	assignment = {'camera':[], 'camera_left':[], 'camera_right':[], 'camera_depth':[], 'laser_scan':[], 'imu':[], 'odometry':[]} # final assignment
	for score in topicscores: # assign topics
		ss = [(topicscores[score][i], i) for i in range(len(topicscores[score]))]
		s = reversed(sorted(ss))
		for c in s:
			idx = c[1]
			assignment[score].append(topicmatches[score][idx])

	return assignment

#bag=rosbag.Bag("/home/user/data/2011-01-25-06-29-26.bag")
#bag=rosbag.Bag("/home/user/data/2011-01-27-07-49-54.bag")
#bag=rosbag.Bag("/home/user/data/2011-04-11-07-34-27.bag")
bag=rosbag.Bag("/home/user/2011-01-24-06-18-27.bag")
topics = extract_topics(bag)
#################################### print topics #############################################           
tab = texttable.Texttable()                                                                   #
tab.header(["topic name","msgs type","msgs count","connections","frequency"])                 #
for topic, info in topics.items():                                                            #
	tab.add_row((topic,info.msg_type,info.message_count,info.connections,info.frequency)) #
print(tab.draw())                                                                             #
###############################################################################################

frames_roots = extract_frames(bag)
################### print tf frames ##############
for root in frames_roots:                        #
	for pre, fill, node in RenderTree(root): #
		print("%s%s" % (pre,node.name))  #
##################################################

laser_frames = extract_parrent_and_child_frames(frames_roots,["world","odom"],["laser","robot","base"])
camera_frames = extract_parrent_and_child_frames(frames_roots,["world","odom"],["cam","stereo","robot","base"])

laser_costs = {"laser":1,"robot":0.5,"base":0.5}
camera_costs = {"cam":2,"stereo":1,"robot":0.5,"base":0.5}

print("laser frames:")
for k,vs in laser_frames.items():
	for v in vs:
		total_cost = 0
		for sensor, cost in laser_costs.items():
			if sensor in v.name:
				total_cost += cost
		print(k.name,"->",v.name,total_cost)
print("camera frames:")
for k,vs in camera_frames.items():
	for v in vs:
		total_cost = 0
		for sensor, cost in camera_costs.items():
			if sensor in v.name:
				total_cost += cost
print(k.name,"->",v.name,total_cost)

assignments = match_topic_types(topics.items())
for a in assignments:
	print(a, assignments[a])