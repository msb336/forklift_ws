import setup_path 
import airsim
import os
import numpy as np
import pdb
import time
import datetime
import pprint
import keyboard  # using module keyboard
import math
import argparse
import copy

parser = argparse.ArgumentParser()
parser.add_argument('--start_x', '-start_x', help='x coordinate of the forklift start point', default=83.770279, type=float)
parser.add_argument('--start_y', '-start_y', help='y coordinate of the forklift start point', default=-1772.335938, type=float)
parser.add_argument('--start_z', '-start_z', help='z coordinate of the forklift start point', default=0.304142, type=float)
parser.add_argument('--map', '-map', help='path to the numpy of the map', default=".\\output_map\\static05\\map.npy", type=str)
args = parser.parse_args()

# actual map boundaries
map_x_range = [-4045.1, 2334.9] # start and end
map_y_range = [-2525.2, 1604.9]

def toEulerianAngle(q):
	z = q.z_val
	y = q.y_val
	x = q.x_val
	w = q.w_val
	ysqr = y * y

	# roll (x-axis rotation)
	t0 = +2.0 * (w*x + y*z)
	t1 = +1.0 - 2.0*(x*x + ysqr)
	roll = math.atan2(t0, t1)

	# pitch (y-axis rotation)
	t2 = +2.0 * (w*y - z*x)
	if (t2 > 1.0):
		t2 = 1
	if (t2 < -1.0):
		t2 = -1.0
	pitch = math.asin(t2)

	# yaw (z-axis rotation)
	t3 = +2.0 * (w*z + x*y)
	t4 = +1.0 - 2.0 * (ysqr + z*z)
	yaw = math.atan2(t3, t4)

	return (roll, pitch, yaw)
	
def toQuaternion(roll, pitch, yaw):
	t0 = math.cos(yaw * 0.5)
	t1 = math.sin(yaw * 0.5)
	t2 = math.cos(roll * 0.5)
	t3 = math.sin(roll * 0.5)
	t4 = math.cos(pitch * 0.5)
	t5 = math.sin(pitch * 0.5)

	q = airsim.Quaternionr()
	q.w_val = t0 * t2 * t4 + t1 * t3 * t5 #w
	q.x_val = t0 * t3 * t4 - t1 * t2 * t5 #x
	q.y_val = t0 * t2 * t5 + t1 * t3 * t4 #y
	q.z_val = t1 * t2 * t4 - t0 * t3 * t5 #z
	return q
	
# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()

# upload map numpy
map = np.load(args.map)

# create experiments directories
experiment_dir = os.path.join(os.path.expanduser('~'), 'Documents\AirSim', datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
images_dir = os.path.join(experiment_dir, 'images')
os.makedirs(images_dir)

# create txt file
airsim_rec = open(os.path.join(experiment_dir,"airsim_rec.txt"),"w") 
airsim_rec.write("TimeStamp\tPOS_X\tPOS_Y\tPOS_Z\tRoll\tPitch\tYaw\tRPM\tSpeed\tImageFile\n") 

initial_pose = client.simGetVehiclePose()
idx = 0
while True:

	# create position and orientation vectors
	pose = copy.deepcopy(initial_pose)
	pose.position.x_val = pose.position.x_val + np.random.uniform(low=(map_x_range[0]-args.start_x)/100.0, high=(map_x_range[1]-args.start_x)/100.0)
	pose.position.y_val = pose.position.y_val + np.random.uniform(low=(map_y_range[0]-args.start_y)/100.0, high=(map_y_range[1]-args.start_y)/100.0)
		
	alpha = np.random.uniform(low=-math.pi, high=math.pi)
	
	# computation of related constants
	short_diagonal = 47 # 37 # distance from the pallet origin to the closer edge
	long_diagonal = 145 # 135.5 # distance from the pallet origin to the farther
	short_angle = 67.4 # angle between the straight axis to the short diagonal
	long_angle = 17.6 # angle between the opposite straight axis to the long diagonal
	
	# computation of polygon coordinates as (x,y) for each point
	p0 = (short_diagonal*math.sin(math.radians(alpha-short_angle)), short_diagonal*math.cos(math.radians(alpha-short_angle)))
	p1 = (short_diagonal*math.sin(math.radians(alpha+short_angle)), short_diagonal*math.cos(math.radians(alpha+short_angle)))
	p2 = (long_diagonal*math.sin(math.radians(alpha+180.0-long_angle)), long_diagonal*math.cos(math.radians(alpha+180.0-long_angle)))
	p3 = (long_diagonal*math.sin(math.radians(alpha+180.0+long_angle)), long_diagonal*math.cos(math.radians(alpha+180.0+long_angle)))
	
	# computation of bounding box surrounding the polygon, in relation to the numpy map
	pb0 = (int(min(p0[0], p1[0], p2[0], p3[0]) + (pose.position.x_val * 100) + args.start_x - map_x_range[0]), int(min(p0[1], p1[1], p2[1], p3[1]) + (pose.position.y_val * 100) + args.start_y - map_y_range[0]))
	pb1 = (int(min(p0[0], p1[0], p2[0], p3[0]) + (pose.position.x_val * 100) + args.start_x - map_x_range[0]), int(max(p0[1], p1[1], p2[1], p3[1]) + (pose.position.y_val * 100) + args.start_y - map_y_range[0]))
	pb2 = (int(max(p0[0], p1[0], p2[0], p3[0]) + (pose.position.x_val * 100) + args.start_x - map_x_range[0]), int(max(p0[1], p1[1], p2[1], p3[1]) + (pose.position.y_val * 100) + args.start_y - map_y_range[0]))
	pb3 = (int(max(p0[0], p1[0], p2[0], p3[0]) + (pose.position.x_val * 100) + args.start_x - map_x_range[0]), int(min(p0[1], p1[1], p2[1], p3[1]) + (pose.position.y_val * 100) + args.start_y - map_y_range[0]))
	
	# check if the drawed pose is available
	if 255 in map[pb0[0]:pb2[0], pb0[1]:pb2[1]]:
		continue
	# check if it's not too close to the walls (1 meter at least)
	if min(pb0[1], pb1[1], pb2[1], pb3[1]) < 100 or \
	   max(pb0[1], pb1[1], pb2[1], pb3[1]) > 4030 or \
	   min(pb0[0], pb1[0], pb2[0], pb3[0]) < 100 or \
	   max(pb0[0], pb1[0], pb2[0], pb3[0]) > 6280:
		continue
	
	orientation = toEulerianAngle(pose.orientation)  # roll, pitch, yaw
	pose.orientation = toQuaternion(orientation[0], orientation[1], alpha)
	
	client.simSetVehiclePose(pose, True)
	car_controls.brake = 1.0
	client.setCarControls(car_controls)
	
	# wait for the forklift to stablize
	time.sleep(0.5)
	
	
	# get point cloud from lidar
	lidarData = client.getLidarData(lidar_name='LidarSensor1', vehicle_name='ML_Manual')
	time_stamp = lidarData.time_stamp
	pos = (lidarData.pose.position).to_numpy_array()  # position is relative to "player start" point
	state = client.getCarState(vehicle_name='ML_Manual')

	if (len(lidarData.point_cloud) < 3):
		print("\tNo points received from Lidar data")
	else:
		
		# reshape array of floats to array of [X,Y,Z]
		points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
		points = np.reshape(points, (int(points.shape[0]/3), 3))
		#pdb.set_trace()
		print("\tReading %d: time_stamp: %d number_of_points: %d" % (idx, time_stamp, len(points)))
		#print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
		#print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))

		np.save(os.path.join(images_dir, "ptc_{}".format(time_stamp)), points)
		
		airsim_rec.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(time_stamp,pos[0],pos[1],pos[2],orientation[0],orientation[1],alpha,state.rpm,state.speed,"ptc_{}.npy".format(time_stamp))) 
		
	if keyboard.is_pressed('q'):  # if key 'q' is pressed
		airsim_rec.close()
		quit()
	
	idx += 1

airsim_rec.close() 
