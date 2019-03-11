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
	
# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()

# create experiments directories
experiment_dir = os.path.join(os.path.expanduser('~'), 'Documents\AirSim', datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S"))
images_dir = os.path.join(experiment_dir, 'images')
os.makedirs(images_dir)

# create txt file
airsim_rec = open(os.path.join(experiment_dir,"airsim_rec.txt"),"w") 
airsim_rec.write("TimeStamp\tPOS_X\tPOS_Y\tPOS_Z\tRoll\tPitch\tYaw\tRPM\tSpeed\tImageFile\n") 

idx = 0
while True:

	# get point cloud from lidar
	lidarData = client.getLidarData(lidar_name='LidarSensor1', vehicle_name='ML_Manual')
	time_stamp = lidarData.time_stamp
	pos = (lidarData.pose.position).to_numpy_array()  # position is relative to "player start" point
	orientation = toEulerianAngle(lidarData.pose.orientation)  # pitch, roll, yaw
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
		
		airsim_rec.write("{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(time_stamp,pos[0],pos[1],pos[2],orientation[0],orientation[1],orientation[2],state.rpm,state.speed,"ptc_{}.npy".format(time_stamp))) 
		
	if keyboard.is_pressed('q'):  # if key 'q' is pressed
		airsim_rec.close()
		quit()
	
	idx += 1
	time.sleep(0.2)

airsim_rec.close() 
