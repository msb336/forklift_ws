import tensorflow as tf
import numpy as np
import argparse
import socket
import importlib
import time
import os
import scipy.misc
import sys
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)
sys.path.append(os.path.join(BASE_DIR, 'model.models'))
sys.path.append(os.path.join(BASE_DIR, 'model.utils'))
import model.provider
import model.models.pointnet_cls as model
import setup_path 
import airsim
import keyboard  # using module keyboard
import math
import pdb

parser = argparse.ArgumentParser()
parser.add_argument('--gpu', type=int, default=0, help='GPU to use [default: GPU 0]')
#parser.add_argument('--model', default='pointnet_cls', help='Model name: pointnet_cls or pointnet_cls_basic [default: pointnet_cls]')
parser.add_argument('--num_point', type=int, default=2048, help='Point Number [256/512/1024/2048] [default: 1024]')
parser.add_argument('--model_path', default='log/model.ckpt', help='model checkpoint file path [default: log/model.ckpt]')
parser.add_argument('--dump_dir', default='dump', help='dump folder path [dump]')
parser.add_argument('--start_x', '-start_x', help='x coordinate of the forklift start point in the simulation', default=1340.00061, type=float)
parser.add_argument('--start_y', '-start_y', help='y coordinate of the forklift start point in the simulation', default=-49.999756, type=float)
parser.add_argument('--start_z', '-start_z', help='z coordinate of the forklift start point in the simulation', default=0.0, type=float)
parser.add_argument('--pallet_x', '-pallet_x', help='x coordinate of the destination pallet in the simulation', default=1605.164917, type=float)
parser.add_argument('--pallet_y', '-pallet_y', help='y coordinate of the destination pallet in the simulation', default=0.0, type=float)
parser.add_argument('--pallet_z', '-pallet_z', help='z coordinate of the destination pallet in the simulation', default=0.004776, type=float)
parser.add_argument('--pallet_roll', '-pallet_roll', help='roll coordinate of the destination pallet in the simulation', default=0.0, type=float)
parser.add_argument('--pallet_pitch', '-pallet_pitch', help='pitch coordinate of the destination pallet in the simulation', default=0.0, type=float)
parser.add_argument('--pallet_yaw', '-pallet_yaw', help='yaw coordinate of the destination pallet in the simulation', default=0.0, type=float)
parser.add_argument('--pdt_max', '-pdt_max', help='max distance to the destination pallet', default=750.0, type=float)
parser.add_argument('--edt', '-edt', help='environment distance threshold in meters', default=10, type=int)
parser.add_argument('--z_const', '-z_const', help='constant to add to the z axis of the forklift', default=216.4, type=float)
parser.add_argument('--drive', dest='drive', action='store_true')
FLAGS = parser.parse_args()

NUM_POINT = FLAGS.num_point
MODEL_PATH = FLAGS.model_path
GPU_INDEX = FLAGS.gpu
#MODEL = importlib.import_module(FLAGS.model) # import network module
DUMP_DIR = FLAGS.dump_dir
if not os.path.exists(DUMP_DIR): os.mkdir(DUMP_DIR)
LOG_FOUT = open(os.path.join(DUMP_DIR, 'log_evaluate.txt'), 'w')
LOG_FOUT.write(str(FLAGS)+'\n')

HOSTNAME = socket.gethostname()

def log_string(out_str):
    LOG_FOUT.write(out_str+'\n')
    LOG_FOUT.flush()
    print(out_str)
	
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
	
def preprocess_input(pointcloud, pos):

	# draw relative position vector, convert them to centimeters and change them to be non-relative
	pos[0] = float(pos[0])*100+FLAGS.start_x
	pos[1] = float(pos[1])*100+FLAGS.start_y
	pos[2] = float(pos[2])*100+FLAGS.start_z + FLAGS.z_const
	
	pallets_relative = np.subtract(np.array([FLAGS.pallet_x, FLAGS.pallet_y, FLAGS.pallet_z]), pos) / FLAGS.pdt_max

	# delete irrelevant far data points
	idxs = np.where(abs(pointcloud) > FLAGS.edt)[0]
	lidar_data_trimmed = np.delete(pointcloud, idxs, axis=0)
	
	# randomly remove points to keep constant number of data points
	if lidar_data_trimmed.shape[0] > FLAGS.num_point:
		max_points = lidar_data_trimmed.shape[0]-FLAGS.num_point
		
		rejected_idxs = np.random.permutation(lidar_data_trimmed.shape[0])[:max_points]
		lidar_data_trimmed = np.delete(lidar_data_trimmed, rejected_idxs, axis=0)
	# randomly add points if there is not enough data points
	elif lidar_data_trimmed.shape[0] < FLAGS.num_point:
		added_data_points = np.random.normal(loc=0.0, scale=FLAGS.edt, size=(FLAGS.num_point-lidar_data_trimmed.shape[0],3))
		lidar_data_trimmed = np.concatenate((lidar_data_trimmed, added_data_points), axis=0)

	lidar_data_trimmed = (np.expand_dims(lidar_data_trimmed, axis=0)) / float(FLAGS.edt)
	pallets_relative = np.expand_dims(pallets_relative, axis=0)

	return lidar_data_trimmed, pallets_relative
 
def rotate_pointcloud(vector):
 
	cosval = np.cos(np.pi)
	sinval = np.sin(np.pi)
	rotation_matrix = np.array([[cosval, sinval, 0], [-sinval, cosval, 0], [0, 0, 1]])
	
	vector = np.dot(vector.reshape((-1, 3)),rotation_matrix)
	
	return vector
 
with tf.device('/gpu:'+str(GPU_INDEX)):
	pointclouds_pl, distance_pl, labels_pl = model.placeholder_inputs(1, NUM_POINT)
	is_training_pl = tf.placeholder(tf.bool, shape=())

	# simple model
	pred, end_points = model.get_model(pointclouds_pl, distance_pl, is_training_pl)
	#loss = model.get_loss(pred, labels_pl, end_points)
	
	# Add ops to save and restore all the variables.
	saver = tf.train.Saver()

# connect to the AirSim simulator 
client = airsim.CarClient()
client.confirmConnection()

# take control over the forklift if requested
if FLAGS.drive:
	client.enableApiControl(True)
	car_controls = airsim.CarControls()
	
	car_controls.is_manual_gear = True;
	car_controls.manual_gear = -1
	
	# initiate control sequence
	car_controls.throttle = 0
	car_controls.steering = 0


# Create a session
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
config.allow_soft_placement = True
config.log_device_placement = True
sess = tf.Session(config=config)

# Restore variables from disk.
saver.restore(sess, MODEL_PATH)
log_string("Model restored.")

ops = {'pointclouds_pl': pointclouds_pl,
	   'distance_pl': distance_pl,
	   'is_training_pl': is_training_pl,
	   'pred': pred}
	   #'labels_pl': labels_pl,
	   #'loss': loss}
is_training = False

list_predictions = []
while True:
	
	# get point cloud from lidar
	lidarData = client.getLidarData(lidar_name='LidarSensor1', vehicle_name='ML_Manual')
	pos = (lidarData.pose.position).to_numpy_array()
	gt_orientation_z = math.degrees((toEulerianAngle(lidarData.pose.orientation))[2]) - FLAGS.pallet_yaw  # pitch, roll, yaw
	gt_orientation_z = -1*(gt_orientation_z - 180.0) if  gt_orientation_z > 0 else -1*(gt_orientation_z + 180.0)
	#print("gt_orientation_z: {}".format(gt_orientation_z))
	
	if (len(lidarData.point_cloud) < 3):
		print("\tNo points received from Lidar data")
		continue
	else:
		# reshape array of floats to array of [X,Y,Z]
		points = np.array(lidarData.point_cloud, dtype=np.dtype('f4'))
		points = np.reshape(points, (int(points.shape[0]/3), 3))
		
		points, distance_vector = preprocess_input(points, pos)
		
		points = np.expand_dims(rotate_pointcloud(points), axis=0)
		distance_vector = rotate_pointcloud(distance_vector)
		
		feed_dict = {ops['pointclouds_pl']: points,
                             ops['distance_pl']: distance_vector,
                             ops['is_training_pl']: is_training}
		pred_val = sess.run([ops['pred']], feed_dict=feed_dict)
		predicted_value = pred_val[0][0][0]*180

		list_predictions.append(predicted_value)
		if len(list_predictions) > 20:
			list_predictions = list_predictions[1:]
		orientation_avg = sum(list_predictions) / float(len(list_predictions))
		print("predicted orientation: {}, error: {}".format(round(orientation_avg,2), round(abs(orientation_avg - gt_orientation_z),2)))
		
		#swapped_prediction = predicted_value - 180 if predicted_value > 0 else predicted_value + 180
		#swapped_mean = orientation_avg - 180 if orientation_avg > 0 else orientation_avg + 180

		#print("swapped predicted orientation: {}".format(swapped_prediction))
		#print("swapped prediction mean: {}".format(swapped_mean))
		
		#print("norm: {}:".format(np.linalg.norm(distance_vector[0][:2] * FLAGS.pdt_max)))
		# navigate to destination pallet
		if FLAGS.drive:
			
			# compute distance vector to palletc
			distance_to_pallet = distance_vector * FLAGS.pdt_max
			#distance_to_pallet[0][0] -= 210
			#print("distance_to_pallet: {}".format(distance_to_pallet))
			# set controls and update trajectory
			car_controls.throttle = -0.45
			car_controls.steering = 0.0
			
			# forklift already aligned, compute steering to drive towards the pallet
			if abs(predicted_value) < 3:
			
				# compute steering depend on the distance vector
				car_controls.steering = math.degrees(math.atan2(distance_to_pallet[0][0], distance_to_pallet[0][1])) / 45.0
			
			# forklift is not aligned, use the orientation to compute steering
			else:
				# compute relative angle in which we'll use to compute desired steering angle
				relative_angle = (predicted_value) + math.degrees(math.atan2(distance_to_pallet[0][0], distance_to_pallet[0][1])) * 0.8
				
				#print("distance_to_pallet: {}".format(distance_to_pallet))
				#print("pallet angle: {}, forklift angle: {}".format(pred_val[0][0][0]*180, math.degrees(math.atan2(distance_to_pallet[0][1], distance_to_pallet[0][0]))))
				car_controls.steering = -relative_angle / 180.0
			
			# if reached pallet surrounding, stop
			if np.linalg.norm(distance_to_pallet[0][:2]) < 230:
				car_controls.steering = 0.0
				car_controls.throttle = 0.0
				car_controls.brake = 1.0
			
			# update controls
			client.setCarControls(car_controls)

		time.sleep(0.1)
		
	if keyboard.is_pressed('q'):  # if key 'q' is pressed
		client.enableApiControl(False)
		quit()