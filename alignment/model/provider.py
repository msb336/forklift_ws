import os
import sys
import numpy as np
import h5py
import pdb
import math
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(BASE_DIR)

# Download dataset for point cloud classification
DATA_DIR = os.path.join(BASE_DIR, 'data')

def shuffle_data(data, distance, labels):
    """ Shuffle data and labels.
        Input:
          data: B,N,... numpy array
          label: B,... numpy array
        Return:
          shuffled data, label and shuffle indices
    """
    idx = np.arange(len(labels))
    np.random.shuffle(idx)
    return data[idx, ...], distance[idx, ...], labels[idx, ...], idx

def rotate_point_cloud_aroud_z_axis(batch_data, batch_distance, batch_label):
    """ Randomly rotate the point clouds to augument the dataset, around the z axis
        rotation is per shape based along up direction
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, rotated batch of point clouds
    """
    rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
    rotated_distance = np.zeros(batch_distance.shape, dtype=np.float32)
    rotated_label = np.zeros(batch_label.shape, dtype=np.float32)
	
    for k in range(batch_data.shape[0]):
        rotation_angle = np.random.uniform() * 2 * np.pi
        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        rotation_matrix = np.array([[cosval, sinval, 0],
                                    [-sinval, cosval, 0],
                                    [0, 0, 1]])
        shape_pc = batch_data[k, ...]
        shape_distance = np.expand_dims(batch_distance[k,...], axis=0)
        rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
        rotated_distance[k, ...] = np.dot(shape_distance.reshape((-1, 3)), rotation_matrix)
        rotated_label[k] = batch_label[k] + math.degrees(rotation_angle) / 180.0
        if rotated_label[k] > 1:
            rotated_label[k] = batch_label[k] - 2
    return rotated_data, rotated_distance, rotated_label

def rotate_point_cloud(batch_data):
    """ Randomly rotate the point clouds to augument the dataset
        rotation is per shape based along up direction
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, rotated batch of point clouds
    """
    rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
    for k in range(batch_data.shape[0]):
        rotation_angle = np.random.uniform() * 2 * np.pi
        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        rotation_matrix = np.array([[cosval, 0, sinval],
                                    [0, 1, 0],
                                    [-sinval, 0, cosval]])
        shape_pc = batch_data[k, ...]
        rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
    return rotated_data


def rotate_point_cloud_by_angle(batch_data, rotation_angle):
    """ Rotate the point cloud along up direction with certain angle.
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, rotated batch of point clouds
    """
    rotated_data = np.zeros(batch_data.shape, dtype=np.float32)
    for k in range(batch_data.shape[0]):
        #rotation_angle = np.random.uniform() * 2 * np.pi
        cosval = np.cos(rotation_angle)
        sinval = np.sin(rotation_angle)
        rotation_matrix = np.array([[cosval, 0, sinval],
                                    [0, 1, 0],
                                    [-sinval, 0, cosval]])
        shape_pc = batch_data[k, ...]
        rotated_data[k, ...] = np.dot(shape_pc.reshape((-1, 3)), rotation_matrix)
    return rotated_data

def jitter_point_cloud_and_distance(batch_data, batch_distance, sigma=0.01, clip=0.05):
    """ Randomly jitter points. jittering is per point.
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, jittered batch of point clouds 
    """
    B, N, C = batch_data.shape
    assert(clip > 0)
    jittered_data = np.clip(sigma * np.random.randn(B, N, C), -1*clip, clip)
    jittered_data += batch_data
    jittered_distance = np.clip(sigma * np.random.randn(B, C), -1*clip, clip)
    jittered_distance += batch_distance
    return jittered_data, jittered_distance

def jitter_point_cloud(batch_data, sigma=0.01, clip=0.05):
    """ Randomly jitter points. jittering is per point.
        Input:
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, jittered batch of point clouds 
    """
    B, N, C = batch_data.shape
    assert(clip > 0)
    jittered_data = np.clip(sigma * np.random.randn(B, N, C), -1*clip, clip)
    jittered_data += batch_data
    return jittered_data

def getDataFiles(list_filename):
    return [line.rstrip() for line in open(list_filename)]

def load_h5_data_label_seg(h5_filename):
    f = h5py.File(h5_filename)
    data = f['data'][:]
    label = f['label'][:]
    seg = f['pid'][:]
    return (data, label, seg)

def loadDataFile_with_seg(filename):
    return load_h5_data_label_seg(filename)
	
def load_lidar_data(data_path, is_train=True):
    if is_train:
        dataset = h5py.File(os.path.join(data_path, 'train.h5'), 'r')
    else:  
        dataset = h5py.File(os.path.join(data_path, 'eval.h5'), 'r')
    
    # load dataset
    lidar_data = np.asarray(dataset['lidar_data'])
    #distance = np.asarray(dataset['distance'])
    #distance = np.expand_dims(distance, axis=1)
    #data = np.concatenate((lidar_data,distance), axis=1)
    
    # load distance and labels
    distance = np.asarray(dataset['distance']) / 350.0
    labels = np.asarray(dataset['label']) / 180.0
    labels = np.delete(labels, [0,1], 1) # predict only orientation on z-axis

    return lidar_data, distance, labels
