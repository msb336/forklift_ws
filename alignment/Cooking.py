import random
import csv
from PIL import Image
import numpy as np
import pandas as pd
import sys
import os
import errno
from collections import OrderedDict
import h5py
from pathlib import Path
import copy
import re
import math
import pdb
import pptk

# This constant is used as an upper bound  for normalizing the car's speed to be between 0 and 1 
MAX_SPEED = 7.0
MAX_DISTANCE = 5.0


def checkAndCreateDir(full_path):
    """Checks if a given path exists and if not, creates the needed directories.
            Inputs:
                full_path: path to be checked
    """
    if not os.path.exists(os.path.dirname(full_path)):
        try:
            os.makedirs(os.path.dirname(full_path))
        except OSError as exc:  # Guard against race condition
            if exc.errno != errno.EEXIST:
                raise
    
def splitTrainValidationAndTestData(all_data_mappings, split_ratio=(0.7, 0.2, 0.1)):
    """Simple function to create train, validation and test splits on the data.
            Inputs:
                all_data_mappings: mappings from the entire dataset
                split_ratio: (train, validation, test) split ratio

            Returns:
                train_data_mappings: mappings for training data
                validation_data_mappings: mappings for validation data
                test_data_mappings: mappings for test data

    """
    if round(sum(split_ratio), 5) != 1.0:
        print("Error: Your splitting ratio should add up to 1")
        sys.exit()

    train_split = int(len(all_data_mappings) * split_ratio[0])
    val_split = train_split + int(len(all_data_mappings) * split_ratio[1])

    train_data_mappings = all_data_mappings[0:train_split]
    validation_data_mappings = all_data_mappings[train_split:val_split]
    test_data_mappings = all_data_mappings[val_split:]

    return [train_data_mappings, validation_data_mappings, test_data_mappings]
    
def generateDataMapAirSim(folders, pallets_file, fl_origin, pdt, edt, num_points, z_const):
    """ Data map generator for simulator(AirSim) data. Reads the driving_log csv file and returns a list of 'center camera image name - label(s)' tuples
           Inputs:
               folders: list of folders to collect data from

           Returns:
               mappings: All data mappings as a dictionary. Key is the image filepath, the values are a 2-tuple:
                   0 -> label(s) as a list of double
                   1 -> previous state as a list of double
    """

    mappings = []
    for folder in folders:
        print('Reading data from {0}...'.format(folder))
        current_df = pd.read_csv(os.path.join(folder, 'airsim_rec.txt'), sep='\t')
		
        for i in range(0, current_df.shape[0] - 1): 

			# draw relative position vector, convert them to centimeters and change them to be non-relative
            pos_x = float(current_df.iloc[i][['POS_X']])*100+fl_origin[0]
            pos_y = float(current_df.iloc[i][['POS_Y']])*100+fl_origin[1]
            pos_z = float(current_df.iloc[i][['POS_Z']])*100+fl_origin[2] + z_const
            
			# draw rotation vector and convert to degrees
            roll = math.degrees(float(current_df.iloc[i][['Roll']]))
            pitch = math.degrees(float(current_df.iloc[i][['Pitch']]))
            yaw = math.degrees(float(current_df.iloc[i][['Yaw']]))
			
			# load pallets numpy
            pallets = np.load(pallets_file)
			
            # build numpy of the values we want to subtract from the pallets values
            diff_values = np.array([[[pos_x, pos_y, pos_z], [0, 0, 0], [roll, pitch, yaw]]]*pallets.shape[0])

            # subtract diff values from the pallets numpy
            pallets_relative = np.subtract(pallets, diff_values)
            
            # extract only nearby pallets as possible data points
            relevant_pallets_list = []
            for j in range(pallets_relative.shape[0]):
                # add the pallet to the list only if it close enough to the forklift
                if np.linalg.norm(pallets_relative[j][0]) <= pdt:
                    relevant_pallets_list.append(pallets_relative[j])
            
            # if there is no nearby pallets, continue to the next sample
            if not relevant_pallets_list:
                continue
            
            # load lidar data numpy
            lidar_data_path = os.path.join(os.path.join(folder, 'images'), current_df.iloc[i]['ImageFile']).replace('\\', '/')
            lidar_data = np.load(lidar_data_path)
            
            # delete irrelevant far data points
            idxs = np.where(abs(lidar_data) > edt)[0]
            lidar_data_trimmed = np.delete(lidar_data, idxs, axis=0)
            
            # randomly remove points to keep constant number of data points
            if lidar_data_trimmed.shape[0] > num_points:
                rejected_idxs = np.random.permutation(lidar_data_trimmed.shape[0])[:lidar_data_trimmed.shape[0]-num_points]
                lidar_data_trimmed = np.delete(lidar_data_trimmed, rejected_idxs, axis=0)
            # randomly add points if there is not enough data points
            elif lidar_data_trimmed.shape[0] < num_points:
                added_data_points = np.random.normal(loc=0.0, scale=edt, size=(num_points-lidar_data_trimmed.shape[0],3))
                lidar_data_trimmed = np.concatenate((lidar_data_trimmed, added_data_points), axis=0)
            
            # add the data and label to the mapping list
            # mapping will be [lidar_data, distance, label]
            for pallet in relevant_pallets_list:
                mappings.append([lidar_data_trimmed, pallet[0], pallet[2]])

    random.shuffle(mappings) 
    
    return mappings


def generatorForH5py(data_mappings, chunk_size=32):
    """
    This function batches the data for saving to the H5 file
    """
    for chunk_id in range(0, len(data_mappings), chunk_size):
        # Data is expected to be a list of [lidar_data, distance, label]
        data_chunk = data_mappings[chunk_id:chunk_id + chunk_size]
        if (len(data_chunk) == chunk_size):
            
            lidar_data_chunk = np.asarray([a for [a, b, c] in data_chunk])
            distance_chunk = np.asarray([b for [a, b, c] in data_chunk])
            labels_chunk = np.asarray([c for [a, b, c] in data_chunk])
            
            #Flatten and yield as tuple
            yield (lidar_data_chunk, distance_chunk, labels_chunk)
            if chunk_id + chunk_size > len(data_mappings):
                raise StopIteration
    raise StopIteration
    
    
def saveH5pyData(data_mappings, target_file_path, chunk_size):
    """
    Saves H5 data to file
    """
    gen = generatorForH5py(data_mappings,chunk_size)
    
    lidar_data_chunk, distance_chunk, labels_chunk = next(gen)
    row_count = lidar_data_chunk.shape[0]

    checkAndCreateDir(target_file_path)
    with h5py.File(target_file_path, 'w') as f:

        # Initialize a resizable dataset to hold the output
        lidar_data_chunk_maxshape = (None,) + lidar_data_chunk.shape[1:]
        distance_chunk_maxshape = (None,) + distance_chunk.shape[1:]
        labels_chunk_maxshape = (None,) + labels_chunk.shape[1:]

        dset_lidar_data = f.create_dataset('lidar_data', shape=lidar_data_chunk.shape, maxshape=lidar_data_chunk_maxshape,
                                chunks=lidar_data_chunk.shape, dtype=lidar_data_chunk.dtype)

        dset_distances = f.create_dataset('distance', shape=distance_chunk.shape, maxshape=distance_chunk_maxshape,
                                       chunks=distance_chunk.shape, dtype=distance_chunk.dtype)
        
        dset_labels = f.create_dataset('label', shape=labels_chunk.shape, maxshape=labels_chunk_maxshape,
                                       chunks=labels_chunk.shape, dtype=labels_chunk.dtype)
                                       
        dset_lidar_data[:] = lidar_data_chunk
        dset_distances[:] = distance_chunk
        dset_labels[:] = labels_chunk

        for lidar_data_chunk, distance_chunk, labels_chunk in gen:
            
            # Resize the dataset to accommodate the next chunk of rows
            dset_lidar_data.resize(row_count + lidar_data_chunk.shape[0], axis=0)
            dset_distances.resize(row_count + distance_chunk.shape[0], axis=0)
            dset_labels.resize(row_count + labels_chunk.shape[0], axis=0)
            # Create the next chunk
            dset_lidar_data[row_count:] = lidar_data_chunk
            dset_distances[row_count:] = distance_chunk
            dset_labels[row_count:] = labels_chunk

            # Increment the row count
            row_count += lidar_data_chunk.shape[0]
            
            
def cook(folders, output_directory, pallets_file, fl_origin, pdt, edt, num_points, z_const, train_eval_test_split, chunk_size):
    """ Primary function for data pre-processing. Reads and saves all data as h5 files.
            Inputs:
                folders: a list of all data folders
                output_directory: location for saving h5 files
                train_eval_test_split: dataset split ratio
    """
    output_files = [os.path.join(output_directory, f) for f in ['train.h5', 'eval.h5', 'test.h5']]
    if (any([os.path.isfile(f) for f in output_files])):
       print("Preprocessed data already exists at: {0}. Skipping preprocessing.".format(output_directory))

    else:
        all_data_mappings = generateDataMapAirSim(folders, pallets_file, fl_origin, pdt, edt, num_points, z_const)
        split_mappings = splitTrainValidationAndTestData(all_data_mappings, split_ratio=train_eval_test_split)
        
        for i in range(0, len(split_mappings)-1, 1):
            print('Processing {0}...'.format(output_files[i]))
            saveH5pyData(split_mappings[i], output_files[i], chunk_size)
            print('Finished saving {0}.'.format(output_files[i]))