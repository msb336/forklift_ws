#%matplotlib inline
import numpy as np
import pandas as pd
import h5py
from matplotlib import use
use("TkAgg")
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw
import os
import Cooking
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--data_dir', '-data_dir', help='path to raw data folder', default='C:\\Users\\user1\\Documents\\AirSim', type=str)
parser.add_argument('--output_dir', '-output_dir', help='path to destination folder', default='cooked_data', type=str)
parser.add_argument('--start_x', '-start_x', help='x coordinate of the forklift start point', default=83.770279, type=float)
parser.add_argument('--start_y', '-start_y', help='y coordinate of the forklift start point', default=-1772.335938, type=float)
parser.add_argument('--start_z', '-start_z', help='z coordinate of the forklift start point', default=0.304142, type=float)
parser.add_argument('--pdt_max', '-pdt_max', help='pallet distance threshold max in centimeters', default=350, type=float)
parser.add_argument('--pdt_min', '-pdt_min', help='pallet distance threshold min in centimeters', default=100, type=float)
parser.add_argument('--edt', '-edt', help='environment distance threshold in meters', default=5, type=float) # for each axis. throwing points that are larger than ...
parser.add_argument('--num_points', '-num_points', help='number of data points to store eventually', default=2048, type=int) # defined by the input to the net
parser.add_argument('--z_const', '-z_const', help='constant to add to the z axis of the forklift', default=216.4, type=float)
parser.add_argument('--batch_size', '-batch_size', help='number of samples in a batch', default=32, type=int)
args = parser.parse_args()
# No test set needed, since testing in our case is running the model on an unseen map in AirSim
train_eval_test_split = [0.8, 0.2, 0.0]

# list of all recordings data_folders
data_folders = [name for name in os.listdir(args.data_dir)]
data_folders = [os.path.join(args.data_dir, f) for f in data_folders]

Cooking.cook(data_folders, args.output_dir, [args.start_x, args.start_y, args.start_z], args.pdt_max, args.pdt_min, args.edt, args.num_points, args.z_const, train_eval_test_split, args.batch_size)
