import numpy as np
import pdb
import argparse
from PIL import Image
import os

parser = argparse.ArgumentParser()
parser.add_argument('--log', '-log', help='path to log file', default='..\\..\\..\\..\\Unreal Projects\\Factory\\Saved\\Logs\\Factory.log', type=str)
parser.add_argument('--dest_dir', '-dest_dir', help='path to destination folder', default='.\\output_map', type=str)
args = parser.parse_args()

# find last occurrence of printing
log_file = open(args.log, "r")
start_idx = -1
end_idx = -1
for i, line in enumerate(log_file):
	if "Printing actors locations" in line:
		start_idx = i
	if "Done printing actors locations" in line:
		end_idx = i

# quit if there is no legal printing		
if start_idx == -1 or end_idx == -1 or start_idx >= end_idx:
	print("didn't found any printing of the objects")
	quit()

# export printing to list
log_file = open(args.log, "r")
printing_list = []
for i, line in enumerate(log_file):
	if i < start_idx:
		continue
	if i > end_idx:
		break
	
	printing_list.append(line)

# filter relevant obstacles
obstacles_names = ["Pillar", "EuroPalletSingle", "SparePartsRack", "Wall_Euro_pallets_static", "ProducedGoodsRack", "WrappedEuroPallet", "Pallet_BP", "Wall_pallets_static", "High_Density_ShuttleRacks", "pallet_boxes", "ultralifter_rescaled", "shrinkWrap", "Europalette", "USpalette"]

# Dictionary to store relevant obstacles data
# key: mesh name
# value: [origin, bounding box, rotation]
obstacles_dict = {}

for i in range(1,len(printing_list),3):

	for obstacle in obstacles_names:
		if obstacle in printing_list[i]:
		
			# obsctacle is needed for map - filter object name, origin and bounding box
			key = printing_list[i].split("] ")[-1][:-1]
			origin_str = printing_list[i+1].split("] ")[-1][:-1]
			bbox_str = printing_list[i+2].split("] ")[-1][:-1]
			rotation_str = printing_list[i+3].split("] ")[-1][:-1]

			# convert strings to 3-dimensional vectors
			origin_vec = [float(x[2:]) for x in origin_str.split(" ")]
			bbox_vec = [float(x[2:]) for x in bbox_str.split(" ")]
			rotation_vec = [float(x[2:]) for x in rotation_str.split(" ")]
			
			# add to dictionary
			obstacles_dict[key] = [origin_vec, bbox_vec, rotation_vec]

# Dictionary to store manually added relevant obstacles data
# key: mesh name
# value: [lower origin, dimensions]
manual_obstacles_dict = {}

manual_obstacles_dict['B_Stack_StaticMesh1'] = [[-2378,-330],[260,1180]]
manual_obstacles_dict['B_Stack_StaticMesh2'] = [[-2875,-330],[260,1180]]
manual_obstacles_dict['B_Stack_StaticMesh3'] = [[-3360,-330],[260,1180]]
manual_obstacles_dict['H_Stack_static1'] = [[700,-1072],[1090,240]]
manual_obstacles_dict['H_Stack_static2'] = [[700,-1545],[1090,240]]
manual_obstacles_dict['H_Stack_static3'] = [[700,-2045],[1090,240]]
manual_obstacles_dict['BeltF1'] = [[-1340,1025],[810,130]]
manual_obstacles_dict['BeltF2'] = [[-1340,-280],[120,1435]]
manual_obstacles_dict['BeltF_part1'] = [[-1180,-240],[60,50]]
manual_obstacles_dict['BeltF_part2'] = [[-1178,52],[60,50]]
manual_obstacles_dict['BeltF_part3'] = [[-1178,152],[48,52]]
manual_obstacles_dict['BeltF_part4'] = [[-1163,254],[38,133]]
manual_obstacles_dict['BeltF_part5'] = [[-1178,432],[48,52]]
manual_obstacles_dict['BeltF_part6'] = [[-1163,533],[38,133]]
manual_obstacles_dict['BeltF_part7'] = [[-1178,743],[48,52]]
manual_obstacles_dict['BeltF_part8'] = [[-1050,938],[133,38]]
manual_obstacles_dict['BeltF_part9'] = [[-894,918],[50,60]]
manual_obstacles_dict['BeltF_part10'] = [[-766,938],[133,38]]
manual_obstacles_dict['BeltE1'] = [[-1665,-2247],[1105,127]]
manual_obstacles_dict['BeltE2'] = [[-1665,-2247],[125,1090]]
manual_obstacles_dict['BeltE_part1'] = [[-725,-2087],[52,48]]
manual_obstacles_dict['BeltE_part2'] = [[-797,-2085],[50,60]]
manual_obstacles_dict['BeltE_part3'] = [[-1005,-2087],[52,48]]
manual_obstacles_dict['BeltE_part4'] = [[-1507,-1813],[60,50]]
manual_obstacles_dict['BeltE_part5'] = [[-1507,-1412],[60,50]]

# iterate all obstacles to add to the original list
for key, value in manual_obstacles_dict.items():
	obstacles_dict[key] = [[value[0][0]+(value[1][0]/2),value[0][1]+(value[1][1]/2),92],[value[1][0]/2,value[1][1]/2,150]]

# save the obstacles as numpy in the current format:
# [[origin, bounding box, rotation]*N]
# copy pallets details to a different dictionary, to prevent printing unnecessary objects
pallets_dict = {}
pallets_possible_names = ["EuroPalletSingle", "WrappedEuroPallet", "pallet_boxes", "Pallet_BP", "Europalette", "USpalette"]

for key, value in obstacles_dict.items():	
	for possible_name in pallets_possible_names:
		if possible_name in key:
			pallets_dict[key] = value

pallets_list = []
for key, value in pallets_dict.items():	
	pallets_list.append(value)

np.save(os.path.join(args.dest_dir, "obstacles"), np.array(pallets_list))

# actual map boundaries
map_x_range = [-4045.1, 2334.9]
map_y_range = [-2525.2, 1604.9]

# 2-dimensional navigation map: on obstacles - 0, on empty road - 100
# width: (map_x_range[1]-map_x_range[0]) = 6380
# height: (map_y_range[1]-map_y_range[0]) = 4130
navigation_map = np.zeros((int(map_x_range[1]-map_x_range[0]),int(map_y_range[1]-map_y_range[0])), dtype=int)

# iterate all relevant obstacles to paint unauthorized space in the navigation map
for key, value in obstacles_dict.items():
	
	# compute the exact indexes to paint in the navigation map, according to the obstacle dimensions
	paint_x_range = [int(value[0][0]-(value[1][0])-map_x_range[0]), int(value[0][0]+(value[1][0])-map_x_range[0])]
	paint_y_range = [int(value[0][1]-(value[1][1])-map_y_range[0]), int(value[0][1]+(value[1][1])-map_y_range[0])]
	
	# paint
	navigation_map[paint_x_range[0]:paint_x_range[1],paint_y_range[0]:paint_y_range[1]] = 255

# save map array in dest folder
np.save(os.path.join(args.dest_dir, "map"), navigation_map)

# align map with the simulation's axis for better visualization
navigation_map = navigation_map.astype(np.uint8)
navigation_map = np.flip(navigation_map, axis=1)
navigation_map = np.flip(navigation_map, axis=0)
navigation_map = np.transpose(navigation_map)

# save map array as image for visualization
map_img = Image.fromarray(navigation_map)
map_img.save(os.path.join(args.dest_dir,"map.png"))