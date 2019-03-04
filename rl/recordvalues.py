'''
Data Collection for supervised learning
'''

from SimpleController import *
from simfunctions import *
from LidarAcquisition import *
import airsim
import numpy as np




def gen_string(lidar_x, lidar_y, action):
    # convert input data into string to be printed to file
    x_data = numpyToString(lidar_x)
    y_data = numpyToString(lidar_y)
    action_string = "{} ".format(action) #numpyToString(action)
    return '|labels ' + action_string + '|features ' + x_data + y_data +"\n"

def numpyToString(array_arg):
    output_string = ""
    for i in range(array_arg.size):
        output_string += "{} ".format(array_arg[i])
    return output_string

# initialize airsim client
cli, car_controls, pallet_spot, car_spot = setup()
controller = SimpleController(pallet_spot)
[init_offset, angle, init_dist] = controller.getOffset(car_spot)
planner = ForkliftPlanner(pallet_spot, init_dist, init_offset)
lidar_getter = LidarAcquisition(cli, return_size=484)
f = open('lidar_test.txt', 'a')


# run control loop
while True:

    lidar_x, lidar_y = lidar_getter.getLidar()

    local_goal, global_goal = planner.update()
    action = controller.calculateMotorControl(local_goal)
    car_controls = interpret_action(action, cli)
    cli.setCarControls(car_controls)
    [offset, angle, dist] = controller.getOffset(getCar(cli))

    if lidar_x != []:
        fstring = gen_string(lidar_x, lidar_y, action)
        f.write(fstring)

    if np.abs(dist) < 1:
        cli.reset()
        theta, offset = setRange()
        random_pose = setRandomPose(cli, pallet_spot, offset, theta)[0]
        setCar(cli, car_spot)
        car_control = interpret_action(2,cli)
        cli.setCarControls(car_control)
        controller.reset(random_pose)
        [init_offset, angle, init_dist] = controller.getOffset(car_spot)
        planner.initialize(random_pose, init_dist, init_offset)
        tnow = time.clock()
        time.sleep(0.2)


f.close()
