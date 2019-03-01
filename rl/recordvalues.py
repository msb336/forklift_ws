from SimpleController import *
from simfunctions import *
from LidarAcquisition import *
import airsim
import numpy as np

def gen_string(lidar_x, lidar_y, throttle, action):
    x_data = np.array2string(lidar_x, separator = ' ')[1:-3]
    y_data = np.array2string(lidar_y, separator = ' ')[1:-3]
    action_string = np.array2string(action, separator = ' ')[1:-3]
    string = '|labels ' + action_string + '|features ' + x_data + y_data


cli, car_controls, pallet_spot, car_spot = setup()
controller = SimpleController(pallet_spot)
lidar_getter = LidarAcquisition(cli)
#f = open('lidar_train.txt')

while True:
    
    vehicle_pose = getCar(cli)
    controller.getOffset(vehicle_pose)
    action = controller.sendMotorControl()
    car_controls = interpret_action(action, cli)
    cli.setCarControls(car_controls)

    if np.abs(controller.distance_) < 1:
        print(controller.distance_)
        cli.reset()
        theta, offset = setRange()
        random_pose = setRandomPose(cli, pallet_spot, offset, theta)[0]
        setCar(cli, car_spot)
        car_control = interpret_action(2,cli)
        cli.setCarControls(car_control)
        tnow = time.clock()
        time.sleep(0.2)


#f.close()
