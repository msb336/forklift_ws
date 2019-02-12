# from error import *
from Reward import *
from simfunctions import *
import airsim
import numpy as np



cli, car_controls, spot = setup()
reward_calculator = Reward(spot)

# f = open('reward-random.csv', 'w')
# f.write("delx,del_y,del_quat_z,inner,outer,reward,inner_reward,outer_reward\n")
# y_range = [-0.3, 0.3]
# iterations = 200
while True:
    vehicle_pose = getCar(cli)
    [y,r,d] = reward_calculator.getOffset(vehicle_pose)
    reward = reward_calculator.getReward()
    print(y,r,d, reward)
    sleep(5)

    # f.write("{},{},{},{},{},{},{},{}\n".format(del_x,del_y,car_rot,inner,outer,reward,inner_reward,outer_reward))


# f.close()