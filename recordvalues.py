from error import *
from simfunctions import *
import airsim
import numpy as np



cli, car_controls, spot = setup()
reward_calculator = Reward(cli)

f = open('reward-random.csv', 'w')
f.write("delx,del_y,del_quat_z,inner,outer,reward,inner_reward,outer_reward\n")
y_range = [-0.3, 0.3]
iterations = 200
while True:
    inner,outer = reward_calculator.avgError(3)
    reward, inner_reward, outer_reward = reward_calculator.calculateReward()
    car_pos, car_rot = getCar(cli)
    del_x = spot[0] -car_pos[0]
    del_y = spot[1] - car_pos[1]

    f.write("{},{},{},{},{},{},{},{}\n".format(del_x,del_y,car_rot,inner,outer,reward,inner_reward,outer_reward))


f.close()