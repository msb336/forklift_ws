from error import *
from simfunctions import *
import airsim
import numpy as np



cli, car_controls, spot = setup()
reward_calculator = Reward(cli)

f = open('reward-random.csv', 'w')
f.write("delx,del_y,del_quat_z,inner,outer,reward,inner_reward,outer_reward\n")
y_range = [-0.3, 0.3]
rot_range = [-0.2,0.2]
iterations = 50

plot_data= np.zeros((iterations,3))
for i in range(iterations):
    pose, disp, rot = setRandomPose(cli, spot, y_range, rot_range)
    inner,outer = reward_calculator.avgError(10)
    reward, inner_reward, outer_reward = reward_calculator.calculateReward()
    del_x = disp[0]
    del_y = disp[1]
    print(del_x,del_y,rot,inner,outer,reward,inner_reward,outer_reward)
    f.write("{},{},{},{},{},{},{},{}\n".format(del_x,del_y,rot,inner,outer,reward,inner_reward,outer_reward))
    plot_data[i][0] = del_y*100
    plot_data[i][1] = min(2-abs(rot), abs(rot))*100
    plot_data[i][2] = reward
    sleep(0.1)
viewChart(plot_data)

f.close()

#while True:
#    r,theta = reward_calculator.getPolar(True)
#    print(min(theta), max(theta))
#    sleep(0.1)