import setup_path 
import airsim
from airsim.types import Vector3r, Quaternionr, Pose
import math
import numpy as np
import time
from argparse import ArgumentParser

from LidarAcquisition import *
from simfunctions import *
from Reward import *
from Agent import *


def interpret_action(action, client, distance):
    car_speed = client.getCarState().speed
    car_controls.throttle = -(distance+2)*0.5/4 + 0.5*(-1 - car_speed)
    car_controls.is_manual_gear = True
    car_controls.manual_gear = -1

    if action == 0:
        car_controls.steering = 1
    elif action == 1:
        car_controls.steering = -1
    else:
        car_controls.steering = 0
    return car_controls

def isDone(client, reward, distance, del_distance, time, time_threshold=10):
    successful = False
    if (distance < 0.7) and reward > 0.9:
        done = True
        car_controls.throttle=0
        car_controls.steering=0
        client.setCarControls(car_controls)
        print("inside pallet! distance", distance, "reward", reward)
        successful = True
    elif (time > time_threshold) or (client.simGetCollisionInfo().has_collided):
        done = True
        car_controls.throttle=0
        car_controls.steering=0
        client.setCarControls(car_controls)
        print("didn't win", time, client.simGetCollisionInfo().has_collided, "reward", reward)
    elif del_distance > 0:
        print("del distance > 0", del_distance, "reward", reward)
        done = True
    else:
        done = False
        # reward = reward * (time_threshold - time)/(time_threshold)-0.5
    
    #reward -= 0.5
    return done, reward, successful

def setRange(count):
    #if success_count < 10:
    #    theta_range =[-0.03,0.03]
    #    offset_range =[-0.1, 0.1]
    #elif success_count < 20:
    #    theta_range =[-0.1, 0.1]
    #    offset_range =[-0.1, 0.1]
    #else:
    r = count*0.4 / 2000
    theta_range =[0,0] #[-0.15, 0.15]
    offset_range =[-r, r]
    return theta_range, offset_range



input_size=600
 # Make RL agent
NumBufferFrames = 4
SizeRows = input_size*2
SizeCols = 1
NumActions = 3
agent = DeepQAgent((NumBufferFrames, SizeRows, SizeCols), NumActions, monitor=True, train_after=0)

# Train
epoch = 100
current_step = 0
max_steps = epoch * 250000

client,car_controls,center, car_center = setup()
reward_calculator = Reward(center)
lidar_getter = LidarAcquisition(client, return_size=input_size)

responses = lidar_getter.getPolar()
current_state = responses


tnow = time.clock()
max_time = 10
done=0
increment = 50
save_increment = 20
success=0
cumulative_reward = 0
old_distance = np.inf
distance = 4

while True:

    action = agent.act(current_state)
    car_controls = interpret_action(action,client, distance)
    client.setCarControls(car_controls)

    vehicle_pose = getCar(client)
    [y,r,d] = reward_calculator.getOffset(vehicle_pose)


    reward = reward_calculator.getReward()
    distance = math.fabs(d)
    del_d = distance - old_distance
    old_distance = distance


    done, reward, successful = isDone(client, reward, distance, del_d, time.clock()-tnow)
    cumulative_reward += reward


    agent.observe(current_state, action, reward, done)
    

    if done:
        old_distance = np.inf
        current_step +=1
        if successful:
            success+=1
            print("success count:", success, "accuracy:", 100*success/current_step, "cumulative reward:", cumulative_reward)
        done=0

        cumulative_reward = 0
        agent.train()

        client.reset()
        theta, offset = setRange(success)
        random_pose = setRandomPose(client, center, offset, theta)[0]
        reward_calculator.reset(random_pose)
        setCar(client, car_center)
        car_control = interpret_action(2,client, 4)
        client.setCarControls(car_control)
        tnow = time.clock()
        time.sleep(0.2)

        if current_step % save_increment == 0:
            print("saving model")
            agent._trainer.save_checkpoint("reward_(e^-dy)_input_rtheta1d_2")

    responses = lidar_getter.getPolar()
    if len(responses) == input_size*2:
        current_state = responses
    else:
        print("didnt update")