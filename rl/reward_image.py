import numpy as np
import matplotlib.pyplot as plt

def savetoimage(response, name):
    print(np.amax(response), np.amin(response))
    from PIL import Image
    image = Image.fromarray(response)
    image =image.convert('L')
    #image.show()
    input()
    image.save(name)


def heatmap(array):
    fig, ax = plt.subplots()
    ax.imshow(array, cmap='hot', interpolation='nearest')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    plt.show()

def polynomial(value, function):
    f = 0
    for i in range(len(function)):
        factor = i
        f += function[factor]*np.power(value, factor)
    return f
alpha = [0,0, 1]
beta = [0,0,5]
iterations = 200
offset_max = 0.25
angle_max = np.pi/4
reward = np.zeros((iterations,iterations))

for i in range(iterations):
    y = abs(i*offset_max*2/iterations - offset_max)
    for j in range(iterations):
        r = abs(j*angle_max*2/iterations - angle_max)
        offset_reward = 1-np.exp(12*abs(y))
        angle_reward = -polynomial(r, beta) #1-np.exp(2*abs(r))
        reward[i,j] = offset_reward + angle_reward


print(np.amin(reward), np.amax(reward))
reward = -reward
heatmap(reward)

input()

minimum = np.amin(reward)
reward = reward - minimum
reward = reward*255 / np.amax(reward)

savetoimage(reward, "{}y{}r.png".format(alpha,beta))


