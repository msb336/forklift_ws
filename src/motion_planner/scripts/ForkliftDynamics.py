import numpy as np

class ForkliftDynamics:
    x = []
    y = []
    theta =[]
    input = []
    v = []
    def __init__(self, init_v=0, init_x=0, init_y=0, init_theta=0):
        
        self.tau = 1.95
        self.setKinematics(init_v, init_x, init_y, init_theta)
        self.max_steering_angle = np.pi/2.5
        self.input = [0]

    def estimateKinematics(self, throttle, steering, dt):
        self.input += [(0.01*throttle +0.995*self.input[-1]) * (200 - steering*steering)/200]
        a = (self.input[-1] - self.v[-1])*dt/self.tau
        self.v += [a + self.v[-1]]

        self.theta += [((self.v[-1]*dt*steering*self.max_steering_angle + self.theta[-1] + np.pi) % (2*np.pi))  -np.pi]
        self.x += [self.x[-1] + self.v[-2] * dt * np.cos(self.theta[-1])]
        self.y += [self.y[-1] + self.v[-2] * dt * np.sin(self.theta[-1])]

        self.cleanup()
    def extrapolate(self, throttle, steering, dt):
        input = (0.01*throttle +0.995*self.input[-1]) * (200 - steering*steering)/200
        a = (input - self.v[-1])*dt/self.tau
        v = a + self.v[-1]
        theta = ((v*dt*steering*self.max_steering_angle + self.theta[-1] + np.pi) % (2*np.pi)) - np.pi
        x = self.x[-1] + v * dt * np.cos(theta)
        y = self.y[-1] + v * dt * np.sin(theta)
        return x,y

    def setKinematics(self, velocity, x,y,theta=None):
        self.x += [x]
        self.y += [y]
        if theta is not None:
            self.theta += [theta]
        self.v += [velocity]
    def cleanup(self, values=100):
        if len(self.x) > values:
            self.x = self.x[-values:]
            self.y = self.y[-values:]
            self.theta = self.theta[-values:]
            self.input = self.input[-values:]
            self.v = self.v[-values:]
