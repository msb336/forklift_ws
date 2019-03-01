# import numpy as np

class PID:
    kp=2
    ki=1
    kd=0.5

    time=0
    prev_time=0

    error=0
    prev_error=0

    d_error=0
    i_error=0

    control_input=0
    def __init__(self, kp=0.1, ki=0.1, kd=0.1):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.error
    def getIntegralError(self):
        if self.prev_time != 0:
            dt = self.time-self.prev_time
            self.i_error = self.i_error + (self.error+self.prev_error*0.5)*dt
    def getDerivativeError(self):
        if self.prev_time != 0:
            dt = self.time-self.prev_time
            self.d_error = (self.error-self.prev_error)/dt
    def updateControl(self):
        self.getIntegralError()
        self.getDerivativeError()
        self.control_input = self.kp*self.error + self.kd*self.d_error + self.ki*self.i_error
    def update(self, goal, position, time):
        self.error = goal - position
        self.prev_time = self.time
        self.time = time
        self.updateControl()
        # print("time", time, "dt", time-self.prev_time, "error", [self.error, self.d_error, self.i_error])
        return self.control_input



    