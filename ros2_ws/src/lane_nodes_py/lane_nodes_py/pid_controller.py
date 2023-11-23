import time


class PID:

    def __init__(self, Kp, Ki, Kd):

        # initialize the three PID values
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Set the current time for when the contrller starts
        self.last_time = time.time()

        # Reset integral value and last error
        self.clear()

    def clear(self):
        # ITerm needed as the integral value is cumulative
        self.ITerm = 0.0

        # Last Error needed for hte derivative calculation
        self.last_error = 0.0


    def update(self, error):
        """Calculates PID value for given reference feedback"""

        current_time = time.time()
        delta_time = current_time - self.last_time
        delta_error = error - self.last_error

        # The proportional term is just hte error
        self.PTerm =  error

        # The integral term is just the cumulative sum of the change in time times the current error
        self.ITerm += error * delta_time

        # The derivative term is the change in error over change in time
        DTerm = 0.0
        if delta_time > 0:
            DTerm = delta_error / delta_time

        # calculate PID output
        # print("Error: " + str(error)+ ", components of output: "+ str(self.Kp *self.PTerm) + ", " + str((self.Ki * self.ITerm)) + ", " + str(self.Kd * DTerm))
        output = (self.Kp *self.PTerm) + (self.Ki * self.ITerm) + (self.Kd * DTerm)

        # Remember last time and last error for next calculation
        self.last_time = current_time
        self.last_error = error

        ## Return output
        return output

    def setKp(self, new_kp):
        self.Kp = new_kp

    def setKi(self, new_ki):
        self.Ki = new_ki

    def setKd(self, new_kd):
        self.Kd = new_kd


if __name__ == '__main__':
    test_pid = PID(1, 0.5, 1)
    y_val = 0
    goal = 1
    vertical_velocity = 0
    while(1 in range(10)):
        error = goal - y_val
        print("Current y_val: " + str(y_val))
        time.sleep(0.1)
        output = test_pid.update(error)
        vertical_velocity += output
        y_val = y_val + vertical_velocity/100

