import math
import robot_common.global_defs as defs
import numpy as np

class kinematics:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, dt_in = 0.1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
        self.dt = dt_in

    def get_current_meas_state(self):
        return [self.x, self.y, self.yaw, self.v]

    #update the state with the latest measurements
    def update_state(self, x_meas, y_meas, yaw_meas, v_meas, dt_meas=0.1):
        self.x = x_meas
        self.y = y_meas
        self.yaw = yaw_meas
        self.v = v_meas
        self.dt = dt_meas

    #maybe T is not required as a_vec and d_vec have the required elements -> T
    def predict_motion(self, a_vec, d_vec, T):
        x_bar = np.zeros((defs.NX, T + 1))
        x_pred = self.x
        y_pred = self.y
        yaw_pred = self.yaw
        v_pred = self.v
        x_bar[:,0] = np.array([x_pred, y_pred, yaw_pred, v_pred]).transpose()

        for (acc_i, delta_i, i) in zip(a_vec, d_vec, range(1, T + 1)):

            if delta_i > defs.MAX_STEER:
                delta_i = defs.MAX_STEER
            elif delta_i < -defs.MAX_STEER:
                delta_i = -defs.MAX_STEER

            x_pred = x_pred + v_pred * math.cos(yaw_pred) * self.dt
            y_pred = y_pred + v_pred * math.sin(yaw_pred) * self.dt
            yaw_pred = yaw_pred + v_pred / defs.WB * math.tan(delta_i) * self.dt
            v_pred = v_pred + acc_i * self.dt

            if v_pred > defs.MAX_SPEED:
                v_pred = defs.MAX_SPEED
            elif v_pred < defs.MIN_SPEED:
                v_pred = defs.MIN_SPEED

            x_bar[0, i] = x_pred
            x_bar[1, i] = y_pred
            x_bar[2, i] = yaw_pred
            x_bar[3, i] = v_pred
        
        return x_bar