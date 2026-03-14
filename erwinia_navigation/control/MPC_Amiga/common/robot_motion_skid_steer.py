import math
import common.global_defs as defs
import numpy as np
class kinematics:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, w=0.0, dt_in = 0.1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = w
        self.predelta = None
        self.dt = dt_in

        self.x_measured = 0
        self.y_measured = 0
        self.yaw_measured = 0
        self.v_measured = 0
        self.w_measured = 0

        self.IsFresh = False

    #update the state with the latest measurements
    def set_meas(self, x_meas, y_meas, yaw_meas, v_meas, w_meas):

        self.x_measured = x_meas
        self.y_measured = y_meas
        self.yaw_measured = yaw_meas
        self.v_measured = v_meas
        self.w_measured = w_meas
        #self.IsFresh = True

    #maybe T is not required as a_vec and d_vec have the required elements -> T
    def predict_motion(self, a_vec, w_vec, T):
        x_bar = np.zeros((defs.NX, T + 1))
        x_pred = self.x
        y_pred = self.y
        yaw_pred = self.yaw
        v_pred = self.v
        w_pred = self.w
        x_bar[:,0] = np.array([x_pred, y_pred, yaw_pred, v_pred, w_pred]).transpose()

        for (acc_i, w_i, i) in zip(a_vec, w_vec, range(1, T + 1)):

            #if delta_i > defs.MAX_STEER:
            #    delta_i = defs.MAX_STEER
            #elif delta_i < -defs.MAX_STEER:
            #    delta_i = -defs.MAX_STEER

            x_pred = x_pred + v_pred * math.cos(yaw_pred) * self.dt
            y_pred = y_pred + v_pred * math.sin(yaw_pred) * self.dt
            yaw_pred = yaw_pred + w_pred * self.dt
            v_pred = v_pred + acc_i * self.dt
            w_pred = w_pred + w_i* self.dt

            if v_pred > defs.MAX_SPEED:
                v_pred = defs.MAX_SPEED
            elif v_pred < defs.MIN_SPEED:
                v_pred = defs.MIN_SPEED

            x_bar[0, i] = x_pred
            x_bar[1, i] = y_pred
            x_bar[2, i] = v_pred
            x_bar[3, i] = yaw_pred
            x_bar[4, i] = w_pred
            
        return x_bar

    def get_current_meas_state(self):
        #self.IsFresh = True
        #return [self.x_measured, self.y_measured, self.yaw_measured, self.v_measured]
        self.x = self.x_measured
        self.y = self.y_measured
        self.yaw = self.yaw_measured
        self.v = self.v_measured
        self.w = self.w_measured

    def get_current_pos_meas(self):
        return [self.x_measured, self.y_measured] #for the moment is 2d     

    def refreshState(self):
        self.IsFresh = False        
