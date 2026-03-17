import numpy as np
from scipy.interpolate import interp1d

class A_star_path:

    def __init__(self, x_list=[], y_list=[],yaw_list=[]):
        if len(x_list)>0 and len(y_list) > 0 and len(yaw_list)>0:
            self.x_path = x_list
            self.y_path = y_list
            self.yaw_path = yaw_list
            self.new_path_acq = False
        else:
            self.x_path = []
            self.y_path = []
            self.yaw_path = []
            self.new_path_acq = False

    def update_path(self, x_path, y_path, yaw_path = []):
        _, ids = np.unique([x_path, y_path], axis=1, return_index=True)
        ids = np.sort(ids)
        print("ids", ids)
        x_path_ = [x_path[idx] for idx in ids]
        y_path_ = [y_path[idx] for idx in ids]
        yaw_path_ = [yaw_path[idx] for idx in ids]

        # x_path_new, y_path_new, yaw_path_new = self.initial_interp(x_path_, y_path_, yaw_path_)
        # self.x_path = x_path_new
        # self.y_path = y_path_new
        # self.yaw_path = yaw_path_new
                
        self.x_path = x_path_
        self.y_path = y_path_
        self.yaw_path = yaw_path_        
        self.new_path_acq = True

    def read_path(self):
        self.new_path_acq = False
        # return self.x_path, self.y_path, self.yaw_path, self.new_path_acq

    def initial_interp(self, x_path, y_path, yaw):
        t = np.linspace(0, 1, len(x_path))
        interp_func_x = interp1d(t, x_path, kind='linear')
        interp_func_y = interp1d(t, y_path, kind='linear')
        interp_func_th = interp1d(t, yaw, kind='linear')

        t_new = np.arange(0, 1, 0.01)
        interpolated_x = interp_func_x(t_new)
        interpolated_y = interp_func_y(t_new)
        interpolated_yaw = interp_func_th(t_new)

        return interpolated_x.tolist(), interpolated_y.tolist(), interpolated_yaw.tolist()



