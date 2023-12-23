# !/usr/bin python
import os
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from data_loader import DataLoader

class TrajPloter(object):
    def __init__(self):
        self.ref_pos = []
        self.uav_pos = []
        self.ref_traj_x_coef = []
        self.ref_traj_y_coef = []
        self.ref_traj_z_coef = []
        self.ref_traj_hori = []
        self.ref_traj_order = 0
        self.data_prepared = False
        self.plot_settings = {
            'ref_color': 'orange',
            'pos_color': 'darkcyan',
            'ref_linestyle': 'dashdot',
            'pos_linestyle': 'solid',
            'traj_linestyle': 'dotted',
            'ref_label': 'Reference Trajectory',
            'pos_label': 'Quadrotor Track',
            'line_width': 1.5,
        }
        
        self.draw_ref_signal = True
        self.draw_ref_traj = True
        self.draw_real_signal = True
        
    def load_data(self, data_loader: DataLoader):
        if not data_loader.loaded:
            print('CSV file not loaded. Return...')
            self.data_prepared = False
            return
        
        ref_pos_x = data_loader.get_data('ref_pos_x')
        ref_pos_y = data_loader.get_data('ref_pos_y')
        ref_pos_z = data_loader.get_data('ref_pos_z')
        ref_pos_num = len(ref_pos_x)
        if len(ref_pos_y) != ref_pos_num or len(ref_pos_z) != ref_pos_num:
            print('Reference pos data mismatch!')
            self.data_prepared = False
            return
        for i in range(ref_pos_num):
            _ref_p = [ref_pos_x[i], ref_pos_y[i], ref_pos_z[i]]
            self.ref_pos.append(_ref_p)
            
        pos_x = data_loader.get_data('pos_x')
        pos_y = data_loader.get_data('pos_y')
        pos_z = data_loader.get_data('pos_z')
        pos_num = len(pos_x)
        if len(pos_y) != pos_num or len(pos_z) != pos_num:
            print('UAV pos data mismatch!')
            self.data_prepared = False
            return
        for i in range(pos_num):
            _p = [pos_x[i], pos_y[i], pos_z[i]]
            self.uav_pos.append(_p)
            
        order = data_loader.get_param('traj_poly_order')
        if order is None:
            return
        else:
            self.ref_traj_order  = order
        self.ref_traj_hori = data_loader.get_data('traj_time_horizon')
        for i in range(self.ref_traj_order+1):
            self.ref_traj_x_coef.append(data_loader.get_data('coef_x_'+str(i)))
            self.ref_traj_y_coef.append(data_loader.get_data('coef_y_'+str(i)))
            self.ref_traj_z_coef.append(data_loader.get_data('coef_z_'+str(i)))
        traj_num = len(self.ref_traj_hori)
        if traj_num != len(self.ref_traj_x_coef[0]) or traj_num != len(self.ref_traj_y_coef[0]) or traj_num != len(self.ref_traj_z_coef[0]):
            print('Reference trajectory coefficients mismatch!')
            self.data_prepared = False
            return

        self.data_prepared = True
        
    def plot(self, plot_handle: plt.Axes):
        if not self.data_prepared:
            print('Data not loaded. Return...')
            return
        ref_pos = np.array(self.ref_pos)
        uav_pos = np.array(self.uav_pos)
        plot_handle.plot(ref_pos[:,0], ref_pos[:,1], ref_pos[:,2], 
                         color=self.plot_settings['ref_color'], 
                         linestyle=self.plot_settings['ref_linestyle'], 
                         linewidth=self.plot_settings['line_width'],
                         label=self.plot_settings['ref_label'])
        plot_handle.plot(uav_pos[:,0], uav_pos[:,1], uav_pos[:,2],
                         color=self.plot_settings['pos_color'], 
                         linestyle=self.plot_settings['pos_linestyle'], 
                         linewidth=self.plot_settings['line_width'],
                         label=self.plot_settings['pos_label'])
        for i in range(len(self.ref_traj_hori)):
            _th = self.ref_traj_hori[i]
            _traj = []
            for _t in np.linspace(0, _th, 20, endpoint=True):
                _tmp_x = 0
                _tmp_y = 0
                _tmp_z = 0
                for j in range(self.ref_traj_order+1):
                    _tmp_x += self.ref_traj_x_coef[j][i] * _t**j
                    _tmp_y += self.ref_traj_y_coef[j][i] * _t**j
                    _tmp_z += self.ref_traj_z_coef[j][i] * _t**j
                _traj.append([_tmp_x, _tmp_y, _tmp_z])
            _traj_mat = np.array(_traj)
            if i == 0:
                plot_handle.plot(_traj_mat[:,0], _traj_mat[:,1], _traj_mat[:,2],
                                 color=self.plot_settings['ref_color'], 
                                 linestyle=self.plot_settings['traj_linestyle'], 
                                 linewidth=self.plot_settings['line_width'],
                                 label=self.plot_settings['ref_label'])
            else:
                plot_handle.plot(_traj_mat[:,0], _traj_mat[:,1], _traj_mat[:,2],
                                 color=self.plot_settings['ref_color'], 
                                 linestyle=self.plot_settings['traj_linestyle'], 
                                 linewidth=self.plot_settings['line_width'])