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
        self.ref_path = []
        self.ref_traj_x_coef = []
        self.ref_traj_y_coef = []
        self.ref_traj_z_coef = []
        self.ref_traj_hori = []
        self.ref_traj_order = 0
        self.data_prepared = False
        self.plot_settings = {
            'ref_traj_color': 'orange',
            'pos_traj_color': 'darkcyan',
            'ref_traj_linestyle': 'dashdot',
            'pos_traj_linestyle': 'solid',
            'path_color': 'red',
            'path_linestyle': 'dashed',
            'traj_linestyle': 'dotted',
            'ref_label': 'Reference Trajectory',
            'path_label': 'Reference Path',
            'pos_label': 'Quadrotor Track',
            'line_width': 2.0,
            'disturb_color': 'slategrey',
            'disturb_transp': 0.3,
        }
        self.plot_disturb = True
        self.draw_ref_signal = False
        self.draw_ref_traj = True
        self.draw_real_signal = True
    
    def change_setting(self, key: str, val_str: str):
        if not key in self.plot_settings:
            print('No such setting as [%s] in this plot.' % key)
        else:
            self.plot_settings[key] = val_str
        
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
        
        ref_path_x = data_loader.get_data('ref_path_px')
        ref_path_y = data_loader.get_data('ref_path_py')
        ref_path_z = data_loader.get_data('ref_path_pz')
        ref_path_num = len(ref_path_x)
        if len(ref_path_y) != ref_path_num or len(ref_path_z) != ref_path_num:
            print('Reference path data mismatch!')
            self.data_prepared = False
            return
        for i in range(ref_path_num):
            _ref_p = [ref_path_x[i], ref_path_y[i], ref_path_z[i]]
            self.ref_path.append(_ref_p)
            
        order = data_loader.get_param('traj_poly_order')
        if order is None:
            return
        else:
            self.ref_traj_order = int(order)
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
        
        if self.plot_disturb:
            self.ellipse_num = data_loader.get_param('disturb_num')
            if self.ellipse_num is None:
                self.ellipse_num = 0
            else:
                self.ellipse_num = int(self.ellipse_num)
            self.disturb_poses = []
            self.disturb_dirs = []
            self.disturb_hs = []
            self.disturb_rads = []
            self.disturb_biases = []
            self.disturb_trans_mats = []
            for i in range(self.ellipse_num):
                self.disturb_poses.append([data_loader.get_param('disturb_'+str(i)+'_pos_x'),
                                           data_loader.get_param('disturb_'+str(i)+'_pos_y'),
                                           data_loader.get_param('disturb_'+str(i)+'_pos_z')])
                self.disturb_dirs.append([data_loader.get_param('disturb_'+str(i)+'_dir_x'),
                                          data_loader.get_param('disturb_'+str(i)+'_dir_y'),
                                          data_loader.get_param('disturb_'+str(i)+'_dir_z')])
                self.disturb_hs.append(data_loader.get_param('disturb_'+str(i)+'_cylinder_h'))
                self.disturb_rads.append(data_loader.get_param('disturb_'+str(i)+'_cylinder_rad'))
                self.disturb_biases.append(data_loader.get_param('disturb_'+str(i)+'_cylinder_bias'))
                _trans_mat = np.zeros((4,4))
                _trans_mat[0,0] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_00')
                _trans_mat[0,1] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_01')
                _trans_mat[0,2] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_02')
                _trans_mat[0,3] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_03')
                _trans_mat[1,0] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_10')
                _trans_mat[1,1] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_11')
                _trans_mat[1,2] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_12')
                _trans_mat[1,3] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_13')
                _trans_mat[2,0] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_20')
                _trans_mat[2,1] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_21')
                _trans_mat[2,2] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_22')
                _trans_mat[2,3] = data_loader.get_param('disturb_'+str(i)+'_trans_mat_23')
                _trans_mat[3,3] = 1
                self.disturb_trans_mats.append(_trans_mat)
            
            if len(self.disturb_hs) == 0 or len(self.disturb_rads) == 0 or len(self.disturb_biases) == 0 or len(self.disturb_trans_mats) == 0:
                print('Unable to plot disturbance range.')
                self.plot_disturb = False
            else:
                self.disturb_poses = np.array(self.disturb_poses)
                self.disturb_dirs = np.array(self.disturb_dirs)
                self.disturb_hs = np.array(self.disturb_hs)
                self.disturb_rads = np.array(self.disturb_rads)
                self.disturb_biases = np.array(self.disturb_biases)
                self.disturb_trans_mats = np.array(self.disturb_trans_mats)

        self.data_prepared = True
        
    def plot(self, plot_handle: Axes3D):
        if not self.data_prepared:
            print('Data not loaded. Return...')
            return
        ref_pos = np.array(self.ref_pos)
        ref_path = np.array(self.ref_path)
        uav_pos = np.array(self.uav_pos)
        plot_handle.plot(ref_path[:,0], ref_path[:,1], ref_path[:,2], 
                         color=self.plot_settings['path_color'], 
                         linestyle=self.plot_settings['path_linestyle'], 
                         linewidth=self.plot_settings['line_width'],
                         label=self.plot_settings['path_label'])
        if self.draw_ref_signal:
            plot_handle.plot(ref_pos[:,0], ref_pos[:,1], ref_pos[:,2], 
                            color=self.plot_settings['ref_traj_color'], 
                            linestyle=self.plot_settings['ref_traj_linestyle'], 
                            linewidth=self.plot_settings['line_width'],
                            label=self.plot_settings['ref_label'])
        if self.draw_real_signal:
            plot_handle.plot(uav_pos[:,0], uav_pos[:,1], uav_pos[:,2],
                            color=self.plot_settings['pos_traj_color'], 
                            linestyle=self.plot_settings['pos_traj_linestyle'], 
                            linewidth=self.plot_settings['line_width'],
                            label=self.plot_settings['pos_label'])
        if self.draw_ref_traj:
            for i in range(len(self.ref_traj_hori)):
                _th = self.ref_traj_hori[i]
                _traj = []
                for _t in np.linspace(0, _th, 20, endpoint=True):   # _th-1.0 is a modification for better illustrating the result
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
                                    color=self.plot_settings['ref_traj_color'], 
                                    linestyle=self.plot_settings['traj_linestyle'], 
                                    linewidth=self.plot_settings['line_width'],
                                    label=self.plot_settings['ref_label'])
                else:
                    plot_handle.plot(_traj_mat[:,0], _traj_mat[:,1], _traj_mat[:,2],
                                    color=self.plot_settings['ref_traj_color'], 
                                    linestyle=self.plot_settings['traj_linestyle'], 
                                    linewidth=self.plot_settings['line_width'])
                    
        if self.plot_disturb:
            if len(self.disturb_hs) == 0 or len(self.disturb_rads) == 0 or len(self.disturb_biases) == 0 or len(self.disturb_trans_mats) == 0:
                print('Unable to plot disturbance model.')
                self.plot_disturb = False
                return
            if not self.ellipse_num:
                return
            u = np.linspace(0, 2 * np.pi, 25)
            v = np.linspace(0, np.pi, 25)
            for id in range(self.ellipse_num):
                disturb_h = self.disturb_hs[id]
                disturb_bias = self.disturb_biases[id]
                disturb_rad = self.disturb_rads[id]
                disturb_trans_mat = self.disturb_trans_mats[id]
                for i in range(3):
                    k = (i+1) / 3.0
                    x = k * (disturb_h / 2 * np.outer(np.cos(u), np.sin(v)) + (disturb_h / 2 - disturb_bias))
                    y = k * disturb_rad * np.outer(np.sin(u), np.sin(v))
                    z = k * disturb_rad * np.outer(np.ones(np.size(u)), np.cos(v))
                    d = np.outer(np.ones(np.size(u)), np.ones(np.size(v)))
                    org_crds = np.stack((x, y, z, d))
                    new_crds = np.zeros(org_crds.shape)
                    for m in range(org_crds.shape[1]):
                        for n in range(org_crds.shape[2]):
                            new_crds[:,m,n] = disturb_trans_mat @ org_crds[:,m,n]
                    plot_handle.plot_surface(new_crds[0,:], new_crds[1,:], new_crds[2,:], 
                                            color=self.plot_settings['disturb_color'], alpha=self.plot_settings['disturb_transp'])
                
            
                

if __name__ == "__main__":
    disturb_gen_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/disturbance_generator.csv'
    quad_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/quadrotor.csv'
    ref_gov_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/reference_governor.csv'
    glob_pln_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/global_planner.csv'
    csv_paths = [disturb_gen_path, quad_path, ref_gov_path, glob_pln_path]
    
    loader = DataLoader()
    loader.set_csv_paths(csv_paths)
    if(loader.load()):
        plter = TrajPloter()
        plter.load_data(loader)
        fig = plt.figure("NNYY")
        ax = fig.add_axes([0.15,0.15,0.8,0.8], projection='3d')
        plter.plot(ax) # type: ignore
        ax.legend()
        plt.show()
    
    exit(0)