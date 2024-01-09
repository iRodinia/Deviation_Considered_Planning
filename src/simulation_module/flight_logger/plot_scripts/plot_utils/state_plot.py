# !/usr/bin python
import os
import numpy as np
from matplotlib import pyplot as plt
from data_loader import DataLoader

class PositionPlot(object):
    def __init__(self):
        self.ref_pos = []
        self.ref_pos_ts = []
        self.uav_pos = []
        self.uav_pos_ts = []
        self.data_prepared = False
        self.plot_settings = {
            'ref_x_color': 'firebrick',
            'ref_y_color': 'mediumseagreen',
            'ref_z_color': 'dodgerblue',
            'ref_x_label': 'reference_x',
            'ref_y_label': 'reference_y',
            'ref_z_label': 'reference_z',
            'pos_x_color': 'red',
            'pos_y_color': 'green',
            'pos_z_color': 'blue',
            'pos_x_label': 'uav_trajectory_x',
            'pos_y_label': 'uav_trajectory_y',
            'pos_z_label': 'uav_trajectory_z',
            'ref_linestyle': 'dashdot',
            'pos_linestyle': 'solid',
            'line_width': 1.5,
            'disturb_color': 'slategrey',
            'disturb_transp': 0.4,
        }
        self.plot_disturb = True
        self.ellipse_num = 0
    
    def load_data(self, data_loader: DataLoader):
        if not data_loader.loaded:
            print('CSV file not loaded. Return...')
            self.data_prepared = False
            return
        
        ref_pos_x, ref_pos_times = data_loader.get_data_with_timestamp('ref_pos_x')
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
            self.ref_pos_ts.append(ref_pos_times[i])
            
        pos_x, pos_times = data_loader.get_data_with_timestamp('pos_x')
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
            self.uav_pos_ts.append(pos_times[i])
        
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
            
            if len(self.disturb_hs) == 0 or len(self.disturb_rads) == 0 or len(self.disturb_biases) == 0:
                print('Unable to plot disturbance range.')
                self.plot_disturb = False
            else:
                self.disturb_poses = np.array(self.disturb_poses)
                self.disturb_dirs = np.array(self.disturb_dirs)
                self.disturb_hs = np.array(self.disturb_hs)
                self.disturb_rads = np.array(self.disturb_rads)
                self.disturb_biases = np.array(self.disturb_biases)

        self.data_prepared = True
    
    def change_setting(self, key: str, val_str: str):
        if not key in self.plot_settings:
            print('No such setting as [%s] in this plot.' % key)
        else:
            self.plot_settings[key] = val_str
    
    def check_disturb(self, pos: np.ndarray):
        if not self.plot_disturb or not self.data_prepared or not self.ellipse_num:
            return
        
        for i in range(self.ellipse_num):
            disturb_pose = self.disturb_poses[i]
            disturb_dir = self.disturb_dirs[i]
            disturb_h = self.disturb_hs[i]
            disturb_rad = self.disturb_rads[i]
            disturb_bias = self.disturb_biases[i]
            
            vec_CP = pos - disturb_pose
            vec_CO = disturb_dir
            alpha = np.arccos(np.dot(vec_CP, vec_CO) / (np.linalg.norm(vec_CP) * np.linalg.norm(vec_CO))) # type: ignore
            sin_alpha = np.sin(alpha)
            cos_alpha = np.cos(alpha)
            range_a = disturb_h / 2
            bias_a = range_a - disturb_bias
            A = (range_a * sin_alpha)**2 + (disturb_rad * cos_alpha)**2
            B = 2 * range_a * bias_a * sin_alpha**2
            C = (bias_a * sin_alpha)**2 - (disturb_rad * cos_alpha)**2
            cos_theta = (np.sqrt(B*B - 4*A*C) - B) / 2 / A
            sin_theta = np.sqrt(1 - cos_theta**2)
            ellipse_range = np.sqrt((range_a*cos_theta + bias_a)**2 + (disturb_rad*sin_theta)**2)
            
            if np.linalg.norm(vec_CP) <= ellipse_range:
                return True
        
        return False
            
    def plot(self, x_handle: plt.Axes, y_handle: plt.Axes, z_handle: plt.Axes):
        if not self.data_prepared:
            print('Data not loaded. Return...')
            return
        ref_pos = np.array(self.ref_pos)
        ref_pos_t = np.array(self.ref_pos_ts)
        uav_pos = np.array(self.uav_pos)
        uav_pos_t = np.array(self.uav_pos_ts)
        x_handle.plot(ref_pos_t, ref_pos[:,0], 
                        color=self.plot_settings['ref_x_color'], 
                        linestyle=self.plot_settings['ref_linestyle'], 
                        linewidth=self.plot_settings['line_width'],
                        label=self.plot_settings['ref_x_label'])
        x_handle.plot(uav_pos_t, uav_pos[:,0], 
                        color=self.plot_settings['pos_x_color'], 
                        linestyle=self.plot_settings['pos_linestyle'], 
                        linewidth=self.plot_settings['line_width'],
                        label=self.plot_settings['pos_x_label'])
        x_handle.set_xlabel('timestamp /s')
        x_handle.set_xlabel('x /m')
        y_handle.plot(ref_pos_t, ref_pos[:,1], 
                        color=self.plot_settings['ref_y_color'], 
                        linestyle=self.plot_settings['ref_linestyle'], 
                        linewidth=self.plot_settings['line_width'],
                        label=self.plot_settings['ref_y_label'])
        y_handle.plot(uav_pos_t, uav_pos[:,1], 
                        color=self.plot_settings['pos_y_color'], 
                        linestyle=self.plot_settings['pos_linestyle'], 
                        linewidth=self.plot_settings['line_width'],
                        label=self.plot_settings['pos_y_label'])
        y_handle.set_xlabel('timestamp /s')
        y_handle.set_xlabel('y /m')
        z_handle.plot(ref_pos_t, ref_pos[:,2], 
                        color=self.plot_settings['ref_z_color'], 
                        linestyle=self.plot_settings['ref_linestyle'], 
                        linewidth=self.plot_settings['line_width'],
                        label=self.plot_settings['ref_z_label'])
        z_handle.plot(uav_pos_t, uav_pos[:,2], 
                        color=self.plot_settings['pos_z_color'], 
                        linestyle=self.plot_settings['pos_linestyle'], 
                        linewidth=self.plot_settings['line_width'],
                        label=self.plot_settings['pos_z_label'])
        z_handle.set_xlabel('timestamp /s')
        z_handle.set_xlabel('z /m')
        
        if self.plot_disturb:
            if len(self.disturb_hs) == 0 or len(self.disturb_rads) == 0 or len(self.disturb_biases) == 0:
                print('Unable to plot disturbance range.')
                self.plot_disturb = False
                return
            
            disturb_area = []
            for i in range(uav_pos.shape[0]):
                _p = uav_pos[i,:]
                disturb_area.append(self.check_disturb(_p))
                        
            x_handle.fill_between(uav_pos_t, uav_pos[:,0].min()-0.05, uav_pos[:,0].max()+0.05, where=disturb_area, 
                                  facecolor=self.plot_settings['disturb_color'], alpha=self.plot_settings['disturb_transp'])
            y_handle.fill_between(uav_pos_t, uav_pos[:,1].min()-0.05, uav_pos[:,1].max()+0.05, where=disturb_area,
                                  facecolor=self.plot_settings['disturb_color'], alpha=self.plot_settings['disturb_transp'])
            z_handle.fill_between(uav_pos_t, uav_pos[:,2].min()-0.05, uav_pos[:,2].max()+0.05, where=disturb_area,
                                  facecolor=self.plot_settings['disturb_color'], alpha=self.plot_settings['disturb_transp'])


if __name__ == "__main__":
    disturb_gen_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/disturbance_generator.csv'
    quad_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/quadrotor.csv'
    ref_gev_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/reference_governor.csv'
    csv_paths = [disturb_gen_path, quad_path, ref_gev_path]
    
    loader = DataLoader()
    loader.set_csv_paths(csv_paths)
    if(loader.load()):
        plter = PositionPlot()
        plter.load_data(loader)
        fig = plt.figure("NNYY")
        ax_x = fig.add_subplot(131)
        ax_y = fig.add_subplot(132)
        ax_z = fig.add_subplot(133)
        plter.plot(ax_x, ax_y, ax_z)
        ax_x.legend()
        ax_y.legend()
        ax_z.legend()
        plt.show()
    
    exit(0)