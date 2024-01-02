# !/usr/bin python
import os
import numpy as np
from matplotlib import pyplot as plt
from data_loader import DataLoader

class PlanResultPlot(object):
    def __init__(self):
        self.traj_plan_sec = np.array([])
        self.data_prepared = False
        self.plot_settings = {
            'plan_sec_color': 'royalblue',
            'average_color': 'firebrick',
            'average_linestyle': 'dashed',
            'line_width': 1.5,
        }
    
    def load_data(self, data_loader: DataLoader):
        if not data_loader.loaded:
            print('CSV file not loaded. Return...')
            self.data_prepared = False
            return
        
        traj_plan_sec = data_loader.get_data('traj_plan_time')
        if len(traj_plan_sec) <= 1:
            print('Invalid data.')
            self.data_prepared = False
            return
        self.traj_plan_sec = np.array(traj_plan_sec)
        self.avg_sec = self.traj_plan_sec.sum() / len(traj_plan_sec)
        self.data_prepared = True
    
    def change_setting(self, key: str, val_str: str):
        if not key in self.plot_settings:
            print('No such setting as [%s] in this plot.' % key)
        else:
            self.plot_settings[key] = val_str
        
    def plot(self, handle: plt.Axes):
        if not self.data_prepared:
            print('Data not loaded. Return...')
            return
        
        handle.stem(self.traj_plan_sec, linefmt=self.plot_settings['plan_sec_color'], 
                    markerfmt='o', use_line_collection=True)
        handle.axhline(self.avg_sec, 0, self.traj_plan_sec.shape[0], 
                       linestyle=self.plot_settings['average_linestyle'],
                       color=self.plot_settings['average_color'],
                       linewidth=self.plot_settings['line_width'])
        
        tex = 'avg = '+'{:.3f}'.format(self.avg_sec)+' s'
        handle.text(self.traj_plan_sec.shape[0]-5, self.avg_sec+0.01, tex, 
                    color=self.plot_settings['average_color'], size=10,
                    ha='center', va='bottom')
        handle.set_xlabel('trajectory No.')
        handle.set_ylabel('planning time /s')
        
        

if __name__ == "__main__":
    disturb_gen_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/disturbance_generator.csv'
    quad_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/quadrotor.csv'
    ref_gev_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/reference_governor.csv'
    csv_paths = [disturb_gen_path, quad_path, ref_gev_path]
    
    loader = DataLoader()
    loader.set_csv_paths(csv_paths)
    if(loader.load()):
        plter = PlanResultPlot()
        plter.load_data(loader)
        fig = plt.figure("NNYY")
        ax_t = fig.add_axes([0.15,0.15,0.8,0.8])
        plter.plot(ax_t)
        plt.show()
    
    exit(0)