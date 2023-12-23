# !/usr/bin python
import os
import csv

class DataLoader(object):
    def __init__(self):
        self.csv_paths = []
        self.loaded = False
        self.params = {}
        self.datas = {}

    def set_csv_paths(self, csv_path_list):
        for _p in csv_path_list:
            self.csv_paths.append(_p)

    def load(self):
        if(len(self.csv_paths) <= 0):
            print('CSV paths not set. Load nothing.')
            return
        for path in self.csv_paths:
            if not os.path.exists(path):
                print('File paths not exist, received CSV path: %s' % path)
                print('Exit...')
                self.loaded = False
                return False
            
            with open(path, newline='', mode='r+') as disturb_file:
                file_data = csv.reader(disturb_file, delimiter=',')
                line_count = 0
                param_num = 0
                data_num = 0
                for row in file_data:
                    if line_count == 0:
                        if row[0] == 'parameter_number':
                            param_num = int(row[1])
                        else:
                            print('Unsupported file format, return...')
                            return False
                    elif line_count == 1:
                        if row[0] == 'data_type_number':
                            data_num = int(row[1])
                        else:
                            print('Unsupported file format, return...')
                            return False
                    else:
                        if line_count < 2 + param_num:
                            self.params[row[0]] = float(row[1])
                        elif line_count < 2 + param_num + data_num:
                            if row[0] in self.datas:
                                print('The key: %s has already been loaded, delete existing data.' % row[0])
                            self.datas[row[0]] = []
                            for i in range(1,len(row)):
                                self.datas[row[0]].append(float(row[i]))
                        else:
                            break
                    line_count += 1
                    
        self.loaded = True
        return True

    def get_param(self, key: str):
        if not self.loaded:
            print('CSV not loaded. Return...')
            return None
        if key in self.params:
            return self.params[key]
        else:
            print('No parameter named: %s. Return nothing.' % key)
            return None
    
    def get_data(self, key: str):
        if not self.loaded:
            print('CSV not loaded. Return...')
            return []
        if key in self.datas:
            return self.datas[key]
        else:
            print('No data named: %s. Return nothing.' % key)
            return []
    
    def get_data_with_timestamp(self, key: str):
        if not self.loaded:
            print('CSV not loaded. Return...')
            return [], []
        if key in self.datas:
            return self.datas[key], self.datas[key+'_ts']
        else:
            print('No data named: %s. Return nothing.' % key)
            return [], []


if __name__ == "__main__":
    disturb_gen_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/disturbance_generator.csv'
    quad_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/quadrotor.csv'
    ref_gev_path = '/home/cz_linux/Documents/Deviation_Considered_Planning/src/simulation_module/flight_logger/flight_logs/log_test_exp/reference_governor.csv'
    csv_paths = [disturb_gen_path, quad_path, ref_gev_path]
    
    loader = DataLoader()
    loader.set_csv_paths(csv_paths)
    if(loader.load()):
        print("YES")
    
    exit(0)