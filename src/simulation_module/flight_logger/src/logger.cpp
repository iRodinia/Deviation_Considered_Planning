#include "flight_logger/logger.h"

#define RED "\033[31m"
#define YELLOW "\033[33m"
#define CYAN "\033[36m"

FlightLogger::FlightLogger(string folder_name, string file_name){
    f_name = file_name;
    target_file_folder = LOG_PATH_PREFIX + folder_name;
    target_file_path = LOG_PATH_PREFIX + folder_name + "/" + file_name + ".csv";
    param_num = 0;
    data_type_num = 0;
    tags_set = false;
}

void FlightLogger::logParameter(string param_name, double param_value){
    if(params_map.find(param_name) == params_map.end()){
        param_num++;
    }
    params_map[param_name] = param_value;
}

void FlightLogger::logParameter(vector<string> param_names, vector<double> param_vals){
    if(param_names.size() != param_vals.size()){
        cout << YELLOW << "Log parameters with names (" << param_names.size() << ") and values (" << param_vals.size() << ") size mismatch." << endl;
        return;
    }
    for(int i=0; i<param_names.size(); i++){
        params_map[param_names[i]] = param_vals[i];
        if(params_map.find(param_names[i]) == params_map.end()){
            param_num++;
        }
    }
}

void FlightLogger::setDataTags(vector<string> tags){
    datas_map.clear();
    data_type_num = tags.size();
    if(data_type_num <= 0){
        cout << YELLOW << "Attempt to set " << data_type_num << " tags, not allowed." << endl;
        data_type_num = 0;
        tags_set = false;
        return;
    }
    for(int i=0; i<data_type_num; i++){
        datas_map[tags[i]] = vector<double>();
        datas_map[tags[i]].reserve(1e5);
        datas_map[tags[i]+"_ts"] = vector<double>();
        datas_map[tags[i]+"_ts"].reserve(1e5);
    }
    tags_set = true;
}

void FlightLogger::logData(double time_sec, string tag, double val){
    if(!tags_set){
        cout << RED << "Log data in [" << f_name << "] before set tags is not allowed." << endl;
        return;
    }
    if(datas_map.find(tag) == datas_map.end()){
        cout << YELLOW << "Tag [" << tag << "] not in the data list." << endl;
        return;
    }
    datas_map[tag].push_back(val);
    datas_map[tag+"_ts"].push_back(time_sec);
}

void FlightLogger::logData(double time_sec, vector<string> tags, vector<double> vals){
    if(!tags_set){
        cout << RED << "Log data in [" << f_name << "] before set tags is not allowed." << endl;
        return;
    }
    int len = tags.size();
    if(vals.size() != len){
        cout << YELLOW << "Log data with tags (" << tags.size() << ") and values (" << vals.size() << ") size mismatch." << endl;
        return;
    }
    bool legal_tags = true;
    for(auto t : tags){
        if(datas_map.find(t) == datas_map.end()){
            cout << YELLOW << "Tag [" << t << "] not in the data list." << endl;
            legal_tags = false;
        }
    }
    if(!legal_tags){
        cout << YELLOW << "Abandon data log at time " << time_sec << " due to illegal data tags." << endl;
        return;
    }
    for(int i=0; i<len; i++){
        datas_map[tags[i]].push_back(vals[i]);
        datas_map[tags[i]+"_ts"].push_back(time_sec);
    }
}

bool FlightLogger::saveFile(){
    if(param_num <= 0 && data_type_num <= 0){
        cout << RED << "Nothing to record in [" << f_name << "], abort creating log file." << endl;
        return true;
    }

    cout << CYAN << "Set log path to: " << target_file_path << endl;
    if(access(target_file_path.c_str(), F_OK) != -1){
        if(remove(target_file_path.c_str()) == 0){
            cout << YELLOW << "Delete existing log file [" << f_name << "]..." << endl;
        }
    }

    if (!boost::filesystem::is_directory(target_file_folder)){
        cout << YELLOW << "Create log folder: " << target_file_folder << endl;
        if (!boost::filesystem::create_directory(target_file_folder)){
            cout << RED << "Create folder failed: " << target_file_folder << endl;
            return false;
        }
    }

    fstream fout;
    fout.open(target_file_path.c_str(), ios::out);
    if(!fout.good()){
        cout << RED << "Open/Create log file [" << f_name << "] failed!" << endl;
        return false;
    }
    fout << "parameter_number: " << param_num << '\n';
    fout << "data_type_number: " << data_type_num << '\n';
    if(param_num > 0){
        for(auto it = params_map.begin(); it != params_map.end(); it++){
            string _name = it->first;
            double _val = it->second;
            fout << _name << ", " << _val << '\n';
        }
    }
    if(data_type_num > 0){
        for(auto it = datas_map.begin(); it != datas_map.end(); it++){
            string _name = it->first;
            if(it->second.size() > 0){
                fout << _name;
                for(auto _d : it->second){
                    fout << ", " << _d;
                }
                fout << '\n';
            }
        }
    }
    fout.close();
    return true;
}