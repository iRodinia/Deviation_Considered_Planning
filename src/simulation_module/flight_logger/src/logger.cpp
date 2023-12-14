#include "flight_logger/logger.h"

FlightLogger::FlightLogger(string folder_name, string file_name){
    f_name = file_name;
    target_file_path = LOG_PATH_PREFIX + folder_name + "/" + file_name + ".csv";
    param_num = 0;
    data_type_num = 0;
    data_length = 0;
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
        ROS_WARN("Log parameters with names (%ld) and values (%ld) size mismatch.", param_names.size(), param_vals.size());
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
        ROS_WARN("Attempt to set %d tags, not allowed.", data_type_num);
        data_type_num = 0;
        tags_set = false;
        return;
    }
    datas_map["timestamps"] = vector<double>();
    for(int i=0; i<data_type_num; i++){
        datas_map[tags[i]] = vector<double>();
    }
    data_length = 0;
    tags_set = true;
}

void FlightLogger::logData(double time_sec, vector<string> tags, vector<double> vals){
    if(!tags_set){
        ROS_INFO("Log data in [%s] before set tags is not allowed.", f_name.c_str());
        return;
    }
    int len = tags.size();
    if(vals.size() != len){
        ROS_WARN("Log data with tags (%ld) and values (%ld) size mismatch.", tags.size(), vals.size());
        return;
    }
    if(len != data_type_num){
        ROS_INFO("Log %d datas, but %d expected.", len, data_type_num);
        return;
    }
    bool legal_tags = true;
    for(auto t : tags){
        if(datas_map.find(t) == datas_map.end()){
            ROS_WARN("Tag [%s] not in the data list.", t.c_str());
            legal_tags = false;
        }
    }
    if(!legal_tags){
        ROS_INFO("Abandon data log at time %f due to illegal data tags.", time_sec);
        return;
    }
    datas_map["timestamps"].push_back(time_sec);
    for(int i=0; i<len; i++){
        datas_map[tags[i]].push_back(vals[i]);
    }
    data_length++;
}

bool FlightLogger::saveFile(){
    if(param_num <= 0 && data_type_num <= 0){
        ROS_INFO("Nothing to record in [%s], abort creating log file.", f_name.c_str());
        return true;
    }
    fstream fout;
    fout.open(target_file_path.c_str(), ios::out | ios::app);
    if(!fout.good()){
        ROS_WARN("Open/Create log file [%s] failed!", f_name.c_str());
        return false;
    }
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