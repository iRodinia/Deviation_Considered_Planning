#ifndef FLIGHT_LOGGER
#define FLIGHT_LOGGER

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>

using namespace std;

string LOG_PATH_PREFIX = ros::package::getPath("flight_logger") + "/flight_logs/";

/* Generate a CSV log file */

class FlightLogger {
public:
    FlightLogger(string folder_name, string file_name);
    ~FlightLogger() {}
    void logParameter(string param_name, double param_val);
    void logParameter(vector<string> param_names, vector<double> param_vals);
    void setDataTags(vector<string> tags);
    void logData(double time_sec, vector<string> tags, vector<double> vals);
    bool saveFile();

private:
    string target_file_path, f_name;
    unordered_map<string, double> params_map;
    unordered_map<string, vector<double>> datas_map;
    int param_num, data_type_num, data_length;
    bool tags_set = false;
};

#endif