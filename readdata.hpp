#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

/**
 * dataset : Time,ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,Roll,Pitch
 *
 * direction :
 *              ^ X
 *              |
 *              |
 *            Z(.)------>Y
 *
 * data unit:
 *   Time : ms
 *   ACC_* : m/s^2
 *   GYRO_* : rad/s
 *   Roll : deg
 *   Pitch : deg
 *
 */

// T usually use float
template <typename T>
struct DataStruct {
  unsigned int time;
  T acc_x;
  T acc_y;
  T acc_z;
  T gyro_x;
  T gyro_y;
  T gyro_z;
  T roll;
  T pitch;
  T yaw;

  // can convert from vector<string>
  DataStruct<T>(vector<string> input) {
    std::stringstream ss;
    ss.str(input[0]);
    ss >> time;
    ss.clear();
    ss.str(input[1]);
    ss >> acc_x;
    ss.clear();
    ss.str(input[2]);
    ss >> acc_y;
    ss.clear();
    ss.str(input[3]);
    ss >> acc_z;
    ss.clear();
    ss.str(input[4]);
    ss >> gyro_x;
    ss.clear();
    ss.str(input[5]);
    ss >> gyro_y;
    ss.clear();
    ss.str(input[6]);
    ss >> gyro_z;
    ss.clear();
    ss.str(input[7]);
    ss >> roll;
    ss.clear();
    ss.str(input[8]);
    ss >> pitch;
    ss.clear();
    ss.str(input[9]);
    ss >> yaw;
    ss.clear();
  }
};

// convert data file to <vector<string>>
inline vector<vector<string>> ReadFile(string filename) {
  vector<vector<string>> data;
  ifstream infile(filename);
  if (!infile.is_open()) {
    std::cout << "Failed to open file!" << std::endl;
    std::terminate();
  }

  string line;
  while (getline(infile, line)) {
    stringstream ss(line);
    string field;
    vector<string> row;
    while (getline(ss, field, ',')) {
      row.push_back(field);
    }
    data.push_back(row);
  }
  infile.close();

  return data;
}

// convert <vector<string>> to vector<DataStruct<T>>
template <typename T>
inline void ConvertData(vector<vector<string>> data,
                        vector<DataStruct<T>>& output) {
  for (int i = 1; i < data.size(); i++) {
    // output.emplace_back(DataStruct<T>(data[i]));
    output.push_back(data[i]);
  }
}
