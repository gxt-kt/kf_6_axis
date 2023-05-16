#include <string>
#include <vector>
#include "kf_eigen.hpp"
#include "readdata.hpp"

using namespace std;

string file_path = "../data.txt";

int main() {
  vector<vector<string>> file_data = ReadFile(file_path);
  vector<DataStruct<float>> data;
  ConvertData(file_data, data);
  cout << "data.size=" << data.size() << endl;

  for (int i{1}; i < data.size(); i++) {
    auto ret = KF_6_Axis(data[i].time / 1000.0,
                         data[i].acc_x,
                         data[i].acc_y,
                         data[i].acc_z,
                         data[i].gyro_x,
                         data[i].gyro_y,
                         data[i].gyro_z);
    cout << "data_set: roll=" << data[i].roll << "  pitch=" << data[i].pitch << "  yaw=" << data[i].yaw << "\n";
    cout << "kf      : roll=" << ret(0) * 57.3 << "  pitch=" << ret(1) * 57.3 << " yaw=" << ret(2) * 57.3 << "\n";
    cout << "\n";
  }

  return 0;
}
