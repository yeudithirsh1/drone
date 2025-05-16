#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include "rpc/client.h"
#include "common/Common.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "Reading_sensors.h"

using LidarDataSet = vector<vector<Point>>;
using namespace std;
// === הצהרת פונקציית זיהוי מכשולים ===
void detect_obstacles_from_lidar(const LidarDataSet& lidar_data_set);

// === פונקציה לשליפת נתוני LiDAR מחיישן מסוים ===
std::vector<Point> get_lidar_points(msr::airlib::MultirotorClient& client, const std::string& sensor_name) {
    std::vector<Point> points;
    auto data = client.getLidarData(sensor_name, "Drone1");

    for (size_t i = 0; i + 2 < data.point_cloud.size(); i += 3) {
        points.push_back({ data.point_cloud[i], data.point_cloud[i + 1], data.point_cloud[i + 2] });
    }

    return points;
}

// === קריאת כל ששת חיישני הלידאר והעברת הנתונים לעיבוד ===
void read_all_lidars(msr::airlib::MultirotorClient& client) {
    while (true) {
        LidarDataSet all_lidar_data;

        all_lidar_data.push_back(get_lidar_points(client, "LidarFront"));
        all_lidar_data.push_back(get_lidar_points(client, "LidarLeft"));
        all_lidar_data.push_back(get_lidar_points(client, "LidarRight"));
        all_lidar_data.push_back(get_lidar_points(client, ""))

        detect_obstacles_from_lidar(all_lidar_data);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz
    }
}

// === קריאה מחיישן IMU ===
void read_imu(msr::airlib::MultirotorClient& client) {
    while (true) {
        auto imu = client.getImuData("ImuSensor1", "Drone1");
        std::cout << "IMU Accel: ("
            << imu.linear_acceleration.x_val << ", "
            << imu.linear_acceleration.y_val << ", "
            << imu.linear_acceleration.z_val << ")\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }
}

// === קריאה מחיישן GPS ===
void read_gps(msr::airlib::MultirotorClient& client) {
    while (true) {
        auto gps = client.getGpsData("GpsSensor1", "Drone1");
        std::cout << "GPS: ("
            << gps.gnss.geo_point.latitude << ", "
            << gps.gnss.geo_point.longitude << ", "
            << gps.gnss.geo_point.altitude << ")\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz
    }
}

int main() {
    msr::airlib::MultirotorClient client;
    client.confirmConnection();

    std::thread lidar_thread(read_all_lidars, std::ref(client));
    std::thread imu_thread(read_imu, std::ref(client));
    std::thread gps_thread(read_gps, std::ref(client));

    lidar_thread.join();
    imu_thread.join();
    gps_thread.join();

    return 0;
}
