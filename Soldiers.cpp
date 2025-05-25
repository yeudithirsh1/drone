#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <queue>
#include <mutex>
#include <atomic>
#include <vector>
#include <chrono>
#include <random>
#include <map>
using namespace std;

queue<string> task_queue;  // ��� ������ ������
mutex task_queue_mutex;         // ����� ����
mutex data_store_mutex;        // ����� ������
map<string, string> data_store; // ����� ������ �����

// ������� ������ �����
void sensor_writer(const string& file_path) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, 100);

    while (true) {
        int new_value = dis(gen);
        ofstream file(file_path);
        if (file.is_open()) {
            file << new_value;
            file.close();
        }
        this_thread::sleep_for(chrono::milliseconds(200));  // 5 ����� ������
    }
}

// ����� ������ ����� ���� � �� 0.2 �����
void periodic_enqueue_reads(const vector<string>& file_paths) {
    while (true) {
        for (const auto& path : file_paths) {
            lock_guard<std::mutex> lock(task_queue_mutex);
            task_queue.push(path);
        }
        this_thread::sleep_for(chrono::milliseconds(200));  // 5 ����� ������
    }
}

// ����� ������ ����� ������
void worker() {
    while (true) {
        string path;
        {
            lock_guard<mutex> lock(task_queue_mutex);
            if (!task_queue.empty()) {
                path = task_queue.front();
                task_queue.pop();
            }
            else {
                this_thread::sleep_for(chrono::milliseconds(50));  // ����� ���� �� ��� ������ ����
                continue;
            }
        }

        std::ifstream file(path);
        if (file.is_open()) {
            string new_value;
            getline(file, new_value);
            file.close();

            lock_guard<mutex> lock(data_store_mutex);
            data_store[path] = new_value;  // ����� ������
        }
        else {
            std::cerr << "Error reading " << path << std::endl;
        }
    }
}

// ���� ����������
void stats_monitor() {
    while (true) {
        this_thread::sleep_for(chrono::seconds(1));
        lock_guard<std::mutex> lock(data_store_mutex);
        cout << "[STAT] Files tracked: " << data_store.size() << " | Tasks in queue: " << task_queue.size() << std::endl;
    }
}

//int main() {
//    const string folder_to_watch = "C:/Users/This User/Documents/project/Soldiers";
//    vector<string> file_paths;
//
//    // ����� �����
//    for (int i = 0; i < 6; ++i) {
//        string file_path = folder_to_watch + "/sensor_" + to_string(i + 1) + ".txt";
//        ofstream file(file_path);
//        if (file.is_open()) {
//            file << "0";  // ����� ��� ������
//            file.close();
//        }
//        file_paths.push_back(file_path);
//    }
//
//    // ����� ��������� ������
//    for (const auto& path : file_paths) {
//        thread(sensor_writer, path).detach();
//    }
//
//    // ����� �������� �����
//    for (int i = 0; i < 6; ++i) {
//        thread(worker).detach();
//    }
//
//    // ������� ������ ������ ����
//    thread(periodic_enqueue_reads, file_paths).detach();
//
//    // ������� �����������
//    thread(stats_monitor).detach();
//
//    // ����� ������� ����� �����
//    this_thread::sleep_for(chrono::hours(24));
//
//    return 0;
//}
