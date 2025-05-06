#include "Soldiers.h"  
#include <iostream>  
#include <fstream>  
#include <unordered_map>  
#include <string>  
#include <filesystem>  

namespace fs = filesystem;  
using namespace std;  

int main() {  
   string folder_path = "C:/Users/This User/Documents/project/Soldiers";  

   unordered_map<string, FileData> file_hash;  

   try {  
       for (const auto& entry : fs::directory_iterator(folder_path)) {  
           if (entry.is_regular_file() && entry.path().extension() == ".txt") {  
               ifstream file(entry.path());  
               if (file) {  
                   string line1, line2;  
                   getline(file, line1);  
                   getline(file, line2);  

                   FileData data;  
                   data.gps_id = line1;  
                   data.start_position = line2;  

                   file_hash[entry.path().filename().string()] = data;  
               }  
               else {  
                   cerr << "The file cannot be opened: " << entry.path() << endl;  
               }  
           }  
       }  
   }  
   catch (const fs::filesystem_error& e) {  
       cerr << "File system error: " << e.what() << endl;  
   }  
   return 0;  
}
