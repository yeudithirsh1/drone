#include <string>

using namespace std;

struct YoloDetection {
    int classId;
    float bbox[4];
};

string getLastCreatedFile(const string& directoryPath);
void processDirectoryAndCopyFilesOnly();
bool runPythonScript();