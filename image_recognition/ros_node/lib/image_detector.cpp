#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <json/json.h>
#include <vector>

#include "image_detector.h"

// for zed cam
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

// to comunicate with image recognition program
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "object.h"

using namespace std;

image_detector::image_detector(){
    
    //inizializing zed camera:
    imgNumber = 25;
    sleepDuration = 10;

    // Create a ZED Camera object
    sl::Camera zed;

    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    

    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        cout << "ERROR" << endl;
        return -1;
    }

    // Print camera information
    auto camera_info = zed.getCameraInformation();
    cout << endl <<"inizializing camera..."<< endl;
    cout <<"ZED Model                 : "<< camera_info.camera_model << endl;
    cout <<"ZED Serial Number         : "<< camera_info.serial_number << endl;
    cout <<"ZED Camera Firmware       : "<< camera_info.camera_configuration.firmware_version <<"/"<<camera_info.sensors_configuration.firmware_version<< endl;
    cout <<"ZED Camera Resolution     : "<< camera_info.camera_configuration.resolution.width << "x" << camera_info.camera_configuration.resolution.height << endl;
    cout <<"ZED Camera FPS            : "<< zed.getInitParameters().camera_fps << endl;

    return 0;
}

image_detector::~image_detector(){
    // Exit
    zed.close();
}

vector<Object> image_detector::distance(vector<Object> v1, vector<Object> v2){

    vector<Object> vetRet;

    // sort the two vector to semplify the operations
    sort(v1.begin(), v1.end(),[](const Object &o1, const Object &o2) {
        return o1 < o2;
    });
    sort(v2.begin(), v2.end(),[](const Object &o1, const Object &o2) {
        return o1 < o2;
    });

    // Camera intrinsics
    float fx = zed.getCameraInformation().calibration_parameters.left_cam.fx;
    float fy = zed.getCameraInformation().calibration_parameters.left_cam.fy;
    float cx = zed.getCameraInformation().calibration_parameters.left_cam.cx;
    float cy = zed.getCameraInformation().calibration_parameters.left_cam.cy;

    // Triangulation to estimate object's 3D position
    float depth1 = 1.0; // Dummy depth value for image1
    float depth2 = 1.0; // Dummy depth value for image2


    // calculate the distance     
    auto getDistance = [](const Object o1, const Object o2){
        if(!strcmp(o1.getName().c_str(), o2.getName().c_str())) return NULL;
        float x = (x1 - cx) * depth1 / fx;
        float y = (y1 - cy) * depth1 / fy;
        float z = (fx * depth1 + fy * depth2) / (fx + fy);
        cout << "Estimated 3D position: (" << x << ", " << y << ", " << z << ")" << endl;
        Object ret = new Object(o1.getName(), x, y, z);
    };

    size_t size=v1.size;
    if(size > v2.size){
        size = v2.size;
    }

    for(size_t i=0; size; i++){
        Object app= getDistance(v1[i], v2[i]);
        if(app!=NULL){
            vetRet.insert(app);
        }
    }
    return vetRet;
}

/*
bool readJSONFromFile( Json::Value& jsonData) {
    ifstream jsonFile("objects.json");
    if (!jsonFile.is_open()) {
        cerr << "Error opening JSON file: " << filename << endl;
        return false;
    }

    Json::CharReaderBuilder builder;
    JSONCPP_STRING errs;
    if (!Json::parseFromStream(builder, jsonFile, jsonData, &errs)) {
        cerr << "Error parsing JSON: " << errs << endl;
        jsonFile.close();
        return false;
    }
    jsonFile.close();

    return true;
}
*/

vector<Object> image_detector::call_image_recognition_routine(const char* arguments) {

    vector<Object> objVector;

    // chiama il programma per il riconoscimento delle imagini 
    string command = "python3 object_detector.py ";
    command += arguments;
    int returnCode = system(command.c_str());
    if (returnCode != 0) {
        cerr << "Command failed with exit code: " << returnCode << endl;
        return NULL;
    }
    
    // legge il file json creato 
    Json::Value jsonData;

    auto readJSONFromFile = [&jsonData]() {
        ifstream jsonFile("objects.json");
        if (!jsonFile.is_open()) {
            cerr << "Error opening JSON file" << endl;
            return false;
        }

        Json::CharReaderBuilder builder;
        JSONCPP_STRING errs;
        if (!Json::parseFromStream(builder, jsonFile, jsonData, &errs)) {
            cerr << "Error parsing JSON: " << errs << endl;
            jsonFile.close();
            return false;
        }
        jsonFile.close();

        return true;
    };

    if (readJSONFromFile()){
        
        // count how many elements there are in the json file
        int numElements = jsonData.size();
        cout << "Number of elements in the JSON array: " << numElements << endl;

        // Access the JSON data
        for (const Json::Value& object : jsonData) {
            string name = object["name"].asString();
            double probability = object["probability"].asDouble();
            int x1 = object["x1"].asInt();
            int y1 = object["y1"].asInt();
            int x2 = object["x2"].asInt();
            int y2 = object["y2"].asInt();
           
            cout << "Name: " << name << ", Probability: " << probability <<endl;
            cout << "Bounding Box: (" << x1 << ", " << y1 << "), (" << x2 << ", " << y2 << ")" <<endl;
   
            objVector.push_back(new Object(name, probability, x1, y1));
        }
    }
    return objVector;
}

vector<Object> image_detector::image_recognition(){

    vector<Object> vetRet;

    // Create a Mat to store images
    sl::Mat image1, image2;
    string imgName1,imgName2; 

    // Check that a new image is succesfully acquired
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
        zed.retrieveImage(image1, sl::VIEW_LEFT);
        
        // Convert sl::Mat to cv::Mat (share buffer)
        cv::Mat cvImage1 = cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, image1.getPtr<sl::uchar1>(sl::MEM::CPU));
        imgName1 = "../images/image1.jpg";
        bool check = imwrite(imgName1, cvImage1);
        // detect object of first image
        const char* detectorArguments = "../images/image1.jpg"; 
        vector<Object> ImgVector1 = call_image_recognition_routine(detectorArguments);

        // getting second image
        zed.grab();
        if (zed.grab() == sl::ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image2, sl::VIEW_LEFT);
            
            // Convert sl::Mat to cv::Mat (share buffer)
            cv::Mat cvImage2 = cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, image2.getPtr<sl::uchar1>(sl::MEM::CPU));
            imgName2 = "../images/image2.jpg";
            bool check = imwrite(imgName2, cvImage2);
            // detect object of first image
            const char* detectorArguments = "../images/image2.jpg"; 
            vector<Object> ImgVector2 = call_image_recognition_routine(detectorArguments);

            retVet = distance(ImgVector1, ImgVector2);
        } else {
            cout << "Error grabbing images" << endl;
            zed.close();
            return -1;
        }
    } else {
        cout << "Error grabbing images" << endl;
        zed.close();
        return -1;
    }
    return vetRet
}
