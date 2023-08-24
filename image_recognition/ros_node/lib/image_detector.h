/**
 * @file image_detector.h
 * class to dectect image with zedcam and recognize objects in images
 */

#ifndef __IMAGE_DETECTOR_H__
#define __IMAGE_DETECTOR_H__

#include <iostream>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <json/json.h>
#include <vector>

// for zed cam
#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

// to comunicate with image recognition program
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include "object.h"

using namespace std;

class image_detector{
    private:
        //inizializing zed camera:
        imgNumber = 25;
        sleepDuration = 10;
        // Create a ZED Camera object
        sl::Camera zed;

        sl::InitParameters init_parameters;
        init_parameters.sdk_verbose = true;
        init_parameters.camera_resolution = sl::RESOLUTION::HD720;
        init_parameters.depth_mode = sl::DEPTH_MODE::NONE;

        vector<Object> distance(vector<Object> v1, vector<Object> v2);
        vector<Object> call_image_recognition_routine(const char* arguments);
    public:
        image_detector();
        ~image_detector();
        vector<Object> image_recognition();
};

#endif
