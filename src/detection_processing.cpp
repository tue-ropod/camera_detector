using namespace std;

/// [headers]

// ROS
//#include <ros/ros.h>
//#include "camera_detector/objPosVel.h"
//#include "camera_detector/detections.h"
#include "ed_gui_server/objsPosVel.h"

//#include<geometry_msgs/Point.msg>


// Openpose API (including opencv)
#include <openpose/headers.hpp>
#include <camera_detector/detection_processing.h>
#include <ctime>
#include <iostream>
#include <chrono>
#include <sys/time.h>
struct timeval timeC;
#include <sstream>

void detection_processing::display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr,double timeD)
{
    try
    {
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Display image
            cv::Mat flipImage;
            cv::flip(datumsPtr->at(0)->cvOutputData, flipImage,1);
//            cv::imshow(OPEN_POSE_NAME_AND_VERSION + " - detector on Kinect input", flipImage);
//            time_t current_time;
//            string name = "/media/nvidia/TOSHIBA/pictures/kinectDetection_"  + to_string(time(NULL)) + ".jpg";
//            gettimeofday(&timeC, NULL);
            std::stringstream stream;
            stream << std::fixed << std::setprecision(2) << timeD;
            string name = "/home/nvidia/pictures/kinectDetection_"  + stream.str() + ".jpg";
            cout<<name<<endl;
            cv::imwrite(name,flipImage);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

void detection_processing::depthMeasurement(int indx,int indy, cv::Mat &rgbd2, double &dist) {
    // calculate mean distance over 25 points
    double distItTemp;
    double countDistIt=0;
    double distIt=0.0;
    for (int ii=-2;ii<3;ii++) {
        for (int jj=-2;jj<3;jj++) {
            distItTemp = rgbd2.at<float>(indy+ii,indx+jj);
            if (distItTemp!=0 && !isnan(distItTemp)) { // check if measurement is valid
                distIt = distIt + distItTemp; // add measurement
                countDistIt = countDistIt+1.0; // count amount of valid measurements
            }
        }
    }
    dist = distIt/countDistIt/1000.0+0.5*rBody; // determine average distance (in meter) AND compensate for measuring front instead of inside leg
}


camera_detector::detection detection_processing::xyzPoints(const auto poseKeypoints,auto person,vector<int> bodyParts,cv::Mat &rgbd2) { // measure depth of object
    double dist,thetaH,thetaV,x,y,z,p;
    int indx,indy,bodyPart;
    camera_detector::detection m;
    int count=0;
    x=y=p=0.0;
    m.x=m.y=m.p=0.0;
    point pTemp;
    for (int i=0; i<bodyParts.size(); i++) {
        bodyPart = bodyParts[i];
        indx = round(poseKeypoints[vector<int> {person, bodyPart, 0}])-1; // look for index in depth image from rgb detection
        if (indx>0) { // for all detected points
            indy = round(poseKeypoints[vector<int> {person, bodyPart, 1}])-1; // look for index in depth image from rgb detection
            // calculate mean distance over 25 points
            depthMeasurement(indx,indy,rgbd2,dist);
            if (dist>0 && dist<20) { // only use points with distance between zero and 20 meter
                count++;
                thetaH = (indx/resH-0.5)*thetaHKinect/180.0*M_PI;
                thetaV = -(indy/resV-0.5)*thetaVKinect/180.0*M_PI;
                toXYZ(pTemp, thetaH, thetaV, dist);
                x = x+pTemp.x;
                y = y+pTemp.y;
                z = z+pTemp.z;
                p = p+poseKeypoints[vector<int> {person, bodyPart, 2}];

            }
        }
    }
    m.p = p/double(count);
    m.x = x/double(count);
    m.y = y/double(count);
    m.z = z/double(count);
    return m;
}

void detection_processing::process(const auto poseKeypoints, cv::Mat &rgbd2, camera_detector::detections &detectionData) {
    camera_detector::detection m;
    const vector<int> all = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24}; // vector with all bodyparts
    for (int person = 0 ; person < poseKeypoints.getSize(0); person++)
    {
        m = xyzPoints(poseKeypoints,person,all,rgbd2);
        if (m.x>-20 && m.x<20 && m.y>-20 && m.y<20 && m.z>-20 && m.z<20) {
            detectionData.detections.push_back(m);
        }
    }
}

void detection_processing::toXYZ(point &p, double thetaH, double thetaV, double dist) {
    p.x = cos(thetaH)*cos(thetaV)*dist;
    p.y = sin(thetaH)*cos(thetaV)*dist;
    p.z = sin(thetaV)*dist;
}

void detection_processing::processKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr,cv::Mat &rgbd2,camera_detector::detections &detectionData)
{
    try
    {
        if (datumsPtr != nullptr && !datumsPtr->empty())
        {
            // Accesing each element of the keypoints
            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            process(poseKeypoints,rgbd2,detectionData);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High);

    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

