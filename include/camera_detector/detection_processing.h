
#include<camera_detector/detections.h>
#include<camera_detector/config.h>



class detection_processing
{
public:

void toXYZ(point &p, double thetaH, double thetaV, double dist);
void depthMeasurement(int indx,int indy, cv::Mat &rgbd2, double &dist);
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr, double timeD);
camera_detector::detection xyzPoints(const auto poseKeypoints,auto person,vector<int> bodyParts,cv::Mat &rgbd2);
void process(const auto poseKeypoints, cv::Mat &rgbd2, camera_detector::detections &detectionData);
void processKeypoints(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr,cv::Mat &rgbd2,camera_detector::detections &detectionData);
};
