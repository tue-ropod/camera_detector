
#include<camera_detector/detections.h>
#include <visualization_msgs/MarkerArray.h>
#include <camera_detector/math_functions.h>


class detection_processing_sim
{
public:

void toXYZ(point &p, double thetaH, double thetaV, double dist);
void linkObjects(camera_detector::detections detectionData, vector<measurement> &measurements, vector<visitedTemp> &visited, int N, int M, ed_gui_server::objsPosVel &objsInfo);
bool validateSize(double dist);
bool validateHeight(double zKnee, double zAnkle);
double distContinuous(camera_detector::detection &m, cv::Mat &rgbd2);
void depthMeasurement(int indx,int indy, cv::Mat &rgbd2, double &dist);
void hypothesize(gates &G,int ind,double p,hypotheses hypothesesTemp,int gate);
void createValidationGates(vector<measurement> measurements, gates &G, vector<visitedTemp> visited);
void display(const std::shared_ptr<std::vector<std::shared_ptr<op::Datum>>>& datumsPtr);
camera_detector::detection xyzPoints(const auto poseKeypoints,auto person,vector<int> D,cv::Mat &rgbd2);
void initializeLegs(visualization_msgs::MarkerArray& cameraData, camera_detector::detections &detectionData);
void interpolatePoints(camera_detector::detection &m, double dalpha);
//bool validateUniqueness(camera_detector::detection m,double rLeg, double zCamera, cv::Mat &rgbd2);
void processGates(camera_detector::detections &detectionData, gates &G);
void processKeypoints(visualization_msgs::MarkerArray& cameraData,camera_detector::detections &postestvec,ed_gui_server::objsPosVel &objsInfo);
double Euclidean(double Px,double Py,double Mx,double My);
void print2DVec(vector<vector<double>> &mat);
};
