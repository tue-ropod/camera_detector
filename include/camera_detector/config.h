using namespace std;
#include<vector>
struct measurement {
    vector<int> o; // possible associated object number
    vector<double> d; // Mahalanobis distance from measurement to object
    vector<double> p; //association probability
    int m; // measurement label
};

//struct hypotheses {
//    vector<int> measurement; // labels with measurements
//    vector<int> object; // labels with possibly associated objects
//    double probability; // probability of hypothesis
//};

//struct gate {
//    vector<hypotheses> H;
//    vector<measurement> m;
//};

//struct gates {
//    vector<gate> g_i;
//};

//struct visitedTemp {
//    vector<int> m;
//};

struct point {
    double x;
    double y;
    double z;
};

//const double R11 = 0.010; // std x
//const double R22 = 0.010; // std y
//const double R33 = 0.1; // std r
//const double pFP = 0.05;
const double rBody = 0.3;
const double resH = 480.0;
const double resV = 270.0;
const double thetaHKinect = 84.1;
const double thetaVKinect = 53.8;
//const double zLRF = 0.42;
//const double zCamera = 0.26;
//const double zCamera = 0.42;
//const double xCamera = 0.09;
//const double chiSquared = 7.8147;
//const string IDMeasurement = "1";
