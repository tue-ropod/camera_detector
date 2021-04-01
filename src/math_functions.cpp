// Min cost bipartite matching via shortest augmenting paths
//
// This is an O(n^3) implementation of a shortest augmenting path
// algorithm for finding min cost perfect matchings in dense
// graphs.  In practice, it solves 1000x1000 problems in around 1
// second.
//
//   cost[i][j] = cost for pairing left node i with right node j
//   Lmate[i] = index of right node that left node i pairs with
//   Rmate[j] = index of left node that right node j pairs with
//
// The values in cost[i][j] may be positive or negative.  To perform
// maximization, simply negate the cost[][] matrix.

using namespace std;
#include <vector>
#include <math.h>
#include <iostream>
using namespace std;
#include <camera_detector/math_functions.h>
#include "camera_detector/detections.h"
#include "ed_gui_server/objsPosVel.h"

double math_functions::Euclidean(double Px,double Py,double Mx,double My) {
    float r_turn = 0.05; // turnover point in centimeter cost funtion
    float c_p = 2; // scale parameter cost detection probability (default 2)
    Mx = Mx/100.0; // transform measurement from centimeters to meters
    My = My/100.0;
    double c = sqrt(pow((Mx-Px)/r_turn,2)+pow((My-Py)/r_turn,2));
    return c;
}

double math_functions::Mahalanobis(camera_detector::detection& detection,ed_gui_server::objPosVel& object) {
    double dz_x,dz_y,dz_r,S11,S22,S33;
    S11 = object.circle.xPosStdDev + R11;
    S22 = object.circle.yPosStdDev + R22;
    S33 = object.circle.radiusStdDev + R33;
    dz_x = detection.x-object.circle.pose.position.x;
    dz_y = detection.y-object.circle.pose.position.y;
    dz_r = detection.r-object.circle.radius;

    double d = sqrt(pow(dz_x,2.0)/S11+pow(dz_y,2.0)/S22+pow(dz_r,2.0)/S33);
    return d;
}


void math_functions::print2DVec(vector<vector<double>> &mat) {
    double Inf = HUGE_VAL;
    for (int i=0;i<mat.size();i++) {
        for (int j=0;j<mat[0].size();j++) {
            if (mat[i][j]==Inf||isnan(mat[i][j])) {
                cout<<"  "<<mat[i][j]<<" ";
            } else {
                printf("%.3f", mat[i][j]);
                cout<<" ";
            }

        }
        cout<<endl;
    }
}
