using namespace std;

/// [headers]

// ROS
//#include <ros/ros.h>
//#include "camera_detector/objPosVel.h"
//#include "camera_detector/detections.h"
#include "ed_gui_server/objsPosVel.h"


// Openpose API (including opencv)
#include <openpose/headers.hpp>
#include <camera_detector/detection_processing_sim.h>

void validateUniqueness(camera_detector::detections &detectionData) {
    double dxKnee,dyKnee,dzKnee,dKnee;
    double dxAnkle,dyAnkle,dzAnkle,dAnkle;
    for (int i=0;i<detectionData.detections.size();i++) {
        if (detectionData.detections[i].validUniqueness) {
            if (detectionData.detections[i].validHeight&&detectionData.detections[i].validSize) {
                for (int j=i+1;j<detectionData.detections.size();j++) {
                    if (detectionData.detections[j].distContinuous && detectionData.detections[i].distContinuous){
                        if (detectionData.detections[j].validHeight&&detectionData.detections[j].validSize) {
                            dxKnee = detectionData.detections[i].xKnee-detectionData.detections[j].xKnee; // legs and knees different!
                            dyKnee = detectionData.detections[i].yKnee-detectionData.detections[j].yKnee;
                            dzKnee = detectionData.detections[i].zKnee-detectionData.detections[j].zKnee;
                            dKnee = sqrt(pow(dxKnee,2.0)+pow(dyKnee,2.0)+pow(dzKnee,2.0));
                            dxAnkle = detectionData.detections[i].xAnkle-detectionData.detections[j].xAnkle; // legs and knees different!
                            dyAnkle = detectionData.detections[i].yAnkle-detectionData.detections[j].yAnkle;
                            dzAnkle = detectionData.detections[i].zAnkle-detectionData.detections[j].zAnkle;
                            dAnkle = sqrt(pow(dxAnkle,2.0)+pow(dyAnkle,2.0)+pow(dzAnkle,2.0));
                            if (dKnee<rLeg&&dAnkle<rLeg) {
                                cout<<"double leg detection"<<endl;
                                if (detectionData.detections[i].p>detectionData.detections[j].p) {
                                    detectionData.detections[j].validUniqueness=false;
                                } else {
                                    detectionData.detections[i].validUniqueness=false;
                                }
                            } else if(dKnee<rLeg||dAnkle<rLeg) {
                                cout<<"double knee detection"<<endl;
                                if (detectionData.detections[i].distContinuous>detectionData.detections[j].distContinuous) {
                                    detectionData.detections[i].validUniqueness = false;
                                } else {
                                    detectionData.detections[j].validUniqueness = false;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void detection_processing_sim::hypothesize(gates &G,int ind,double p,hypotheses hypothesesTemp,int gate) {
    int N = G.g_i[gate].m[ind].o.size();
    bool ignore;
    for (int i=0;i<N;i++) {
        ignore=false;
        for (int iCheck=0;iCheck<hypothesesTemp.object.size();iCheck++) {

            if (G.g_i[gate].m[ind].o[i]!=-1) {
                if (G.g_i[gate].m[ind].o[i]==hypothesesTemp.object[iCheck]) {
                    ignore=true;
                }
            }
        }
        if (ignore==false) {
            if (ind!=0) {
                hypothesesTemp.measurement.push_back(G.g_i[gate].m[ind].m);
                hypothesesTemp.object.push_back(G.g_i[gate].m[ind].o[i]);
                hypothesize(G,ind-1,p*G.g_i[gate].m[ind].p[i],hypothesesTemp,gate);
                hypothesesTemp.measurement.pop_back();
                hypothesesTemp.object.pop_back();

            } else {
                hypothesesTemp.probability=p*G.g_i[gate].m[ind].p[i];
                //                cout<<"probability hypothesis: "<<hypothesesTemp.probability<<endl;
                hypothesesTemp.measurement.push_back(G.g_i[gate].m[ind].m);
                hypothesesTemp.object.push_back(G.g_i[gate].m[ind].o[i]);
                G.g_i[gate].H.push_back(hypothesesTemp);
                hypothesesTemp.measurement.pop_back();
                hypothesesTemp.object.pop_back();
            }
        }

    }


}

void detection_processing_sim::createValidationGates(vector<measurement> measurements, gates &G, vector<visitedTemp> visited) {
    vector<int> toVisitM;
    vector<int> toVisitO;
    vector<int> toVisitMInd;

    gate gateTemp;
    int visitCount=0;
    int countVM=0;
    int countVO=0;
    int A = measurements.size();
    if (A>0) {

        int maxCount=0;
        for (int i=0;i<A;i++) { // loop over measurements
            maxCount = maxCount + measurements[i].o.size();
        }

        while (visitCount<maxCount) {
            while (countVO<toVisitO.size()) {
                for (int i=0;i<A;i++) { // loop over measurements
                    for (int j=0;j<measurements[i].o.size();j++) { // check all possible association objects in a measurement
                        if (!visited[i].m[j]) { // check if measurement is not visited
                            if (measurements[i].o[j]==toVisitO[countVO]) { // check if measurement is connected to currently analyzed object
                                toVisitM.push_back(measurements[i].m); // add the measurement to the visit list
                                toVisitMInd.push_back(i);
                                visited[i].m[j]=true; // add a visited label to this measurement
                                visitCount++;
                            }
                        }
                    }
                }

                countVO++;
                countVM=0;
                while (countVM<toVisitM.size()) {
                    for (int i=0;i<A;i++) { // loop over points
                        for (int j=0;j<measurements[i].o.size();j++) {
                            if (!visited[i].m[j]) {
                                if (measurements[i].m==toVisitM[countVM]) {
                                    toVisitO.push_back(measurements[i].o[j]);
                                    visited[i].m[j]=true;
                                    visitCount++;
                                }
                            }
                        }
                    }
                    countVM++;
                }
            }

            countVO=0;
            if (toVisitM.size()>0) {
                for (int i=0;i<toVisitM.size();i++) {
                    gateTemp.m.push_back(measurements[toVisitMInd[i]]);
                }
                G.g_i.push_back(gateTemp);
                gateTemp.m.clear();
            }

            toVisitM.clear();
            toVisitMInd.clear();
            toVisitO.clear();
            for (int i=0;i<A;i++) {
                for (int j=0;j<measurements[i].o.size();j++) {
                    if (!visited[i].m[j]&&toVisitO.size()==0) {
                        toVisitO.push_back(measurements[i].o[j]);
                    }
                }
            }
        }
    } else {
        cout<<"INFO: no measurements in any gate"<<endl;
    }
}

void detection_processing_sim::depthMeasurement(int indx,int indy, cv::Mat &rgbd2, double &dist) {
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
    dist = distIt/countDistIt/1000.0+0.5*rLeg; // determine average distance (in meter) AND compensate for measuring front instead of inside leg
}

bool detection_processing_sim::validateSize(double dist) {

    if ((dist<0.6)&&(dist>0.2)) {
        return true;
    }
    return false;
}

bool detection_processing_sim::validateHeight(double zKnee, double zAnkle) {

    if (zKnee>0.0 && zAnkle<0.0) {
        return true;
    }
    return false;
}

double detection_processing_sim::distContinuous(camera_detector::detection &m, cv::Mat &rgbd2) {
    double distContinuous = 0.0;
    double xC,yC,zC,dC,distC, thetaHC, thetaVC;
    int ix,iy;
    point pC;
    int count=0;
    for (int i=1;i<m.xArray.size()-1;i++) {
        xC = m.xArray[i];
        yC = m.yArray[i];
        zC = m.zArray[i];
        thetaHC = atan2(yC,xC);
        thetaVC = atan2(zC,sqrt(pow(xC,2.0)+pow(yC,2.0)));
        ix = (thetaHC*180.0/(thetaHKinect*M_PI)+0.5)*resH;
        iy = (-thetaVC*180.0/(thetaVKinect*M_PI)+0.5)*resV;
        dC = sqrt(pow(xC,2.0)+pow(yC,2.0)+pow(zC,2.0));
        depthMeasurement(ix,iy,rgbd2,distC);
        if (distC>0 && distC<20) {
            distContinuous = distContinuous + fabs(distC-dC);
            toXYZ(pC,thetaHC,thetaVC,distC);
            m.xValArray.push_back(pC.x);
            m.yValArray.push_back(pC.y);
            m.zValArray.push_back(pC.z);
            count++;
        }
    }
    distContinuous = distContinuous/(double)count;
    if (distContinuous == 0.0) { // if all measurements are neglected, initiate a high value
        distContinuous = 5.0;
    }
    return distContinuous;
}

void detection_processing_sim::interpolatePoints(camera_detector::detection &m, double dalpha) {
    m.xArray.push_back(m.xAnkle);
    m.yArray.push_back(m.yAnkle);
    m.zArray.push_back(m.zAnkle);
    for (double alpha=dalpha;alpha<1.0;alpha=alpha+dalpha) {
        //        cout<<"value of alpha: "<<alpha<<endl;
        m.xArray.push_back(alpha*(m.xAnkle-m.xKnee)+m.xKnee);
        m.yArray.push_back(alpha*(m.yAnkle-m.yKnee)+m.yKnee);
        m.zArray.push_back(alpha*(m.zAnkle-m.zKnee)+m.zKnee);
    }
    m.xArray.push_back(m.xKnee);
    m.yArray.push_back(m.yKnee);
    m.zArray.push_back(m.zKnee);
}

void detection_processing_sim::toXYZ(point &p, double thetaH, double thetaV, double dist) {
    p.x = cos(thetaH)*cos(thetaV)*dist;
    p.y = sin(thetaH)*cos(thetaV)*dist;
    p.z = sin(thetaV)*dist;
}

camera_detector::detection detection_processing_sim::xyzPoints(visualization_msgs::MarkerArray& cameraData) { // measure depth of object
    double dist,thetaH,thetaV,x,y,p,alpha;
    int indx,indy,bodyPart;
    camera_detector::detection m;
    int count=0;
    x=y=p=0.0;
    m.x=m.y=m.p=0.0;
    point pTemp;
            // calculate mean distance over 25 points
                    m.xKnee = cameraData[index].points[4].x;
                    m.yKnee = cameraData[index].points[4].y;
                    m.zKnee = cameraData[index].points[4].z;
                    m.xAnkle = cameraData[index].points[0].x;
                    m.yAnkle = cameraData[index].points[0].y;
                    m.zAnkle = cameraData[index].points[0].z;
                p = p+poseKeypoints[vector<int> {person, bodyPart, 2}];

        alpha = (zLRF-zCamera-m.zKnee)/(m.zAnkle-m.zKnee);
        m.d = sqrt(pow(m.xAnkle-m.xKnee,2.0)+pow(m.yAnkle-m.yKnee,2.0)+pow(m.zAnkle-m.zKnee,2.0));
        m.x = alpha*(m.xAnkle-m.xKnee)+m.xKnee;
        m.y = alpha*(m.yAnkle-m.yKnee)+m.yKnee;
        m.z = alpha*(m.zAnkle-m.zKnee)+m.zKnee;
        interpolatePoints(m, 0.25);
        m.validSize = validateSize(m.d);
        m.validHeight = validateHeight(m.zKnee,m.zAnkle);
        m.distContinuous = distContinuous(m,rgbd2);
    m.p = p/double(count);
    m.r = rLeg; // constant estimate for size lower leg
    m.associated = false; // initialize no association
    m.validUniqueness=true; // initially the measurement is unique (checked later)
    return m;
}

void detection_processing_sim::initializeLegs(visualization_msgs::MarkerArray& cameraData, camera_detector::detections &detectionData) {
    camera_detector::detection pL,pR;
    const vector<int> L = {13,14}; // vector with bodyparts left lower leg
    const vector<int> R = {10,11}; // vector with bodyparts right lower leg
    for (int person = 0 ; person < poseKeypoints.getSize(0); person++)
    {
        pL = xyzPoints(poseKeypoints,person,L,rgbd2);
        pR = xyzPoints(poseKeypoints,person,R,rgbd2);
        if (pL.x!=0 && pL.y!=0 && !isnan(pL.x) && !isnan(pL.y)){ // detect valid left leg
            detectionData.detections.push_back(pL);
        }
        if (pR.y!=0 && pR.y!=0 && !isnan(pR.x) && !isnan(pR.y)) { // detect valid right leg
            detectionData.detections.push_back(pR);
        }
    }
}

void detection_processing_sim::linkObjects(camera_detector::detections detectionData, vector<measurement> &measurements, vector<visitedTemp> &visited,int N, int M, ed_gui_server::objsPosVel &objsInfo) {
    visitedTemp visitedTempM;
    measurement measurementTemp;
    double d, p;

    for(int i=0;i<M;i++) {
        if (detectionData.detections[i].validSize&&detectionData.detections[i].validHeight&&detectionData.detections[i].validUniqueness) {//&&detectionData.detections[i].validContinuous) {
            measurementTemp.m = i;
            for (int j=0;j<N;j++) {
                d = math_functions::Mahalanobis(detectionData.detections[i],objsInfo.objects[j]);
                if (pow(d,2.0)<chiSquared) {
                    p = exp(-pow(d,2.0)/2.0)*detectionData.detections[i].p*1.0;
                    visitedTempM.m.push_back(false);
                    measurementTemp.d.push_back(d);
                    measurementTemp.o.push_back(j);
                    measurementTemp.p.push_back(p);
                }
            }
            // add false positive
            visitedTempM.m.push_back(false);
            measurementTemp.o.push_back(-1);
            measurementTemp.p.push_back(0.1); // false positive probability
            measurementTemp.d.push_back(-1);
            // push in measurement array
            measurements.push_back(measurementTemp);
            visited.push_back(visitedTempM);
            measurementTemp.d.clear();
            measurementTemp.o.clear();
            measurementTemp.p.clear();
            visitedTempM.m.clear();
        }
    }


}

void detection_processing_sim::processGates(camera_detector::detections &detectionData, gates &G) {
    for (int i=0;i<G.g_i.size();i++) {
        hypotheses hypothesesTemp;
        hypothesize(G,G.g_i[i].m.size()-1,1.0,hypothesesTemp,i);
        double maxP = 0.0;
        double sumP = 0.0;
        int indMax=-1;
        for (int ii=0;ii<G.g_i[i].H.size();ii++) { // normalize probability in gate
            sumP = sumP+G.g_i[i].H[ii].probability;
            if (maxP<G.g_i[i].H[ii].probability) {
                maxP = G.g_i[i].H[ii].probability;
                indMax = ii;
            }
        }
        for (int ii=0;ii<G.g_i[i].H.size();ii++) {
            G.g_i[i].H[ii].probability=G.g_i[i].H[ii].probability/sumP;
        }

        if (indMax!=-1) {
            cout<<"Association certainty: "<<G.g_i[i].H[indMax].probability<<endl;
            for (int ii=0;ii<G.g_i[i].H[indMax].measurement.size();ii++) {
                if (G.g_i[i].H[indMax].object[ii]!=-1) {
                    cout<<"Measurement: "<< G.g_i[i].H[indMax].measurement[ii] <<" associated to object: "<< G.g_i[i].H[indMax].object[ii]<<endl;
                    detectionData.detections[G.g_i[i].H[indMax].measurement[ii]].associated=true;
                    detectionData.detections[G.g_i[i].H[indMax].measurement[ii]].associatedPoint=G.g_i[i].H[indMax].object[ii];
                }
            }
        }
    }
}

void detection_processing_sim::processKeypoints(visualization_msgs::MarkerArray& cameraData,camera_detector::detections &detectionData,ed_gui_server::objsPosVel &objsInfo)
{
    try
    {
        int N,M; // amount of measurements and points respectively

//        if (datumsPtr != nullptr && !datumsPtr->empty())
        if (1)
        {
            // Accesing each element of the keypoints
//            const auto& poseKeypoints = datumsPtr->at(0)->poseKeypoints;
            // transform keyPoints to lower legs
            initializeLegs(cameraData,detectionData);
            // validate uniqueness lower legs
            validateUniqueness(detectionData);

            M=detectionData.detections.size(); // Amount of measurements
            N = objsInfo.objects.size(); // amount of detected points
            cout<<"amount of measurements: "<<M<< ", amount of points: "<< N<<endl;
            gates G;
            vector<measurement> measurements;
            vector<visitedTemp> visited;
            linkObjects(detectionData,measurements,visited,N,M,objsInfo);
            createValidationGates(measurements,G,visited);
            processGates(detectionData,G);
        }
        else
            op::log("Nullptr or empty datumsPtr found.", op::Priority::High);

    }
    catch (const std::exception& e)
    {
        op::error(e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}

