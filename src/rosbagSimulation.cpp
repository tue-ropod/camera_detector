using namespace std;

/// [headers]

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include <visualization_msgs/MarkerArray.h>
#include <camera_detector/associationsWouter.h>
#include <camera_detector/persons.h>
#include "ed_gui_server/objsPosVel.h"
#include "tf/transform_listener.h"
#include <tf2/LinearMath/Quaternion.h>




/// [headers]


void createLineAssociations(ed_gui_server::objsPosVel& objsInfo,int i, int j, visualization_msgs::Marker &marker, camera_detector::personAssociation pA) {
    double dx,dy,dz,xL,yL,zL,xR,yR,zR, dist;
    double t3,t2,t1;

    xL = objsInfo.objects[pA.leftLegObject].circle.pose.position.x;
    yL = objsInfo.objects[pA.leftLegObject].circle.pose.position.y;
    zL = 0.0;
    xR = objsInfo.objects[pA.rightLegObject].circle.pose.position.x;
    yR = objsInfo.objects[pA.rightLegObject].circle.pose.position.y;
    zR = 0.0;

    dx = xL-xR;
    dy = yL-yR;
    dz = zL-zR;

    dist = sqrt(pow(dx,2.0)+pow(dy,2.0)+pow(dz,2.0));

    t1 = atan2(-dy,dx); //atan2(dy,dz)
    t2 = M_PI/2.0;
    t3 = 0.0;
    cout<<atan2(dy,dx)<<endl;
    tf2::Quaternion angleQ;
    angleQ.setEuler(t2,t1,t3);
    marker.header.stamp = ros::Time();
    marker.id = 100*i+j;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.pose.position.x = (xL+xR)/2.0;
    marker.pose.position.y = (yL+yR)/2.0;
    marker.pose.position.z = (zL+zR)/2.0+0.42;

    marker.pose.orientation.x = angleQ.x();
    marker.pose.orientation.y = angleQ.y();
    marker.pose.orientation.z = angleQ.z();
    marker.pose.orientation.w = angleQ.w();
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = dist;
    marker.color.a = pA.associationProbability;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}



camera_detector::persons personsStore;

void storeData(const camera_detector::persons& persons)
{
    personsStore=persons;
}
ed_gui_server::objsPosVel objsInfo_store; // create global object to store lrf info

void showData1(const ed_gui_server::objsPosVel& objsInfo)
{
    objsInfo_store=objsInfo;
}

/// [main]
int main(int argc, char *argv[])
/// [main]
{
    // Start ROS listener
    ros::Publisher objAssociations_pub;
    ros::Subscriber sub2;
    ros::Subscriber sub1;

    cout<<"Starting ROS listener..."<<endl;
    ros::init(argc, argv, "associationTransformer");
    ros::NodeHandle n;
    sub1 = n.subscribe("/ropod_tue_2/ed/gui/objectPosVel", 3, showData1);
    sub2 = n.subscribe("/Jetson/persons", 3, storeData);
    visualization_msgs::MarkerArray markerArrayA;
    visualization_msgs::Marker markerA;
    markerA.ns = "LRFAssociatinos";
    markerA.header.frame_id = "/map";
    markerA.action = visualization_msgs::Marker::ADD;
    objAssociations_pub = n.advertise<visualization_msgs::MarkerArray>("/Jetson/LRFAssociationsFast",3);


//    string leftLegObjectID;
//    string rightLegObjectID;
    double N;
//    double maxP;
//    camera_detector::associationWouter associationTemp;
//    camera_detector::associationsWouter associationsWouter;
    /// [loop start]
    while(true)
    {
        cout<<"check"<<endl;
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1)); // Call ROS stream with lrf detections and let it sleep max x seconds until skipping
        N = markerArrayA.markers.size();
        markerArrayA.markers.clear();

        visualization_msgs::MarkerArray deleteAllMarkerArray;
        visualization_msgs::Marker deleteAllMarker;
        deleteAllMarker.action = visualization_msgs::Marker::DELETEALL;
        for (int i=0;i<N;i++){
            deleteAllMarkerArray.markers.push_back(deleteAllMarker);
        }

        objAssociations_pub.publish (deleteAllMarkerArray);


        for (int i=0;i<personsStore.persons.size();i++) {
            for (int j=0;j<personsStore.persons[i].p.size();j++) {
                createLineAssociations(objsInfo_store, i, j, markerA,personsStore.persons[i].p[j]);
                markerArrayA.markers.push_back(markerA);
            }
        }

        objAssociations_pub.publish (markerArrayA);

//        cv::imshow("esc",cv::Mat(1,1,CV_32FC1,1));



//        associationsWouter.humans.clear();
//        for (int i=0;i<personsStore.persons.size();i++) {
//            cout<<"person ID: "<<i<<endl;
//            maxP = 0.0;
//            for (int j=0;j<personsStore.persons[i].p.size();j++) {
//                cout<<"amount of checks: "<<j<<"with p:" <<maxP <<endl;
//                if (maxP<personsStore.persons[i].p[j].associationProbability) {
//                    cout<<"new maxP"<<endl;
//                    maxP = personsStore.persons[i].p[j].associationProbability;
//                    cout<<"DEBUG"<< maxP << " "<<personsStore.persons[i].p[j].associationProbability<<endl;
//                    associationTemp.p = maxP;
//                    associationTemp.leftLegObjectID = personsStore.persons[i].p[j].leftLegObjectID;
//                    associationTemp.rightLegObjectID = personsStore.persons[i].p[j].rightLegObjectID;
//                }
//            }
//            cout<<"current maxP"<<maxP<<endl;
//            if (maxP>0.0) {
//                cout<<"newHuman"<<endl;
//                associationsWouter.humans.push_back(associationTemp);
//            }
//            associationsWouter_pub.publish(associationsWouter);
//        }
    }
    /// [loop end]

    return 0;
}

