#include <iostream>
#include "camera_detector/detections.h"

using namespace std;



struct measurement {
    vector<int> o; // possible associated object number
    vector<double> d; // Mahalanobis distance from measurement to object
    vector<double> p; //association probability
    int m; // measurement label
};

struct hypotheses {
    vector<int> measurement; // labels with measurements
    vector<int> object; // labels with possibly associated objects
    double probability; // probability of hypothesis
};

struct gate {
    vector<hypotheses> H;
    vector<measurement> m;
};

struct gates {
    vector<gate> g_i;
};





void hypothesize2(gates &G,int ind,double p,hypotheses hypothesesTemp) {
    int N = G.g_i[0].m[ind].o.size();
    bool ignore;

    for (int i=0;i<N;i++) {
        ignore=false;
        for (int iCheck=0;iCheck<hypothesesTemp.object.size();iCheck++) {
            if (G.g_i[0].m[ind].o[i]==hypothesesTemp.object[iCheck]) {
                ignore=true;
            }
        }
        if (ignore==false) {
            cout<<"i: "<<i<<endl;
            if (ind!=0) {
                hypothesesTemp.measurement.push_back(G.g_i[0].m[ind].m);
                hypothesesTemp.object.push_back(G.g_i[0].m[ind].o[i]);
                hypothesize2(G,ind-1,p*G.g_i[0].m[ind].p[i],hypothesesTemp);
                hypothesesTemp.measurement.pop_back();
                hypothesesTemp.object.pop_back();

            } else {
                hypothesesTemp.probability=p*G.g_i[0].m[ind].p[i];
                hypothesesTemp.measurement.push_back(G.g_i[0].m[ind].m);
                hypothesesTemp.object.push_back(G.g_i[0].m[ind].o[i]);
                G.g_i[0].H.push_back(hypothesesTemp);
                hypothesesTemp.measurement.pop_back();
                hypothesesTemp.object.pop_back();
            }
        }

    }


}


int main () {
    gate gateTemp;
    vector<int> measurementsTemp;
    vector<int> objectsTemp;
    measurement measurementTemp;
    hypotheses hypothesesTemp;
    measurementTemp.o.push_back(0);
//    measurementTemp.o.push_back(1);
//    measurementTemp.o.push_back(2);
    measurementTemp.p.push_back(0.2);
//    measurementTemp.p.push_back(0.4);
//    measurementTemp.p.push_back(0.3);
    measurementTemp.m=3;
    gateTemp.m.push_back(measurementTemp);
    measurementTemp.o.clear();
    measurementTemp.p.clear();
    measurementTemp.o.push_back(1);
//    measurementTemp.o.push_back(3);
    measurementTemp.p.push_back(0.4);
//    measurementTemp.p.push_back(0.5);
    measurementTemp.m=1;
    gateTemp.m.push_back(measurementTemp);
//    measurementTemp.o.clear();
//    measurementTemp.p.clear();
//    measurementTemp.o.push_back(0);
//    measurementTemp.p.push_back(1);
//    measurementTemp.o.push_back(1);
//    measurementTemp.p.push_back(0.8);
//    measurementTemp.m=3;
//    gateTemp.m.push_back(measurementTemp);
//    measurementTemp.o.clear();
//    measurementTemp.p.clear();
//    measurementTemp.o.push_back(0);
//    measurementTemp.o.push_back(3);
//    measurementTemp.o.push_back(4);
//    measurementTemp.p.push_back(0.5);
//    measurementTemp.p.push_back(0.4);
//    measurementTemp.p.push_back(0.3);
//    measurementTemp.m=4;
//    gateTemp.m.push_back(measurementTemp);
    gates G;
    G.g_i.push_back(gateTemp);
    hypothesize2(G,1,1,hypothesesTemp);//measurementsTemp,objectsTemp);
    cout<<"print results"<<endl;
    for (int i=0;i<G.g_i[0].H.size();i++) {
        cout<<G.g_i[0].H[i].probability<<" ";
        for (int j=0;j<G.g_i[0].H[i].measurement.size();j++) {
            cout<<G.g_i[0].H[i].object[j]<<" ";
        }
        cout<<endl;
    }
    cout<<endl;

    double maxP = 0.0;
    int indMax=-1;
    for (int i=0;i<G.g_i[0].H.size();i++) {
        if (maxP<G.g_i[0].H[i].probability) {
            maxP = G.g_i[0].H[i].probability;
            indMax = i;
        }
    }
    cout<<G.g_i[0].H[indMax].probability<<endl;

    for (int i=0;i<G.g_i[0].H[indMax].measurement.size();i++) {
        cout<<"Measurement: "<< G.g_i[0].H[indMax].measurement[i] <<" associated to object: "<< G.g_i[0].H[indMax].object[i] <<endl;
    }




    return 0;
}
