#include "../include/ZScoreAnomalyDetector.h"
#include "../include/AnomalyReport.h"

ZScoreAnomalyDetector::~ZScoreAnomalyDetector() {
    // TODO Auto-generated destructor stub
}

void ZScoreAnomalyDetector::learnNormal(const TimeSeries& ts) {

    //Calculating tx
    vector<string> attributes = ts.gettAttributes();
    //attributes.erase(attributes.begin());
    for(string s: attributes) {
        vector<float> col = ts.getAttributeData(s);
        thArr_.push_back(zScore(col, col.size() - 1));
    }
}

float ZScoreAnomalyDetector::zScore(vector<float> col, int lastIndex) {
    float th = 0;
    float z;
    vector <float> tmp;
    tmp.resize(col.size(), 0);
    //copy(col.begin(), col.end(), tmp.begin());
    for(int i = 0; i < col.size(); i++)
        tmp.at(i) = col[i];
    float *tmpArr = &tmp[0];
    float variance = (float) sqrt(var(tmpArr, tmp.size()));
    float  average = avg(tmpArr, tmp.size());
    for(int i = 0; i <= lastIndex; i++) {
        z = (abs(col[i] - average) / variance);
        if(th < z)
            th = z;
    }
    return th;
}

vector<AnomalyReport> ZScoreAnomalyDetector::detect(const TimeSeries& ts) {

    vector<AnomalyReport> detected;
    vector<string> attributes = ts.gettAttributes();
    //attributes.erase(attributes.begin());
    if(attributes.at(0)== "TimeStep") {
        attributes.erase(attributes.begin());
    }
    vector<float> curr;

    for(int i = 0; i < attributes.size(); i++) {
        curr = ts.getAttributeData(attributes.at(i));
        for(int j = 0; j < curr.size(); j++) {
            float z = zScore(curr, j);
            if(i < thArr_.size() && z > ZScoreAnomalyDetector::getThArr().at(i)) {
                std_msgs::String d;
                d.data = "sensor 1: " + ts.gettAttributes().at(i);
                //int timeStep = ts.getAttributeData(ts.gettAttributes().at(0)).at(j);
                float time = ts.getTimeSteps().at(j);
                detected.push_back(*(new AnomalyReport(d, time)));
            }
        }
    }
    return detected;
}