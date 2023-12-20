#ifndef ALGORITHMS_ZSCOREANOMALYDETECTOR_H
#define ALGORITHMS_ZSCOREANOMALYDETECTOR_H

#include "anomaly_detection_util.h"
#include "AnomalyReport.h"
#include "timeseries.h"
#include "std_msgs/String.h"
#include "anomaly_detection_util.h"
#include <vector>
#include <algorithm>
#include <string.h>
#include <math.h>


class ZScoreAnomalyDetector {

private:
    vector<float> thArr_;
public:
    ZScoreAnomalyDetector() {};
    virtual ~ZScoreAnomalyDetector();
public:
    void learnNormal (const TimeSeries& ts);
    vector<AnomalyReport> detect(const TimeSeries& ts);
    float zScore(vector<float> col, int lastIndex);
    vector<float> getThArr() {
        return this->thArr_;
    }
    void setThArr(vector<float> v) {
        this->thArr_.clear();
        for(float f: v) {
            this->thArr_.push_back(f);
        }
    }
};


#endif //ALGORITHMS_ZSCOREANOMALYDETECTOR_H
