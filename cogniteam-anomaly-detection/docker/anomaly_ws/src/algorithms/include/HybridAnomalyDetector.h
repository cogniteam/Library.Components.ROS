#include "anomaly_detection_util.h"
#include "AnomalyReport.h"
#include "timeseries.h"
#include "SimpleAnomalyDetector.h"
#include "ZScoreAnomalyDetector.h"
#include "minCircle.h"
#include <vector>
#include <map>
#include <set>
#include <unordered_set>
#include <algorithm>
#include <string.h>
#include <math.h>
#include <unordered_map>

#ifndef HYBRIDANOMALYDETECTOR_H_
#define HYBRIDANOMALYDETECTOR_H_

#include "minCircle.h"

using namespace std;

class HybridAnomalyDetector {
public:
    HybridAnomalyDetector();
    HybridAnomalyDetector(float th, float lowerBound, float upperBound);

    map<string, vector<correlatedFeatures>> featuresToAlgorithm;
    SimpleAnomalyDetector* regressionDetector;
    ZScoreAnomalyDetector* zScoreDetector;
    map<string,Circle> welzlCircleModel;//storing circles for each pair of correlated features

    float upperBound_;
    float lowerBound_;

    void learnNormal (TimeSeries& ts);
    vector<AnomalyReport> detect(const TimeSeries& ts);


    virtual ~HybridAnomalyDetector();
};

#endif /* HYBRIDANOMALYDETECTOR_H_ */
