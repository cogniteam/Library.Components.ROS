#ifndef SIMPLEANOMALYDETECTOR_H_
#define SIMPLEANOMALYDETECTOR_H_

#include "anomaly_detection_util.h"
#include "AnomalyReport.h"
#include "timeseries.h"
#include <vector>
#include <algorithm>
#include <string.h>
#include <math.h>

using namespace std;

struct correlatedFeatures{
	string feature1,feature2;  // names of the correlated features
	float correlation;
	Line lin_reg;
	float threshold;
	float cx,cy;

    bool operator <(correlatedFeatures const & obj) {
        return((this->feature1.compare(obj.feature1)) && (this->feature2.compare(obj.feature2)) && (this->threshold < obj.threshold));
    }

    string hash() {
        return(this->feature1 + ","  + this->feature2 + ","  + to_string(this->correlation)  + ","  + to_string(this->threshold)  + ","   + to_string(this->cx) + ","
        + to_string(this->cy));
    }
};


class SimpleAnomalyDetector {
public:
	vector<correlatedFeatures> cf;
    vector<correlatedFeatures> correlatedOnlyForThemself;
	float threshold;
public:
	SimpleAnomalyDetector(float th): threshold(th){};
	virtual ~SimpleAnomalyDetector();

    void learnNormal(const TimeSeries& ts);
    vector<AnomalyReport> detect(const TimeSeries& ts);
	vector<correlatedFeatures> getNormalModel() {
		return cf;
	}
	void setCorrelationThreshold(float threshold) {
		this->threshold=threshold;
	}
    vector<correlatedFeatures> getCorrelatedOnlyForThemself() {
        return correlatedOnlyForThemself;
    }

public:
	virtual void learnHelper(const TimeSeries& ts,float p/*pearson*/,string f1, string f2,Point** ps);
	virtual bool isAnomalous(float x, float y,correlatedFeatures c);
	float findThreshold(Point** ps,size_t len,Line rl);
};



#endif /* SIMPLEANOMALYDETECTOR_H_ */
