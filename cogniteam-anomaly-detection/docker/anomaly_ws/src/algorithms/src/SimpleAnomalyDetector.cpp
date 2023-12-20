#include "../include/SimpleAnomalyDetector.h"
#include "anomaly_detection_util.cpp"
#include "../include/AnomalyReport.h"
#include <math.h>

SimpleAnomalyDetector::~SimpleAnomalyDetector() {
	// TODO Auto-generated destructor stub
}

float SimpleAnomalyDetector::findThreshold(Point** ps,size_t len,Line rl){
	float max=0;
	for(size_t i=0;i<len;i++){
		float d=abs(ps[i]->y - rl.f(ps[i]->x));
		if(d>max)
			max=d;
	}
	return max;
}

void SimpleAnomalyDetector::learnNormal(const TimeSeries& ts) {
    //setCorrelationThreshold(0.9);
	vector<string> atts = ts.gettAttributes();
    //atts.erase(atts.begin());
	size_t len = ts.getRowSize();
	float vals[atts.size()][len];

	for(size_t i = 0; i < atts.size(); i++) {
		vector<float> x = ts.getAttributeData(atts[i]);
		for(size_t j = 0; j < len; j++) {
			vals[i][j]=x[j];
		}
	}

	for(size_t i = 0; i < atts.size(); i++) {
		string f1=atts[i];
		float max = 0;
		size_t maxPlace = -1;
		for(size_t j = i + 1; j < atts.size(); j++) {
			float p = abs(pearson(vals[i], vals[j], len));
			if(p > max) {
				max = p;
				maxPlace = j;
			}
		}
        if(maxPlace == -1) {
            correlatedFeatures c;
            c.feature1 = ts.gettAttributes().at(i);
            c.feature2 = ts.gettAttributes().at(i);
            c.correlation = 0;
            c.threshold = 0;
            correlatedOnlyForThemself.push_back(c);
        }
        else {
            string f2 = atts[maxPlace];
            Point** ps = toPoints(ts.getAttributeData(f1),ts.getAttributeData(f2));

            //learnHelper(ts,max,f1,f2,ps);
            if(max > threshold) {
                size_t len = ts.getRowSize();
                correlatedFeatures c;
                c.feature1 = f1;
                c.feature2 = f2;
                c.correlation = max;
                c.lin_reg = linear_reg(ps,len);
                c.threshold = findThreshold(ps,len,c.lin_reg) *1.1; // 10% increase
                cf.push_back(c);
            }
            // delete points
            for(size_t k=0;k<len;k++)
                delete ps[k];
            delete[] ps;
        }
	}
}

void SimpleAnomalyDetector::learnHelper(const TimeSeries& ts,float p/*pearson*/,string f1, string f2,Point** ps) {
	if(p > threshold) {
		size_t len = ts.getRowSize();
		correlatedFeatures c;
		c.feature1 = f1;
		c.feature2 = f2;
		c.correlation = p;
		c.lin_reg = linear_reg(ps,len);
		c.threshold = findThreshold(ps,len,c.lin_reg) * 1.1; // 10% increase
		cf.push_back(c);
	}
}

vector<AnomalyReport> SimpleAnomalyDetector::detect(const TimeSeries& ts) {
	vector<AnomalyReport> detected;
	/*for_each(cf.begin(),cf.end(),[&detected, &ts, this](correlatedFeatures c) {
		vector<float> x = ts.getAttributeData(c.feature1);
		vector<float> y = ts.getAttributeData(c.feature2);
		for(size_t i = 0; i < x.size(); i++) {
			if(isAnomalous(x[i], y[i], c)) {
				string d = c.feature1 + "-" + c.feature2;
				detected.push_back(AnomalyReport(d,(i+1)));
			}
		}
	});*/
    Point *p;
    for(correlatedFeatures f:cf) {
        vector<float> x = ts.getAttributeData(f.feature1);
        vector<float> y = ts.getAttributeData(f.feature2);
        for (size_t i = 0; i < x.size(); i++) {
            if (isAnomalous(x[i], y[i], f)) {
                std_msgs::String d;
                d.data = "sensor 1: " + f.feature1 + " " + "sensor 2: " + f.feature2;
                float time = ts.getTimeSteps().at(i);
                detected.push_back(*(new AnomalyReport(d, time)));
            }
        }
    }
    return detected;
}

bool SimpleAnomalyDetector::isAnomalous(float x, float y,correlatedFeatures c) {
	return (abs(y - c.lin_reg.f(x))>c.threshold);
}
