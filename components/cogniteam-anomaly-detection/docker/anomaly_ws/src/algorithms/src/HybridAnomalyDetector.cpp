#include "../include/HybridAnomalyDetector.h"
#include "../include/AnomalyReport.h"
#include "../include/anomaly_detection_util.h"
#include "../include/minCircle.h"

HybridAnomalyDetector::HybridAnomalyDetector() {
	// TODO Auto-generated constructor stub
    regressionDetector = new SimpleAnomalyDetector(0.7);
    zScoreDetector = new ZScoreAnomalyDetector();
}

HybridAnomalyDetector::HybridAnomalyDetector(float th, float lowerBound, float upperBound) {
    // TODO Auto-generated constructor stub
    regressionDetector = new SimpleAnomalyDetector(th);
    this->lowerBound_ = lowerBound;
    this->upperBound_ = upperBound;
    zScoreDetector = new ZScoreAnomalyDetector();
}

HybridAnomalyDetector::~HybridAnomalyDetector() {
	 delete(this->regressionDetector);
     delete(this->zScoreDetector);
}

void HybridAnomalyDetector::learnNormal (TimeSeries& ts) {
    regressionDetector->learnNormal(ts);
    vector <correlatedFeatures> mostCorrelated = regressionDetector->getNormalModel();

    //vector<correlatedFeatures> ZScoreSet;
    //vector<correlatedFeatures> RegressionSet;
    //vector<correlatedFeatures> WelzlSet;

    /*--------------------sorting the features by they correlation value--------------*/
    featuresToAlgorithm.emplace("ZScore", vector<correlatedFeatures>());
    featuresToAlgorithm.emplace("Regression", vector<correlatedFeatures>());
    featuresToAlgorithm.emplace("Welzl", vector<correlatedFeatures>());
    vector<correlatedFeatures> onlyForThemselves = regressionDetector->getCorrelatedOnlyForThemself();
    featuresToAlgorithm.at("ZScore").insert(std::end(featuresToAlgorithm.at("ZScore")), std::begin(onlyForThemselves), std::end(onlyForThemselves));

    for (correlatedFeatures c: mostCorrelated) {
        if (abs(c.correlation) >= this->upperBound_)
            featuresToAlgorithm["Regression"].push_back(c);
        else if(abs(c.correlation) > this->lowerBound_ /*&& abs(c.correlation) < regressionDetector->threshold*/ ) {
            correlatedFeatures cf;
            Point** arrPoints = toPoints(ts.getAttributeData(c.feature1), ts.getAttributeData(c.feature2));
            Circle c1 = findMinCircle(arrPoints, ts.getRowSize());
            cf.feature1 = c.feature1;
            cf.feature2 = c.feature2;
            cf.correlation = c.correlation;
            cf.threshold = c1.radius*1.1; // 10% increase
            cf.cx=c1.center->x;
            cf.cy=c1.center->y;
            featuresToAlgorithm["Welzl"].push_back(cf);

        }
    }
    vector<correlatedFeatures> example = featuresToAlgorithm.at("Welzl");
    for(correlatedFeatures c: featuresToAlgorithm.at("Welzl")) {
        //Point** arrPoints = toPoints(ts.getAttributeData(c.feature1), ts.getAttributeData(c.feature2));
        Circle* c1 = new Circle(c.cx,c.cy,c.threshold);
        welzlCircleModel.emplace(c.hash(), *c1);
    }
    zScoreDetector->learnNormal(ts);
    regressionDetector = new SimpleAnomalyDetector((float) 0.95);
    regressionDetector->learnNormal(ts);
}

vector<AnomalyReport> HybridAnomalyDetector::detect(const TimeSeries& ts) {
    vector<AnomalyReport> detected;
    vector<AnomalyReport> regressionAr = regressionDetector->detect(ts);
    detected.insert(std::end(detected), std::begin(regressionAr), std::end(regressionAr));
    TimeSeries* testZScoreAlgorithm = new TimeSeries();
    testZScoreAlgorithm->setTimeSteps(ts.getTimeSteps());
    //-----------------build new TimeSeries-------------------
    for(correlatedFeatures c: featuresToAlgorithm.at("Welzl")) {
        int count = 0;
        float time = ts.getTimeSteps().at(count);
        vector<Point> dataFeature = getListPoint(ts.getAttributeData(c.feature1), ts.getAttributeData(c.feature2));
        for(Point p: dataFeature) {
            if(!welzlCircleModel.at(c.hash()).containsPoint(p)) {
                std_msgs::String d;
                d.data = "sensor 1: " + c.feature1 + " " + "sensor 2: " + c.feature2;
                detected.push_back(*(new AnomalyReport(d, time)));
            }
            /*map<string, Circle>::iterator it;
            for(it = welzlCircleModel.begin(); it != welzlCircleModel.end(); it++) {
                if((it->first. == c.feature1) && (it->first.feature2 == c.feature2) && (it->first.threshold == c.threshold) && (
                        static_cast<Line>(it->first.lin_reg) == c.lin_reg) && (it->first.correlation == c.correlation)) {
                    Circle circle = it->second;
                    if(!circle.containsPoint(p)) {
                        AnomalyReport* ar = new AnomalyReport(c.feature1 + "-" + c.feature2, count);
                        detected.push_back(*ar);
                    }
                }
            }*/
            ++count;
        }
    }
    for(correlatedFeatures c: featuresToAlgorithm.at("ZScore")) {
        testZScoreAlgorithm->addCol(c.feature1, ts.getAttributeData(c.feature1));
        testZScoreAlgorithm->addCol(c.feature2, ts.getAttributeData(c.feature2));
    }
    vector<AnomalyReport> resultOfZScore = zScoreDetector->detect(*testZScoreAlgorithm);
    detected.insert(std::end(detected), std::begin(resultOfZScore), std::end(resultOfZScore));
    return detected;
}

/*void HybridAnomalyDetector::learnHelper(const TimeSeries& ts,float pearson,string f1, string f2,Point** ps){
	SimpleAnomalyDetector::learnHelper(ts,p,f1,f2,ps);
	if(p > 0.5 && p < threshold) {
		Circle cl = findMinCircle(ps,ts.getRowSize());
		correlatedFeatures c;
		c.feature1=f1;
		c.feature2=f2;
		c.correlation=p;
		c.threshold=cl.radius*1.1; // 10% increase
		c.cx=cl.center->x;
		c.cy=cl.center->y;
		cf.push_back(c);
	}
}

bool HybridAnomalyDetector::isAnomalous(float x,float y,correlatedFeatures c){
	return (c.correlation>=threshold && SimpleAnomalyDetector::isAnomalous(x,y,c)) ||
			(c.correlation>0.5 && c.correlation<threshold && dist(Point(c.cx,c.cy),Point(x,y))>c.threshold);
}*/
