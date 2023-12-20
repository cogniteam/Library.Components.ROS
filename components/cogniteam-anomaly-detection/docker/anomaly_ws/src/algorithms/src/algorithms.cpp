#include "../include/algorithms.h"
#include "../include/ZScoreAnomalyDetector.h"
#include "../include/SimpleAnomalyDetector.h"
#include "../include/HybridAnomalyDetector.h"
#include "../include/timeseries.h"
#include <iostream>
#include <fstream>
#include <string.h>


Algorithms::Algorithms() {

    ros::NodeHandle nodePrivate("~");

    /*set parameters*/
    nodePrivate.param("csv_path", pathTrain_, string("/home/") );
    nodePrivate.param("detection_mode", detectionMode_, false);
    nodePrivate.param("algorithm_type", algorithmType_, 1);
    nodePrivate.param("result_file_path", pathRes_, string("/home"));
    nodePrivate.param("simple_detector_threshold", simpleAnomalyTh_, float (0.7));
    nodePrivate.param("upper_bound", upperBound_, float (0.9));
    nodePrivate.param("lower_bound", lowerBound_, float (0.5));


    /*if it's learning mode, subscribe to the topic /state. (that says if the msg_convertor node has finished writing to the file or not)*/
    if(detectionMode_ == false)
        this->subscriber_ = node_.subscribe<std_msgs::String>("state", 1000, &Algorithms::subCallBack, this);

    /*if it's detection mode, subscribe to the /csv topic, that publishes each time step vector (in csv format) of the feature's values*/
    if(detectionMode_ == true)
        this->detectionSub_ = node_.subscribe<std_msgs::String>("csv",1000, &Algorithms::detectionSubCallback, this);
}

void Algorithms::updateTimerCallback(const ros::TimerEvent&) {}


/*If the node is in learning mode, the subscriber_ will be directed to this callback.
 * This callback will learn the correlation between the features that are in the csv file (path_) and will write the results in the result file (pathRes_) */
void Algorithms::subCallBack(const std_msgs::String::ConstPtr& msg) {
    string state = "Finished";
    if(state.compare(msg->data)==0) {
        if(testCount_ == 1) {
            this->tsTrain_ = new TimeSeries(this->pathTrain_.c_str());

            this->simpleAnomalyDetector_ = new SimpleAnomalyDetector(this->simpleAnomalyTh_);
            this->zScoreDetector_ = new ZScoreAnomalyDetector();
            this->hybridAnomalyDetector_ = new HybridAnomalyDetector(this->simpleAnomalyTh_, this->lowerBound_, this->upperBound_);

            ofstream of;
            of.open(pathRes_, std::ios_base::app); //TODO: Don't so sure about the permission of the file, std::ios::trunc

            switch (algorithmType_) {
                case 1:
                    simpleAnomalyDetector_->learnNormal(*tsTrain_);
                    for(correlatedFeatures c :this->simpleAnomalyDetector_->getNormalModel()) {
                        of<<c.feature1 <<","<<c.feature2 + ","<<to_string(this->tsTrain_->getColOfFeature(c.feature1))<<","<<to_string(this->tsTrain_->getColOfFeature(c.feature2))<<","<<to_string(c.correlation)<<","<<to_string(c.threshold)<<","<<to_string(c.lin_reg.a)<<","<<to_string(c.lin_reg.b)<<endl;
                    }
                    //cerr<<"DONE SIMPLE ANOMALY DETECTOR"<<endl;
                    break;
                case 2:
                    zScoreDetector_->learnNormal(*tsTrain_);
                    for(int i = 0; i < zScoreDetector_->getThArr().size(); i++) {
                        of<<tsTrain_->gettAttributes().at(i)<<","<<(i+1)<<","<<to_string(zScoreDetector_->getThArr().at(i))<<endl;
                    }
                    //cerr<<"DONE ZSCORE ANOMALY DETECTOR"<<endl;
                    break;
                case 3:
                    hybridAnomalyDetector_->learnNormal(*tsTrain_);
                    if(this->hybridAnomalyDetector_->regressionDetector->getNormalModel().size() != 0) {
                        of<<"Simple Regression Algorithm"<<endl;
                        for(correlatedFeatures c :this->hybridAnomalyDetector_->regressionDetector->getNormalModel()) {
                            of<<c.feature1 <<","<<c.feature2 + ","<<to_string(this->tsTrain_->getColOfFeature(c.feature1))<<","<<to_string(this->tsTrain_->getColOfFeature(c.feature2))<<","<<to_string(c.correlation)<<","<<to_string(c.threshold)<<","<<to_string(c.lin_reg.a)<<","<<to_string(c.lin_reg.b)<<endl;
                        }
                    }
                    if(this->hybridAnomalyDetector_->zScoreDetector->getThArr().size() != 0) {
                        of<<"ZScore Algorithm"<<endl;
                        for(int i = 0; i < hybridAnomalyDetector_->zScoreDetector->getThArr().size(); i++) {
                            of<<tsTrain_->gettAttributes().at(i)<<","<<(i+1)<<","<<to_string(this->hybridAnomalyDetector_->zScoreDetector->getThArr().at(i))<<endl;
                        }
                    }
                    if(this->hybridAnomalyDetector_->welzlCircleModel.size() != 0) {
                        of<<"Welzl Algorithm"<<endl;
                        for (auto const& x : this->hybridAnomalyDetector_->welzlCircleModel)
                        {
                            vector<string> unhash;
                            stringstream ss(x.first);
                            while (ss.good()) {
                                string substr;
                                getline(ss, substr, ',');
                                unhash.push_back(substr);
                            }
                            of<<unhash.at(0)<<","<<unhash.at(1)<<","<<unhash.at(2)<<","<<unhash.at(3)<<","<<to_string(x.second.center->x)<<","<<to_string(x.second.center->y)<<","<<to_string(x.second.radius)<<endl;
                        }
                    }
                    //cerr<<"DONE HYBRID ANOMALY DETECTOR"<<endl;
                    break;
            }
            of.close();
            testCount_++;
        }
    }
    else {
        //The state from the msg_convertor node is NotFinished, which means that the msg_convertor node didn't finish yet to write to the csv file.
        return;
    }
}

/*If the node in detection mode, the detectionSub_ will be directed to this callback.
 *This callback will read from the result file (pathRes_), the results of the learning mode (according to the algorithmType_), will cal the appropriate detection and publish the anomalies in the /anomalies topic */
void Algorithms::detectionSubCallback(const std_msgs::String::ConstPtr& msg) {
    this->outputStream_ = vector<AnomalyReport>();
    this->pub_ = node_.advertise<std_msgs::String>("anomalies", 20);

    this->simpleAnomalyDetector_ = new SimpleAnomalyDetector(this->simpleAnomalyTh_);
    this->zScoreDetector_ = new ZScoreAnomalyDetector();
    this->hybridAnomalyDetector_ = new HybridAnomalyDetector(this->simpleAnomalyTh_, this->lowerBound_, this->upperBound_);

    this->tsTest_ = new TimeSeries(msg->data);

    this->algorithmToCallback[1] = &Algorithms::simpleAnomalyDetectionCallback;
    this->algorithmToCallback[2] = &Algorithms::zScoreAnomalyDetectionCallback;
    this->algorithmToCallback[3] = &Algorithms::hybridAnomalyDetectionCallback;

    auto it = this->algorithmToCallback.find(algorithmType_);
    if ( it != this->algorithmToCallback.end() )
        (*this.*algorithmToCallback[algorithmType_])();

}



void Algorithms::simpleAnomalyDetectionCallback() {
    ifstream in;
    in.open(pathRes_, ios::in);
    vector<string> v;
    while(!in.eof()) {
        string line;
        in>>line;
        string val;
        stringstream lss(line);
        while(getline(lss,val,',')) {
            v.push_back(val);
        }
        if(in.eof()) {
            break;
        }
        string feature1 = v.at(0);
        string feature2 = v.at(1);
        float correlation = stof(v.at(4));
        float threshold = stof(v.at(5));
        Line* lin_reg = new Line(stof(v.at(6)), stof(v.at(7)));
        correlatedFeatures c;
        c.feature1 = feature1;
        c.feature2 = feature2;
        c.correlation = correlation;
        c.threshold = threshold;
        c.lin_reg = *lin_reg;
        simpleAnomalyDetector_->cf.push_back(c);
        v.clear();
    }
    this->outputStream_ = this->simpleAnomalyDetector_->detect(*tsTest_);
    for(AnomalyReport ar : this->outputStream_) {
        std_msgs::String toPublish;
        toPublish.data = "TimeStep: " + to_string(ar.timeStep) + " " + "Description: " + ar.description.data;
        this->pub_.publish(toPublish);
    }
    in.close();
}

void Algorithms::zScoreAnomalyDetectionCallback() {
    ifstream in;
    in.open(pathRes_, ios::in);
    vector<string> v;
    vector<float> thArray;
    while(!in.eof()) {
        string line;
        in >> line;
        string val;
        stringstream lss(line);
        while (getline(lss, val, ',')) {
            v.push_back(val);
        }
        if(in.eof()) {
            break;
        }
        string feature = v.at(0);
        float th = stof(v.at(2));
        thArray.push_back(th);
    }
    zScoreDetector_->setThArr(thArray);
    this->outputStream_ = this->zScoreDetector_->detect(*tsTest_);
    for(AnomalyReport ar : this->outputStream_) {
        std_msgs::String toPublish;
        toPublish.data = "TimeStep: " + to_string(ar.timeStep) + " " + "Description: " + ar.description.data;
        this->pub_.publish(toPublish);
    }
    //cerr<<"ZScore anomaly detection done"<<endl;
    in.close();
}

void Algorithms::hybridAnomalyDetectionCallback() {
    ifstream in;
    in.open(pathRes_, ios::in);
    vector<string> v1;
    vector<string> v2;
    vector<string> v3;
    vector<float> thArray;
    string currentAlgorithm;
    hybridAnomalyDetector_->featuresToAlgorithm.emplace("ZScore", vector<correlatedFeatures>());
    hybridAnomalyDetector_->featuresToAlgorithm.emplace("Regression", vector<correlatedFeatures>());
    hybridAnomalyDetector_->featuresToAlgorithm.emplace("Welzl", vector<correlatedFeatures>());
    while(!in.eof()) {
        string line;
        getline(in, line);
        if(line.compare("Simple Regression Algorithm") == 0) {
            currentAlgorithm = "Simple";
            continue;
        }
        if(line.compare("ZScore Algorithm") == 0) {
            currentAlgorithm = "ZScore";
            continue;
        }
        if(line.compare("Welzl Algorithm") == 0) {
            currentAlgorithm = "Welzl";
            continue;
        }
        string val;
        stringstream lss(line);
        if(currentAlgorithm.compare("Simple") == 0) {
            while (getline(lss, val, ',')) {
                v1.push_back(val);
            }
            if(in.eof())
                break;
            string feature1 = v1.at(0);
            string feature2 = v1.at(1);
            float correlation = stof(v1.at(4));
            float threshold = stof(v1.at(5));
            Line* lin_reg = new Line(stof(v1.at(6)), stof(v1.at(7)));
            correlatedFeatures c;
            c.feature1 = feature1;
            c.feature2 = feature2;
            c.correlation = correlation;
            c.threshold = threshold;
            c.lin_reg = *lin_reg;
            hybridAnomalyDetector_->featuresToAlgorithm["Regression"].push_back(c);
            if(feature1.compare(feature2) == 0) {
                hybridAnomalyDetector_->regressionDetector->correlatedOnlyForThemself.push_back(c);
            }
            else
                hybridAnomalyDetector_->regressionDetector->cf.push_back(c);
            v1.clear();
        }
        if(currentAlgorithm.compare("ZScore") == 0) {
            if(in.eof())
                break;
            while (getline(lss, val, ',')) {
                v2.push_back(val);
            }
            string feature1 = v2.at(0);
            string feature2 = v2.at(0);
            float th = stof(v2.at(2));
            thArray.push_back(th);
            correlatedFeatures c;
            c.feature1 = feature1;
            c.feature2 = feature2;
            c.correlation = th;
            hybridAnomalyDetector_->featuresToAlgorithm["ZScore"].push_back(c);
            hybridAnomalyDetector_->zScoreDetector->setThArr(thArray);
            v2.clear();
        }
        if(currentAlgorithm.compare("Welzl") == 0) {
            if(in.eof()) {
                break;
            }
            while (getline(lss, val, ',')) {
                v3.push_back(val);
            }
            string feature1 = v3.at(0);
            string feature2 = v3.at(1);
            float correlation = stof(v3.at(2));
            float threshold = stof(v3.at(3));
            Circle* circle  = new Circle(stof(v3.at(4)),stof(v3.at(5)), stof(v3.at(6)));
            correlatedFeatures c;
            c.feature1 = feature1;
            c.feature2 = feature2;
            c.correlation = correlation;
            c.threshold = threshold;
            c.cx = stof(v3.at(4));
            c.cy = stof(v3.at(5));
            hybridAnomalyDetector_->welzlCircleModel.emplace(c.hash(), *circle);
            hybridAnomalyDetector_->featuresToAlgorithm["Welzl"].push_back(c);
            v3.clear();
        }
    }
    this->outputStream_ = this->hybridAnomalyDetector_->detect(*tsTest_);
    for(AnomalyReport ar : this->outputStream_) {
        std_msgs::String toPublish;
        toPublish.data = "TimeStep: " + to_string(ar.timeStep) + " " + "Description: " + ar.description.data;
        this->pub_.publish(toPublish);
    }
    //cerr<<"Hybrid anomaly detection done"<<endl;
    in.close();
}