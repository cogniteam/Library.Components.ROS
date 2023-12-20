#ifndef TIMESERIES_H_
#define TIMESERIES_H_
#include <iostream>
#include <string.h>
#include <sstream>
#include <fstream>
#include <map>
#include <vector>
#include <string.h>
#include <bits/stdc++.h>
#include <algorithm>

using namespace std;

class TimeSeries {

	map<string,vector<float>> ts; //mapping between the name of the attributes and their values.
	vector<string> atts; //vector of strings that contains the name of the attributes.
	size_t dataRowSize; //the number of values for each attribute.
    vector<float> timeSteps;

public:

    TimeSeries(){this->dataRowSize = 0;}

	TimeSeries(const char* CSVfileName) {
		ifstream in;
        in.open(CSVfileName, ios::in);
		string head;
		in>>head;
		string att;
		stringstream hss(head);
		while(getline(hss,att,',')) { //only separates the first line (the names of the attributes) according to ','
			ts.emplace(att,vector<float>());
		    atts.push_back(att);
		}

		while(!in.eof()) {
			string line;
			in>>line;
			string val;
			stringstream lss(line);
			int i=0;
			while(getline(lss,val,',')) {
				ts[atts[i]].push_back(stof(val));
			     i++;
			}
		}
		in.close();

        for(float timeStep:  ts.at("TimeStep")) {
            timeSteps.push_back(timeStep);
            //cerr<<"timeStep: "<<timeStep<<endl;
        }
        atts.erase(atts.begin());
        ts.erase("TimeStep");

		dataRowSize = ts[atts[0]].size();
	}

    TimeSeries(string csv) {

        stringstream ss(csv);

        int countStr = 0;
        int countFloat = 0;
        while (ss.good()) {
            string substr;
            getline(ss, substr, ',');
            if(countStr < 22) {
                ts.emplace(substr,vector<float>());
                atts.push_back(substr); //TODO: Check if the logic here is right. Update: Checked! the logic is good
                countStr++;
            }
            else {
                if(countFloat < 22) {
                    ts[atts[countFloat]].push_back(stof(substr));
                    countFloat++;
                }
            }
        }
        for(float timeStep:  ts.at("TimeStep")) {
            timeSteps.push_back(timeStep);
            //cerr<<"timeStep: "<<timeStep<<endl;
        }
        atts.erase(atts.begin());
        ts.erase("TimeStep");

        dataRowSize = ts[atts[0]].size();
    }

	const vector<float>& getAttributeData(string name) const {
		return ts.at(name);
	}

	const vector<string>& gettAttributes() const {
		return atts;
	}

	size_t getRowSize() const {
		return dataRowSize;
	}

    size_t getAttribtesSize() const {
        return atts.size();
    }

    void setTimeSteps(const vector<float>& v) {
        for(double li: v) {
            this->timeSteps.push_back(li);
        }
    }

    void addCol(string feature, vector<float> data) {
        this->atts.push_back(feature);
        this->ts.emplace(feature, data);
        this->dataRowSize = data.size();
    }
    
     void addRow(vector<float> data) {
        int i = 0;
        for(string feature: this->atts) {
            this->ts.at(feature).push_back(data[i]);
            i++;
        }
    }

    const vector<float>& getTimeSteps() const {
        return this->timeSteps;
    }

    int getColOfFeature(string f){
        for(int i = 0; i < this->getAttribtesSize(); i++) {
            if(this->atts.at(i) == f)
                return i+1;
        }
        return 0;
    }

    bool isFloat( string myString ) {
        std::istringstream iss(myString);
        float f;
        iss >> noskipws >> f; // noskipws considers leading whitespace invalid
        // Check the entire string was consumed and if either failbit or badbit is set
        return iss.eof() && !iss.fail();
    }

	~TimeSeries() {}
};



#endif /* TIMESERIES_H_ */
