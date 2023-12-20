#ifndef ANOMALYDETECTORUTIL_H_
#define ANOMALYDETECTORUTIL_H_

#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace std;

float avg(float* x, int size);

// returns the variance of X and Y
float var(float* x, int size);

// returns the covariance of X and Y
float cov(float* x,  float* y, int size);


// returns the Pearson correlation coefficient of X and Y
float pearson(float* x, float* y, int size);

class Line{
public:
	float a,b;
	Line():a(0),b(0){};
	Line(float a, float b):a(a),b(b){}
	float f(float x){
		return a*x+b;
	}
    bool operator == (Line const &obj) {
        return ((this->a == obj.a) && (this->b == obj.b));
    }
};

class Point{
public:
	float x,y;
	Point(float x, float y):x(x),y(y){}

    float distanceSquaredTo(Point p) {
        float DX = x - p.x;
        float DY = y - p.y;

        return DX * DX + DY * DY;
    }

    float distanceTo(Point p) {
        return (float)sqrt(distanceSquaredTo(p));
    }

    bool operator == (Point const &obj) {
        return ((this->x == obj.x) && (this->y == obj.y));
    }
};


// performs a linear regression and returns the line equation
Line linear_reg(Point** points, int size);

// returns the deviation between point p and the line equation of the points
float dev(Point p,Point** points, int size);

// returns the deviation between point p and the line
float dev(Point p,Line l);

vector<Point> getListPoint(vector<float> x, vector<float> y);

Point** toPoints(vector<float> x, vector<float> y);

#endif
