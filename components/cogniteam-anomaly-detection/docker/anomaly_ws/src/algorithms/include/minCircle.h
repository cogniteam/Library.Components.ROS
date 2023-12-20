
#ifndef MINCIRCLE_H_
#define MINCIRCLE_H_

#include <iostream>
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include "anomaly_detection_util.h"
#include <algorithm>

using namespace std;


// ------------ DO NOT CHANGE -----------
//class Point{
//public:
//	float x,y;
//	Point(float x,float y):x(x),y(y){}
//};

class Circle{
public:
	Point* center;
	float radius;
	Circle(Point* c,float r):center(c),radius(r){}

    Circle(float x, float y, float radius) {
        center = new Point(x, y);
        this->radius = radius;
    }

    Circle(Point p, float r) {
        center = &p;
        this->radius = radius;

    }

    Circle(Point p1, Point p2) {
        center = new Point((float)((p1.x + p2.x) * 0.5), (float)((p1.y + p2.y) * 0.5));
        radius = center->distanceTo(p1);
    }

    Circle(Point p1, Point p2, Point p3) {
        float P2_MINUS_P1_Y = p2.y - p1.y;
        float P3_MINUS_P2_Y =  p3.y - p2.y;

        if (P2_MINUS_P1_Y == 0.0 || P3_MINUS_P2_Y == 0.0) {
            center = new Point((float)0,(float)0);
            radius = (float)0;
        }
        else {
            float A = -(p2.x - p1.x) / P2_MINUS_P1_Y;
            float A_PRIME = -(p3.x - p2.x) / P3_MINUS_P2_Y;
            float A_PRIME_MINUS_A = A_PRIME - A;

            if (A_PRIME_MINUS_A == 0.0) {
                center = new Point((float)0, (float)0);
                radius = (float)0;
            }
            else {
                float P2_SQUARED_X = p2.x * p2.x;
                float P2_SQUARED_Y = p2.y * p2.y;


                float B = (float) ((P2_SQUARED_X - p1.x * p1.x + P2_SQUARED_Y - p1.y * p1.y) /
                                         (2.0 * P2_MINUS_P1_Y));
                float B_PRIME = (float) ((p3.x * p3.x - P2_SQUARED_X + p3.y * p3.y - P2_SQUARED_Y) /
                                               (2.0 * P3_MINUS_P2_Y));


                float XC = (B - B_PRIME) / A_PRIME_MINUS_A;
                float YC = A * XC + B;

                float DXC = p1.x - XC;
                float DYC = p1.y - YC;

                center = new Point(XC, YC);
                radius = (float) sqrt(DXC * DXC + DYC * DYC);
            }
        }
    }

    bool containsPoint(Point p) {
        return p.distanceSquaredTo(*center) <= radius * radius;
    }
};
// --------------------------------------



float dist(Point a, Point b);

Circle from2points(Point a,Point b);

Point circumcenter(Point b, Point c);

Circle from3Points(Point a, Point b, Point c);

Circle trivial(vector<Point>& P);

Circle welzl(Point** P,vector<Point> R, size_t n);

//Circle welzl(vector<Point> points, vector<Point> boundary);

//Circle findMinCircle(vector<Point> points);

Circle findMinCircle(Point **points, size_t size);

#endif /* MINCIRCLE_H_ */
