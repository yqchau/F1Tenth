#pragma once

#include <vector>
#include "geometry_msgs/Point.h"
#include <Eigen/Geometry>
#include <cmath>
#include <Eigen/Dense>

using namespace std;

struct Point {
  // 2D Point Coordinate defined in radius(r) and angle(theta)
  float r, theta;

  //Constructors
  Point() : r(0), theta(0){};
  Point(float range, float angle) : r(range), theta(angle){};

  /**
   * @brief find the distance to another point
   * @param pt2 the another Point
   * @return distance to the another point in meter (m)
   */
  float distToPoint(const Point* pt2) const {
    return sqrt(r*r + pt2->r*pt2->r - 2*r*pt2->r*cos(pt2->theta-theta));
  }

  /**
   * @brief find the square of the distance to another point
   * @param pt2 the another Point
   * @return square of distance to the another point in meter square (m**2)
   */
  float distToPoint2(const Point* pt2) const {
    return r*r + pt2->r*pt2->r - 2*r*pt2->r*cos(pt2->theta-theta);
  }

  /**
   * @brief find the radial gap to another point
   * @param pt2 the another point
   * @return radial dsitance to another point in meter (m)
   */
  float radialGap(const Point* pt2) const{
    return abs(r-pt2->r);
  }

  // Get the x coordinate of the point
  float getX() { return r * cos(theta); }

  // Get the y coordinate of the point
  float getY() { return r * sin(theta); }

  // Overloading < and > operator functions by comparing two Points' theta
  bool operator<(const Point& p) { return theta < p.theta; }
  bool operator>(const Point& p) { return theta > p.theta; }

  // Get point as a geometry message
  geometry_msgs::Point getPoint() const {
    geometry_msgs::Point p;
    p.x = r * cos(theta); p.y = r * sin(theta); p.z = 0.0;
    return p;
  }

  // Wrap the theta to be constrainted within -PI and PI
  void wrapTheta(){
    while(theta > M_PI){
      theta -= 2*M_PI;
    }
    while(theta <= -M_PI){
      theta += 2*M_PI;
    }
  }

  // Rotate the point by angle phi
  void rotate(float phi){
    theta = theta + phi;
    wrapTheta();
  }

  // Translate the point by distance x and y
  void translate(float x, float y){
    float newx = getX()+x;
    float newy = getY()+y;
    r = sqrt(newx*newx+newy*newy);
    theta = atan2(newy,newx);
  }

  // Get point as a vector message
  Eigen::Vector2f getVector() {
    return Eigen::Vector2f(getX(), getY());
  }
};

struct Correspondence{
  // Transformed point
  Point *p;
  // Original point
  Point *po;
  // First best point
  Point *pj1;
  // Second best point
  Point *pj2;

  float pix, piy;
  Eigen::Vector2f v;

  // Constructor
  Correspondence(Point *p, Point *po, Point *pj1, Point *pj2) : p(p), po(po), pj1(pj1), pj2(pj2) {
    float p1x = pj1->getX(), p1y = pj1->getY();
    float p2x = pj2->getX(), p2y = pj2->getY();
    float px = p->getX(), py = p->getY();
    float d = (p1x*p2x + p1y*p2y + p1x*px - p2x*px + p1y*py - p2y*py - p1x*p1x - p1y*p1y)/
          (p1x*p1x - 2*p1x*p2x + p2x*p2x + p1y*p1y - 2*p1y*p2y + p2y*p2y);
    pix = p1x + (p1x - p2x)*d;
    piy = p1y + (p1y - p2y)*d;
    v << p1y - p2y, p2x - p1x;
  }

  // Get normal of the correspondence 
  Eigen::Vector2f getNormalNorm(){
    // Eigen::Vector2f v;
    // v << p->getX() - pix,  p->getY() - piy;
    if (v.norm() > 0) { v = v / v.norm(); }
    return v;
  }

  // Get correspondence point as a geometry message
  geometry_msgs::Point getPiGeo(){
    geometry_msgs::Point pi;
    pi.x = pix;
    pi.y = piy;
    return pi;
  }

  // Get correspondence point as a vector message
  Eigen::Vector2f getPiVec(){
    Eigen::Vector2f pi;
    pi << pix , piy;
    return pi;
  }
};

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob);

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                                                vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob);

void computeJump(vector< vector<int> >& table, vector<Point>& points);
