#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>

#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;

using namespace std;

///base abstract kinematics class

class Kinematics
{
public:
    Kinematics();

private:
};

///Table lookup kinematics

class TableKinematics : public Kinematics
{
public:
    TableKinematics();
    TableKinematics(string path);
    long GetIK(long theta, long phi, vector<double> &lengths);

private:
    long Initialize(string csvfileName, vector<int> tableDimensions);
    vector < vector<long> > lookupIndex;
    vector < vector<double> > lookupTable;
};


///Inverse kinematic algorithm
class GeoInkinematics
{
public:
    GeoInkinematics();
    GeoInkinematics(double  new_a, double new_b, double new_L0);
    long GetIK(double incl,double orien, vector<double> &lengths);

private:
    double theta,phi;
    double a,b;
    double L0;
    double t11, t12, t13, t21, t22, t23, t31, t32, t33, R, s0, t0, P, L;
};

#endif // KINEMATICS_H

