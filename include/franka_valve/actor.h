#pragma once
#ifndef __ACTOR_H
#define __ACTOR_H
#include <iostream>
#include <fstream>
#include <string.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <time.h>

#include <vector>
#include <sstream>
#include <chrono>

using namespace std;
using namespace Eigen;

class Actor
{
public:
    Actor();
    virtual ~Actor();

    VectorXd Fc_layer1(VectorXd x);
    VectorXd Fc_layer2(VectorXd x);
    VectorXd Fc_layer3(VectorXd x);
    VectorXd Forward(VectorXd x);
    void state_update(VectorXd x);
    void setup_weight(const char weight_hh[], const char weigh_hi[], const char bias_hh[], const char bias_hi[], const char fcWeight1[], const char fcbias1[]);


private:
    void Initialize();
    VectorXd V_ReLU(VectorXd x);

    ifstream weight;

    double weight0[256][97];     //FC1_weight
    double weight1[256];        //FC1_bias 
    double weight2[256][256];   //FC2_weight
    double weight3[256];        //FC2_bias
    
    double weight4[6][256];     //FC3_weight
    double weight5[6];          //FC3_bias


    int input_size, output_size, hidden_units;

    VectorXd x;
    VectorXd _Fcb1, _Fcb2, _Fcb3;
    MatrixXd _FcW1, _FcW2, _FcW3, buffer;

    VectorXd Fc1_output, output1;
    VectorXd Fc2_output, output2;
    VectorXd Fc3_output, output3;
    
    VectorXd  FC1_out, FC2_out, FC3_out;
};
#endif