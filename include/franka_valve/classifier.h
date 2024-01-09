#pragma once
#ifndef __Classifier_H
#define __Classifier_H
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

class Classifier
{
public:
    Classifier();
    // void Initialize(string folder_name);
    virtual ~Classifier();

    
    VectorXd Fc_layer1(VectorXd x);
    VectorXd Fc_layer2(VectorXd x);
    VectorXd Fc_layer3(VectorXd x);
    VectorXd Fc_layer4(VectorXd x);
    VectorXd Forward(VectorXd x);
    void Initialize(string folder_name);
    void state_update(VectorXd x);
    void setup_weight(string model_path);


private:
   

    VectorXd V_ReLU(VectorXd x);

    ifstream weight;

    double weight0[64][20];     //FC1_weight
    double weight1[64];        //FC1_bias 
    double weight2[256][64];   //FC2_weight
    double weight3[256];        //FC2_bias
    
    double weight4[256][256];     //FC3_weight
    double weight5[256];          //FC3_bias
    double weight6[20][256];     //FC4_weight
    double weight7[20];          //FC4_bias


    int input_size, output_size, hidden1, hidden2;

    VectorXd x;
    VectorXd _Fcb1, _Fcb2, _Fcb3, _Fcb4;
    MatrixXd _FcW1, _FcW2, _FcW3, _FcW4, buffer;

    VectorXd Fc1_output, output1;
    VectorXd Fc2_output, output2;
    VectorXd Fc3_output, output3;
    VectorXd Fc4_output, output4;
    
    VectorXd  FC1_out, FC2_out, FC3_out, FC4_out;
};
#endif