#include "franka_valve/classifier.h"
#define MAX_NAME_LEN 50
Classifier::Classifier()
{
    
}
Classifier::~Classifier()
{
    cout << "Classifier Memory delete" << endl;
}

void Classifier::setup_weight(string model_path)
{
    string line;
    vector<vector<float>> weight_w;
    vector<float> weight_b;

    string FC1_weight = model_path + "/w1.csv";
    string FC2_weight = model_path + "/w2.csv";
    string FC3_weight = model_path + "/w3.csv";
    string FC4_weight = model_path + "/w4.csv";
    string FC1_bias = model_path + "/b1.csv";
    string FC2_bias = model_path + "/b2.csv";
    string FC3_bias = model_path + "/b3.csv";
    string FC4_bias = model_path + "/b4.csv";

    ifstream file;
    file.open(FC1_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    for (int r = 0; r < hidden1; r++)
    {
        for (int c = 0; c < input_size; c++)
        {
            weight0[r][c] = weight_w[r][c];
        }
    }
    weight_w.clear();
    file.close();

    // ifstream file(FC1_bias);
    file.open(FC1_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < hidden1; r++)
    {
        weight1[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();


    file.open(FC2_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    
    for (int r = 0; r < hidden2; r++)
    {
        for (int c = 0; c < hidden2; c++)
        {
            weight2[r][c] = weight_w[r][c];
        }
    }
    weight_w.clear();
    file.close();

    file.open(FC2_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < hidden2; r++)
    {
        weight3[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();

    file.open(FC3_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    
    for (int r = 0; r < hidden2; r++)
    {
        for (int c = 0; c < hidden2; c++)
        {
            weight4[r][c] = weight_w[r][c];
        }
    }
    weight_w.clear();
    file.close();

    file.open(FC3_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < hidden2; r++)
    {
        weight5[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();

    file.open(FC4_weight);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            row.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }

        weight_w.push_back(row);
    }
    
    for (int r = 0; r < output_size; r++)
    {
        for (int c = 0; c < hidden2; c++)
        {
            weight6[r][c] = weight_w[r][c];
        }
    }
    weight_w.clear();
    file.close();

    file.open(FC4_bias);
    while (getline(file, line))
    {
        vector<float> row;
        istringstream iss(line);
        float value;
        while (iss >> value)
        {
            weight_b.push_back(value);
            if (iss.peek() == ',')
                iss.ignore();
        }
    }
    
    for (int r = 0; r < output_size; r++)
    {
        weight7[r] = weight_b[r];
        
    }
    weight_b.clear();
    file.close();



    // Fc1
    for (int i = 0; i < hidden1; i++)
    {
        for (int j = 0; j < input_size; j++)
        {
            _FcW1(i, j) = weight0[i][j];
        }
    }
    for (int i = 0; i < hidden1; i++)
    {
        _Fcb1(i) = weight1[i];
    }
    // Fc2
    for (int i = 0; i < hidden2; i++)
    {
        for (int j = 0; j < hidden1; j++)
        {
            _FcW2(i, j) = weight2[i][j];
        }
    }
    for (int i = 0; i < hidden2; i++)
    {
        _Fcb2(i) = weight3[i];
    }
    // Fc3
    for (int i = 0; i < hidden2; i++)
    {
        for (int j = 0; j < hidden1; j++)
        {
            _FcW3(i, j) = weight4[i][j];
        }
    }
    for (int i = 0; i < hidden2; i++)
    {
        _Fcb3(i) = weight5[i];
    }
    // Fc3
    for (int i = 0; i < output_size; i++)
    {
        for (int j = 0; j < hidden2; j++)
        {
            _FcW3(i, j) = weight6[i][j];
        }
    }
    for (int i = 0; i < output_size; i++)
    {
        _Fcb3(i) = weight7[i];
    }

    cout << "pass setup weights" << endl;
}
VectorXd Classifier::Fc_layer1(VectorXd x)
{
    Fc1_output = (_FcW1 * x) + _Fcb1;
    output1 = V_ReLU(Fc1_output);
    return output1;
}
VectorXd Classifier::Fc_layer2(VectorXd x)
{
    Fc2_output = (_FcW2 * x) + _Fcb2;
    output2 = V_ReLU(Fc2_output);

    return output2;
}
VectorXd Classifier::Fc_layer3(VectorXd x)
{
    Fc3_output = (_FcW3 * x) + _Fcb3;
    output3 = V_ReLU(Fc3_output);

    return output2;
}
VectorXd Classifier::Fc_layer4(VectorXd x)
{
    Fc4_output = (_FcW4 * x) + _Fcb4;
    output4 = (Fc4_output);

    return output4;
}
VectorXd Classifier::Forward(VectorXd x)
{
    FC1_out = Fc_layer1(x);
    FC2_out = Fc_layer2(FC1_out);
    FC3_out = Fc_layer3(FC2_out);
    FC4_out = Fc_layer3(FC3_out);
    return FC4_out;
}

void Classifier::Initialize(string folder_name)
{
    input_size = 8; 
    output_size = 20;
    hidden1 = 64;
    hidden2 = 256;

    x.setZero(input_size);

    _FcW1.setZero(hidden1, input_size);
    _Fcb1.setZero(hidden1);
    _FcW2.setZero(hidden2, hidden1);
    _Fcb2.setZero(hidden2);
    _FcW3.setZero(hidden2, hidden2);
    _Fcb3.setZero(hidden2);
    _FcW4.setZero(output_size, hidden2);
    _Fcb4.setZero(output_size);

    Fc1_output.setZero(hidden1);
    output1.setZero(hidden1);

    Fc2_output.setZero(hidden2);
    output2.setZero(hidden2);

    Fc3_output.setZero(hidden2);
    output3.setZero(hidden2);

    Fc4_output.setZero(output_size);
    output4.setZero(output_size);

    string model_path = "/home/kist/catkin_ws/src/franka_ros/franka_valve/src/model_weight/" + folder_name;
    
    
    
    setup_weight(model_path);

}
VectorXd Classifier::V_ReLU(VectorXd x)
{
    VectorXd _x, _relu_x;
    if (x.rows() > 2)
    {
        _relu_x.setZero(x.rows());
        _x.setZero(x.rows());
        _x = x;
        for (int i = 0; i < x.rows(); i++)
        {
            if (_x(i) < 0)
            {
                _relu_x(i) = 0;
            }
            else
            {
                _relu_x(i) = _x(i);
            }
        }
    }
    else
    {
        _relu_x.setZero(x.rows());
        _x.setZero(x.rows());
        _x = x;
        for (int i = 0; i < x.rows(); i++)
        {
            if (_x(i) < 0)
            {
                _relu_x(i) = 0;
            }
            else
            {
                _relu_x(i) = _x(i);
            }
        }
    }
    return _relu_x;
}
