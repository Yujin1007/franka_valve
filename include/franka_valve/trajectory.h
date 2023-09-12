#pragma once
#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "custommath.h"

using namespace std;
using namespace Eigen;

class CTrajectory
{

public:
	CTrajectory();
	virtual ~CTrajectory();

	void set_size(int dof);
	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);
	int check_trajectory_complete();

	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();


private:
	void Initialize();
	void check_vector_size(VectorXd X);
	
	int _vector_size;
	double _time_start, _time, _time_end;	
	VectorXd _init_pos, _init_vel, _goal_pos, _goal_vel;
	bool _bool_trajectory_complete;
	double _motion_threshold;
};


class HTrajectory
{

public:
	HTrajectory();
	virtual ~HTrajectory();

	void reset_initial(double time0, VectorXd init_pos, VectorXd init_vel);
	void update_time(double time);
	void update_goal(VectorXd goal_pos, VectorXd goal_vel, double goal_time);

	VectorXd position_cubicSpline();
	VectorXd velocity_cubicSpline();

	Matrix3d rotationCubic();
	Vector3d rotationCubicDot();

	int check_trajectory_complete();


private:
	void Initialize();

	double cubic(double x_0, double x_f, double x_dot_0, double x_dot_f);
	double cubicDot(double x_0, double x_f, double x_dot_0, double x_dot_f);

	CTrajectory XTrajectory;

	double _time_start, _time, _time_end;

	Matrix3d rotation_0, rotation_f;
	VectorXd _init_pos, _init_vel;
	VectorXd _goal_pos, _goal_vel;
	Vector3d w_0, a_0;

	bool _bool_trajectory_complete;
};


class RTrajectory
{

public:
	RTrajectory();
	virtual ~RTrajectory();


	void reset_initial(double time0, Vector3d tangent_vector, Vector3d normal_vector, double radius, MatrixXd Tvr, double dt);

	double update_time(double time);
	
	void update_goal(double goal_time, double init_theta, double goal_theta);
	
	Vector3d drpy2nextrpy(Vector3d drpy, Vector3d rpy);
	VectorXd position_circular();
	VectorXd velocity_circular();
	Vector3d rotation_circular();
	Vector3d rotationdot_circular();

	int check_trajectory_complete();


private:
	void Initialize();

	double angle_cubicSpline();
	
	double angledot_cubicSpline();
	Vector3d rotation_circular_calc(double theta);

	int _vector_size;

	double _time_start, _time, _time_end, _dt;
	double _acc_theta, _acc_time, _acc_dtheta, _add_theta;
	double _theta, _dtheta, _r, _goal_theta, _theta_des;
	double _theta_next, _dtheta_next;
	Vector3d _tangent_vector, _normal_vector;
	MatrixXd _Tvr;
	bool _bool_trajectory_complete;
	double _init_theta;
	VectorXd _rpy_next, _rpy;


	
};

#endif
