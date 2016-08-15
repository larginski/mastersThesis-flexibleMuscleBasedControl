#ifndef FLEXIBLEMUSCLEBASEDCONTROL_H_
#define FLEXIBLEMUSCLEBASEDCONTROL_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <math.h>
#include <iostream>
#include <list>

#include "RobotGeometry.h"

typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

class FlexibleMuscleBasedControl {
public:
	FlexibleMuscleBasedControl();
	virtual ~FlexibleMuscleBasedControl();

	// Set desired muscle activations
	void setMuscleActivations(RobotGeometry* &robot);

	// Setter
	void setKp(double kp);
	void setKv(double kv);
	void setKq(double kq);
	void setKw(double kw);

private:
	// Gain for proportional position difference
	double kp_;
	// Gain for derivative velocity difference
	double kv_;
	// Gain for proportional orientation difference
	double kq_;
	// Gain for derivative angular velocity difference
	double kw_;
};

#endif /* FLEXIBLEMUSCLEBASEDCONTROL_H_ */
