#ifndef ROBOTGEOMETRY_H_
#define ROBOTGEOMETRY_H_

#include <Eigen/Dense>
#include <iostream>
#include <list>

#include "HillMuscleModel.h"
#include "SpringMuscle.h"

//namespace std
//{

//namespace matrix
//{

typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

class RobotGeometry {
public:
	RobotGeometry();
	virtual ~RobotGeometry();

	// Test function
	void bla();

	// Updates transformation matrices with current joint states
	void updateTransformationMatrices();
	// Updates all points on all bodies
	void updateBodies();


	// Setters
	void setJointAngles(std::list<double> jointAngles);
	void setDesJointAngles(double desX, double desY);
	void setRailJoint(double railInput); // Sets the member variable
	// Set length for internal muscle models
	void setTotalMuscleLengths();

	// Set length of springs
	void setSpringMuscleLength();

	// Set muscle length
	//void calculateTotalMuscleLength();

	// Transforms input for rail joint [mm] in inputs for low level position controller
	// and enforces limits
	double getRailInput();

	// Getter methods
	Vector3d getBodyPosition(int bodyID);
	Vector3d getBodyVelocity(int bodyID);
	Matrix3d getBodyOrientation(int bodyID);
	Vector3d getBodyAngularVelocity(int bodyID);

	Vector3d getDesBodyPosition(int bodyID);
	Vector3d getDesBodyVelocity(int bodyID);
	Matrix3d getDesBodyOrientation(int bodyID);
	Vector3d getDesBodyAngularVeloctiy(int bodyID);

	Vector3d getJointPosition(int bodyID);
	Vector3d getMomentArm(int bodyID);
	Vector3d getSpringMomentArm(int springID);
	double getRealSpringMomentArm(int springID);

	double getMuscleMaxIsometForce(int muscleID);
	void setMuscleActivationLevel(int muscleID, double activationLevel);
	double getTotalMuscleForce(int muscleID);

	double getTotalSpringForce(int springID);

	void initJointAngles(std::list<double> jointAngles);

private:
	// List to store all transformation matrices needed for forward kinematics
	std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> > forwardKinematics_;
	// List to store all transformation matrices of the desired configuration
	std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> > desForwardKinematics_;

	// List to store all joint angles coming from the sensors of the robot
	std::list<double> jointAngles_;
	// List to store all joint angles from last loop iteration to get angular momentum
	std::list<double> oldJointAngles_;
	// List to store all desired joint angles from inverse kinematics based on user input
	std::list<double> desJointAngles_;
	// List of max limits for joint angles
	std::list<double> maxAngles_;
	// List of min limits for joint angles
	std::list<double> minAngles_;
	// Container for current rail joint position
	double railJointPosition_;

	// Struct to define a body, which is the sum of its attachment/detachment points
	struct body {
		// spring points
		Vector4d springAttach;
		Vector4d refSpringAttach;
		Vector4d springDetach;
		Vector4d refSpringDetach;
		Vector4d springGuide;
		Vector4d refSpringGuide;

		// muscle points
		Vector4d muscleAttach;
		Vector4d refMuscleAttach;
		Vector4d muscleDetach;
		Vector4d refMuscleDetach;
		Vector4d muscleGuide;
		Vector4d refMuscleGuide;

		// joint position (is origin of frame)
		Vector4d refJointPosition_;
		Vector4d jointPosition_;

		// state
		Vector4d position;
		Vector4d refPosition;
		Vector3d velocity;
		Vector3d angVelocity;
		Matrix3d orientation;
		Matrix3d refOrientation;

		// desired state
		Vector4d desPosition;
		Vector4d refDesPosition;
		Vector3d desVelocity;
		Vector3d desAngVelocity;
		Matrix3d desOrientation;
		Matrix3d refDesOrientation;
	};

	//keep a list of all bodies
	std::list<struct body> bodies_;

	// All muscles
	HillMuscleModel* muscleUpperArm_;
	HillMuscleModel* muscleLowerArm_;
	HillMuscleModel* muscleEndeffector_;

	// All passive muscles
	//SpringMuscle* springUpperArm_;
	//SpringMuscle* springLowerArm_;
	//SpringMuscle* springEndeffector_;
	std::list<SpringMuscle> springMuscles_;
};
//}
//}
#endif /* ROBOTGEOMETRY_H_ */
