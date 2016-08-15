/*
 * FlexibleMuscleBasedControl.cpp
 *
 *  Created on: Feb 17, 2016
 *      Author: myo
 */

#include "FlexibleMuscleBasedControl.h"

FlexibleMuscleBasedControl::FlexibleMuscleBasedControl() {
	// Default values of gains
	this->kp_ = 1.0;
	this->kv_ = 1.0;
	this->kq_ = 500.0;
	this->kw_ = 1.0;

}

FlexibleMuscleBasedControl::~FlexibleMuscleBasedControl() {
	// TODO Auto-generated destructor stub
}

void FlexibleMuscleBasedControl::setMuscleActivations(RobotGeometry* &robot) {
	// iterate over all bodies
	for(int i=1; i<=3; i++) {
		// Print all input values for feedback force
		//std::cout << "Position gain is: " << this->kp_ << std::endl;
		//std::cout << "Velocity gain is: " << this->kv_ << std::endl;
		//std::cout << "Position of body " << i << " is: " << robot->getBodyPosition(i) << std::endl;
		//std::cout << "Desired position of body " << i << " is: " << robot->getDesBodyPosition(i) << std::endl;
		//std::cout << "Velocity of body " << i << " is: " << robot->getBodyVelocity(i) << std::endl;
		//std::cout << "Desired velocity of body " << i << " is: " << robot->getDesBodyVelocity(i) << std::endl;

		// Get feedback force
		//Vector3d positionError = robot->getDesBodyPosition(i) - robot->getBodyPosition(i);
		//Vector3d velocityError = robot->getDesBodyVelocity(i) - robot->getBodyVelocity(i);
		//Vector3d feedbackForce = this->kp_ * positionError;
		// Print feedback force
		//std::cout << "Feedback force of muscle " << i << " is: " << feedbackForce << std::endl;

		/// Feedback Torque ///
		// Print all input values for feedback torque
		//std::cout << "Orientation of body " << i << " is: " << robot->getBodyOrientation(i) << std::endl;
		//std::cout << "Inverse of orientation of body " << i << " is: " << robot->getBodyOrientation(i).inverse() << std::endl;
		//std::cout << "Desired orientation of body " << i << " is: " << robot->getDesBodyOrientation(i) << std::endl;
		// First, calculate matrix exponential
		//Matrix3d matrixExponential;
		//matrixExponential = (robot->getDesBodyOrientation(i) * robot->getBodyOrientation(i).inverse());
		//std::cout << "Matrix exponential input is: " << matrixExponential << std::endl;
		//matrixExponential = matrixExponential.exp();

		/*
		// Test to see what .exp() really does
		Matrix3d test_desOrient;
		test_desOrient << 	0, -1, 0,
							1, 0, 0,
							0, 0, 1;
		Matrix3d test_curOrient;
		test_curOrient << 	1, 0, 0,
						0, 1, 0,
						0, 0, 1;
		Matrix3d printMatrix = (test_desOrient*test_curOrient);
		std::cout << "Matrix exponential test (before exponential): " << printMatrix << std::endl;
		printMatrix.exp();
		std::cout << "Matrix exponential test (after exponential): " << printMatrix << std::endl;
		 */

		//std::cout << "matrixExponentital of body: " << i << " is: " << matrixExponential << std::endl;

		// ToDo: Test without additional multiplication of current orientation
		//Matrix3d propErr = robot->getBodyOrientation(i)*matrixExponential;
		//Matrix3d propErr = matrixExponential;
		//std::cout << "Proportional error of orientation of muscle " << i << " is: " << propErr << std::endl;

		// ToDo: does z error matter ?
		//propErr(2,2) = 0.0;

		//double acosForAngle = (propErr.trace()-1)/2;
		//std::cout << "acos input for angle is: " << acosForAngle << std::endl;
		//if(acosForAngle > 1.0) {
		//	acosForAngle = 1.0;
		//}
		//else if (acosForAngle < -1.0) {
		//	acosForAngle = -1.0;
		//}
		//double angle  = acos(acosForAngle);
		//std::cout << "Angle of angle-axis of muscle " << i << " is: " << angle << std::endl;

		//Vector3d angleAxis;
		//angleAxis << propErr(2,1)-propErr(1,2), propErr(0,2)-propErr(2,0), propErr(1,0)-propErr(0,1);
		//Vector3d angleAxisNormed = (1/(2*sin(angle)))*angleAxis;
		//Vector3d angleAxisNormed = angleAxis/angleAxis.norm();
		/*
		std::cout << "Normalized Angle Axis of muscle " << i << " is: " << angleAxisNormed << std::endl;

		std::cout << "Gain orientation: " << this->kq_ << std::endl;
		std::cout << "Gain angular velocity: " << this->kw_ << std::endl;
		std::cout << "Angle is: " << angle << std::endl;
		std::cout << "Axis is: " << angleAxis << std::endl;
		std::cout << "Desired angular velocity: " << robot->getDesBodyAngularVeloctiy(i) << std::endl;
		std::cout << "Angular velocity: " << robot->getBodyVelocity(i) << std::endl;

		Vector3d feedbackTorque = this->kq_ * (angle*angleAxisNormed) +
				(this->kw_*(robot->getDesBodyAngularVeloctiy(i) - robot->getBodyAngularVelocity(i)));
		std::cout << "Feedback Torque of muscle " << i << " is: " << feedbackTorque << std::endl;
		*/

		Matrix3d currentOrientation = robot->getBodyOrientation(i);
		double currentAngle = (currentOrientation.trace()-1)/2;
		//std::cout << "Input for acos of current angle is: " << currentAngle << std::endl;
		currentAngle = acos(currentAngle);
		//std::cout << "current angle of muscle " << i << " is: " << currentAngle << std::endl;
		Matrix3d desiredOrientation = robot->getDesBodyOrientation(i);
		double desiredAngle = (desiredOrientation.trace()-1)/2;
		//std::cout << "Input for acos of desired angle is: " << desiredAngle << std::endl;
		desiredAngle = acos(desiredAngle);
		//std::cout << "Desired angle of muscle " << i << " is: " << desiredAngle << std::endl;
		Vector3d angleAxis;
		angleAxis << 0, 0, 1;
		Vector3d feedbackTorque = this->kq_ * ((desiredAngle-currentAngle) * angleAxis);
		//std::cout << "Feedback Torque of muscle " << i << " is: " << feedbackTorque << std::endl;
		// For active muscles
		Vector3d momentArm = robot->getMomentArm(i);
		//std::cout << "Moment arm of muscle "  << i << " is: " << momentArm << std::endl;
		Vector3d normMomentArm = (momentArm/momentArm.norm()); // Divided by 1000 to get meters
		// Now find desired torque of joint of body
		//Eigen::Matrix<double, 1, 1> desiredTorque = (normMomentArm.cross(robot->getBodyPosition(i)-robot->getJointPosition(i))).transpose()
		//			* feedbackForce + (normMomentArm.transpose() * feedbackTorque);

		Eigen::Matrix<double, 1, 1> desiredTorque = normMomentArm.transpose() * feedbackTorque;
		//std::cout << "Desired torque of muscle " << i << " is: " << desiredTorque << std::endl;

		// Compute desired Force
		double desiredMuscleForce = (desiredTorque(0,0)/momentArm.norm())/1000;
		//std::cout << "Desired Muscle Force of muscle " << i << " is: "  << desiredMuscleForce << std::endl;

		// For passive muscles (springs)
		Vector3d springMomentArm = robot->getSpringMomentArm(i);
		Vector3d normSpringMomentArm = springMomentArm/springMomentArm.norm();
		/*
		Eigen::Matrix<double, 1, 1> desiredSpringTorque = (normSpringMomentArm.cross(robot->getBodyPosition(i)-robot->getJointPosition(i))).transpose()
					* feedbackForce + (normSpringMomentArm.transpose() * feedbackTorque);
		*/
		Eigen::Matrix<double, 1, 1> desiredSpringTorque = normSpringMomentArm.transpose() * feedbackTorque;
		double desiredSpringForce = (desiredSpringTorque(0,0)/springMomentArm.norm())/1000;
		//std::cout << "Desired Spring force for spring " << i << " is: " << desiredSpringForce << std::endl;
		// compute desired activation level and set it
		double activationLevel = desiredMuscleForce/robot->getMuscleMaxIsometForce(i);
		//std::cout << "Activation level of muscle " << i << " is: " << activationLevel << std::endl;
		robot->setMuscleActivationLevel(i, activationLevel);
	}
}


// Setters
void FlexibleMuscleBasedControl::setKp(double kp) {
	this->kp_ = kp;
}

void FlexibleMuscleBasedControl::setKv(double kv) {
	this->kv_ = kv;
}

void FlexibleMuscleBasedControl::setKq(double kq) {
	this->kq_ = kq;
}

void FlexibleMuscleBasedControl::setKw(double kw) {
	this->kw_ = kw;
}
