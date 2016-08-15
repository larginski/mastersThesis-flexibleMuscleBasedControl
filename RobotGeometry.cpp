#include "RobotGeometry.h"
#include <iostream>
#include <cmath>

RobotGeometry::RobotGeometry() {
	// First, generate all transformation matrices
	// Note: row based insertion, translational offsets in [mm]
	Matrix4d transToFirstJoint;
	transToFirstJoint << 1, 0, 0, 627,
						 0, 1, 0, -65,
						 0, 0, 1, 250,
						 0, 0, 0, 1; // z-position is from WCF origin to the start of rail joint
	this->forwardKinematics_.push_back(transToFirstJoint);
	this->desForwardKinematics_.push_back(transToFirstJoint);
	Matrix4d transFirstToSecond;
	transFirstToSecond << 1, 0, 0, 103,
					      0, 1, 0, 23,
					      0, 0, 1, -43,
					      0, 0, 0, 1;
	this->forwardKinematics_.push_back(transFirstToSecond);
	this->desForwardKinematics_.push_back(transFirstToSecond);
	Matrix4d transSecondToThird;
	transSecondToThird << 1, 0, 0, 404,
						  0, 1, 0, 0,
						  0, 0, 1, 0,
						  0, 0, 0, 1;
	this->forwardKinematics_.push_back(transSecondToThird);
	this->desForwardKinematics_.push_back(transSecondToThird);
	Matrix4d transThirdToFourth;
	transThirdToFourth << 1, 0, 0, 404,
						  0, 1, 0, 0,
						  0, 0, 1, 0,
						  0, 0, 0, 1;
	this->forwardKinematics_.push_back(transThirdToFourth);
	this->desForwardKinematics_.push_back(transThirdToFourth);

	// Second, populate joint angle lists to be on the safe side
	this->jointAngles_.push_back(0.0);
	this->jointAngles_.push_back(0.0);
	this->jointAngles_.push_back(0.0);
	this->desJointAngles_.push_back(0.0);
	this->desJointAngles_.push_back(0.0);
	this->desJointAngles_.push_back(0.0);

	// Now, set all points for all body parts (points w.r.t to own reference frame in [mm])
	// also, set states that are known in advance (desVelocity, desAngVelocity always zero)
	// ToDo: Find out the rest of the points

	// rail body (body 0)
	struct body rail;
	rail.refSpringAttach << 0, 0, 0, 1;
	rail.refSpringDetach << -30, -54, -43, 1;
	rail.refSpringGuide << 106, -30, -45, 1;
	rail.refMuscleAttach << 0, 0, 0, 1;
	rail.refMuscleDetach << -52, 72, -43, 1;
	rail.refMuscleGuide << 0, 0, 0, 1;
	// just need the position of rail joint
	rail.refPosition << 0, 0, 0, 1;
	rail.refOrientation << 1, 0, 0,
						   0, 1, 0,
						   0, 0, 1;
	rail.refJointPosition_ = rail.refPosition;
	this->bodies_.push_back(rail);

	// upper arm (body 1)
	struct body upperArm;
	upperArm.refSpringAttach << -30, -53, 0, 1;
	upperArm.refSpringDetach << 280, -76, 0, 1;
	upperArm.refSpringGuide << 415, -45, 0, 1;
	upperArm.refMuscleAttach << 123, -22, 0, 1;
	upperArm.refMuscleDetach << 340, 10, 0, 1;
	upperArm.refMuscleGuide << 404, 15, 0, 1;
	upperArm.refJointPosition_ << 0, 0, 0, 1;
	// states
	upperArm.velocity << 0, 0, 0;
	upperArm.refPosition << 404, 0, 0, 1;
	upperArm.refOrientation << 1, 0, 0,
							   0, 1, 0,
							   0, 0, 1;
	upperArm.desVelocity << 0, 0, 0;
	upperArm.desAngVelocity << 0, 0, 0;
	this->bodies_.push_back(upperArm);

	// lower arm (body 2)
	struct body lowerArm;
	lowerArm.refSpringAttach << -30, -53, 0, 1;
	lowerArm.refSpringDetach << 271, -76, 0, 1;
	lowerArm.refSpringGuide << 415, -45, 0, 1;
	lowerArm.refMuscleAttach << 142, -22, 0, 1;
	lowerArm.refMuscleDetach << 340, 10, 0, 1;
	lowerArm.refMuscleGuide << 404, 15, 0, 1;
	lowerArm.refJointPosition_ << 0, 0, 0, 1;
	//states
	lowerArm.refPosition << 404, 0, 0, 1;
	lowerArm.refOrientation << 1, 0, 0,
							   0, 1, 0,
							   0, 0, 1;
	lowerArm.desVelocity << 0, 0, 0;
	lowerArm.desAngVelocity << 0, 0, 0;
	this->bodies_.push_back(lowerArm);

	// endeffector (body 3)
	struct body endeffector;
	endeffector.refSpringAttach << -20, -53, 0, 1;
	endeffector.refSpringDetach << 0, 0, 0, 1;
	endeffector.refSpringGuide << 0, 0, 0, 1;
	endeffector.refMuscleAttach << 28, 10, 0, 1;
	endeffector.refMuscleDetach << 0, 0, 0, 1;
	endeffector.refMuscleGuide << 0, 0, 0, 1;
	endeffector.refJointPosition_ << 0, 0, 0, 1;
	//states
	endeffector.refPosition << 169, 10, 0, 1;
	endeffector.refOrientation << 1, 0, 0,
							   0, 1, 0,
							   0, 0, 1;
	endeffector.desVelocity << 0, 0, 0;
	endeffector.desAngVelocity << 0, 0, 0;
	// also, the desired orientation for the endeffector is always PI/2 so it face down
	endeffector.refDesOrientation << cos(M_PI), -sin(M_PI), 0,
			   	   	   	   	   	     sin(M_PI), cos(M_PI), 0,
			   	   	   	   	   	     0, 0, 1;
	this->bodies_.push_back(endeffector);

	// Initialize muscles
	muscleUpperArm_ = new HillMuscleModel(130.0, 200.0);
	muscleLowerArm_ = new HillMuscleModel(150.0, 205);
	muscleEndeffector_ = new HillMuscleModel(60.0, 90.0);

	// Initialize springs
	// ToDo: spring rest force and muscle rest length need to be identified
	SpringMuscle springUpperArm = SpringMuscle(6.2361, 128.0, 0.0, 80.0);
	SpringMuscle springLowerArm = SpringMuscle(1.8630, 173.0, 0.0, 80.0);
	SpringMuscle springEndeffector = SpringMuscle(0.5240, 173.0, 0.0, 80.0);

	springMuscles_.push_back(springUpperArm);
	springMuscles_.push_back(springLowerArm);
	springMuscles_.push_back(springEndeffector);

	// Get joint limits populated
	this->maxAngles_.push_back(1.93);
	this->maxAngles_.push_back(1.39);
	this->maxAngles_.push_back(1.57);
	this->minAngles_.push_back(1.39);
	this->minAngles_.push_back(0.46);
	this->minAngles_.push_back(0.19);
}

RobotGeometry::~RobotGeometry() {
	delete muscleUpperArm_;
	delete muscleLowerArm_;
	delete muscleEndeffector_;
}


// Just a function to test stuff, not important
void RobotGeometry::bla() {
	/*
	for(std::list<double>::iterator itJointAngles = this->jointAngles_.begin();
			itJointAngles != this->jointAngles_.end(); itJointAngles++) {
		std::cout << *itJointAngles << std::endl;
	}
	*/
	/*
	for(std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> >::iterator itTrans =
			this->forwardKinematics_.begin(); itTrans != this->forwardKinematics_.end();
			itTrans++) {
		std::cout << "Current Orientation: "<< *itTrans << std::endl;
	}
	*/
	/*
	for(std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> >::iterator itDesTrans =
			this->desForwardKinematics_.begin(); itDesTrans != this->desForwardKinematics_.end();
			itDesTrans++) {
		std::cout << "Desired Orientation: " << *itDesTrans << std::endl;
	}
	*/

	for(std::list<double>::iterator itDesJointAngles = this->desJointAngles_.begin();
			itDesJointAngles != this->desJointAngles_.end(); itDesJointAngles++) {
		std::cout << "Desired joint angle: " << *itDesJointAngles << std::endl;
	}


	for(std::list<struct body>::iterator itBodies = this->bodies_.begin();
			itBodies != this->bodies_.end(); itBodies++) {
			std::cout << "Position of body designated point: " << (*itBodies).position << std::endl;
			std::cout << "Desired position of designated points: " << (*itBodies).desPosition << std::endl;
	}

	/*
	struct body itBodies = this->bodies_.back();
	std::cout << "Current Position: " << itBodies.position << std::endl;
	std::cout << "Desired Position: " << itBodies.desPosition << std::endl;
	*/
	/*
	for(std::list<struct body>::iterator itBodies = this->bodies_.begin();
			itBodies != this->bodies_.end(); itBodies++) {
			std::cout << "Orientation of body: " << (*itBodies).orientation << std::endl;
	}
	*/
	/*
	for(std::list<struct body>::iterator itBodies = this->bodies_.begin();
			itBodies != this->bodies_.end(); itBodies++) {
			std::cout << "Desired orientation of body: " << (*itBodies).desOrientation << std::endl;
	}
	*/
}

// Setters (Sets the difference between old and new joint angles)
void RobotGeometry::setJointAngles(std::list<double> jointAngles) {
	this->jointAngles_ = jointAngles;
}

// Updates transformation matrices based on joint states
void RobotGeometry::updateTransformationMatrices() {
	// First set iterator for joint Angles
	std::list<double>::iterator itJointAngles = this->jointAngles_.begin();

	// and set iterator for desired joint Angles
	std::list<double>::iterator itDesJointAngles = this->desJointAngles_.begin();

	// Now, update current rail joint position
	// ToDo: how to get current position of rail joint
	std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> >::iterator itTrans =
				this->forwardKinematics_.begin(); // iterator for transformation matrices

	std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> >::iterator itDesTrans =
				this->desForwardKinematics_.begin(); // iterator for desired transformation matrices

	// Overwrite x-offset with (row,coloumn) operator
	//Matrix4d test = *itTrans;
	(*itTrans)(2,3) = this->railJointPosition_ + 245; // ToDo: Reference to rail joint position calculation
	(*itDesTrans)(2,3) = this->railJointPosition_ + 245; // 245 is offset of rail joint in [mm]

	//std::cout << "transformation matrix: " << std::endl;
	//std::cout << *itTrans << std::endl;

	// Go the next transformation matrix
	++itTrans;
	++itDesTrans;

	// Now iterate over the remaining transformation matrices and update them with joint angles
	for(; itTrans != this->forwardKinematics_.end(); itTrans++) {

		// Update rotational values
		// Note: we just rotate over z-axis in all joints
		(*itTrans)(0,0) = cos(*itJointAngles);
		(*itTrans)(0,1) = -sin(*itJointAngles);
		(*itTrans)(1,0) = sin(*itJointAngles);
		(*itTrans)(1,1) = cos(*itJointAngles);
		// Somewhere 1s are written to positions (2,0), (3,0) so fix it here for now
		(*itTrans)(2,0) = 0;
		(*itTrans)(3,0) = 0;

		// and update desired transformation matrices
		(*itDesTrans)(0,0) = cos(*itDesJointAngles);
		(*itDesTrans)(0,1) = -sin(*itDesJointAngles);
		(*itDesTrans)(1,0) = sin(*itDesJointAngles);
		(*itDesTrans)(1,1) = cos(*itDesJointAngles);

		(*itDesTrans)(2,0) = 0;
		(*itDesTrans)(3,0) = 0;

		//std::cout << "transformation matrix: " << std::endl;
		//std::cout << *itTrans << std::endl;

		++itJointAngles;
		++itDesJointAngles;
		++itDesTrans;
	}
}

// Set desired joint angles based on user input and inverse kinematics
void RobotGeometry::setDesJointAngles(double desX, double desY) {
	// Analytical method, using geometric porperties and trigonometry

	// First define all helper variables to keep it manageable
	double l1 = 404; // length of upper arm in [mm]
	double l2 = 404; // length of lower arm in [mm]
	double l3 = 169; // length of endeffector in [mm]

	// Desired orientation of the endeffector is always PI so it faces down
	double desOrient = M_PI;

	// Next, calculate where wrist of endeffector is
	double wristX = desX -l3*cos(desOrient);
	double wristY = desY -l3*sin(desOrient);

	// Now, express wrist coordinates w.r.t. frame of first rotational joint
	wristX -= 714;
	wristY -= -50;

	// Now, get vector bewteen "root" joint and wrist
	// "root" joint is the first rotational joint, since we can do the inverese kinematics
	// just for the planar arm structure
	double powWristX = pow(wristX,2.0);
	double powWristY = pow(wristY,2.0);
	double rootToWrist = sqrt(powWristX+powWristY);

	// Get argument for acos function of upper arm and check it it is in limits
	double acosUpperArm = (pow(rootToWrist,2.0)+pow(l1,2.0)-pow(l2,2.0))/(2*rootToWrist*l1);
	if (acosUpperArm > 1.0) {
		acosUpperArm = 1.0;
	}
	else if (acosUpperArm < -1.0) {
		acosUpperArm = -1.0;
	}

	// For the angle of the first joint, we must substract the angle between the first joint and
	// the wrist of the endeffector with the angle inside the triangle
	double outerAngle = atan2(wristY, wristX);
	double innerAngle = acos(acosUpperArm);
	double angleUpperArm = outerAngle-innerAngle;

	// Check angle limits for upper arm
	std::list<double>::iterator itMaxAngles = this->maxAngles_.begin();
	std::list<double>::iterator itMinAngles = this->minAngles_.begin();
	if(angleUpperArm > *itMaxAngles) {
		angleUpperArm = *itMaxAngles;
	}
	else if(angleUpperArm < *itMinAngles) {
		angleUpperArm = *itMinAngles;
	}

	// Get argument for acos function and check it is in limits
	double acosLowerArm = (pow(l1,2.0)+pow(l2,2.0)-pow(rootToWrist,2.0))/(2*l1*l2);
	if(acosLowerArm > 1.0) {
		acosLowerArm = 1.0;
	}
	else if (acosLowerArm < -1.0) {
		acosLowerArm = -1.0;
	}

	// The angle of the second joint corresponds to the angle inside the triangle
	// described by the length l1, l2 and the vector rootToWrist substracted from pi
	double angleLowerArm = M_PI - acos(acosLowerArm);

	// check angle limtis for lower arm
	itMaxAngles++;
	itMinAngles++;
	if(angleLowerArm > *itMaxAngles) {
		angleLowerArm = *itMaxAngles;
	}
	else if(angleLowerArm < *itMinAngles) {
		angleLowerArm = *itMinAngles;
	}

	// And finally the joint angle for the endeffector, we just substract the first and second
	// joint angle from the desired orientation
	double endeffectorAngle = desOrient-(angleLowerArm+angleUpperArm);

	// check angle limtis for endeffector
	itMaxAngles++;
	itMinAngles++;
	if(endeffectorAngle > *itMaxAngles) {
		endeffectorAngle = *itMaxAngles;
	}
	else if(endeffectorAngle < *itMinAngles) {
		endeffectorAngle = *itMinAngles;
	}

	// Set values in container
	std::list<double>::iterator itDesJointAngles = this->desJointAngles_.begin();
	*itDesJointAngles = angleUpperArm;
	++itDesJointAngles;
	*itDesJointAngles = angleLowerArm;
	++itDesJointAngles;
	*itDesJointAngles = endeffectorAngle;
}

// Update all points on all bodies via forward kinematics
void RobotGeometry::updateBodies() {

	// Iterator for transformations
	std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> >::iterator itTrans =
				this->forwardKinematics_.begin();

	// Iterator for desired transformations
	std::list<Matrix4d, Eigen::aligned_allocator<Matrix4d> >::iterator itDesTrans =
				this->desForwardKinematics_.begin();
	Matrix4d desTransformation = *itDesTrans;

	// iterator for body part
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// container to gather current transformation matrix (depends on which body to transform)
	Matrix4d transformation = *itTrans;

	// Apply transformation to rail body points (all that are of interest)
	(*itBodies).position = transformation * (*itBodies).refPosition;
	(*itBodies).springDetach = transformation * (*itBodies).refSpringDetach;
	(*itBodies).springGuide = transformation * (*itBodies).refSpringGuide;
	(*itBodies).muscleDetach = transformation * (*itBodies).refMuscleDetach;
	(*itBodies).jointPosition_ = transformation * (*itBodies).refJointPosition_;

	// Go to next body
	++itBodies;

	// Get iterators over joint angles
	std::list<double>::iterator itJointAngles = jointAngles_.begin();
	std::list<double>::iterator itOldJointAngles = oldJointAngles_.begin();

	// Now iterate over the remaining bodies and apply transformations
	for(; itBodies != this->bodies_.end(); itBodies++) {
		// First update general transformation matrices
		++itTrans;
		transformation *= *itTrans;
		++itDesTrans;
		desTransformation *= *itDesTrans;
		//std::cout << *itDesTrans << std::endl;
		//std::cout << desTransformation << std::endl;
		// Now apply general transformation to body parts
		// ToDo: find out how to update velocity and angular velocity
		(*itBodies).position = transformation * (*itBodies).refPosition;
		(*itBodies).orientation = transformation.block<3,3>(0,0);
		(*itBodies).springAttach = transformation * (*itBodies).refSpringAttach;
		(*itBodies).springDetach = transformation * (*itBodies).refSpringDetach;
		(*itBodies).springGuide = transformation * (*itBodies).refSpringGuide;
		(*itBodies).muscleAttach = transformation * (*itBodies).refMuscleAttach;
		(*itBodies).muscleDetach = transformation * (*itBodies).refMuscleDetach;
		(*itBodies).muscleGuide = transformation * (*itBodies).refMuscleGuide;

		(*itBodies).jointPosition_ = transformation * (*itBodies).refJointPosition_;

		// and transformation to desired states
		(*itBodies).desPosition = desTransformation * (*itBodies).refPosition;
		(*itBodies).desOrientation = desTransformation.block<3,3>(0,0);

		// Also update angular momentum and velocity
		(*itBodies).angVelocity(2) = (*itJointAngles) - (*itOldJointAngles);
		double radius = (*itBodies).refMuscleAttach(0)/1000;
		//std::cout << "Radius is: " << radius << std::endl;
		(*itBodies).velocity = (*itBodies).angVelocity * radius;
		//std::cout << "Angular velocity: " << (*itBodies).angVelocity << std::endl;
		//std::cout << "Velocity: " << (*itBodies).velocity << std::endl;
		*itOldJointAngles = *itJointAngles;
		++itJointAngles;
		++itOldJointAngles;

		//std::cout << (*itBodies).refPosition << std::endl;
		//std::cout << (*itBodies).position << std::endl;
		//std::cout << (*itBodies).desPosition << std::endl;
	}
}

double RobotGeometry::getRailInput() {
	// Apply "polynomial" (is linear for some reason) to translate absolute position in WCF into
	// commands for low level position controller
	// coefficients experimentically evaluated
	return (-53.2263+0.1060*this->railJointPosition_);
}

void RobotGeometry::setRailJoint(double railInput) {
	// Enforce limits of rail joint (values are experimentically evaluated)
	if (railInput > 924) {
		railInput = 924;
	} else if (railInput < 171) {
		railInput = 171;
	}
	this->railJointPosition_ = railInput;
}

// Getter methods
Vector3d RobotGeometry::getBodyPosition(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).position.block<3,1>(0,0));
}

Vector3d RobotGeometry::getBodyVelocity(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).velocity);
}

Matrix3d RobotGeometry::getBodyOrientation(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).orientation);
}

Vector3d RobotGeometry::getBodyAngularVelocity(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).angVelocity);
}

Vector3d RobotGeometry::getDesBodyPosition(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).desPosition.block<3,1>(0,0));
}

Vector3d RobotGeometry::getDesBodyVelocity(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).desVelocity);
}

Matrix3d RobotGeometry::getDesBodyOrientation(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).desOrientation);
}

Vector3d RobotGeometry::getDesBodyAngularVeloctiy(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).desAngVelocity);
}

Vector3d RobotGeometry::getJointPosition(int bodyID) {
	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// Go to specified body
	std::advance(itBodies, bodyID);
	// return value
	return((*itBodies).jointPosition_.block<3,1>(0,0));
}


// set total muscle length here
void RobotGeometry::setTotalMuscleLengths() {
	// Value storage
	Vector3d upperArmDetach;
	Vector3d upperArmAttach;
	Vector3d lowerArmDetach;
	Vector3d lowerArmAttach;
	Vector3d endeffectorDetach;
	Vector3d endeffectorAttach;

	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();

	// Get attachment/detachment points depending on current body
	upperArmDetach = (*itBodies).muscleDetach.block<3,1>(0,0);
	itBodies++;
	upperArmAttach = (*itBodies).muscleAttach.block<3,1>(0,0);
	lowerArmDetach = (*itBodies).muscleDetach.block<3,1>(0,0);
	itBodies++;
	lowerArmAttach = (*itBodies).muscleAttach.block<3,1>(0,0);
	endeffectorDetach = (*itBodies).muscleDetach.block<3,1>(0,0);
	itBodies++;
	endeffectorAttach = (*itBodies).muscleAttach.block<3,1>(0,0);

	// Calculate muscle length
	double lengthUpperArm = (upperArmAttach-upperArmDetach).norm();
	double lengthLowerArm = (lowerArmAttach-lowerArmDetach).norm();
	double lengthEndeffector = (endeffectorAttach-endeffectorDetach).norm();

	// Set total muscle length on model
	this->muscleUpperArm_->setTotalMuscleLength(lengthUpperArm);
	this->muscleLowerArm_->setTotalMuscleLength(lengthLowerArm);
	this->muscleEndeffector_->setTotalMuscleLength(lengthEndeffector);
}


Vector3d RobotGeometry::getMomentArm(int bodyID) {
	/*
	 * Okay, the moment arm of a muscle is defined by the section of the muscle that crosses
	 * the joint. That means, that for joint k belonging to body k the muscle detaches from body
	 * k-1 and attaches to body k.
	 */

	// Get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();

	// Now go to body k-1 to get muscle detachment point and muscle guide
	std::advance(itBodies, bodyID-1);
	Vector3d muscleDetachment = (*itBodies).muscleDetach.block<3,1>(0,0);
	//std::cout << "Muscle detachment of body " << bodyID-1 << " is: " << muscleDetachment << std::endl;

	// Next, go to body k to get muscle attachment point
	std::advance(itBodies, 1);
	Vector3d muscleAttachment = (*itBodies).muscleAttach.block<3,1>(0,0);
	Vector3d jointPosition = (*itBodies).jointPosition_.block<3,1>(0,0);
	Vector3d muscleSegment = muscleAttachment - muscleDetachment;

	//std::cout << "Muscle Attachment of body " << bodyID << " is: " << muscleAttachment << std::endl;
	//std::cout << "Muscle Segment of body " << bodyID << " is: " << muscleSegment << std::endl;
	//std::cout << "Joint Position of body " << bodyID << " is: " << jointPosition << std::endl;

	Vector3d momentArm = -(muscleAttachment-jointPosition).cross(muscleSegment/muscleSegment.norm());
	//std::cout << "Moment arm of muscle " << bodyID << " is: " << momentArm << std::endl;
	return(momentArm);
}

// set length of spring muscle
void RobotGeometry::setSpringMuscleLength() {
	// Iterators
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	std::list<double>::iterator itJointAngles = this->jointAngles_.begin();
	std::list<SpringMuscle>::iterator itSpringMuscles = this->springMuscles_.begin();
	for(; itSpringMuscles != this->springMuscles_.end(); itSpringMuscles++) {
		Vector3d springDetachment = (*itBodies).springDetach.block<3,1>(0,0);
		//std::cout << "Spring Detachment: " << springDetachment << std::endl;
		Vector3d springGuide = (*itBodies).springGuide.block<3,1>(0,0);
		//std::cout << "Spring Guide: " << springGuide << std::endl;
		std::advance(itBodies, 1);
		Vector3d springAttachment = (*itBodies).springAttach.block<3,1>(0,0);
		//std::cout << "Spring Attachment: " << springAttachment << std::endl;
		std::list<double>::iterator itJointAngles = this->jointAngles_.begin();
		double springLength = 0.0;
		// Check if spring guide comes into action
		// ToDo: Check angle on system
		if(*itJointAngles > 1.22) {
			double firstSegment = (springGuide - springDetachment).norm();
			double secondSegment = (springAttachment - springGuide).norm();
			springLength = (firstSegment+secondSegment);
		}
		else {
			springLength = (springAttachment - springDetachment).norm();
		}
		(*itSpringMuscles).setSpringMuscleLength(springLength);
		itJointAngles++;
		//std::cout << "Length of spring is: " << springLength << std::endl;
	}
}


// Get moment arm of springs
Vector3d RobotGeometry::getSpringMomentArm(int springID) {
	// get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// go the body k-1 for detachment point
	std::advance(itBodies, springID-1);
	Vector3d springDetachment = (*itBodies).springDetach.block<3,1>(0,0);
	Vector3d springGuide = (*itBodies).springGuide.block<3,1>(0,0);
	// advance to next body
	std::advance(itBodies, 1);
	Vector3d springAttachment = (*itBodies).springAttach.block<3,1>(0,0);
	Vector3d jointPosition = (*itBodies).jointPosition_.block<3,1>(0,0);

	Vector3d springSegment = springAttachment - springDetachment; // default, when spring guide not in action
	// At a specific angle, the spring guide comes into action,
	// changing the effective spring segment spanning over the joint
	std::list<double>::iterator itJointAngles = this->jointAngles_.begin();
	std::advance(itJointAngles, springID-1);
	// ToDo: check angle conditions on system
	if ((springID == 2) && (*itJointAngles > 1.22)) {
		springSegment = springAttachment - springGuide;
	}
	else if ((springID == 3) && (*itJointAngles > 1.22)) {
		springSegment = springAttachment - springGuide;
	}

	Vector3d springMomentArm = -(springAttachment-jointPosition).cross(springSegment/springSegment.norm());
	return(springMomentArm);
}

double RobotGeometry::getRealSpringMomentArm(int springID) {
	// get pointer on body list
	std::list<struct body>::iterator itBodies = this->bodies_.begin();
	// go the body k-1 for detachment point
	std::advance(itBodies, springID-1);
	Vector3d springDetachment = (*itBodies).springDetach.block<3,1>(0,0);
	Vector3d springGuide = (*itBodies).springGuide.block<3,1>(0,0);
	// advance to next body
	std::advance(itBodies, 1);
	Vector3d springAttachment = (*itBodies).springAttach.block<3,1>(0,0);
	Vector3d jointPosition = (*itBodies).jointPosition_.block<3,1>(0,0);

	//std::cout << "spring attachment: " << springAttachment << std::endl;
	//std::cout << "spring detachment: " << springDetachment << std::endl;

	Vector3d springSegment = springAttachment - springDetachment; // default, when spring guide not in action
	// At a specific angle, the spring guide comes into action,
	// changing the effective spring segment spanning over the joint
	std::list<double>::iterator itJointAngles = this->jointAngles_.begin();
	std::advance(itJointAngles, springID-1);
	// ToDo: check angle conditions on system
	if ((springID == 2) && (*itJointAngles > 1.22)) {
		springSegment = springAttachment - springGuide;
	}
	else if ((springID == 3) && (*itJointAngles > 1.22)) {
		springSegment = springAttachment - springGuide;
	}
	//std::cout << "joint position: " << jointPosition << std::endl;
	//std::cout << "springSegment: " << springSegment << std::endl;
	double realMomentArm = (springSegment.cross(jointPosition)).norm();
	return(realMomentArm);
}

// Return maxIsometForce for muscle with "muscleID"
double RobotGeometry::getMuscleMaxIsometForce(int muscleID) {
	double maxIsometForce = 1.0;
	if(muscleID == 1) {
		maxIsometForce = this->muscleUpperArm_->getMaxIsometForce();
	}
	if(muscleID == 2) {
		maxIsometForce = this->muscleLowerArm_->getMaxIsometForce();
	}
	if(muscleID == 3) {
		maxIsometForce = this->muscleEndeffector_->getMaxIsometForce();
	}
	return (maxIsometForce);
}

// Set activationLevel for corresponding muscle
void RobotGeometry::setMuscleActivationLevel(int muscleID, double activationLevel) {
	if(muscleID == 1) {
		this->muscleUpperArm_->setActivation(activationLevel);
	}
	if(muscleID == 2) {
		this->muscleLowerArm_->setActivation(activationLevel);
	}
	if(muscleID == 3) {
		this->muscleEndeffector_->setActivation(activationLevel);
	}
}

// Get total muscle force of muscle
double RobotGeometry::getTotalMuscleForce(int muscleID) {
	double muscleForce = 0.0;
	//std::cout << "Updates for muscle " << muscleID << std::endl;
	if(muscleID == 1) {
		muscleForce = this->muscleUpperArm_->getTotalMuscleForce();
	}
	if(muscleID == 2) {
		muscleForce = this->muscleLowerArm_->getTotalMuscleForce();
	}
	if(muscleID == 3) {
		muscleForce = this->muscleEndeffector_->getTotalMuscleForce();
	}
	return (muscleForce);
}

double RobotGeometry::getTotalSpringForce(int springID) {
	std::list<SpringMuscle>::iterator itSprings = this->springMuscles_.begin();
	std::advance(itSprings, springID-1);
	return((*itSprings).getSpringForce());
}

// Init joint angles
void RobotGeometry::initJointAngles(std::list<double> jointAngles) {
	this->jointAngles_ = jointAngles;
	this->oldJointAngles_ = jointAngles;
}
