#include "SpringMuscle.h"

// default constructor
SpringMuscle::SpringMuscle() {
}

// custom constructor
SpringMuscle::SpringMuscle(double springCoefficient, double springRestLength, double springRestForce, double muscleRestLength) {
	this->springCoefficient_ = springCoefficient;
	this->springRestLength_ = springRestLength;
	this->springRestForce_ = springRestForce;
	this->muscleRestLength_ = muscleRestLength;
}

// default destructor
SpringMuscle::~SpringMuscle() {
}

// sets spring length
void SpringMuscle::setSpringMuscleLength(double length) {
	this->muscleLength_ = length;
}

// get spring force
double SpringMuscle::getSpringForce() {
	double displacement = (this->muscleLength_ - this->muscleRestLength_);
	double springForce = this->springCoefficient_ * displacement;
	//std::cout << "Real force on spring is: " << springForce << std::endl;
	return(springForce);
}
