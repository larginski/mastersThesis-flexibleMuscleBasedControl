#include "HillMuscleModel.h"

// Constructors
HillMuscleModel::HillMuscleModel() {
	// ToDo: Adjust values when they become available
	this->totalMuscleLength_ = 0.0;
	this->optLengthCE_ = 50.0;
	this->minLengthCE_ = 20.0;
	this->maxLengthCE_ = 150.0;
	this->restLengthSEE_ = 10.0;
	this->maxIsometForce_ = 200.0;
	this->maxVel_ = -10.0;
	this->tendonReferenceStrain_ = 5.0;
}

HillMuscleModel::HillMuscleModel(double minTotalLength, double maxTotalLength) {
	// Values are input from measurment on massageboy
	this->minTotalLength_ = minTotalLength;
	std::cout << "Min total length: " << this->minTotalLength_ << std::endl;
	this->maxTotalLength_ = maxTotalLength;
	std::cout << "Max total length: " << this->maxTotalLength_ << std::endl;
	// Set total muscle length to maxTotalLength as default
	this->totalMuscleLength_ = maxTotalLength;
	std::cout << "Total length: " << this->totalMuscleLength_ << std::endl;
	// Optimael length of CE is (for now) 2/3 of maxTotalLength
	this->optLengthCE_ = (maxTotalLength-minTotalLength)/4;
	std::cout << "Optimal CE length: " << this->optLengthCE_ << std::endl;
	this->minLengthCE_ = this->optLengthCE_ - (this->optLengthCE_/10);
	std::cout << "Min CE length: " << this->minLengthCE_ << std::endl;
	this->maxTotalLength_ = this->optLengthCE_ + (this->optLengthCE_/10);
	std::cout << "Max CE length: " << this->maxTotalLength_ << std::endl;
	// resting length of SEE is rest of total muscle length after setting optimal CE length
	this->restLengthSEE_ = maxTotalLength - this->optLengthCE_;
	std::cout << "Resting length of SEE: " << this->restLengthSEE_ << std::endl;
	this->maxIsometForce_ = 300.0;
	std::cout << "Maximal Isometric Force: " << this->maxIsometForce_ << std::endl;
	this->maxVel_ = -10.0;
	std::cout << "Max velocity: " << this->maxVel_ << std::endl;
	this->tendonReferenceStrain_ = 5.0;
	std::cout << "Tendon Reference strain: " << this->tendonReferenceStrain_ << std::endl;
}

// Destructor
HillMuscleModel::~HillMuscleModel() {
}

/// INPUTS FROM SYSTEM ///
// Get total muscle length from system
void HillMuscleModel::setTotalMuscleLength(double totalMuscleLength) {
	this->totalMuscleLength_ = totalMuscleLength;
	//std::cout << "Total muscle length: " << this->totalMuscleLength_ << std::endl;
}
// Set activation level from controller output
void HillMuscleModel::setActivation(double activation) {
	// enforce limits to activation level
	if(activation > 1.0) {
		activation = 1.0;
	}
	else if (activation < 0.0){
		activation = 0.0;
	}
	// set activation level
	this->activation_ = activation;
}

/// INTERNAL UPDATES ///
// Dynamics functions of CE
double HillMuscleModel::forceLengthRelation() {
	/*
	// Parameters
	double residualForce = log(0.05);
	double bellWidth = 0.4*this->optLengthCE_;
	double retValue = exp(residualForce*(abs(pow(((this->lengthCE_-this->optLengthCE_)/(this->optLengthCE_*bellWidth)),3.0))));
	//std::cout << "Force-Length outputs: " << retValue << std::endl;
	return(retValue);
	*/
	double resForce = log(0.05); // residual force factor
	double lce = this->lengthCE_/this->optLengthCE_; // normalized CE length
	double retValue = exp(resForce*pow((lce-1)/0.56,4)); // 0.56 is the width of the bell-shaped curve
	//std::cout << "Force-Length-Relation is: " << retValue << std::endl;
	return(retValue);
}
double HillMuscleModel::forceVelocityRelation() {
	/*
	 *  Notes: the curvature constant K = 5, eccentric force enhancement = 1.5,
	 *         7.56 is an additional scalar used
	 */
	double vce = this->contractionVel_/this->optLengthCE_; // normalized velocity (does this make sense?)
	double retValue = 0.0;
	if(vce < 0.0) {
		retValue = (-10 - vce)/(-10 + 5*vce);
	}
	else {
		retValue = 1.5 + 0.5*(-10+vce)/(37.8*vce+10);
	}
	//std::cout << "Force-Velocity Realtion is: " << retValue << std::endl;
	return (retValue);
}
// Update length of SEE
void HillMuscleModel::setLenghtSEE() {
	this->lengthSEE_ = this->totalMuscleLength_ - this->lengthCE_;
	//std::cout << "Length of SEE is: " << this->lengthSEE_ << std::endl;
}
// Set CE force
void HillMuscleModel::setCEForce() {
	this->forceCE_ = this->activation_ * this->maxIsometForce_
			* forceLengthRelation() * forceVelocityRelation();
	//std::cout << "CE force is: " << this->forceCE_ << std::endl;
}
// Set SEE force
void HillMuscleModel::setSEEForce() {
	/*
	double tendonStrain = (this->lengthSEE_ - this->restLengthSEE_)/this->restLengthSEE_;
	//std::cout << "Tendon strain is: " << tendonStrain << std::endl;
	if(tendonStrain > 0.0) {
		this->forceSEE_ = pow((tendonStrain/this->tendonReferenceStrain_),2.0);
	}
	else {
		this->forceSEE_ = 0.0;
	}
	*/
	double lsee = this->lengthSEE_/this->restLengthSEE_; // normalized see length
	this->forceSEE_ = this->maxIsometForce_ * pow((lsee-1)/(0.04),2);
	//std::cout << "SEE force is: " << this->forceSEE_ << std::endl;
}
// Set PEE force
void HillMuscleModel::setPEEForce() {
	/*
	double peeStrain = (this->lengthCE_ - this->optLengthCE_)/this->optLengthCE_;
	//std::cout << "PEE strain is: " << peeStrain << std::endl;
	if(peeStrain > this->lengthCE_) {
		this->forcePEE_ = this->maxIsometForce_ * pow((this->lengthCE_ - this->optLengthCE_)
				/(this->optLengthCE_*peeStrain),2.0);
	}
	else {
		this->forcePEE_ = 0.0;
	}
	*/
	double lce = this->lengthCE_/this->optLengthCE_; // normalized CE length
	double f_hpe = this->maxIsometForce_ * pow((lce-1)/(0.56),2);
	double f_lpe = this->maxIsometForce_ * pow((0.44-lce)/(0.28),2);
	this->forcePEE_ = f_hpe - f_lpe;
	//std::cout << "PEE force: " << this->forcePEE_ << std::endl;
}
// Update the length of the CE
void HillMuscleModel::updateLengthCE() {
	// First, calculate f_v(V_CE) in terms of forces
	if(this->activation_ < 0.0) {
		this->activation_ = 0.0001; // Can not let a fall below 0
	}
	double f_v = (this->forceSEE_ - this->forcePEE_)/
			(this->activation_*this->maxIsometForce_*forceLengthRelation());
	// Then, take f_v as input for inverted f_v function to get v_ce
	double v_ce = (-10 + 10*f_v)/(5*f_v + 1) * this->optLengthCE_;
	//std::cout << "Contraction velocity is: " << v_ce << std::endl;
	// Set v_ce
	setContractionVelocity(v_ce);
	// Update l_ce
	this->lengthCE_ += v_ce;
	// Limits
	if(this->lengthCE_ < this->minLengthCE_) {
		this->lengthCE_ = this->minLengthCE_;
	}
	else if(this->lengthCE_ > this->maxLengthCE_) {
		this->lengthCE_ = this->maxLengthCE_;
	}
	//std::cout << "CE length after update: " << this->lengthCE_ << std::endl;
}
// Set current contraction velocity
void HillMuscleModel::setContractionVelocity(double contractionVelocity) {
	this->contractionVel_ = contractionVelocity;
}
void HillMuscleModel::updateAll() {
	// Update all internal parameters in the right order
	// First, update length of SEE
	HillMuscleModel::setLenghtSEE();
	// Then, calculate forces of all muscle elements
	HillMuscleModel::setSEEForce();
	HillMuscleModel::setPEEForce();
	HillMuscleModel::setCEForce();
	// Lastly, update length of CE
	HillMuscleModel::updateLengthCE();
}

/// HILL MUSCLE OUTPUT ///
// This function returns the current force of the muscle and calls the internal updates
double HillMuscleModel::getTotalMuscleForce() {
	HillMuscleModel::updateAll();
	double retValue = this->forceCE_ + this->forcePEE_;
	if(retValue < 0.0) {
		retValue = 0.0;
	}
	return (retValue);
}

double HillMuscleModel::getMaxIsometForce() {
	return (this->maxIsometForce_);
}
