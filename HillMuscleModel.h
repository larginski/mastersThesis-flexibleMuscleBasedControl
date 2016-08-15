#ifndef HILLMUSCLEMODEL_H_
#define HILLMUSCLEMODEL_H_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>

class HillMuscleModel {
public:
	// Default constructor
	HillMuscleModel();
	// "Special" constructor
	HillMuscleModel(double minTotalLength, double maxTotalLength);
	// Destructor
	virtual ~HillMuscleModel();

	/// INPUTS FROM SYSTEM ///
	// Sets the total muscle length depending on the current geometry of the robot
	void setTotalMuscleLength(double totalMuscleLength);
	// Sets the activation level depending on controller output
	void setActivation(double activation);

	/// INTERNAL UPDATES ///
	// Master function to call all other update functions
	void updateAll();
	// Sets the length of the SEE depending on the current length of the CE
	void setLenghtSEE();
    // Update of the CE length depending on the current contraction velocity
	void updateLengthCE();
	// Set force of CE
	void setCEForce();
	// Set force of SEE
	void setSEEForce();
	// Set force of PEE
	void setPEEForce();
	// Dynamics of CE
	double forceLengthRelation();
	double forceVelocityRelation();
	void setContractionVelocity(double contractionVelocity);

	/// HILL MUSCLE OUTPUT ///
	// Get total muscle force
	double getTotalMuscleForce();
	double getMaxIsometForce();

private:
	// Length values
	double totalMuscleLength_; // input
	double lengthCE_; // inner DoF
	double optLengthCE_; // constant parameter
	double lengthSEE_; // inner DoF
	double restLengthSEE_; // constant parameter
	double minLengthCE_; // constant parameter
	double maxLengthCE_; // constant parameter
	double minTotalLength_; // constant parameter
	double maxTotalLength_; // constant parameter

	// Force values
	double maxIsometForce_; // constant parameter
	double forceCE_; // inner DoF
	double forceSEE_; // inner DoF
	double forcePEE_; // inner DoF

	// Velocity values
	double contractionVel_; // ???
	double maxVel_; // constant parameter

	// Strain values
	double tendonReferenceStrain_; // constant parameter

	// Activation value
	double activation_; // input
};

#endif /* HILLMUSCLEMODEL_H_ */
