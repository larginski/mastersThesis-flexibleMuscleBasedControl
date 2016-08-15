#ifndef SPRINGMUSCLE_H_
#define SPRINGMUSCLE_H_

#include <Eigen/Dense>
#include <iostream>

class SpringMuscle {
public:
	// default constructor
	SpringMuscle();
	// custom constructor
	SpringMuscle(double springCoefficient, double springRestLength, double springRestForce, double muscleRestLength);
	// default destructor
	virtual ~SpringMuscle();

	// Setter
	void setSpringMuscleLength(double length);

	// Getter
	double getSpringForce();

private:
	double springRestLength_; // initial length of spring (constant)
	double springRestForce_; // initial force of spring depending on initial length (constant)

	double muscleRestLength_; // initial length of passive spring muscle (constant)
	double muscleLength_; // length of spring depending on current geometric configuration

	double springCoefficient_; // coefficient to calculate spring force form displacement (constant)
};

#endif /* SPRINGMUSCLE_H_ */
