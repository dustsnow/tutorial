/**
 * @file ex2.cpp
 * @author Can Erdogan
 * @date June 06, 2013
 * @brief This file is the second example of runtime errors, specifically focusing on asserting
 * errors, infinite loops and stack overflows.
 * The idea is to implement inverse Jacobian control to move the gripper (hand) of a two-link
 * robotic arm to a location. Let th1 and th2 be the two motors and move the two links of the 
 * robot. Then, the end-effector position (x,y) for link lengths l1 and l2 can be written as:
 * x = l1*cos(th1) + l2*cos(th1+th2), y = l1*sin(th1) + l2*sin(th1+th2). The method is given
 * a location (xi,yi) of the end-effector at time i, we compute the displacement (dx,dy) towards 
 * (x,y) and then get the necessary displacement in the joint space (dth1, dth2) through the
 * Jacobian (google it!). 
 * Note that we will use the symbol x for the position vector (x,y) and q for the joint values
 * (th1, th2).
 */

#include <stdlib.h>
#include <iostream>
#include <vector>
#include </usr/include/eigen3/Eigen/Dense>
#include <fstream>

using namespace std;
using namespace Eigen;

const double l1 = 1.0, l2 = 2.0;		///< The link lengths
const double smallLength = 0.05;		///< The small motion towards the goal
VectorXd xGoal;							///< The goal value for the end-effector
vector <VectorXd> joints;		///< The joint values that take the end-effector from start to goal

/// Makes a small displacement towards the goal
void moveTowardsGoal (VectorXd q) {

	static int asdf = 0;
	
	// Compute the current location of the end-effector (gripper/hand)
	VectorXd xi (2);
	xi << l1*cos(q(0)) + l2*cos(q(0)+q(1)), l1*sin(q(0)) + l2*sin(q(0)+q(1)); 

	// Compute the displacement in the workspace
	VectorXd dx = xGoal - xi;

	// Check if the goal is reached
	double dist = dx.norm();
	if(dist < 1e-1) return;
	else joints.push_back(q);

	// Get the Jacobian: take the derivative of x and y values with respect to th1 and th2.
	MatrixXd J (2, 2);
	J << (-l1*sin(q(0)) - l2*sin(q(0)+q(1))), (-l2*sin(q(0)+q(1))),
       ( l1*cos(q(0)) + l2*cos(q(0)+q(1))), ( l2*cos(q(0)+q(1)));

	// Compute the direction in the joint space and compute the small motion on it
	VectorXd dir = (J.inverse() * dx).normalized();
	dir *= smallLength;
	
	// Carry out the motion and attempt to make a new small move
	VectorXd qNew = q + dir;
	moveTowardsGoal(qNew);
}

/// Print the results into a file
void printTrajectory () {

	// Print the path
	ofstream out ("path_ex2", ofstream::out);
	for(int i = joints.size()-1; i >= 0; i--)
		out << joints[i].transpose() << endl;
}

/// The main thread
int main () {

	// Initialize the goal location
	xGoal = VectorXd (2);
	xGoal << -0.5, 2.0;

	// Initialize the initial joint value
	VectorXd q0 (2);
	q0 << 0.1, 0.2;

	// Try to execute Jacobian control
	moveTowardsGoal(q0);

	// Print the trajectory
	printTrajectory();
}
