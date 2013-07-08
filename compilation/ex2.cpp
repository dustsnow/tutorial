/**
 * @file ex2.cpp
 * @author Can Erdogan
 * @date June 06, 2013
 * @brief This file is the second example of compilation errors and focuses more on class 
 * definitions and pointers.
 * The main idea here is A has a class member B which has a value. A also has two values, one
 * given by the user and the other is fixed. We want to increment B's value with A's two values.
 * The problems in this example are related to: loopy includes, private variables, pointers,
 * casting, conflicting function definitions, default constructors and constants.
 */

#include "helper1.h"
#include "helper2.h"
#include <assert.h> 

/// The main thread
int main () {

	// Create an instance of class A and get its B value
	A a(1);
	B* b = a.getB();

	// // // Increment the value of B with A's values.
	a.incrementBsValue();
	// // // double sum = b->getB()->val + a.val2 + a.val3;
	double sum = b->val + a.getVal1() + a.getVal2();

	// // // After adding up all the values, the end value of B should be 6
	assert((sum == 6) && "Something went wrong!");
}
