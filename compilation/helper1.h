/**
 * @file helper1.h
 * @author Can Erdogan
 * @date June 06, 2013
 * @brief Contains the class definition of A for examples 2.
 * What is the difference between a static and normal field member? Google!
 * NOTE: Do not change the public/private definitions of members. That is usually a simple
 * solution/hack but would defeat the purpose of the exercise.
 */
// #ifndef HELPER1_H
// #define HELPER1_H

#include "helper2.h"

/// Forward declare class B
// struct B;

/// The definition of class A
class A {

	B* b;																	///< The B instance
	int val2;															///< The input value

  // How can we declare a static variable in a class? Do not make this variable a const for the
	// purpose of for this exercise.
	static int val1;									///< The fixed value 
public:

	/// The constructor with a parameter
	A (int input) : val2(input) {
		b = new struct B;
	}

	// Add a const here
	void incrementBsValue (); 						///< Increment Bs value

	// The getter function that returns private variables
	int getVal1 () const;									///< Returns val2 - the const at the end means that this file does 
																				///  not change the variables of this class
	int getVal2 ();												///< Returns val2
	B* getB () { return b; }							///< Returns the local instance of B
};


// #endif