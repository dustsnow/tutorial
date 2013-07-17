/**
 * @file ex.cpp
 * @author Can Erdogan
 * @date June 07, 2013
 * @brief This file shows an example usage of a testing library, GTest, to ensure that a
 * collision library, FCL, performs as expected.
 */

#include <gtest/gtest.h>
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/math/transform.h"
#include "fcl/collision_data.h"
#include "fcl/collision_object.h"
#include <iostream>
#include <cmath>
#include </usr/include/eigen3/Eigen/Dense>

using namespace std;
using namespace fcl;
using namespace Eigen;

#define PI 3.14159
#define theta PI/4

FCL_REAL *dist = new FCL_REAL;
Vec3f contact_point[3];
bool collision;

/// The collision detector
GJKSolver_libccd solver;

MatrixXd rotation_result(3,3);

TEST(EXAMPLE, RectRectRotation) {
    /* 
     * Test of collision of two rectangle 
     */
     fcl::Matrix3f rotation_matrix(cos(theta), -1*sin(theta),0,sin(theta),cos(theta),0,0,0,1);
  fcl::Matrix3f rotation_test;
    Box two_rect_box1(2.0,2.0,0.0);
    Box two_rect_box2(2.0,2.0,0.0);
  
    //// Barely miss
    Transform3f tftwo_rect_box1 (Vec3f(0.0, 0.0, 0.0));

    tftwo_rect_box1.setRotation(rotation_matrix);

    Transform3f tftwo_rect_box2 (Vec3f(2.8283, 0.0, 0.0));

    tftwo_rect_box2.setRotation(rotation_matrix);

    collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box1, two_rect_box2, tftwo_rect_box2, contact_point,dist,NULL);
    EXPECT_TRUE(collision);
    ////// Touch
    //Transform3f tftwo_rect_box3 (Vec3f(0.0, 0.0, 0.0));
    //Transform3f tftwo_rect_box4 (Vec3f(2.0, 2.0, 0.0));
    //collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box3, two_rect_box2, tftwo_rect_box4, contact_point,dist,NULL);
    //EXPECT_TRUE(collision);
    ////// Collide
    //Transform3f tftwo_rect_box5 (Vec3f(0.0, 0.0, 0.0));
    //Transform3f tftwo_rect_box6 (Vec3f(1.9, 0.0, 0.0));
    //collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box5, two_rect_box2, tftwo_rect_box6, contact_point,dist,NULL);
    //EXPECT_TRUE(collision);
  
  
  
}
TEST(EXAMPLE, RectCircle) {
  /* 
   * Test the intersection a rectangle and circle 
   */
  //// Barelly Missed
  Box rect_circ_box1(2.0,2.0,2.0);
  Sphere rect_circ_s1(1); 
  Transform3f tfrect_circ_box1 (Vec3f(0.0, 0.0, 0.0));
  Transform3f tfrect_circ_s1 (Vec3f(2.1, 0.0, 0.0));
  collision = solver.shapeIntersect(rect_circ_box1, tfrect_circ_box1, rect_circ_s1, tfrect_circ_s1, NULL,NULL,NULL);
  EXPECT_FALSE(collision);

  //// Touch
  Transform3f tfrect_circ_s1_2 (Vec3f(1.999, 0.0, 0.0));
  collision = solver.shapeIntersect(rect_circ_box1, tfrect_circ_box1, rect_circ_s1, tfrect_circ_s1_2, NULL,NULL,NULL);
  EXPECT_TRUE(collision);


  //// Collide
  Transform3f tfrect_circ_s1_3 (Vec3f(1.0, 0.0, 0.0));
  collision = solver.shapeIntersect(rect_circ_box1, tfrect_circ_box1, rect_circ_s1, tfrect_circ_s1_3, NULL,NULL,NULL);
  EXPECT_TRUE(collision);

}
TEST(EXAMPLE, RectRect) {
  /* 
   * Test of collision of two rectangle 
   */
  Box two_rect_box1(2.0,2.0,0.0);
  Box two_rect_box2(2.0,2.0,0.0);

  //Transform3f tftwo_rect_box1 (Vec3f(0.0, 0.0, 0.0));
  //Transform3f tftwo_rect_box2 (Vec3f(2.1, 0.0, 0.0));
  ////tftwo_rect_box2.setRotation(Matrix3f(Vec3f(),Vec3f(),Vec3f()));
  //collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box1, two_rect_box2, tftwo_rect_box2, contact_point,dist,NULL);
  //EXPECT_FALSE(collision);


  //// Barely miss
  Transform3f tftwo_rect_box1 (Vec3f(0.0, 0.0, 0.0));
  Transform3f tftwo_rect_box2 (Vec3f(2.1, 0.0, 0.0));
  collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box1, two_rect_box2, tftwo_rect_box2, contact_point,dist,NULL);
  EXPECT_FALSE(collision);
  //// Touch
  Transform3f tftwo_rect_box3 (Vec3f(0.0, 0.0, 0.0));
  Transform3f tftwo_rect_box4 (Vec3f(2.0, 2.0, 0.0));
  collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box3, two_rect_box2, tftwo_rect_box4, contact_point,dist,NULL);
  EXPECT_TRUE(collision);
  //// Collide
  Transform3f tftwo_rect_box5 (Vec3f(0.0, 0.0, 0.0));
  Transform3f tftwo_rect_box6 (Vec3f(1.9, 0.0, 0.0));
  collision = solver.shapeIntersect(two_rect_box1, tftwo_rect_box5, two_rect_box2, tftwo_rect_box6, contact_point,dist,NULL);
  EXPECT_TRUE(collision);


}

//TEST(EXAMPLE, SphereSphere) {
//
//  // Test two sphere intersect
//  Sphere s1(1);
//  Sphere s2(1);
//  Transform3f trans1 (Vec3f(0.0, 0.0, 0.0));
//  Transform3f trans2 (Vec3f(2.0, 0, 0.0));
//  collision = solver.shapeIntersect(s1, trans1, s2, trans2,contact_point,dist,NULL);
//  EXPECT_FALSE(collision);
//}

/// Testing triangle sphere collisions at different configurations.
TEST(EXAMPLE, TriangleSphere) {

// Create the sphere with radius 10 and equilateral triangle with 20 side length
  Sphere s1(1);
  Vec3f t1[3];
  t1[0].setValue(0, 0, 0);
  t1[1].setValue(1.5, 0, 0);
  t1[2].setValue(0, 1.5, 0);

	// Set the positions of the sphere so they barely miss (see test1.png)
  Transform3f trans (Vec3f(1.5, 1.5, 0.0));
  collision = solver.shapeTriangleIntersect(s1, trans, t1[0], t1[1], t1[2], NULL, NULL, NULL);
  EXPECT_FALSE(collision);

	// Now move the sphere a bit to the left/bottom and see if they collide
	Transform3f trans2 (Vec3f(1.45, 1.45, 0.0));
  collision = solver.shapeTriangleIntersect(s1, trans2, t1[0], t1[1], t1[2], NULL, NULL, NULL);
	EXPECT_TRUE(collision);

	// Move the sphere to the bottom of the triangle and collide with the corner
	Transform3f trans3 (Vec3f(-0.7, -0.7, 0.0));
  collision = solver.shapeTriangleIntersect(s1, trans3, t1[0], t1[1], t1[2], NULL, NULL, NULL);
	EXPECT_TRUE(collision);

}

/// The main thread runs the tests
int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
