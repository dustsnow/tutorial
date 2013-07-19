/**
 * @file rrt-2-link-arm.cpp
 * @author Peng Hou
 * @date July 19, 2013
 * @brief This file implement the RRT algorithm with a 2 link arm
 */


#include <flann/flann.hpp>
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/collision_data.h"
#include "fcl/collision_object.h"
#include "fcl/math/transform.h"
#include <fstream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace flann;
using namespace std;
using namespace fcl;

int l1 = 1.0;///<length of link 1
int l2 = 2.0;///<length of link 2

int w = 0.1;///<width of links
double step_size = 0.01;///<step size of small move

bool violation_flag = false;///< Collision Checker result. 
bool obstacle_flag = true;///< If Obstacle presented 

GJKSolver_libccd solver;///< fcl collision checker solver

void CalcNewPoint(Matrix<double> pt_nearest, Matrix<double> pt_target,Matrix<double> pt_new);
double CalcDistance(Matrix<double> point1, Matrix<double> point2);
void CalcCenterPoint(Matrix<double> point,Matrix<double> *cp);
bool CollisionCheck(Matrix<double> point);
void FindPath(vector<Matrix<double> > linkset);
double CalcVectorLength(Matrix<double> vector);

int main(int argc, char** argv){
	// Center point for two links
	Matrix<double> pt_cp(new double[4],2,2);
	pt_cp[0][0] = 0;
	pt_cp[0][1] = 0;
	pt_cp[1][0] = 0;
	pt_cp[1][1] = 0;

    // Test points
	Matrix<double> pt_test(new double[2],1,2);
	pt_test[0][0] = 0.7854;
	pt_test[0][1] = 0;

	// RRT data of nodes. 
	Matrix<double> dataset(new double[2],1,2); 
	// Set root point as initial point
	dataset[0][0] = 0.1;
	dataset[0][1] = 0.2;

	// linkset stores all the link in RRT
	vector<Matrix<double> > linkset;


	//Construct index
	Index<L2<double> > index(dataset,flann::KDTreeIndexParams(4));
	index.buildIndex();

	// Target point matrix(randomly picked point or pt_goal point)
	Matrix<double> pt_target(new double[2],1,2);

	// zero point 
	Matrix<double> pt_zero(new double[2],1,2);
	pt_zero[0][0] = 0;
	pt_zero[0][1] = 0;

	// pt_goal point 
	Matrix<double> pt_goal(new double[2],1,2);
	pt_goal[0][0] = 0.45;
	pt_goal[0][1] = 1.83;

	int nn= 3;	
	Matrix<int> indices(new int[pt_target.rows*nn], pt_target.rows,nn);
	Matrix<double> dists(new double[pt_target.rows*nn], pt_target.rows,nn);

	srand(time(NULL));
	int number_of_points = 0;
	while(true){

		// RRT data of links. First row is self node, second row is parent node
		//the first node is root node. Since root doesn't have parent node, make the second row as the root itself.
		Matrix<double> link(new double[4],2,2); 

	    // pt_nearest point 
		Matrix<double> pt_nearest(new double[2],1,2);
		pt_nearest[0][0] = 0.1;
		pt_nearest[0][1] = 0.2;

		// pt_new point 
	    Matrix<double> pt_new(new double[2],1,2);

		if (violation_flag) {
			double rand1 = -0.5 + (double)rand()/((double)RAND_MAX/(1.0));
			double rand2 = -1.83 + (double)rand()/((double)RAND_MAX/(3.66));
			pt_target[0][0]= rand1;
			pt_target[0][1]= rand2;
		} else {
		    pt_target[0][0] = pt_goal[0][0];
		    pt_target[0][1] = pt_goal[0][1];
		}

		// Find Nearest Neighbor Point(pt_nearest)
		index.knnSearch(pt_target, indices, dists, nn, flann::SearchParams(128));
		pt_nearest[0][0] = (index.getPoint(indices[0][0]))[0];
        pt_nearest[0][1] = (index.getPoint(indices[0][0]))[1];


		CalcNewPoint(pt_nearest, pt_target, pt_new);

		// If obstacles present, do collision check on new point
		if(obstacle_flag){
			violation_flag = CollisionCheck(pt_new);	
			if(violation_flag){
				continue;
			}
		} 
		// Calculate distance between new point and pt_goal point(d). 
		// If d less than or equal to the threshold of the pt_goal point, this is the good enough result. Done
		if(CalcDistance(pt_goal, pt_new) <= step_size) break;

		// If collision check pass, Add new point and new edge to tree
		index.addPoints(pt_new);
	
		// put pt_new and pt_nearest into path
		link[0][0] = pt_new[0][0];
        link[0][1] = pt_new[0][1];
        link[1][0] = pt_nearest[0][0];
		link[1][1] = pt_nearest[0][1]; 
		linkset.push_back(link);

		ofstream out("cpp_plot/path",ofstream::app);
		out <<  pt_new[0][0] << " " << pt_new[0][1] << " " 
			<<  pt_nearest[0][0] << " " << pt_nearest[0][1] << " " 
			<< violation_flag << " "
			<< endl;
	}
	FindPath(linkset);
	return 1;
}

/**
 * @brief Calculate the distance of two points
 * @param[in] point1,point2 Points consist of the coordinates
 * @return The distance of two points
 */
double CalcDistance(Matrix<double> point1, Matrix<double> point2){
	double x1 = point1[0][0];
    double y1 = point1[0][1];
    double x2 = point2[0][0];
    double y2 = point2[0][1];
	return sqrt(pow((x1 - x2),2)+pow((y1 - y2),2));
}

/**
 * @brief Simulate "move a small step"
 *
 * Calculate the new point from nearest neighbor point and target point
 *
 * @param[in] pt_nearest nearest neighbor point found in index			 
 * @param[in] pt_target target point picked randomly or set as goal point
 * @param[in,out] pt_new the new point calculated
 */
void CalcNewPoint(Matrix<double> pt_nearest, Matrix<double> pt_target, Matrix<double> pt_new){
	double x_nearest = pt_nearest[0][0];
	double y_nearest = pt_nearest[0][1];
	double x_target = pt_target[0][0];
	double y_target = pt_target[0][1];
	double distance = CalcDistance(pt_target,pt_nearest);
	pt_new[0][0] = (x_target - x_nearest)*step_size/distance+x_nearest;
	pt_new[0][1] = (y_target - y_nearest)*step_size/distance+y_nearest;
}

/**
 * @brief Calculate the length of vector
 * @param[in] vector a space vector consist of three coordinates
 */
double CalcVectorLength(Matrix<double> vector){
	return sqrt( pow(vector[0][0],2)+ pow(vector[0][1],2)+pow(vector[0][2],2) );	
}

/** 
 * @brief Check the collision of the arm with obstacle presented
 * @param[in] point the point consists of three joint values of the arm
 */
bool CollisionCheck(Matrix<double> point){
	// Obstacles
	Box obs1(0.4,0.4,0);
	Transform3f tf_obs1(Vec3f(2.22,1.5,0));
	//Box obs2(0.4,0.4,0);
	//Transform3f tf_obs2(Vec3f(1.22,1.5,0));

	// Calculate center point(cp)
	Matrix<double> cp(new double[4],2,2);
	CalcCenterPoint(point,&cp);

	// Create two rectangulers represent 2 link	
	Box link1(l1,w,0);
	Transform3f tf_link1(Vec3f(cp[0][0],cp[0][1],0));
	Box link2(l2,w,0);
	Transform3f tf_link2(Vec3f(cp[1][0],cp[1][1],0));

	// Rotate links at their joint angle
	double theta1 = point[0][0];
	fcl::Matrix3f rotation_matrix_1(cos(theta1),-sin(theta1),0,sin(theta1),cos(theta1),0,0,0,1);
	tf_link1.setRotation(rotation_matrix_1);	
	double theta2 = point[0][0]+point[0][1];
	fcl::Matrix3f rotation_matrix_2(cos(theta2),-sin(theta2),0,sin(theta2),cos(theta2),0,0,0,1);
	tf_link2.setRotation(rotation_matrix_2);	

	// Now the arm is properly positioned
	// Do collision check
	double collision = solver.shapeIntersect(link1,tf_link1,obs1,tf_obs1,NULL,NULL,NULL);
	collision = collision || solver.shapeIntersect(link2,tf_link2,obs1,tf_obs1,NULL,NULL,NULL);
	return collision;
}

/**
 * @brief Calculate the center point of every link of the arm
 * @param[in] point Consists of three joint values of the arm
 * @param[in,out] center_points Stores center points of every link of the arm
 */
void CalcCenterPoint(Matrix<double> point,Matrix<double> *center_points){
	(*center_points)[0][0] = l1/2.0 * cos(point[0][0]);
	(*center_points)[0][1] = l1/2.0 * sin(point[0][0]);
	(*center_points)[1][0] = 2.0*(*center_points)[0][0] + l2/2.0 * cos(point[0][0]+point[0][1]);
	(*center_points)[1][1] = 2.0*(*center_points)[0][1] + l2/2.0 * sin(point[0][0]+point[0][1]);

}

/** 
 * @brief Finding a path from RRT
 * @param[in] linkset Containing nodes and links of RRT
 */
void FindPath(vector<Matrix<double> > linkset){
	double size = linkset.size();
	double *pt_child = linkset.back()[0];
	double *pt_parent = linkset.back()[1];
	double *pt_output;
	bool root_found = false;
	ofstream out("cpp_plot/mypath",ofstream::app);
	while(!root_found){
		pt_output = pt_child;
		out << pt_output[0] << " " <<  pt_output[1] << endl;	
		// If the pt_parent is the root point, break
		if(pt_parent[0] == 0.1 && pt_parent[1] == 0.2){
			root_found = true;
			break;
		}
		//Search the pt_parent in the child row(first row), if found, update the pt_child and pt_parent
		for(int i = 0; i < size; i++){
			//search for pt_parent_wanted in child position
			if(linkset[i][0][0] == pt_parent[0] && linkset[i][0][1] == pt_parent[1]){
				pt_child = linkset[i][0];
				pt_parent = linkset[i][1];
				break;
			}
		}
	}
}

