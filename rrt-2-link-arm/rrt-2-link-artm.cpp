/*
 *
 *
 */

///input: 
/// 	initial point(double pt_joint_init); 
///     pt_goal point(double pt_joint_pt_goal)); 
///     step size(double pt_joint_step);
///output: a sequence of steps 
///		output (matrix output)


#include <flann/flann.hpp>
//#include <flann/io/hdf5.h>
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/collision_data.h"
#include "fcl/collision_object.h"
#include "fcl/math/transform.h"

#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <vector>

using namespace flann;
using namespace std;
using namespace fcl;

int l1 = 1.0;//length of link 1
int l2 = 2.0;//length of link 2
int w = 0.1;
double node1_x = 0;
double node1_y = 0;

bool violation_flag = false;// Collision Checker result. if true, pick a random point as pt_target; else set pt_target to pt_goal
bool obstacle_flag = false;
GJKSolver_libccd solver;
double step_size = 0.01;

void calculateNewPoint(Matrix<double> pt_nearest, Matrix<double> pt_target,Matrix<double> pt_new);
double calculateDistance(Matrix<double> pt1, Matrix<double> pt2);
void workspaceConversion(Matrix<double> *workspace, Matrix<double> jointspace);

int main(int argc, char** argv){

	/// RRT data of nodes. 
	Matrix<double> dataset(new double[2],1,2); 
	/// Set root point as initial point
	dataset[0][0] = 0.1;
	dataset[0][1] = 0.2;

	/// RRT data of links. First two numbers is the starting point; Last two numbers is the end point;
	Matrix<double> linkset(new double[4],1,4); 
	linkset[0][0] = 0.1;
	linkset[0][1] = 0.2;

	///Construct index
	Index<L2<double> > index(dataset,flann::KDTreeIndexParams(4));
	index.buildIndex();

//    /// points that are searched for
//	Matrix<double> pt_query(new double[2],1,2);
    /// Test points
	Matrix<double> pt_test(new double[2],1,2);

	/// Target point matrix(randomly picked point or pt_goal point)
	Matrix<double> pt_target(new double[2],1,2);

    /// pt_nearest point 

	Matrix<double> pt_nearest(new double[2],1,2);
	pt_nearest[0][0] = 0.1;
	pt_nearest[0][1] = 0.2;

	/// pt_new point 
	Matrix<double> pt_new(new double[2],1,2);

	/// outpoint point 
	Matrix<double> outpoint(new double[2],1,2);

	/// pt_goal point 
	Matrix<double> pt_goal(new double[2],1,2);
	(pt_goal.ptr())[0] = -0.5;
	(pt_goal.ptr())[1] = 2.0;

	int nn= 1;	
	Matrix<int> indices(new int[pt_target.rows*nn], pt_target.rows,nn);
	Matrix<double> dists(new double[pt_target.rows*nn], pt_target.rows,nn);
	//index.knnSearch(pt_query, indices, dists, 1, flann::SearchParams(5));

	int number_of_points = 0;
	while(true){
		if(number_of_points >= 2000) break;
		number_of_points++;
		/* Try "Direct Approach", 
		 * If invalid nearsest accountered, 
		 * 		Go "Randomized Approach"
		 * Back to "Direct Approach" again;
		 */
		
		/// Set pt_target point as pt_goal point or pick a random point
		//if (violation_flag) {
		//	(pt_target.ptr())[0] = (rand()*100+100);
		//	(pt_target.ptr())[1] = (rand()*100+100);
		//} else {
			(pt_target.ptr())[0] = (pt_goal.ptr())[0];
			(pt_target.ptr())[1] = (pt_goal.ptr())[1];
		//}

		/// Find Nearest Neighbor Point(pt_nearest)
		//index.knnSearch(pt_target, indices, dists, 1, flann::SearchParams(50));
		//(pt_nearest.ptr())[0] = dataset[(*indices[0])][0];
        //(pt_nearest.ptr())[1] = dataset[(*indices[0])][1];
		cout << "Indices" << endl;
		cout << (*indices[0]) << endl;
		//cout << dataset[(*indices[0])][0] << endl;
        //cout <<(pt_nearest.ptr())[0]<< endl;
        //cout << dataset[(*indices[0])][1] << endl;
		//cout <<(pt_nearest.ptr())[1]<< endl;

		/// If obstacles present, do collision check
		//if(obstacle_flag){
		//    // Collision Check for NN-Point(pt_nearest) and new edge 
		//} else{
		//	// No obstacle
		//}

		calculateNewPoint(pt_nearest, pt_target, pt_new);
		cout << "Nearest"<<endl;
        cout <<(pt_nearest.ptr())[0]<< endl;
		cout <<(pt_nearest.ptr())[1]<< endl;
		cout << "Target"<<endl;
		cout <<(pt_target.ptr())[0]<< endl;
		cout <<(pt_target.ptr())[1]<< endl;
		cout << "New"<<endl;
		cout <<pt_new[0][0]<< endl;
		cout <<pt_new[0][1]<< endl;

		/// Calculate distance between new point and pt_goal point(d). if d less than or equal to the threshold of the pt_goal point, this is the good enough result. Done
		if(calculateDistance(pt_goal, pt_new) <= step_size) break;

		/// If collision check pass, Add new point and new edge to tree
		index.addPoints(pt_new);
		pt_nearest[0][0] = pt_new[0][0];	
		pt_nearest[0][1] = pt_new[0][1];	

		//workspaceConversion(&outpoint,pt_new);

		ofstream out("path",ofstream::app);
		//out << outpoint[0][0] << " " << outpoint[0][1] << endl;
		out << pt_new[0][0] << " " << pt_new[0][1] << endl;
    }
	return 1;
}

double calculateDistance(Matrix<double> pt1, Matrix<double> pt2){
	double x1 = pt1[0][0];
    double y1 = pt1[0][1];
    double x2 = pt2[0][0];
    double y2 = pt2[0][1];

	double distance = sqrt(pow((x1 - x2),2)+pow((y1 - y2),2));
	return distance;
}

/// Calculate the pt_new from pt_nearest and pt_target
void calculateNewPoint(Matrix<double> pt_nearest, Matrix<double> pt_target,Matrix<double> pt_new){
	double x_nearest = pt_nearest[0][0];
	double y_nearest = pt_nearest[0][1];
	double x_target = pt_target[0][0];
	double y_target = pt_target[0][1];
	//double distance = sqrt(pow((x_target - x_nearest),2)+pow((x_target - x_nearest),2));
	double distance = calculateDistance(pt_target,pt_nearest);
	pt_new[0][0] = (x_target - x_nearest)*step_size/distance+x_nearest;
	pt_new[0][1] = (y_target - y_nearest)*step_size/distance+y_nearest;
}
/* 
 * Convert from joint space to workspace
 * Parameters:
 *		Matrix<double> workspace
 *			workspace[0][0]:	x coordinate of node1
 *			workspace[0][1]:	y coordinate of node1
 *			workspace[1][0]:	x coordinate of node2
 *			workspace[1][1]:	y coordinate of node2
 *			workspace[2][0]:	x coordinate of node3
 *			workspace[2][1]:	y coordinate of node3
 * 		Matrix<double> jointspace
 *			jointspace[0][0]: joint value 1
 *			jointspace[0][1]: joint value 2
 */
void workspaceConversion(Matrix<double> *workspace, Matrix<double> jointspace){
	double theta1 = jointspace[0][0];
	double theta2 = jointspace[0][1];
	(*workspace)[1][0] = l1 * cos(theta1);
	(*workspace)[1][1] = l1 * sin(theta1);
	(*workspace)[2][0] = l2 * cos(theta1 + theta2) + (*workspace)[1][0];
	(*workspace)[2][1] = l2 * sin(theta1 + theta2) + (*workspace)[1][1];
}
bool collisionCheck(Matrix<double> *point){
	// Calculate center point(cp)
	double cp1_x = l1/2;
	double cp1_y = w/2;
	double cp2_x = l2/2;
	double cp2_y = w/2;

	/// Create two rectangulers represent 2 link	
	Box link1(l1,w,0);
	Transform3f tf_link1(Vec3f(cp1_x,cp1_y,0));


	Box link2(l2,w,0);
	Transform3f tf_link2(Vec3f(cp2_x,cp2_y,0));

}

void printWorkspace(Matrix<double> pt){
	ofstream out("path",ofstream::out);
}
//Matrix<double> dataset;
//load_from_file(dataset,"dataset.h5","dataset");

//Matrix<double> pt_joint_goal (new double[2],1,2);
//(pt_joint_goal.ptr())[0] = 5;
//(pt_joint_goal.ptr())[1] = 6;
//index.addPoints(pt_joint_init);
//index.addPoints(pt_joint_goal);

//cout<< "pt_query:"<<endl;
//cout << (pt_query.ptr())[0] << endl;
//cout << (pt_query.ptr())[1] << endl;
//cout<< "indices:"<<endl;
//cout<< (indices.ptr())[0] << endl;
//cout<< "dists:"<<endl;
//cout<< (dists.ptr())[0] << endl;



	

//double *a = index.getPoint(0);
//double *b = index.getPoint(1);
//double *c = index.getPoint(2);
//double *d = index.getPoint(3);
//cout << "a:" << *a << " " << *(a+1) << endl;
//cout << "b:" << *b << " " << *(b+1) << endl;
//cout << "c:" << *c << " " << *(c+1) << endl;
//cout << "d:" << *d << " " << *(d+1) << endl;
//flann::save_to_file(pt_query,"result.h5","pt_query");
//flann::save_to_file(dataset,"result.h5","dataset");
//flann::save_to_file(indices,"result.h5","indices");
//flann::save_to_file(dists,"result.h5","dists");
