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
#include <math.h>
#include <stdlib.h>
#include <vector>

using namespace flann;
using namespace std;
using namespace fcl;

int l1 = 1.0;//length of link 1
int l2 = 2.0;//length of link 2

int w = 0.1;
double step_size = 0.1;

bool violation_flag = false;// Collision Checker result. if true, pick a random point as pt_target; else set pt_target to pt_goal
bool obstacle_flag = true;// If true, do collision check; else, do direct approach

GJKSolver_libccd solver;

void calculateNewPoint(Matrix<double> pt_nearest, Matrix<double> pt_target,Matrix<double> pt_new);
double calculateDistance(Matrix<double> pt1, Matrix<double> pt2);
void workspaceConversion(Matrix<double> *workspace, Matrix<double> jointspace);
void pi(Index<L2<double> > index,int a);
void calcCenterPoint(Matrix<double> pt,Matrix<double> *cp);
bool collisionCheck(Matrix<double> pt);

int main(int argc, char** argv){
	// Center point for two links
	Matrix<double> pt_cp(new double[4],2,2);
	pt_cp[0][0] = 0;
	pt_cp[0][1] = 0;
	pt_cp[1][0] = 0;
	pt_cp[1][1] = 0;

    /// Test points
	Matrix<double> pt_test(new double[2],1,2);
	pt_test[0][0] = 0.7854;
	pt_test[0][1] = 0;

	/// RRT data of nodes. 
	Matrix<double> dataset(new double[2],1,2); 

	/// Set root point as initial point
	dataset[0][0] = 0.1;
	dataset[0][1] = 0.2;

//	/// RRT data of links. First row is the parent node; Middle is the self node; Last ro is the child point;

	Matrix<double> linkset(new double[6],3,2); 
	linkset[0][0] = 0;linkset[1][0] = 0;linkset[2][0] = 0;
	linkset[0][1] = 0;linkset[1][1] = 0;linkset[2][1] = 0;

	vector<Matrix<double> > path;

	///Construct index
	Index<L2<double> > index(dataset,flann::KDTreeIndexParams(4));
	index.buildIndex();

//    /// points that are searched for
//	Matrix<double> pt_query(new double[2],1,2);


	/// Target point matrix(randomly picked point or pt_goal point)
	Matrix<double> pt_target(new double[2],1,2);

    /// pt_nearest point 

	Matrix<double> pt_nearest(new double[2],1,2);
	pt_nearest[0][0] = 0.1;
	pt_nearest[0][1] = 0.2;

	/// outpoint point 
	Matrix<double> outpoint(new double[2],1,2);

	/// zero point 
	Matrix<double> pt_zero(new double[2],1,2);
	pt_zero[0][0] = 0;
	pt_zero[0][1] = 0;

	/// pt_goal point 
	Matrix<double> pt_goal(new double[2],1,2);
	pt_goal[0][0] = 0.45;
	pt_goal[0][1] = 1.83;

	int nn= 3;	
	Matrix<int> indices(new int[pt_target.rows*nn], pt_target.rows,nn);
	Matrix<double> dists(new double[pt_target.rows*nn], pt_target.rows,nn);

	srand(time(NULL));
	int number_of_points = 0;
	while(true){
		/// pt_new point 
	    Matrix<double> pt_new(new double[2],1,2);
	    /* Try "Direct Approach", 
		 * If invalid nearsest accountered, 
		 * 		Go "Randomized Approach"
		 * Back to "Direct Approach" again;
		 */
		//if(number_of_points >= 1000) break;
		//number_of_points++;
		if (violation_flag) {
			double rand1 = -0.5 + (double)rand()/((double)RAND_MAX/(1.0));
			double rand2 = -1.83 + (double)rand()/((double)RAND_MAX/(3.66));
			pt_target[0][0]= rand1;
			pt_target[0][1]= rand2;
		} else {
		    pt_target[0][0] = pt_goal[0][0];
		    pt_target[0][1] = pt_goal[0][1];
		}

		/// Find Nearest Neighbor Point(pt_nearest)
		index.knnSearch(pt_target, indices, dists, nn, flann::SearchParams(128));
		pt_nearest[0][0] = (index.getPoint(indices[0][0]))[0];
        pt_nearest[0][1] = (index.getPoint(indices[0][0]))[1];


		calculateNewPoint(pt_nearest, pt_target, pt_new);

		// If obstacles present, do collision check on new point
		if(obstacle_flag){
			violation_flag = collisionCheck(pt_new);	
			if(violation_flag){
				continue;
			}
		} 
		/// Calculate distance between new point and pt_goal point(d). 
		/// If d less than or equal to the threshold of the pt_goal point, this is the good enough result. Done
		if(calculateDistance(pt_goal, pt_new) <= step_size) break;

		/// If collision check pass, Add new point and new edge to tree
		index.addPoints(pt_new);
	
		ofstream out("cpp_plot/path",ofstream::app);
		out <<  pt_new[0][0] << " " << pt_new[0][1] << " " 
			<<  pt_nearest[0][0] << " " << pt_nearest[0][1] << " " 
			<<  pt_target[0][0] << " " << pt_target[0][1] << " " 
			<<  calculateDistance(pt_target, pt_nearest) << " "  
			<< violation_flag << " "<< endl;
	}
	return 1;
}
/// Print the index
void pi(Index<L2<double> > index,int a){
	Matrix<double> pt_test(new double[2],1,2);
	for(int i = 0; i < a;i++){
		pt_test[0][0] = (index.getPoint(i))[0];
		pt_test[0][1] = (index.getPoint(i))[1];
		cout << pt_test[0][0]<<"\t"<<pt_test[0][1]<<endl;
	}
}
/// print a point
void pp(Matrix<double> pt){
	cout << pt[0][0] << "\t"
	     << pt[0][1] << endl;
}
/// assign a value of a point
void ap(Matrix<double> *pt,double a, double b){
	(*pt)[0][0] = a;
	(*pt)[0][1] = b;
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
	double distance = calculateDistance(pt_target,pt_nearest);
	pt_new[0][0] = (x_target - x_nearest)*step_size/distance+x_nearest;
	pt_new[0][1] = (y_target - y_nearest)*step_size/distance+y_nearest;
}
/// Check the collision of the arm with obstacle presented
/// Parameter
///		pt: a double matrix containing the joint values
bool collisionCheck(Matrix<double> pt){
	/*
	 * construct rect for links
	 *		set height and width
	 * 		set center point to "finally espected" position
	 *		rotate the joint value 
	 */
	// Obstacles
	Box obs1(0.1,0.1,0);
	Transform3f tf_obs1(Vec3f(0.22,2,0));

	// Calculate center point(cp)
	Matrix<double> cp(new double[4],2,2);
	calcCenterPoint(pt,&cp);

	// Create two rectangulers represent 2 link	
	Box link1(l1,w,0);
	Transform3f tf_link1(Vec3f(cp[0][0],cp[0][1],0));
	Box link2(l2,w,0);
	Transform3f tf_link2(Vec3f(cp[1][0],cp[1][1],0));

	// Rotate links at their joint angle
	double theta1 = pt[0][0];
	fcl::Matrix3f rotation_matrix_1(cos(theta1),-sin(theta1),0,sin(theta1),cos(theta1),0,0,0,1);
	tf_link1.setRotation(rotation_matrix_1);	
	double theta2 = pt[0][0]+pt[0][1];
	fcl::Matrix3f rotation_matrix_2(cos(theta2),-sin(theta2),0,sin(theta2),cos(theta2),0,0,0,1);
	tf_link2.setRotation(rotation_matrix_2);	

	/// Now the arm is properly positioned
	/// Do collision check
	double collision = solver.shapeIntersect(link1,tf_link1,obs1,tf_obs1,NULL,NULL,NULL);
	collision = collision || solver.shapeIntersect(link2,tf_link2,obs1,tf_obs1,NULL,NULL,NULL);
	return collision;
}
void calcCenterPoint(Matrix<double> pt,Matrix<double> *cp){
	(*cp)[0][0] = l1/2.0 * cos(pt[0][0]);
	(*cp)[0][1] = l1/2.0 * sin(pt[0][0]);
	(*cp)[1][0] = 2.0*(*cp)[0][0] + l2/2.0 * cos(pt[0][0]+pt[0][1]);
	(*cp)[1][1] = 2.0*(*cp)[0][1] + l2/2.0 * sin(pt[0][0]+pt[0][1]);

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
//void jointsp2worksp(Matrix<double> *jointspace, Matrix<double> workspace){
//	double theta1 = jointspace[0][0];
//	double theta2 = jointspace[0][1];
//	(workspace)[1][0] = l1 * cos(theta1);
//	(workspace)[1][1] = l1 * sin(theta1);
//	(workspace)[2][0] = l2 * cos(theta1 + theta2) + (*workspace)[1][0];
//	(workspace)[2][1] = l2 * sin(theta1 + theta2) + (*workspace)[1][1];
//}
//
//void worksp2jointsp(Matrix<double> *workspace, Matrix<double> jointspace){
//	double theta1 = jointspace[0][0];
//	double theta2 = jointspace[0][1];
//	(*workspace)[1][0] = l1 * cos(theta1);
//	(*workspace)[1][1] = l1 * sin(theta1);
//	(*workspace)[2][0] = l2 * cos(theta1 + theta2) + (*workspace)[1][0];
//	(*workspace)[2][1] = l2 * sin(theta1 + theta2) + (*workspace)[1][1];
//}


void printWorkspace(Matrix<double> pt){
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

//flann::save_to_file(pt_query,"result.h5","pt_query");
//flann::save_to_file(dataset,"result.h5","dataset");
//flann::save_to_file(indices,"result.h5","indices");
//flann::save_to_file(dists,"result.h5","dists");
