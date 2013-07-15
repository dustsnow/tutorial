/*
 *
 *
 */

///input: 
/// 	initial point(double pt_joint_init); 
///     goal point(double pt_joint_goal)); 
///     step size(double pt_joint_step);
///output: a sequence of steps 
///		output (matrix output)
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>
#include <fcl/narrowspace/narrowspace.h>
#include <fcl/collision.h>
#include <fcl/collision_data.h>
#include <fcl/collision_object.h>
#include <fcl/math/transform.h>

#include <stdlib.h>
#include <vector>

using namespace flann;
using namespace std;

int l1 = 1.0;//length of link 1
int l2 = 2.0;//length of link 2


bool violation = false;// Collision Checker result. if true, pick a random point as target; else set target to goal

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

    /// points that are searched for
	Matrix<double> query(new double[2],1,2);

	/// Target point matrix(randomly picked point or goal point)
	Matrix<double> target(new double[2],1,2);

    /// nearse point 
	Matrix<double> nearest(new double[2],1,2);
	// goal point 
	Matrix<double> goal(new double[2],1,2);
	(goal.ptr())[0] = -0.5;
	(goal.ptr())[1] = 2.0;

	int nn= 1;	
	Matrix<int> indices(new int[query.rows*nn], query.rows,nn);
	Matrix<double> dists(new double[query.rows*nn], query.rows,nn);
	//index.knnSearch(query, indices, dists, 1, flann::SearchParams(5));

	while(true){

		/* Try "Direct Approach", 
		 * If invalid nearsest accountered, 
		 * 		Go "Randomized Approach"
		 * Back to "Direct Approach" again;
		 */
		
		/// Set target point as goal point or pick a random point
		if (violation) {
			(target.ptr())[0] = (rand()*100+100);
			(target.ptr())[1] = (rand()*100+100);
		} else {
				(target.ptr())[0] = (goal.ptr())[0];
				(target.ptr())[1] = (goal.ptr())[1];
			}
		/// Find Nearest Neighbor Point(nearest)
		index.knnSearch(query, indices, dists, 1, flann::SearchParams(50));
		(nearest.ptr())[0] = dataset[(*indices[0])][0];
        (nearest.ptr())[1] = dataset[(*indices[0])+1][1];

		/// Collision Check for NN-Point(nearest) and new edge 


//
//		/// Add new point(pt_joint_new) and new edge to tree
//
//		/// Calculate distance between new point and goal point(d). if d less than or equal to the threshold of the goal point, this is the good enough result. Done
    }
	return 1;
}
//Matrix<double> dataset;
//load_from_file(dataset,"dataset.h5","dataset");

//Matrix<double> pt_joint_goal (new double[2],1,2);
	//(pt_joint_goal.ptr())[0] = 5;
	//(pt_joint_goal.ptr())[1] = 6;
	//index.addPoints(pt_joint_init);
	//index.addPoints(pt_joint_goal);

	//cout<< "query:"<<endl;
    //cout << (query.ptr())[0] << endl;
    //cout << (query.ptr())[1] << endl;
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
//flann::save_to_file(query,"result.h5","query");
//flann::save_to_file(dataset,"result.h5","dataset");
//flann::save_to_file(indices,"result.h5","indices");
//flann::save_to_file(dists,"result.h5","dists");
