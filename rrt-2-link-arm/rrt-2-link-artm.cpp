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

#include <stdlib.h>
#include <vector>

using namespace flann;
using namespace std;

int l1 = 1.0;//length of link 1
int l2 = 2.0;//length of link 2


bool violation = false;// Collision Checker result. if true, pick a random point as target; else set target to goal
double *pt_joint_target, *pt_joint_root;

int main(int argc, char** argv){

	//Matrix<double> dataset;
	//load_from_file(dataset,"dataset.h5","dataset");
	Matrix<double> dataset(new double[2],1,2);// RRT data of nodes and links
	(dataset.ptr())[0] = 0.0;
	(dataset.ptr())[1] = 0.0;
	//(dataset.ptr())[2] = 1.0;
	//(dataset.ptr())[3] = 0.0;

	Index<L2<double> > index(dataset,flann::KDTreeIndexParams(4));
	index.buildIndex();

	Matrix<double> pt_joint_init (new double[2],1,2);
	(pt_joint_init.ptr())[0] = 1;
	(pt_joint_init.ptr())[1] = 0;
	Matrix<double> pt_joint_goal (new double[2],1,2);
	(pt_joint_goal.ptr())[0] = 5;
	(pt_joint_goal.ptr())[1] = 6;
	index.addPoints(pt_joint_init);
	index.addPoints(pt_joint_goal);
	Matrix<double> query (new double[2],1,2);// points that are searched for
	(query.ptr())[0] = 1;
	(query.ptr())[1] = 2;
	int nn= 1;	
	Matrix<int> indices(new int[query.rows*nn], query.rows,nn);
	Matrix<double> dists(new double[query.rows*nn], query.rows,nn);
	index.knnSearch(query, indices, dists, 1, flann::SearchParams(5));
	cout<< "query:"<<endl;
	cout << (query.ptr())[0] << endl;
	cout << (query.ptr())[1] << endl;
	cout<< "indices:"<<endl;
	cout<< (indices.ptr())[0] << endl;
	cout<< "dists:"<<endl;
	cout<< (dists.ptr())[0] << endl;
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

//	while(true){
//
//		int nn = 1;
//		/* Try "Direct Approach", 
//		 * If invalid pt_joint_new accountered, 
//		 * 		Go "Randomized Approach"
//		 * Back to "Direct Approach" again;
//		 */
//		/// Set root point as initial point(pt_joint_root)
//			//pt_joint_root = pt_joint_init;
//		/// Set target point as goal point or pick a random point
//			if (violation) {
//			//	pt_joint_rand = rand()*100+100;
//			//	pt_joint_target = pt_joint_rand;
//			} else {
//			//	pt_joint_target = pt_joint_goal;
//			}
//
//		/// Find Nearest Neighbor Point(pt_joint_new)
//
//		/// Collision Check for NN-Point(pt_joint_new) and new edge 
//
//		/// Add new point(pt_joint_new) and new edge to tree
//
//		/// Calculate distance between new point and goal point(d). if d less than or equal to the threshold of the goal point, this is the good enough result. Done
//    }
	return 1;
}

