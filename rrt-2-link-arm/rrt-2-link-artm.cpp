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
#include <random>
#include <stdlib.h>
#include <vector>
using namespace flann;

int l1 = 1.0;//length of link 1
int l2 = 2.0;//length of link 2
double *pt_joint_init = {0.1,0.2};
double pt_joint_goal = {-0.5,2.0};
bool violation = false;
double pt_joint_target, pt_joint_root;

double 
int main(){
	while(true){

		int nn = 1;
		//Matrix<double> dataset;// RRT data of nodes and links
		dataset.push_back(pt_joint_init);
		Matrix<double> query;// points that are searched for
		Index<L2<double>> index(dataset,flann::KDTreeIndexParams(1));
		//Matrix<int> indices(new int[query.rows*nn], query.rows,nn);
		//Matrix<double> dists(new float[query.rows*nn], query.rows,nn);
		/* Try "Direct Approach", 
		 * If invalid pt_joint_new accountered, 
		 * 		Go "Randomized Approach"
		 * Back to "Direct Approach" again;
		 */
		/// Set root point as initial point(pt_joint_root)
			pt_joint_root = pt_joint_init;
		/// Set target point as goal point or pick a random point
			if (violation) {
				pt_joint_rand = rand()*100+100;
				pt_joint_target = pt_joint_rand;
			} else {
				pt_joint_target = pt_joint_goal;
			}

		/// Find Nearest Neighbor Point(pt_joint_new)

		/// Collision Check for NN-Point(pt_joint_new) and new edge 

		/// Add new point(pt_joint_new) and new edge to tree

		/// Calculate distance between new point and goal point(d). if d less than or equal to the threshold of the goal point, this is the good enough result. Done
    }
}

