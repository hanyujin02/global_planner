/*
*	File: rrtBase.h
*	---------------
*   Base class for RRT planner.
*/
#ifndef RRTBASE_H
#define RRTBASE_H
#include <global_planner/KDTree.h>

using std::cout; using std::endl;

namespace rrt{
	template <std::size_t N>
	class rrtBase{
	private:
		KDTree::Point<N> start_;
		KDTree::Point<N> goal_;
		KDTree::Point<N> emptyToken_;
		std::vector<double> collisionBox_; // half of (lx, ly, lz)
		std::vector<double> envBox_; // value of min max of x y and z
		double delQ_; // incremental distance
		double dR_; // criteria for goal reaching


	protected:
		KDTree::KDTree<N, int> ktree_; // KDTree
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_; // for backtracking

	public:
		// Default constructor
		rrtBase(); 

		// Constructor
		rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR);

		rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR);

		virtual ~rrtBase();

		// load map based on different map representaiton
		virtual void updateMap() = 0;

		// collision checking function based on map and collision box:
		virtual bool checkCollision(const KDTree::Point<N>& q) = 0;

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand) = 0;

		// Find the nearest vertex (node) in the tree
		void nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear);

		// Steer function: basd on delta
		void newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew);

		void backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan);

		bool isReach(const KDTree::Point<N>& q);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan) = 0;

		// add the new vertex to the RRT: add to KDTree
		void addVertex(const KDTree::Point<N>& qNew);

		// add the new edge to RRT: add to rrt 
		void addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew);

		// return start point:
		KDTree::Point<N> getStart();

		// return goal point:
		KDTree::Point<N> getGoal();

		// return collision box:
		std::vector<double> getCollisionBox();

		// return env box:
		std::vector<double> getEnvBox();

		// return dR:
		double getReachRadius();
	};
}

#endif