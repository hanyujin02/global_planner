/*
*	File: rrtBase.h
*	---------------
*   Base class for RRT planner.
*/
#ifndef RRTBASE_H
#define RRTBASE_H
#include <global_planner/KDTree.h>
#include <random>

using std::cout; using std::endl;

namespace rrt{
	std::random_device rd;
	std::mt19937 mt(rd());
	// Helper Function: Random Number
	double randomNumber(double min, double max){
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(mt);
	}

	template <std::size_t N>
	class rrtBase{
	private:
		double delQ_; // incremental distance
		double dR_; // criteria for goal reaching

		ros::NodeHandle nh_;

	protected:
		KDTree::Point<N> start_;
		KDTree::Point<N> goal_;
		KDTree::Point<N> emptyToken_;
		KDTree::KDTree<N, int> ktree_; // KDTree
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> parent_; // for backtracking
		std::vector<double> collisionBox_; // half of (lx, ly, lz)
		std::vector<double> envBox_; // value of min max of x y and z
		double connectGoalRatio_;
		double timeout_;

	public:
		// Default constructor
		rrtBase(); 

		// Constructor
		rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(const ros::NodeHandle &nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(const ros::NodeHandle &nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);

		rrtBase(std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout);


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

		// update start position:
		void updateStart(const std::vector<double>& newStart);
		void updateStart(const KDTree::Point<N>& newStart);

		// update goal position:
		void updateGoal(const std::vector<double>& newGoal);
		void updateGoal(const KDTree::Point<N>& newGoal);

		void clearRRT();

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

		// return parent dictionary (edge)
		std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> getParentDict(); 

		// return goal conenct ratio
		double getConnectGoalRatio();

		// return timeout
		double getTimeout();
	};

	// ===============Function Definition===============================
		template <std::size_t N>
	rrtBase<N>::rrtBase(){};

	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout) 
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		KDTree::Point<N> startp = KDTree::vec2Point<N>(start);
		KDTree::Point<N> goalp = KDTree::vec2Point<N>(goal);
		this->start_ = startp;
		this->goal_ = goalp;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}


	// Constructor:
	template <std::size_t N>
	rrtBase<N>::rrtBase(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout) 
	: nh_(nh), collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		this->start_ = start;
		this->goal_ = goal;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: nh_(nh), collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		KDTree::Point<N> startp = KDTree::vec2Point<N>(start);
		KDTree::Point<N> goalp = KDTree::vec2Point<N>(goal);
		this->start_ = startp;
		this->goal_ = goalp;
		this->emptyToken_[0] = -11311; 
		this->parent_[start_] = this->emptyToken_; // set start parent to NULL
	}

	template <std::size_t N>
	rrtBase<N>::rrtBase(std::vector<double> collisionBox, std::vector<double> envBox, double delQ, double dR, double connectGoalRatio, double timeout)
	: collisionBox_(collisionBox), envBox_(envBox), delQ_(delQ), dR_(dR), connectGoalRatio_(connectGoalRatio), timeout_(timeout){
		cout << "[Global Planner INFO]: Please update start and goal!" << endl;
		this->emptyToken_[0] = -11311; 
	}

	template <std::size_t N>
	void rrtBase<N>::nearestVertex(const KDTree::Point<N>& qKey, KDTree::Point<N>& qNear){
		this->ktree_.nearestNeighbor(qKey, qNear);
	}

	template <std::size_t N>
	rrtBase<N>::~rrtBase(){

	}

	template <std::size_t N>
	void rrtBase<N>::newConfig(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qRand, KDTree::Point<N>& qNew){
		// TODO: implement steer function
		double distance = Distance(qNear, qRand);
		KDTree::Point<N> direction = qRand - qNear;
		qNew = qNear + (this->delQ_/distance) * direction;
	}

	template <std::size_t N>
	void rrtBase<N>::backTrace(const KDTree::Point<N>& qGoal, std::vector<KDTree::Point<N>>& plan){
		KDTree::Point<N> ptr = qGoal;
		while (ptr != this->emptyToken_){
			plan.push_back(ptr);
			ptr = this->parent_[ptr];
		}
	}

	template <std::size_t N>
	bool rrtBase<N>::isReach(const KDTree::Point<N>& q){
		return KDTree::Distance(q, this->goal_) <= this->dR_;
	}

	template <std::size_t N>
	void rrtBase<N>::addVertex(const KDTree::Point<N>& qNew){
		this->ktree_.insert(qNew);
	}

	template <std::size_t N>
	void rrtBase<N>::addEdge(const KDTree::Point<N>& qNear, const KDTree::Point<N>& qNew){
		this->parent_[qNew] = qNear;
	}

	template <std::size_t N>
	void rrtBase<N>::updateStart(const std::vector<double>& newStart){
		KDTree::Point<N> newStartp = KDTree::vec2Point<N>(newStart);
		this->updateStart(newStartp);
	}
	
	template <std::size_t N>
	void rrtBase<N>::updateStart(const KDTree::Point<N>& newStart){
		this->start_ = newStart;
		this->parent_.clear();
		this->parent_[this->start_] = this->emptyToken_;
		if (not this->ktree_.empty()){
			this->clearRRT();
		}
	}

	template <std::size_t N>
	void rrtBase<N>::updateGoal(const std::vector<double>& newGoal){
		KDTree::Point<N> newGoalp = KDTree::vec2Point<N>(newGoal);
		this->updateGoal(newGoalp);
	}
	
	template <std::size_t N>
	void rrtBase<N>::updateGoal(const KDTree::Point<N>& newGoal){
		this->goal_ = newGoal;
		this->parent_.clear();
		this->parent_[this->start_] = this->emptyToken_;
		if (not this->ktree_.empty()){
			this->clearRRT();
		}
	}

	template <std::size_t N>
	void rrtBase<N>::clearRRT(){
		this->ktree_.~KDTree();
		new (&this->ktree_) KDTree::KDTree<N, int> ();
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getStart(){
		return this->start_;
	}

	template <std::size_t N>
	KDTree::Point<N> rrtBase<N>::getGoal(){
		return this->goal_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getCollisionBox(){
		return this->collisionBox_;
	}

	template <std::size_t N>
	std::vector<double> rrtBase<N>::getEnvBox(){
		return this->envBox_;
	}

	template <std::size_t N>
	double rrtBase<N>::getReachRadius(){
		return this->dR_;
	}

	template <std::size_t N>
	std::unordered_map<KDTree::Point<N>, KDTree::Point<N>, KDTree::PointHasher> rrtBase<N>::getParentDict(){
		return this->parent_;
	}

	template <std::size_t N>
	double rrtBase<N>::getConnectGoalRatio(){
		return this->connectGoalRatio_;
	}

	template <std::size_t N>
	double rrtBase<N>::getTimeout(){
		return this->timeout_;
	}
}

#endif