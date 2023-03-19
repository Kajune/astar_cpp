#include "boost/python.hpp"
#include "boost/python/numpy.hpp"
#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <memory>
#include <limits>


namespace p = boost::python;
namespace np = boost::python::numpy;


struct SearchNode {
	p::object data_;
	double gscore_, fscore_;
	bool closed_;
	bool out_openset_;
	std::shared_ptr<SearchNode> came_from_;

	SearchNode(p::object data, double gscore, double fscore)
	 : data_(data), gscore_(gscore), fscore_(fscore), closed_(false), out_openset_(true), came_from_(nullptr) {};

	bool operator<(const SearchNode& rhs) const {
		return this->fscore_ < rhs.fscore_;
	}

	bool operator>(const SearchNode& rhs) const {
		return this->fscore_ > rhs.fscore_;
	}
};


template <typename T>
struct less_shared_ptr {
	bool operator()(const std::shared_ptr<T> lhs, const std::shared_ptr<T> rhs) const {
		return (*lhs) < (*rhs);
	}
};


template <typename T>
struct greater_shared_ptr {
	bool operator()(const std::shared_ptr<T> lhs, const std::shared_ptr<T> rhs) const {
		return (*lhs) > (*rhs);
	}
};


template <class T,
		class Container = std::vector<T>,
		class Compare = std::less<typename Container::value_type>>
class removable_priority_queue : public std::priority_queue<T, Container, Compare> {
public:
	bool remove(const T& value) {
		auto it = std::find(this->c.begin(), this->c.end(), value);

		if (it == this->c.end()) {
			return false;
		}
		if (it == this->c.begin()) {
			this->pop();
		} else {
			this->c.erase(it);
			std::make_heap(this->c.begin(), this->c.end(), this->comp);
		}
		return true;
	}
};


class AStar {
public:
	p::list reconstruct_path(std::shared_ptr<SearchNode> last) const {
		p::list ret;

		std::shared_ptr<SearchNode> current;
		current = last;
		while (current) {
			ret.append(current->data_);
			current = current->came_from_;
		}
		return ret;
	}


	p::list find_path(p::object start) const {
		if (this->is_goal_reached(start)) {
			p::list ret;
			ret.append(start);
			return ret;
		}

		std::map<p::object, std::shared_ptr<SearchNode>> searchNodes;
		auto startNode = std::make_shared<SearchNode>(start, 0.0, this->heuristic_cost_estimate(start));
		searchNodes[start] = startNode;

		removable_priority_queue<std::shared_ptr<SearchNode>,
			std::vector<std::shared_ptr<SearchNode>>,
			greater_shared_ptr<SearchNode>> openSet;
		openSet.push(startNode);

		while (!openSet.empty()) {
			auto current = openSet.top();
			openSet.pop();

			if (this->is_goal_reached(current->data_)) {
				return this->reconstruct_path(current);
			}

			current->out_openset_ = true;
			current->closed_ = true;
			
			auto neighbors = this->neighbors(current->data_);
			p::ssize_t n = p::len(neighbors);
			for (p::ssize_t i = 0; i < n; i++) {
				if (searchNodes.find(neighbors[i]) == searchNodes.end()) {
					searchNodes[neighbors[i]] = std::make_shared<SearchNode>(neighbors[i], 
						std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
				}
				const auto& neighbor = searchNodes.at(neighbors[i]);

				if (neighbor->closed_) {
					continue;
				}

				double tentative_gscore = current->gscore_ + this->distance_between(
					current->data_, neighbor->data_
				);

				if (tentative_gscore >= neighbor->gscore_) {
					continue;
				}

				neighbor->came_from_ = current;
				neighbor->gscore_ = tentative_gscore;
				neighbor->fscore_ = tentative_gscore + this->heuristic_cost_estimate(
					neighbor->data_
				);

				if (neighbor->out_openset_) {
					neighbor->out_openset_ = false;
					openSet.push(neighbor);
				} else {
					openSet.remove(neighbor);
					openSet.push(neighbor);
				}
			}
		}

		p::list ret;
		return ret;
	}


	virtual double heuristic_cost_estimate(p::object node) const {
		std::cout << "Override function \"heuristic_cost_estimate\" in python subclass!" << std::endl;
		return 0.0f;
	}

	virtual double distance_between(p::object node1, p::object node2) const {
		std::cout << "Override function \"distance_between\" in python subclass!" << std::endl;
		return 0.0f;
	}

	virtual p::list neighbors(p::object node) const {
		std::cout << "Override function \"neighbors\" in python subclass!" << std::endl;
		return p::list();
	}

	virtual bool is_goal_reached(p::object node) const {
		std::cout << "Override function \"is_goal_reached\" in python subclass!" << std::endl;
		return false;
	}
};



template <typename AStarBase>
struct AStarWrap : public AStarBase, public p::wrapper<AStarBase> {
	AStarWrap() : AStarBase() {};


	p::list find_path(p::object start) const {
		return AStarBase::find_path(start);
	}


	double heuristic_cost_estimate(p::object node) const override {
		if (p::override f = this->get_override("heuristic_cost_estimate")) {
			return this->get_override("heuristic_cost_estimate")(node);
		} else {
			return AStarBase::heuristic_cost_estimate(node);
		}
	}

	double distance_between(p::object node1, p::object node2) const override {
		if (p::override f = this->get_override("distance_between")) {
			return this->get_override("distance_between")(node1, node2);
		} else {
			return AStarBase::distance_between(node1, node2);
		}
	}

	p::list neighbors(p::object node) const override {
		if (p::override f = this->get_override("neighbors")) {
			return this->get_override("neighbors")(node);
		} else {
			return AStarBase::neighbors(node);
		}
	}

	bool is_goal_reached(p::object node) const override {
		if (p::override f = this->get_override("is_goal_reached")) {
			return this->get_override("is_goal_reached")(node);
		} else {
			return AStarBase::is_goal_reached(node);
		}
	}
};


BOOST_PYTHON_MODULE(astar_cpp) {
	p::class_<AStarWrap<AStar>, boost::noncopyable>("AStar", p::init<>())
		.def("find_path", &AStarWrap<AStar>::find_path)
		.def("heuristic_cost_estimate", &AStarWrap<AStar>::heuristic_cost_estimate)
		.def("distance_between", &AStarWrap<AStar>::distance_between)
		.def("neighbors", &AStarWrap<AStar>::neighbors)
		.def("is_goal_reached", &AStarWrap<AStar>::is_goal_reached);
}
