#pragma once 
#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <unordered_set>
#include <functional>
#include <map>
#include <boost/functional/hash.hpp>

#include <random>
using namespace std;

// Define a structure for the state of the search problem
class Node {
    double v_, w_;
public:
    Node():v_(0), w_(0){}
    Node(double v, double w) : 
    v_(v), w_(w) {}
    double getV() const
    {
        return v_; 
    }
    double getW() const 
    {
        return w_;
    }
    bool operator==(const Node &other) const
    { return (v_ == other.v_
            && w_ == other.w_);
    }
    struct HashFunction
    {
        std::size_t operator()(Node const& k) const noexcept
        {
            int v = 100000 * (100.0 + k.v_);
            int w = 100000 * (100.0 + k.w_);
            std::size_t h1 = std::hash<std::string>{}(std::to_string(v));
            std::size_t h2 = std::hash<std::string>{}(std::to_string(w));
            return h1 ^ (h2 << 1); // or use boost::hash_combine
        }
    };

};


// Define a function to generate successor states
inline vector<Node> generateSuccessors(double v, double start, double end, 
    double yawrate_reso) {
    vector<Node> successors;
    for (double y=start; y<=end; y+=yawrate_reso)
    {
       Node node(v, y);
       successors.emplace_back(node);
    }
    return successors;
}


// Define a struct to represent a node in the search tree

struct Tree {
    Node state;
    shared_ptr<Tree> parent;
    double cost; // The total cost from the root node to this node
    double heuristic; // The heuristic cost (if applicable)

    Tree(const Node& s, shared_ptr<Tree> p, double c, double h) : state(s), parent(move(p)), cost(c), heuristic(h) {}
};

// Define a custom comparison function for the priority queue
struct CompareNodeCost {
    bool operator()(const shared_ptr<Tree>& left, const shared_ptr<Tree>& right) const {
        // Use the sum of cost and heuristic as the priority
        return left->cost + left->heuristic > right->cost + right->heuristic;
    }
};

