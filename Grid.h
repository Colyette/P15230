//
//  Grid.h
//  
//
//  Created by Alyssa Colyette on 11/9/14.
//
// just an array of nodes with a constant edge weight

#ifndef ____Grid__
#define ____Grid__

#include <stdio.h>
#include <queue>
#include "Node.h"

class Grid{

private:
    /**
     *
     * Actual representation of the map
     */
    std::vector<std::vector<Node *> > map;
    
    //disable default constructor
    Grid();
    
    //details of grid creation
    int width;
    int length;
    int resolution;
    
    
//ACTUAL A* SEARCH CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     * Maps to each node
     */
    std::vector<std::vector<double> > heuristicData;
    
    /**
     *
     * Calculates all the manhatten distances from every node 
     * to destination node
     */
    void calHeuristicData(Node* node);
    
    /**
     * Calculate the Manhattan Distance, used for heuristic data
     */
    double calManhattanDist(Node* node_A, Node* node_B);
    
public:
    /**
     *
     * Creates a grid of given dimensions divided by given resolution
     *
     */
    Grid(int w,int l,int res);
    
    ~Grid();
    
    /**
     *
     * Adds all nodes to the gride given the instantiations
     */
    void initializeGrid();
    
    /**
     *
     * Adds all of the neighbors, initial map, before objects are detected
     */
    void add_neighbors();
    
    //getter
    std::vector<std::vector<Node*> > getMap(){return map;};
    
    /**
     * Removes the edge between two nodes
     * [ removes both nodes from each others neighbor list]
     */
    bool removeEdge(Node * node_A, Node * node_B);
    
    /**
     *
     * Retrieves pointer of node from certain coordinates
     */
    Node* getNode(int x, int y);
    
    //last pathing performed...TODO may not be needed...
    std::vector<Node*> path;

//ACTUAL A* SEARCH CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     *
     * Returns a queue of the path to the designated sink node
     * from the designated source node
     */
    std::vector<Node*> findPath(Node* S, Node* T);
    
    /**
     *
     * removes the tracking left by the node
     */
    void clearParents();
    
    /**
     *
     * so far only prints the path variable
     */
    void printPath();
};

#endif /* defined(____Grid__) */
