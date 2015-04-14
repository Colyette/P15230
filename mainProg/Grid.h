//
//  Grid.h
//  
//
//  Created by Alyssa Colyette on 11/9/14.
//
// just an array of nodes with a constant edge weight
/**
 * @file Grid.h
 *
 * @author Alyssa Colyette .
 *
 * @class Grid
 *
 *
 * @brief Holds a 2D array of nodes with a constant edge weights. Provides
 *      an method for finding a path using A* Search
 *
 * @note
 *
 * @version $Revision: 1.5 $
 *
 * @date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: axc4954@rit.edu
 *
 * Created on: Sun Nov 9 18:39:37 2014
 *
 */


#ifndef ____Grid__
#define ____Grid__

#include <stdio.h>
#include <queue>
#include "Node.h"

class Grid{

private:
    /**
     *
     * @brief Actual representation of the map
     */
    std::vector<std::vector<Node *> > map;
    
    /**
     * @brief disable default constructor
     */
    Grid();
    
    //details of grid creation
    /** @brief the width of the area to be transversed */
    int width;
    
    /** @brief the length of the area to be transversed */
    int length;
    
    /** @brief the resolution of the area to be transversed, spacing between ea Node*/
    int resolution;
    
    
//ACTUAL A* SEARCH CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    /**
     * @brief Maps to each node
     * TODO: not used
     */
    std::vector<std::vector<double> > heuristicData;
    
    /**
     *
     * @brief Calculates all the manhatten distances from every node
     * to destination node
     * @param 
     * TODO not used
     */
    void calHeuristicData(Node* node);
    
    /**
     * @brief Calculate the Manhattan Distance, used for heuristic data
     * @param node_A: a ptr to start node to calculated distance from
     * @param node_B: a ptr to the destingation node to calculated distance to
     */
    double calManhattanDist(Node* node_A, Node* node_B);
    
public:
    /**
     *
     * @brief Creates a grid of given dimensions divided by given resolution
     * @param w: width of the area being transversed
     * @param l: length of the area being transversed
     * @param res: resolution of the area being transversed
     *
     */
    Grid(int w,int l,int res);
    
    /**
     * @brief removes Nodes created using new
     */
    ~Grid();
    
    /**
     *
     * @brief Adds all nodes to the gride given the instantiations
     */
    void initializeGrid();
    
    /**
     *
     * @brief Adds all of the neighbors creating initial map, before objects are detected
     */
    void add_neighbors();
    
    //getter
    std::vector<std::vector<Node*> > getMap(){return map;};
    
    /**
     * @brief Removes the edge between two nodes
     * [ removes both nodes from each others neighbor list]
     * @param node_A: ptr to one endpoint of the edge
     * @param node_B: ptr to the other endpoint of the edge
     * @return bool: tells whether the provided edge was removed successfully
     */
    bool removeEdge(Node * node_A, Node * node_B);
    
    /**
     *
     * @brief Retrieves pointer of node from certain coordinates
     * @param x: x postition of requested Node
     * @param y: y position of requested Node
     * @return the Node ptr of the requested position, returns NULL if invalid
     */
    Node* getNode(int x, int y);
    
    /**
     * @brief last pathing performed...TODO may not be needed...
     */
    std::vector<Node*> path;

//ACTUAL A* SEARCH CODE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /**
     *
     * @brief Returns a queue of the path to the designated sink node
     * from the designated source node
     * @param S: the source/starting Node for path planning
     * @param T: the sink/destination Node for path planning
     * @return returns the path created, just returns this->path
     */
    std::vector<Node*> findPath(Node* S, Node* T);
    
    /**
     *
     * @brief Removes the tracking left by the node
     */
    void clearParents();
    
    /**
     *
     * @brief So far only prints the Nodes in path variable
     */
    void printPath();
};

#endif /* defined(____Grid__) */
