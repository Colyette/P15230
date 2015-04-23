//
//  Node.h
//  
//
//  Created by Alyssa Colyette on 11/9/14.
//
//
/**
 * @file Node.h
 *
 * @author Alyssa Colyette .
 *
 * \class Node
 *
 *
 * \brief Represent a Node that is used in the Grid class
 *          nodes only contain 4 neighbors at the moment
 *          up,down,left, and right neighbors
 *
 *
 * \note
 *
 * \version $Revision: 1.5 $
 *
 * \date $Date: 2005/04/14 14:16:20 $
 *
 * Contact: axc4954@rit.edu
 *
 * Created on: Sun Nov 9 18:39:37 2014
 *
 * $Id:
 *
 *
 */



#ifndef ____Node__
#define ____Node__

#include <stdio.h>
#include <vector>

//help with selecting the neighbors
#define UP      (0)
#define DOWN    (1)
#define LEFT    (2)
#define RIGHT   (3)

class Node{
private:
    /**
     * \brief Pointers to neighbors of the given Node
     */
    Node * u_neighbor;
    Node * d_neighbor;
    Node * l_neighbor;
    Node * r_neighbor;
    
    
    
    /**
     *
     * \brief THe position of the Node in the rectangular graph
     */
    int x_coord;
    int y_coord;
    
    /** \brief to prevent default construction*/
    Node();
    
public:
    
    /**
     * \brief Constructor with included positions
     * \param pos_x: the x postion of node created
     * \param pos_y: the y position of node created
     */
    Node(int pos_x, int pos_y);
    
    /**
     * \brief prints the Coordinate info of the node
     */
    void printNodeCord();
    
    /**
     * 
     * \brief Getters for Node x Coordinate
     * \return x_coord: the x coordinate of the node
     */
    int getXcoord() {return x_coord;};
    
    /**
     *
     * \brief Getters for Node y Coordinate
     * \return y_coord: the y coordinate of the node
     */
    int getYcoord() {return y_coord;};
    
    /**
     *
     * \brief Returns the pointers to all neighbor Nodes
     * in order u,d,l,r
     * \return an array of all the neighbors assigned to that node
     * for a non-neighbor a NULL ptr is provided
     */
    std::vector<Node*> getNeighbors ();
    
    /**
     *
     * \brief Get a neighbor node of a particular position
     * \param pos: the postion of the neighbor as descripted by #defines
     * \return a ptr to the requested node
     */
    Node* getNeighbor(int pos);
    
    /**
     *
     * \brief Removes the neighbor of a given position
     * one-way
     * \param pos: the postion of the neighbor as descripted by #defines
     * \return tells whether the neighbor was removed
     */
    bool removeNeighbor(int pos);
    
    /**
     *
     * \brief Add a neighbor at the provided position
     * using UP,DOWN,LEFT,RIGHT
     * \param pos: the postion of the neighbor as descripted by #defines
     * \param n: a ptr to the Node being added as a neighbor
     * \return tells whether the neighbor was successfully added
     */
    bool addNeighbor(int pos, Node * n);
    
    /**
     *
     * \brief prints all of the neighbors and their coordinates
     */
    void printNeighbors();
    
    //used for navigation
    /**
     * \brief forward node for backtracking
     */
    Node* forward;
    
    /**
     * \brief backward node for backtracking
     */
    Node* backward;
    
    /**
     * \brief holds heuristic cost in A*
     */
    double h_cost;
    
    /**
     * \brief holds gain (traveled) cost in A*
     */
    double g_cost;
    
    /**
     * \brief holds overall cost in A* (h+g)
     */
    double f_cost;
    
    /**
     * \brief easy to read print of cost data for the node
     */
    void printData();
    
    bool visited;
    
};

#endif /* defined(____Node__) */
