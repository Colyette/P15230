//
//  Node.h
//  
//
//  Created by Alyssa Colyette on 11/9/14.
//
//

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
     * Pointer to neighbors of the given Node
     */
    Node * u_neighbor;
    Node * d_neighbor;
    Node * l_neighbor;
    Node * r_neighbor;
    
    
    
    /**
     *
     * THe position of the Node in the rectangular graph
     */
    int x_coord;
    int y_coord;
    
    /* to prevent default construction*/
    Node();
    
public:
    
    /**
     * Constructor with included positions
     *
     */
    Node(int pos_x, int pos_y);
    
    void printNodeCord();
    
    /**
     * 
     * Getters for Node Coordinates
     */
    int getXcoord() {return x_coord;};
    int getYcoord() {return y_coord;};
    
    /**
     *
     * Returns the pointers to all neighbor Nodes
     * in order u,d,l,r
     */
    std::vector<Node*> getNeighbors ();
    
    /**
     *
     *  Get a neighbor node of a particular position
     */
    Node* getNeighbor(int pos);
    
    /**
     *
     * Removes the neighbor of a given position
     * one-way
     */
    bool removeNeighbor(int pos);
    
    /**
     *
     * Add a neighbor at the provided position
     * using UP,DOWN,LEFT,RIGHT
     */
    bool addNeighbor(int pos, Node * n);
    
    /**
     *
     * prints all of the neighbors and their coordinates
     */
    void printNeighbors();
    
    //used for navigation
    Node* forward;
    Node* backward;
    double h_cost;
    double g_cost;
    double f_cost;
    
    /**
     * easy to read print of data for the node
     */
    void printData();
    
};

#endif /* defined(____Node__) */
