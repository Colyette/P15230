//
//  Node.cpp
//  
//
//  Created by Alyssa Colyette on 11/9/14.
//
//

#include "Node.h"


Node::Node(int pos_x,int pos_y) {
    this->x_coord = pos_x;
    this->y_coord = pos_y;
    
    //instantiate neighbor ptrs to NULL, for later adding
    this->u_neighbor=NULL;
    this->d_neighbor=NULL;
    this->l_neighbor=NULL;
    this->r_neighbor=NULL;
    
    this->forward =NULL;
    this->backward= NULL;
    this->h_cost=0;
    this->g_cost=0;
    this->f_cost=0;
}

/**
 * readable format to dictate its node position
 */
void Node::printNodeCord(){
    printf("Node (%d,%d)\n",x_coord,y_coord);
}

//TODO add legal position checking
bool Node::addNeighbor(int pos, Node * n){
    bool status=0;
    switch (pos) {
        case UP:
            this->u_neighbor=n;
            break;
        case DOWN:
            this->d_neighbor=n ;
            break;
        case LEFT:
            this->l_neighbor=n;
            break;
        case RIGHT:
            this->r_neighbor=n;
            break;
        default:
            printf("addNeighbor: Not Legel position\n");
            status =false;
            break;
    }
    
    return status;
}

Node* Node::getNeighbor(int pos){
    Node* n;
    switch (pos) {
        case UP:
            n=this->u_neighbor;
            break;
        case DOWN:
            n=this->d_neighbor ;
            break;
        case LEFT:
            n=this->l_neighbor;
            break;
        case RIGHT:
            n=this->r_neighbor;
            break;
        default:
            printf("getNeighbor: Not Legel position\n");
            n=NULL;
            break;
    }
    return n;
}

/**
 *
 * Removes the neighbor of a given position
 * one-way
 */
bool Node::removeNeighbor(int pos) {
    bool res =true;
    switch(pos) {
        case UP:
            u_neighbor = NULL;
            break;
        case DOWN:
            d_neighbor = NULL;
            break;
        case LEFT:
            l_neighbor = NULL;
            break;
        case RIGHT:
            r_neighbor = NULL;
            break;
        default:
            printf("removeNeighbor: Not legal position\n");
            res = false;
            break;
    }
    return res;
}

std::vector<Node*> Node::getNeighbors (){
    std::vector<Node*> neighbor_array;
    neighbor_array.push_back(u_neighbor);
    neighbor_array.push_back(d_neighbor);
    neighbor_array.push_back(l_neighbor);
    neighbor_array.push_back(r_neighbor);
    
    return neighbor_array;
}

void Node::printNeighbors() {
    std::vector<Node*> n_list;
    Node* temp_n;
    n_list = this->getNeighbors();
    //printf("Got %d neighbors\n",(int)n_list.size());
    for (int i=0; i<n_list.size(); i++) {
        temp_n =n_list.at(i);
        if (temp_n !=NULL ) { //if the neighbor is available
            printf("neighbor:%d, x:%d y:%d\n", i,temp_n->getXcoord(), temp_n->getYcoord() );
        }
        
    }
}

/**
 * easy to read print of data for the node
 */
void Node::printData(){
    printf("Node (%d,%d):= h:%f g%f f:%f\n",x_coord,y_coord,h_cost,g_cost,f_cost);
    
}

#ifdef TEST_NODE
int main() {
    std::vector<Node*> n_list;
    Node* temp_n;
    Node m_node(2,2);
    Node u_node(2,3);
    Node d_node(2,1);
    Node l_node(1,2);
    Node r_node(3,2);
    
    m_node.addNeighbor(UP,&u_node);
    m_node.addNeighbor(DOWN,&d_node);
    m_node.addNeighbor(LEFT, &l_node);
    m_node.addNeighbor(RIGHT,&r_node);
    
    n_list = m_node.getNeighbors();
    
//    for (int i=0; i<4; i++) {
//        temp_n =n_list.at(i);
//        printf(" neightbor:%d, x:%d y:%d\n", i,temp_n->getXcoord(), temp_n->getYcoord() );
//    }
    m_node.printNeighbors();
    return 0;
}
#endif