/**
 * @file Grid.cpp
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

#include "Grid.h"
#include <cmath>


/**
 *
 * Creates a grid of given dimensions divided by given resolution
 * @var w: width of the navigated area
 * @var l: length of the navigated area
 * @var res: the resolution or edge cost for the grid
 */
Grid::Grid(int w,int l, int res){
    this ->width =w;
    this->length = l;
    this -> resolution = res;
#ifdef TEST_GRID
    printf("Map of size x:%d y:%d \n",(w/res),(l/res));
#endif
    //(this ->map).resize((w/res),str::vector<Node *>(l/res)); //TODO hopefully rounds
    
}

/**
 * Delete all 'new' instantiated objects
 */
Grid::~Grid(){
    Node * temp;
    for(int x=0;x<= this-> width; x+= (this->resolution) ) {
        for(int y= 0; y<= this->length; y+=(this->resolution)) {
            //delete the new nodes created in initialize
            temp = (map.at(x)).at(y);
            delete temp;
        }
        
    }
    printf("Removed all nodes, supposedly\n");

}

/**
 *
 * Adds all nodes to the gride given the instantiations
 */
void Grid::initializeGrid(){
    Node * temp;
    // for every node along the width
    for(int x=0;x<= this-> width; x+= (this->resolution) ) {
        std::vector<Node*> row;
        
        for(int y= 0; y<= this->length; y+=(this->resolution)) {

            temp = new Node(x,y);
//#ifdef TEST_GRID
//            printf("Adding node x:%d y:%d\n",temp->getXcoord(),temp->getYcoord());
//#endif
            row.push_back(temp);
        }
        (this->map).push_back(row); //add initialized row
        //not sure if data disappears after leaving scope
    }

}

/**
 *
 * Adds all of the neighbors, initial map, before objects are detected
 */
void Grid::add_neighbors(){
    Node * curNode;
    Node * nNode;
    // for every node along the width
    for(int x=0;x<= this-> width; x+= (this->resolution) ) {
        
        for(int y= 0; y<= this->length; y+=(this->resolution)) {
            curNode = ( map.at(x) ).at(y); // current node
            //up
            if (y<=(this->length-1)) { //if there is an upper neighbor, length shouldn't be 1
                nNode =( map.at(x) ).at(y+1) ;
//                printf("Adding ");
//                nNode->printNodeCord();
                curNode->addNeighbor( UP,nNode);
            }
            //down
            if (y>= 1) { //if there is a lower neighbor (lowest is y=0
                nNode =( map.at(x) ).at(y-1) ;
//                printf("Adding ");
//                nNode->printNodeCord();
                curNode->addNeighbor(DOWN,nNode );
            }
            //right
            if (x<=(this->width -1)) {
                nNode =( map.at(x+1) ).at(y) ;
//                printf("Adding ");
//                nNode->printNodeCord();
                curNode->addNeighbor(RIGHT,nNode );
            }
            
            //left
            if (x>=1) {
                nNode =( map.at(x-1) ).at(y) ;
//                printf("Adding ");
//                nNode->printNodeCord();
                curNode->addNeighbor(LEFT,nNode );
            }
            
//#ifdef TEST_GRID
//            printf("~~~~Node (%d,%d)~~~~~~~~~~~~~\n",x,y);
//            curNode->printNeighbors();
//            printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
//#endif
            
        }
    }

}

/**
 * Removes the edge between two nodes
 * [ removes both nodes from each others neighbor list]
 */
bool Grid::removeEdge(Node * node_A, Node * node_B){
    std::vector<Node*> neighborsA;
    std::vector<Node*> neighborsB;
    bool status=false;
    if (this->getNode(node_A->getXcoord(),node_A->getYcoord() ) !=NULL  ) {
        if (this->getNode(node_B->getXcoord(),node_B->getYcoord() )  !=NULL) {
            //both nodes are legal in the map
            //find neighbor
            neighborsA = node_A->getNeighbors();
            neighborsB = node_B->getNeighbors();
            for (int i =0; i<neighborsA.size(); i++) {
                if (node_B == neighborsA.at(i) ) {
                    //confirmed as neighbors at least one way, GOod Enough
                    //B is a up neighbor of A
                    if (node_B == node_A->getNeighbor(UP)){
                        //rm
                        node_A->removeNeighbor(UP);
                        node_B->removeNeighbor(DOWN);
#ifdef TEST_GRID
                        printf("Node:: removeEdge: up down\n");
#endif
                        return true;
                    }
                    //B is a down neighbor of A
                    else if(node_B == node_A->getNeighbor(DOWN)) {
                        //rm
                        node_A->removeNeighbor(DOWN);
                        node_B->removeNeighbor(UP);
#ifdef TEST_GRID
                        printf("Node:: removeEdge: down up \n");
#endif
                        return true;
                    }
                    //B is a left neighbor of A
                    else if(node_B == node_A->getNeighbor(LEFT)) {
                        node_A->removeNeighbor(LEFT);
                        node_B->removeNeighbor(RIGHT);
#ifdef TEST_GRID
                        printf("Node:: removeEdge: left right\n");
#endif
                        return true;
                    }
                    //B is a right neighbor of A
                    else if(node_B == node_A->getNeighbor(RIGHT)) {
                        node_A->removeNeighbor(RIGHT);
                        node_B->removeNeighbor(LEFT);
#ifdef TEST_GRID
                        printf("Node:: removeEdge: right left\n");
#endif
                        return true;
                    }
                }//End found matching neighbors
            }//Searched All nodes
            
            
        }
    }
    //should always be false
    return status;
}

/**
 * Calculate the Manhattan Distance, used for heuristic data
 */
double Grid::calManhattanDist(Node* node_A, Node* node_B){
    //knowing the resolution as always 1, and dimensions
    //
    double x_dist,y_dist;
    x_dist= fabs( node_A->getXcoord() -node_B->getXcoord()) ;
    y_dist =fabs( node_A->getYcoord() -node_B->getYcoord()) ;
    
//    node_A->printNodeCord();
//    printf(" from ");
//    node_B->printNodeCord();
//    printf("is %f\n",x_dist+y_dist);
    
    return (x_dist+y_dist);
}

/**
 *
 * Returns a queue of the path to the designated sink node
 * from the designated source node
 */
std::vector<Node*> Grid::findPath(Node* S, Node* T){
    //TODO
    bool done=false;
    std::vector<Node*> neighbors;
    double minCost=-1;
    Node * neighbor_minCost=NULL;
    Node* temp_n;
    Node* prev_Node; // holding for last node
    bool allNull;
    
    ///empty the path
    path.clear();
    
    //mark current block
    Node* curNode = S;
    curNode ->g_cost =0; //no cost to the first node
    curNode->h_cost = calManhattanDist(S,T);
    
    do {
        allNull = true;
        minCost=-1;
        neighbors = curNode->getNeighbors();
        neighbor_minCost=NULL;
        //printf("(%d,%d) neighbor size %d\n",curNode->getXcoord(),curNode->getYcoord(), neighbors.size());
        for (int n=0; n<neighbors.size(); n++) {
            if (neighbors.at(n)!=NULL) {
                allNull = false;    //there was atleast one place to go
                //assign current block as parent, if previously unassigned? Otherwise overwritten
                if(neighbors.at(n)->backward == NULL) {
                    neighbors.at(n)->backward= curNode;
                }
                //calc h distance
                neighbors.at(n)->h_cost=calManhattanDist(neighbors.at(n),T);
                //calc g, if more than 4 neighbors (use diagonal distance measurement)
                neighbors.at(n)->g_cost = curNode->g_cost+resolution; // add the cost to this node
                //cal f
                neighbors.at(n)->f_cost = neighbors.at(n)->h_cost+ neighbors.at(n)->g_cost;
#ifdef TEST_GRID
               neighbors.at(n)->printData();
#elif defined (FLIGHT_DEBUG)
                neighbors.at(n)->printData();
#endif
                
                // save the min cost node to jump to next
                if ((minCost<0) || (minCost > neighbors.at(n)->f_cost)) {
                    if ( (neighbors.at(n))->visited !=true ) {
                        minCost = neighbors.at(n)->f_cost;
                        neighbor_minCost=neighbors.at(n);
#ifdef FLIGHT_DEBUG
                        printf("got a min (%d,%d)\n",neighbors.at(n)->getXcoord(),neighbors.at(n)->getYcoord());
#endif
                    }
                    
                }
                else {
//                    printf("current");
//                    curNode->printNodeCord();
//                    printf("neighborcheck");
//                    (neighbors.at(n))->printNodeCord();
//                    printf("min cost NULL\n");
                }
            }//else not a valid neighbor of the current node
            
            
        } //checked all neighbors
        if (allNull) {
           // printf("(%d,%d) has all NULL neighbors\n");
        }
        //pick min cost as next
        if(neighbor_minCost !=NULL) {
#ifdef FLIGHT_DEBUG
            printf("From ");
            curNode->printNodeCord();
#endif
    
            curNode->forward = neighbor_minCost; //TODO might not be required
            prev_Node = curNode;
            //mark as visited
            curNode->visited = true;
            curNode = neighbor_minCost;
#ifdef FLIGHT_DEBUG
            printf("-> To ");
            curNode->printNodeCord();
#endif
            
  

        }else{
            printf("Need to backtrack?\n");
            curNode->visited=true;
            curNode = curNode->backward;
        }
        //if the heuristic is zero, we're at the sink node
        if (curNode->h_cost ==0) {
#ifdef TEST_GRID
            printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~reached source node\n");
#endif
            break;
            //done=true;
        }
        
    }while (!done);
    
#ifdef TEST_GRID
    printf("Backtracking\n");
//    curNode->printNodeCord();
#endif
    curNode = prev_Node;
    //get path
    while (curNode != S ) {
        //add current node to path
#ifdef TEST_GRID
//        curNode->printNodeCord();
#endif
        path.push_back(curNode);
        //take the previous node transversed
        temp_n = curNode->backward;
        curNode=temp_n;
        
    }
    
    //TODO
    return this->path;
    
}

/**
 *
 * \brief removes the tracking left by the node
 * also removes visited boolean....
 */
void Grid::clearParents(){
    Node * temp;
    for(int x=0;x<= this-> width; x+= (this->resolution) ) {
        for(int y= 0; y<= this->length; y+=(this->resolution)) {
            //delete the new nodes created in initialize
            temp = (map.at(x)).at(y);
            temp->forward = NULL;
            temp->backward = NULL;
            temp->visited = false;
        }
        
    }

}

/**
 *
 * Retrieves pointer of node from certain coordinates
 */
Node* Grid::getNode(int x, int y){
    if ((x >=0) & (x<=this->width)) {
        if ((y >=0) & (y<=this->length)) { //if within dimensions
            return (map.at(x)).at(y);
        }
    }
    printf("Grid::getNode:: INVALID Coordinates\n");
    return NULL;
}

/**
 *
 * so far only prints the path variable
 */
void Grid::printPath(){
    printf("Path:\n");
    printf("___________________\n");
    for (int i = path.size() -1 ; i>=0; i--) {
        path.at(i) -> printNodeCord();
    }
}


#ifdef TEST_GRID
int main() {
    std::vector<Node*> theChosenPath;
    // using course dimensions full field 200'x85'
    Node* start;
    Node* finish;
    
    Grid grid(85,85,1);// testing square section of 1 foot resolution
    printf("#####################adding all initial Nodes to Grid####################\n");
    grid.initializeGrid();
    printf("##################adding all initial neighbors to Nodes#################\n");
    grid.add_neighbors();
    printf("##################Lets Path (0,0) to (85,85)#################\n");
    start = grid.getNode(0,0);
    finish = grid.getNode(85,85);
    if((start!=NULL) &&(finish!=NULL)) {
        grid.findPath(start,finish);
    }
    grid.printPath();
    
    //clear navigation... may need to specifically tune/update for consecutive passes
    grid.clearParents();
    
    printf("##################Edge Removal Test#############################\n");
    start = grid.getNode(0,0);
    finish = grid.getNode(0,1);
    if((start!=NULL) &&(finish!=NULL)) {
        printf("Edges between before:\t\n");
        start->printNodeCord();
        start->printNeighbors();
        
        finish->printNodeCord();
        finish->printNeighbors();
        
        grid.removeEdge(start,finish);
        
        printf("Edges between after:\t\n");
        start->printNodeCord();
        start->printNeighbors();
        
        finish->printNodeCord();
        finish->printNeighbors();
        
//        ///rude test
//        start =grid.getNode(0,1);
//        finish= grid.getNode(0,2);
//        grid.removeEdge(start,finish );
//        
//        //hmm
//        start =grid.getNode(0,2);
//        finish= grid.getNode(0,3);
//        grid.removeEdge(start,finish );
    }
    
    
    
    printf("##################Lets Path (0,0) to (85,85),AGAIN#################\n");
    start = grid.getNode(0,0);
    finish = grid.getNode(85,85);
    if((start!=NULL) &&(finish!=NULL)) {
        theChosenPath =grid.findPath(start,finish);
    }
    grid.printPath();
    

    
    return 0;
}
#elif defined(FLIGHT_DEBUG) // error during implementations testing
int main() {
    std::vector<Node*> theChosenPath;
    // using course dimensions full field 200'x85'
    Node* start;
    Node* finish;
    int i;
    
    Grid grid(85,85,1);// testing square section of 1 foot resolution
    printf("#####################adding all initial Nodes to Grid####################\n");
    grid.initializeGrid();
    printf("##################adding all initial neighbors to Nodes#################\n");
    grid.add_neighbors();
    printf("##################removing edges troubling with##################\n");
    
    //errors when the shortest next target is blacklisted...
    for(i =0; i<11;i++) { //errors when these edges are removed from navigating to first target
        start = grid.getNode(0,i);
        finish = grid.getNode(1,i);
        grid.removeEdge(start,finish);
    }
    
    
    
    
    
    printf("##################Lets Path (0,0) to (10,10)#################\n");
    start = grid.getNode(0,0);
    finish = grid.getNode(10,10);
    if((start!=NULL) &&(finish!=NULL)) {
        grid.findPath(start,finish);
    }
    grid.printPath();
    
    //clear navigation... may need to specifically tune/update for consecutive passes
    grid.clearParents();
    
    printf("##################Lets Path (0,10) to (10,10)#################\n");
    start = grid.getNode(0,10);
    finish = grid.getNode(10,10);
    if((start!=NULL) &&(finish!=NULL)) {
        grid.findPath(start,finish);
    }
    grid.printPath();
    
    //clear navigation... may need to specifically tune/update for consecutive passes
    grid.clearParents();
    
    printf("##################Lets Path (0,11) to (10,10)#################\n");
    start = grid.getNode(0,11);
    finish = grid.getNode(10,10);
    if((start!=NULL) &&(finish!=NULL)) {
        grid.findPath(start,finish);
    }
    grid.printPath();
    
    //clear navigation... may need to specifically tune/update for consecutive passes
    grid.clearParents();
    
    

    return 0;
}
#endif
