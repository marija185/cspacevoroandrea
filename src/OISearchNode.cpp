/*
 * OISearchNode.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: sprunkc
 */

#include "OISearchNode.h"
#include <limits>

#include "MemoryManager.h"

template<> std::vector<OISearchNode*> MemoryManager<OISearchNode>::freeObjects = std::vector<OISearchNode*>();
template<> unsigned int MemoryManager<OISearchNode>::objectsInUse = 0;

#include "AStar.h"


OISearchNode::OISearchNode()
{
}



OISearchNode::OISearchNode(int x, int y, unsigned int o)
: x(x), y(y), o(o), predecessor(NULL)
{
  expanded=false;
  g = std::numeric_limits<int>::max();
  iteration = 0;
}

OISearchNode::~OISearchNode() {
  successors.clear(); //maybe this will silence valgrind
}

void OISearchNode::recycle(){
  predecessor=NULL;
  expanded=false;
  g = std::numeric_limits<int>::max();
  iteration = 0;
}

void OISearchNode::computeHeuristic(SearchNode* goal){
  int dx = abs(((OISearchNode*) goal)->x - x);
  int dy = abs(((OISearchNode*) goal)->x - x);
  h = std::max(dx,dy);
}

SearchNode* OISearchNode::getPredecessor() const{
//  assert(lastIteration == currentIteration);
  return predecessor;
}

void OISearchNode::computeSuccessors(SimplisticMap<OINodeConnector<OISearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations){
  successors.clear();
  successor_costs.clear();

  int xstart = (std::max)(0,x-1);
  int xend = (std::min)(x+1, nodeMap->getMapSizeX()-1);
  int ystart = (std::max)(0,y-1);
  int yend = (std::min)(y+1, nodeMap->getMapSizeY()-1);

  //TODO replace loop?make more efficient?
  OrientationInterval& myInterval = nodeMap->getCellReference(x,y).orientationIntervals[o];
  for(int ny=ystart; ny<=yend; ny++){
    for(int nx=xstart; nx<=xend; nx++){

      if((nx==x && ny==y) || (nx!=x && ny!=y)){
        continue;
      } else {

//        static std::vector<unsigned int> connectedIntervals;
//        admissibleOrientations->getCellReference(nx,ny).getIntervalIndicesWithNonEmptyIntersectionWithInterval(myInterval, &connectedIntervals);
//        for(std::vector<unsigned int>::const_iterator it=connectedIntervals.begin(), end=connectedIntervals.end(); it!=end; ++it){
//          successors.push_back(nodeMap->getCellReference(nx,ny).nodes[*it]);
//        }
        static std::vector<std::pair<unsigned int, unsigned int> > connectedIntervals;
        admissibleOrientations->getCellReference(nx,ny).getIntervalIndicesAndIntersectionSizeWithInterval(myInterval, &connectedIntervals);
        for(std::vector<std::pair<unsigned int, unsigned int> >::const_iterator it=connectedIntervals.begin(), end=connectedIntervals.end(); it!=end; ++it){
          successors.push_back(nodeMap->getCellReference(nx,ny).nodes[it->first]);
          //TODO could be more efficient when caching first summand
          successor_costs.push_back(OrientationIntervals::getMaxOrientation() - OrientationIntervals::getMinOrientation() + 1 - it->second + 1);
        }
      }
    }
  }
}

void OISearchNode::addSuccessorsToSpecificNeighbor(int nx, int ny, SimplisticMap<OINodeConnector<OISearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations){
//  static std::vector<unsigned int> connectedIntervals;
//  admissibleOrientations->getCellReference(nx,ny).getIntervalIndicesWithNonEmptyIntersectionWithInterval(nodeMap->getCellReference(x,y).orientationIntervals[o], &connectedIntervals);
//  for(std::vector<unsigned int>::const_iterator it=connectedIntervals.begin(), end=connectedIntervals.end(); it!=end; ++it){
//    successors.push_back(nodeMap->getCellReference(nx,ny).nodes[*it]);
//  }

  static std::vector<std::pair<unsigned int, unsigned int> > connectedIntervals;
  admissibleOrientations->getCellReference(nx,ny).getIntervalIndicesAndIntersectionSizeWithInterval(nodeMap->getCellReference(x,y).orientationIntervals[o], &connectedIntervals);
  for(std::vector<std::pair<unsigned int, unsigned int> >::const_iterator it=connectedIntervals.begin(), end=connectedIntervals.end(); it!=end; ++it){
    successors.push_back(nodeMap->getCellReference(nx,ny).nodes[it->first]);
    //TODO could be more efficient when caching first summand
    successor_costs.push_back(OrientationIntervals::getMaxOrientation() - OrientationIntervals::getMinOrientation() + 1 - it->second + 1);
  }
}

std::vector<SearchNode*>* OISearchNode::getSuccessors(){
  return &successors;
}

std::vector<double>* OISearchNode::getSuccessorCosts(){
	return &successor_costs;
}
