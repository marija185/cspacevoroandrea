/*
 * OIDStarSearchNode.cpp
 *
 *  Created on: Jan 28, 2013
 *      Author: sprunkc
 */

#include "OIDStarSearchNode.h"
#include "MemoryManager.h"

template<> std::vector<OIDStarSearchNode*> MemoryManager<OIDStarSearchNode>::freeObjects = std::vector<OIDStarSearchNode*>();
template<> unsigned int MemoryManager<OIDStarSearchNode>::objectsInUse = 0;

OIDStarSearchNode::OIDStarSearchNode() {
	// TODO Auto-generated constructor stub

}

OIDStarSearchNode::OIDStarSearchNode(int x, int y, unsigned int o)
: x(x), y(y), o(o), next(NULL)
{
//  expanded=false;
  tag=NEW;
  h=OBSTACLE;
  k=h;
  desiredOrientation = -99;
}


OIDStarSearchNode::~OIDStarSearchNode(){

}

void OIDStarSearchNode::recycle(){
  next=NULL;
//  expanded=false;
  tag=NEW;
  h=OBSTACLE;
  k=h;
desiredOrientation = -88;
}

void OIDStarSearchNode::computeHeuristic(DStarSearchNode* goal){
std::cerr<<"D* heuristic not implemented"<<std::endl;
  //h = 0;
}

void OIDStarSearchNode::computeSuccessors(SimplisticMap<OINodeConnector<OIDStarSearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations){
  successors.clear();
  successor_costs.clear();
  successor_intersections.clear();

  int xstart = (std::max)(0,x-1);
  int xend = (std::min)(x+1, nodeMap->getMapSizeX()-1);
  int ystart = (std::max)(0,y-1);
  int yend = (std::min)(y+1, nodeMap->getMapSizeY()-1);

  static int maxIntersectionWidth = OrientationIntervals::getMaxOrientation() - OrientationIntervals::getMinOrientation() + 1;

  //TODO replace loop?make more efficient?
  myInterval = nodeMap->getCellReference(x,y).orientationIntervals[o];
  for(int ny=ystart; ny<=yend; ny++){
    for(int nx=xstart; nx<=xend; nx++){

#ifdef DSTAR_EIGHT_CONNECTED
      if((nx==x && ny==y)){
#else
      if((nx==x && ny==y) || (nx!=x && ny!=y)){
#endif
        continue;
      } else {

        static std::vector<std::pair<unsigned int, OrientationInterval> > connectedIntervals;
        admissibleOrientations->getCellReference(nx,ny).getIntersectionsWithInterval(myInterval, &connectedIntervals);

        for(std::vector<std::pair<unsigned int, OrientationInterval> >::const_iterator it=connectedIntervals.begin(), end=connectedIntervals.end(); it!=end; ++it){
          successors.push_back(nodeMap->getCellReference(nx,ny).nodes[it->first]);
          successor_costs.push_back(maxIntersectionWidth - OrientationIntervals::getIntervalSize(it->second) + 1);
          successor_intersections.push_back(it->second);
        }
      }
    }
  }
}

void OIDStarSearchNode::addSuccessorsToSpecificNeighbor(int nx, int ny, SimplisticMap<OINodeConnector<OIDStarSearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations){
  static int maxIntersectionWidth = OrientationIntervals::getMaxOrientation() - OrientationIntervals::getMinOrientation() + 1;
  static std::vector<std::pair<unsigned int, OrientationInterval> > connectedIntervals;
  admissibleOrientations->getCellReference(nx,ny).getIntersectionsWithInterval(nodeMap->getCellReference(x,y).orientationIntervals[o], &connectedIntervals);
  for(std::vector<std::pair<unsigned int, OrientationInterval> >::const_iterator it=connectedIntervals.begin(), end=connectedIntervals.end(); it!=end; ++it){
    successors.push_back(nodeMap->getCellReference(nx,ny).nodes[it->first]);
    successor_costs.push_back(maxIntersectionWidth - OrientationIntervals::getIntervalSize(it->second) + 1);
    successor_intersections.push_back(it->second);
  }
}

std::vector<DStarSearchNode*>* OIDStarSearchNode::getSuccessors(){
  return &successors;
}

std::vector<double>* OIDStarSearchNode::getSuccessorCosts(){
    return &successor_costs;
}
std::vector<OrientationInterval >* OIDStarSearchNode::getSuccessorIntersections(){
    return &successor_intersections;
}

const OrientationInterval& OIDStarSearchNode::getOrientationInterval(){
  return myInterval;
}
