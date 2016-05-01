/*
 * OIDStarSearchNode.h
 *
 *  Created on: Jan 28, 2013
 *      Author: sprunkc
 */

#ifndef OIDSTARSEARCHNODE_H_
#define OIDSTARSEARCHNODE_H_

#include "DStarSearchNode.h"
#include "simplisticmap.h"
#include "OrientationIntervals.h"
#include "OINodeConnector.h"

//#define DSTAR_EIGHT_CONNECTED 1

class OIDStarSearchNode: public DStarSearchNode {
public:
	OIDStarSearchNode();
	OIDStarSearchNode(int x, int y, unsigned int o);
	virtual ~OIDStarSearchNode();

  virtual DStarSearchNode* getNext() const{
  	return next;
  };

  inline virtual void setNext(DStarSearchNode* next){
      this->next = next;
  }

  inline virtual void setDesiredOrientation(int orientation){
      this->desiredOrientation = orientation;
  }
  virtual int getDesiredOrientation() const{
  	return desiredOrientation;
  };

  virtual std::vector<DStarSearchNode*>* getSuccessors();
  virtual std::vector<double>* getSuccessorCosts();
  virtual std::vector<OrientationInterval >* getSuccessorIntersections();
  virtual void computeSuccessors(SimplisticMap<OINodeConnector<OIDStarSearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations);
  virtual void addSuccessorsToSpecificNeighbor(int nx, int ny, SimplisticMap<OINodeConnector<OIDStarSearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations);
  virtual void computeHeuristic(DStarSearchNode* goal);
  virtual const OrientationInterval& getOrientationInterval();
  void recycle();
  virtual double getCosts(){
  	return costs;
  };

  int x;
  int y;
  unsigned int o;
  int desiredOrientation;
  std::vector<DStarSearchNode*> successors;
  std::vector<double> successor_costs;
  std::vector<OrientationInterval> successor_intersections;
  DStarSearchNode* next;
  double costs;
  OrientationInterval myInterval;
};

#endif /* OIDSTARSEARCHNODE_H_ */
