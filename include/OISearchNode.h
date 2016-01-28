/*
 * OISearchNode.h
 *
 *  Created on: Dec 14, 2012
 *      Author: sprunkc
 */

#ifndef OISEARCHNODE_H_
#define OISEARCHNODE_H_

#include "SearchNode.h"
#include "simplisticmap.h"
#include "OINodeConnector.h"

class OISearchNode: public SearchNode {
public:
  OISearchNode();
  OISearchNode(int x, int y, unsigned int o);
  virtual ~OISearchNode();

  virtual SearchNode* getPredecessor() const;

  inline virtual void setPredecessor(SearchNode* predecessor){
      this->predecessor = predecessor;
  }

  virtual std::vector<SearchNode*>* getSuccessors();
  virtual std::vector<double>* getSuccessorCosts();
  virtual void computeSuccessors(SimplisticMap<OINodeConnector<OISearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations);
  virtual void addSuccessorsToSpecificNeighbor(int nx, int ny, SimplisticMap<OINodeConnector<OISearchNode> >* nodeMap, SimplisticMap<OrientationIntervals>* admissibleOrientations);
  virtual void computeHeuristic(SearchNode* goal);
  void recycle();

  int x;
  int y;
  unsigned int o;
  std::vector<SearchNode*> successors;
  std::vector<double> successor_costs;
  SearchNode* predecessor;
};

#endif /* OISEARCHNODE_H_ */
