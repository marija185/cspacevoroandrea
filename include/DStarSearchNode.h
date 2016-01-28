/*
 * DStarSearchNode.h
 *
 *  Created on: Jan 25, 2013
 *      Author: sprunkc
 */

#ifndef DSTARSEARCHNODE_H_
#define DSTARSEARCHNODE_H_

#include <vector>
#include "OrientationIntervals.h"

#define NEW 0
#define OPEN 1
#define CLOSED -1
#define OBSTACLE 100000000

class DStarSearchNode {
public:
  DStarSearchNode();
  virtual ~DStarSearchNode();

  virtual DStarSearchNode* getNext() const = 0;
  virtual void setNext(DStarSearchNode*) = 0;
  virtual int getDesiredOrientation() const = 0;
  virtual void setDesiredOrientation(int) = 0;
  virtual std::vector<DStarSearchNode*>* getSuccessors() = 0;
  virtual std::vector<double>* getSuccessorCosts() = 0;
//  virtual std::vector<OrientationInterval>* getSuccessorIntersections() = 0;
  virtual std::vector<OrientationInterval> *getSuccessorIntersections() = 0;
  virtual const OrientationInterval& getOrientationInterval() = 0;

  virtual double getCosts() = 0;
  int k;
  int h;
  int tag;


};

#endif /* DSTARSEARCHNODE_H_ */
