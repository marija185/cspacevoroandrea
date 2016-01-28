/*
 * DStar.h
 *
 *  Created on: Jan 29, 2013
 *      Author: sprunkc
 */

#ifndef DSTAR_H_
#define DSTAR_H_

#include "DStarSearchNode.h"
//#include "OIDStarSearchNode.h"
#include "bucketedqueue.h"
#include <sys/time.h>

#define COSTDIAGONAL 14  
#define COSTSTRAIGHT 10     
#define COSTROTATION 0     


class DStarSearch {
public:
  DStarSearch();
  virtual ~DStarSearch();

  bool initialComputations(DStarSearchNode* Start, DStarSearchNode* Goal);
  std::vector<DStarSearchNode*>* getPath(DStarSearchNode* Start);
  bool update(const std::vector<DStarSearchNode*>& nodesWithChangedCosts);
  void setCurrentStart(DStarSearchNode* Start);
  void setCurrentGoal(DStarSearchNode* Goal);
  int nmbExplored;
  
  
private:

  void insertNode(DStarSearchNode* element, int h_new);
  void processState(DStarSearchNode* n);
  BucketPrioQueue<DStarSearchNode*> queue;

  DStarSearchNode* currentGoal;
  DStarSearchNode* currentStart;

};

#endif /* DSTAR_H_ */
