/*
 * AStar.h
 *
 *  Created on: Dec 14, 2012
 *      Author: sprunkc
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include "SearchNode.h"
#include "bucketedqueue.h"

class AStar {
public:
  AStar();
  virtual ~AStar();

  std::vector<SearchNode*>* searchPath(SearchNode* start, SearchNode* goal);

  unsigned int getCurrentIteration();

private:
  void expandNode(SearchNode* n);

  unsigned int currentIteration;
  SearchNode* currentGoal;

  BucketPrioQueue<SearchNode*> queue;
};

#endif /* ASTAR_H_ */
