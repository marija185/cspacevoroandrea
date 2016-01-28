/*
 * SearchNode.h
 *
 *  Created on: Dec 11, 2012
 *      Author: sprunkc
 */

#ifndef SEARCHNODE_H_
#define SEARCHNODE_H_

#include <vector>

class SearchNode {
public:

public:
  SearchNode();
  virtual ~SearchNode();

  virtual SearchNode* getPredecessor() const = 0;
  virtual void setPredecessor(SearchNode*) = 0;
  virtual std::vector<SearchNode*>* getSuccessors() = 0;
  virtual std::vector<double>* getSuccessorCosts() = 0;
  virtual void computeHeuristic(SearchNode* goal) = 0;
  int g;
  int f;
  int h;
  bool expanded;
  unsigned int iteration;

};

#endif /* SEARCHNODE_H_ */
