/*
 * AStar.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: sprunkc
 */

#include "AStar.h"
#include <iostream>

AStar::AStar()
: currentIteration(1), currentGoal(NULL)
{

}

AStar::~AStar() {
}

std::vector<SearchNode*>* AStar::searchPath(SearchNode* start, SearchNode* goal){

  queue.clear();

  currentIteration++;

  currentGoal = goal;
  //put start in the queue
  start->computeHeuristic(currentGoal);
  start->g=0;
  start->f=start->h;
  start->expanded = false;
  start->iteration = currentIteration;
  queue.push(start->f, start);
  //while queue front != goal
  while(queue.empty()==false){
    SearchNode* n = queue.pop();
    if(n->iteration == currentIteration && n->expanded){
      continue;
    } else {
      if(n==currentGoal){
//        std::cerr<<"PATH FOUND"<<std::endl;

        std::vector<SearchNode*>* result = new std::vector<SearchNode*>();
        SearchNode* s = n;
        while(s!=NULL){
        	result->push_back(s);
        	s = s->getPredecessor();
        }
        return result;

      } else {
        expandNode(n);
      }
    }
  }

  std::cerr<<"search finished w/o result"<<std::endl;

  return NULL;
}

void AStar::expandNode(SearchNode* n){

  std::vector<SearchNode*>* successors = n->getSuccessors();
  std::vector<double>* successor_costs = n->getSuccessorCosts();

  for(std::vector<SearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){

    if((*it)->iteration != currentIteration){
      (*it)->iteration = currentIteration;
      (*it)->computeHeuristic(currentGoal);
      (*it)->expanded = false;

      //(*it)->g = n->g + 1;
      (*it)->g = n->g + successor_costs->at(it-successors->begin());
      (*it)->f = (*it)->g + (*it)->h;
      (*it)->setPredecessor(n);

      queue.push((*it)->f, *it);

    } else if (n->g+1 < (*it)->g) {
      (*it)->g = n->g + 1;
      (*it)->f = (*it)->g + (*it)->h;
      (*it)->setPredecessor(n);

      queue.push((*it)->f, *it);
    }

  }
  n->expanded=true;
}


unsigned int AStar::getCurrentIteration(){
  return currentIteration;
}
