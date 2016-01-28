/*
 * DStar.cpp
 *
 *  Created on: Jan 29, 2013
 *      Author: sprunkc
 */

#include "OIDStar.h"
#include <cmath>
#include "OIDStarSearchNode.h"


DStarSearch::DStarSearch()
: nmbExplored(0), currentGoal(NULL), currentStart(NULL)
{

}

DStarSearch::~DStarSearch() {
}


bool DStarSearch::initialComputations(DStarSearchNode* Start, DStarSearchNode* Goal){
  
  struct timeval timeStart;
  struct timeval timeNow;
  int mySecStart, myMSecStart,mySecNow, myMSecNow;
  int time_difference;
  if (gettimeofday(&timeStart, NULL) == 0)
  {
	  mySecStart = timeStart.tv_sec;
	  myMSecStart = timeStart.tv_usec / 1000;
  }
  currentGoal = Goal;
  currentStart = Start;

  queue.clear();
  nmbExplored=0;
  insertNode(Goal,0);
  int watchdog_counter=0;
  
  while (queue.empty()==false)
  {
    DStarSearchNode* n = queue.pop(); //for exhaustive search this is ok, but it should be top and after pop
    if(n->tag != OPEN){
      continue;
    }else{
	    processState(n);
	    ++watchdog_counter;
    }
  }
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	time_difference=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	printf("DStarSearch> exhaustive search = %d ms, watchdog=%d, No. explored nodes=%d\n",time_difference, watchdog_counter, nmbExplored);
	
//  DStarSearchNode* s = Start;
//  while(s!=NULL){
//   	if (s==currentGoal || std::isinf(s->getCosts())) break;
//   	s = s->getNext();
//  }
//  if (s!=NULL){
//  if (s!= currentGoal || std::isinf(s->getCosts()))
//    return false;
//  }
	return true;
}

void DStarSearch::insertNode(DStarSearchNode* element, int h_new)
{
  if (element->tag==NEW)
  {
	element->k = h_new;
  }
  else if (element->tag==OPEN)
  {
	element->k = std::min(element->k, h_new);
  }
  else //CLOSED
  {
	element->k = std::min(element->h, h_new);
  }
  element->h = h_new;
  element->tag = OPEN;
  nmbExplored++;
  queue.push(element->k, element);
  
}

void DStarSearch::processState(DStarSearchNode* n) {
	int k_val, h_val;
	int c, cellcost;
	double cost;
  std::vector<DStarSearchNode*>* successors = n->getSuccessors();
  std::vector<double>* successor_costs = n->getSuccessorCosts();
  std::vector<OrientationInterval>* successor_intersections = n->getSuccessorIntersections();
  OrientationInterval intersection;
  int middleOri, minUp, minLo;
  int maxOri = OrientationIntervals::getMaxOrientation();
  double angularResolution = OrientationIntervals::getAngularResolution();
  int goalOri = currentGoal->getDesiredOrientation();
  int x,y,nx,ny, travcost;
  double alfa_r;
  int alfa;
  int delta=(maxOri+1)/8; //0
  nx = ((OIDStarSearchNode*) (n))->x;
  ny = ((OIDStarSearchNode*) (n))->y;
  OrientationInterval ninterval = n->getOrientationInterval();
  int closestOrientation;
  int distance;

//  if (nx==88 && ny==60){
//    printf("node\n");
//  }
  
//  if (nx==51 && ny==62){
//    printf("node cost=%d\n", n->h);
//  }

	int h_point;
	n->tag = CLOSED;
	k_val = n->k;
	h_val = n->h;
//avoiding overflow
	if (h_val>OBSTACLE+1) h_val=OBSTACLE+1;
	if (k_val>OBSTACLE+1) k_val=OBSTACLE+1;

//  printf("n=(%d,%d) k_val=%d h_val=%d desiredorientation=%d\n", nx,ny,k_val,h_val,n->getDesiredOrientation());

	// RAISE STATE
	if (k_val < h_val)
	{
//	printf("raise\n");
    for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
      h_point = (*it)->h;
//avoiding overflow
    	if (h_point>OBSTACLE+1) h_point=OBSTACLE+1;
//      cost=std::max(n->getCosts(), (*it)->getCosts());
      cost=successor_costs->at(it-successors->begin());
      if (std::isinf(cost)) cost=OBSTACLE;
      if (cost==OBSTACLE)
        continue;
//      c=1+int(cost);
      c=int(round(cost)); //from 1 to maxOri+1
      cellcost=c;
      //cellcost=1;//when not using cost of the cell
//cost of the rotation between nodes      
      intersection=successor_intersections->at(it-successors->begin());
      int minD;
//for middle orientation path calculation
        int orisize=OrientationIntervals::getIntervalSize(intersection);
        if ( orisize == maxOri + 1){
          middleOri=n->getDesiredOrientation();
        }else{
          if (orisize % 2 == 0){
            middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minLo);
            middleOri = intersection.lower + orisize/2 -1;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minUp);
            if (minLo<minUp) middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
          }else{
            if (orisize == 1){
              middleOri = intersection.lower;
            }else{
              middleOri = intersection.lower + orisize/2;
              if (middleOri>maxOri) middleOri-=(maxOri+1);
            }
          }        
        }
//        if (intersection.lower<=intersection.upper){
//          middleOri=(int) floor((intersection.upper-intersection.lower)/2.)+intersection.lower;
//        }else{
//          middleOri= (int) floor((intersection.upper+maxOri+1-intersection.lower)/2.)+ intersection.lower;
//          if (middleOri>maxOri) middleOri-=(maxOri+1);
//        }
//        if (OrientationIntervals::containsOrientation(intersection, goalOri)){
//          middleOri=goalOri;
//        }
      OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minD);
      if (OrientationIntervals::getIntervalSize(intersection) < maxOri + 1){
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.lower,middleOri,minLo);
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.upper,middleOri,minUp);
        cellcost = maxOri+1 - 2*std::min(minLo,minUp);
      }
      cellcost=1;//when not using cost of the cell
//for optimal path calculation      
//      OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, n->getDesiredOrientation(), closestOrientation, distance);
      minD = distance;
      middleOri = closestOrientation;
      
      x = ((OIDStarSearchNode*) (*it))->x;
      y = ((OIDStarSearchNode*) (*it))->y;
      alfa_r=atan2((y-ny),(x-nx));
      if (alfa_r<0){
      	alfa_r=alfa_r+2*M_PI;
      }
      alfa=floor(alfa_r/angularResolution);
      if (!OrientationIntervals::containsOrientation((*it)->getOrientationInterval(),alfa)){
//      	alfa=(alfa+(maxOri+1)/2)%(maxOri+1);//rikverc
      	if (!OrientationIntervals::containsOrientation((*it)->getOrientationInterval(),alfa)){
      		continue;
      	}
      }

      OrientationIntervals::computeClosestOrientationInSubinterval((*it)->getOrientationInterval(), intersection, alfa, closestOrientation, distance);
      minD = distance;
      middleOri = closestOrientation;
      OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(), alfa, middleOri, distance);
      if (distance>delta){
//        alfa=(alfa+(maxOri+1)/2)%(maxOri+1);//rikverc
      	if (!OrientationIntervals::containsOrientation((*it)->getOrientationInterval(),alfa)){
      		continue;
      	}
        OrientationIntervals::computeClosestOrientationInSubinterval((*it)->getOrientationInterval(), intersection, alfa, closestOrientation, distance);
        minD = distance;
        middleOri = closestOrientation;
        OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(), alfa, middleOri, distance);
        if (distance>delta){
          continue;
        }
      }
      if (x==nx || y==ny){
        travcost=COSTSTRAIGHT;
      }else{
        travcost=COSTDIAGONAL;
      }
      if (abs(x-nx)+abs(y-ny)>2){
        printf("far neighbor (%d,%d) of (%d,%d)!!!!!\n",x,y,nx,ny);
        continue;
      }
      c = cellcost*(travcost) + (COSTROTATION*minD); //1 + minD; //c + minD; //c * (1+minD); 

      minLo=0;
#if 0
      if ((*it)->getDesiredOrientation()!=middleOri){
        if ((*it)->getNext()!=NULL){
//for optimal path calculation      
//          OrientationIntervals::computeClosestOrientationInSubinterval((*it)->getOrientationInterval(), intersection, (*it)->getDesiredOrientation(), closestOrientation, distance);
//          middleOri=closestOrientation;
//          minD=distance;

//for middle orientation path calculation
          if (orisize % 2 == 0){
            middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(),(*it)->getDesiredOrientation(),middleOri,minLo);
            middleOri = intersection.lower + orisize/2 -1;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(),(*it)->getDesiredOrientation(),middleOri,minUp);
            if (minLo<minUp) middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
          }       
      OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(),(*it)->getDesiredOrientation(),middleOri,minD);
      if (OrientationIntervals::getIntervalSize(intersection) == maxOri + 1)
        middleOri=(*it)->getDesiredOrientation();


          c=cellcost*(travcost)+(COSTROTATION*minD);

//          if (h_val > h_point +c){
//            printf("h_val=%d h_point=%d c=%d\n",h_val, h_point, c);
//            printf("c=%d, minUp=%d next cost=%d, next des ori=%d\n",c,minUp, ((*it)->getNext())->h, ((*it)->getNext())->getDesiredOrientation());
//          }
 
//          c=travcost+COSTROTATION*minD; //this is the cost of n if succ is its neighbor
//this line is commented because it produces infinite loop

        }
      }else{
        if ((*it)->getNext()!=NULL){
          minD=0; //because the new connection will go over h_point and desiredOrientation of n will be equal to p
//this fixes 8 more paths!!!
        }
      }
#endif
//  printf("p=(%d,%d) intersection=[%d,%d], h_point=%d desiredorientation=%d, newdesired=%d, minD=%d, c=%d\n", x, y, intersection.lower, intersection.upper, h_point, (*it)->getDesiredOrientation(), middleOri, minD,c);

	    if ( ((*it)->tag!=NEW) && (h_point <= k_val) && (h_val > h_point + c) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE))) 
		  {
		    n->setNext(*it);
		    //calculate new orientation of n - the closest one to the (*it)->getDesiredOrientation()
		    
		    n->setDesiredOrientation(middleOri);
      OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(), alfa, middleOri, distance);
      if (distance>delta){
      printf("1. raise: neighbor (%d,%d) of (%d,%d) angularRes=%f, alfa=%d, alfar=%f, middleOri=%d\n",x,y,nx,ny,angularResolution,alfa,alfa_r,middleOri);
      }

		    if (!OrientationIntervals::containsOrientation(ninterval,middleOri))
		      printf("PANIC ninterval=[%d,%d] middleOri=%d, intersection=[%d,%d], node n (%d,%d), node (*it) (%d,%d)\n",ninterval.lower, ninterval.upper, middleOri, intersection.lower, intersection.upper, nx,ny, x,y);
        c = cellcost*(travcost) + (COSTROTATION*minD);
        h_val = h_point + c;
        n->h = h_val;
//        printf("best (%d,%d) succ (%d,%d) new h_val=%d\n",nx,ny,x,y,h_val);
//avoiding overflow
       	if (h_val>OBSTACLE+1) h_val=OBSTACLE+1;
      }
    }
  }
	// LOWER STATE
  if (k_val == h_val)
	{
//	printf("lower\n");
    for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
      h_point = (*it)->h;
        //avoiding overflow
    	if (h_point>OBSTACLE+1) h_point=OBSTACLE+1;
      cost=successor_costs->at(it-successors->begin());
      if (std::isinf(cost)) cost=OBSTACLE;
      c=int(round(cost));
      cellcost=c;
      //cellcost=1;//when not using cost of the cell
      intersection=successor_intersections->at(it-successors->begin());
      int minD;
//for middle orientation path calculation
        int orisize=OrientationIntervals::getIntervalSize(intersection);
        if ( orisize == maxOri + 1){
          middleOri=n->getDesiredOrientation();
        }else{
          if (orisize % 2 == 0){
            middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minLo);
            middleOri = intersection.lower + orisize/2 -1;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minUp);
            if (minLo<minUp) middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
          }else{
            if (orisize == 1){
              middleOri = intersection.lower;
            }else{
              middleOri = intersection.lower + orisize/2;
              if (middleOri>maxOri) middleOri-=(maxOri+1);
            }
          }        
        }
//        if (((*it)->getDesiredOrientation()!=middleOri) && ((*it)->getNext()!=NULL)){
//          middleOri=(*it)->getDesiredOrientation();
//          if (!(OrientationIntervals::containsOrientation(intersection, middleOri))){
//            continue;
//          }
//        }
//        if (intersection.lower<=intersection.upper){
//          middleOri=(int) floor((intersection.upper-intersection.lower)/2.)+intersection.lower;
//        }else{
//          middleOri= (int) floor((intersection.upper+maxOri+1-intersection.lower)/2.)+ intersection.lower;
//          if (middleOri>maxOri) middleOri-=(maxOri+1);
//        }
//        if (OrientationIntervals::containsOrientation(intersection, goalOri)){
//          middleOri=goalOri;
//        }

      if (cost<OBSTACLE){

      OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minD);
      if (OrientationIntervals::getIntervalSize(intersection) < maxOri + 1){
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.lower,middleOri,minLo);
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.upper,middleOri,minUp);
        cellcost = maxOri+1 - 2*std::min(minLo,minUp);
      }
      cellcost=1;//when not using cost of the cell
      }//for c<OBSTACLE

//for optimal path calculation      
//      OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, n->getDesiredOrientation(), closestOrientation, distance);
//      minD = distance;
//      middleOri = closestOrientation;

      x = ((OIDStarSearchNode*) (*it))->x;
      y = ((OIDStarSearchNode*) (*it))->y;
      alfa_r=atan2((ny-y),(nx-x));
      if (alfa_r<0){
      	alfa_r=alfa_r+2*M_PI;
      }
      alfa=floor(alfa_r/angularResolution);
      if (!OrientationIntervals::containsOrientation(ninterval,alfa)){
//      	alfa=(alfa+(maxOri+1)/2)%(maxOri+1);//rikverc
        if (!OrientationIntervals::containsOrientation(ninterval,alfa)){
   	   	continue;
        }

      }

      OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, alfa, closestOrientation, distance);
      minD = distance;
      middleOri = closestOrientation;
      OrientationIntervals::computeOrientationDistanceInInterval(ninterval, alfa, middleOri, distance);
      if (distance>delta){
//        alfa=(alfa+(maxOri+1)/2)%(maxOri+1);//rikverc
        if (!OrientationIntervals::containsOrientation(ninterval,alfa)){
   	   	continue;
        }
        OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, alfa, closestOrientation, distance);
        minD = distance;
        middleOri = closestOrientation;
        OrientationIntervals::computeOrientationDistanceInInterval(ninterval, alfa, middleOri, distance);
        if (distance>delta){
          continue;
        }
      }
      if (x==nx || y==ny){
        travcost=COSTSTRAIGHT;
      }else{
        travcost=COSTDIAGONAL;
      }
      if (abs(x-nx)+abs(y-ny)>2){
        printf("far neighbor (%d,%d) of (%d,%d)!!!!!\n",x,y,nx,ny);
        continue;
      }
      if (cost<OBSTACLE)
      c = cellcost*(travcost) + (COSTROTATION*minD); //1 + minD; //c + minD; //c * (1+minD);      
      
      minLo=0;
#if 0
      if ((*it)->getDesiredOrientation()!=middleOri){
        if ((*it)->getNext()!=NULL){
//for optimal path calculation
//          OrientationIntervals::computeClosestOrientationInSubinterval((*it)->getOrientationInterval(), intersection, (*it)->getDesiredOrientation(), closestOrientation, distance);

//for middle orientation path calculation
      OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(),(*it)->getDesiredOrientation(),middleOri,distance);

          minLo = distance; //cost to the intersection interval
//          printf("c=%d, minUp=%d next cost=%d, next des ori=%d\n",c,minUp, ((*it)->getNext())->h, ((*it)->getNext())->getDesiredOrientation());
 
        }
      }
#endif
//  printf("n=(%d,%d) p=(%d,%d) intersection=[%d,%d], h_point=%d desiredorientation=%d, newdesired=%d, minD=%d, c=%d\n", nx,ny,x, y, intersection.lower, intersection.upper, h_point, (*it)->getDesiredOrientation(), middleOri, minD,c);

	    if ( (((*it)->tag==NEW) && (1 || c<OBSTACLE) && (1 || h_val<OBSTACLE)) || ( (((*it)->getNext()) == n) && (h_point != h_val + c) && (1||(h_point<OBSTACLE)||(h_val<OBSTACLE)) && (1||(h_point<OBSTACLE)||(c<OBSTACLE))) || ( ((*it)->getNext() != n) && (h_point + 0*minLo > h_val + c) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE))) )
		  {
		    (*it)->setNext(n);
		    (*it)->setDesiredOrientation(middleOri);
      OrientationIntervals::computeOrientationDistanceInInterval(ninterval, alfa, middleOri, distance);
      if (distance>delta){
      printf("lower: neighbor (%d,%d) of (%d,%d) angularRes=%f, alfa=%d, alfar=%f, middleOri=%d\n",x,y,nx,ny,angularResolution,alfa,alfa_r,middleOri);
      }

 		    if (!OrientationIntervals::containsOrientation((*it)->getOrientationInterval(),middleOri))
		      printf("PANIClower interval=[%d,%d] middleOri=%d, intersection=[%d,%d], ninterval=[%d,%d], node n (%d,%d), node (*it) (%d,%d)\n",(*it)->getOrientationInterval().lower, (*it)->getOrientationInterval().upper, middleOri, intersection.lower, intersection.upper, ninterval.lower, ninterval.upper, nx,ny, x,y);

  			insertNode((*it), h_val + c);
//  			        printf("best (%d,%d) succ (%d,%d) new h %d, old was =%d\n",nx,ny,x,y,h_val+c,h_point);
//  			printf("insert\n");
		  }
		  if (((*it)->getNext()==n) && (std::isinf(n->getCosts()))){
		    (*it)->setNext(NULL);
		  }
	  }
	}
  else        //RAISE STATE
  {
//	printf("else\n");
    for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
      h_point = (*it)->h;
        //avoiding overflow
    	if (h_point>OBSTACLE+1) h_point=OBSTACLE+1;
      cost=successor_costs->at(it-successors->begin());
      if (std::isinf(cost)) cost=OBSTACLE;
      c=int(round(cost));
      cellcost=c;
      //cellcost=1;//when not using cost of the cell
      intersection=successor_intersections->at(it-successors->begin());
      int minD;
//for middle orientation path calculation
        int orisize=OrientationIntervals::getIntervalSize(intersection);
        if ( orisize == maxOri + 1){
          middleOri=n->getDesiredOrientation();
        }else{
          if (orisize % 2 == 0){
            middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minLo);
            middleOri = intersection.lower + orisize/2 -1;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
            OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minUp);
            if (minLo<minUp) middleOri = intersection.lower + orisize/2;
            if (middleOri>maxOri) middleOri-=(maxOri+1);
          }else{
            if (orisize == 1){
              middleOri = intersection.lower;
            }else{
              middleOri = intersection.lower + orisize/2;
              if (middleOri>maxOri) middleOri-=(maxOri+1);
            }
          }        
        }
//        if (((*it)->getDesiredOrientation()!=middleOri) && ((*it)->getNext()!=NULL)){
//          middleOri=(*it)->getDesiredOrientation();
//          if (!(OrientationIntervals::containsOrientation(intersection, middleOri))){
//            continue;
//          }
//        }
//        if (intersection.lower<=intersection.upper){
//          middleOri=(int) floor((intersection.upper-intersection.lower)/2.)+intersection.lower;
//        }else{
//          middleOri= (int) floor((intersection.upper+maxOri+1-intersection.lower)/2.)+ intersection.lower;
//          if (middleOri>maxOri) middleOri-=(maxOri+1);
//        }
//        if (OrientationIntervals::containsOrientation(intersection, goalOri)){
//          middleOri=goalOri;
//        }

      if (cost<OBSTACLE){

      OrientationIntervals::computeOrientationDistanceInInterval(ninterval,n->getDesiredOrientation(),middleOri,minD);
      if (OrientationIntervals::getIntervalSize(intersection) < maxOri + 1){
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.lower,middleOri,minLo);
        OrientationIntervals::computeOrientationDistanceInInterval(intersection,intersection.upper,middleOri,minUp);
        cellcost = maxOri+1 - 2*std::min(minLo,minUp);
      }
      cellcost=1;//when not using cost of the cell
      }
//for optimal path calculation      
//      OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, n->getDesiredOrientation(), closestOrientation, distance);
//      minD = distance;
//      middleOri = closestOrientation;

      x = ((OIDStarSearchNode*) (*it))->x;
      y = ((OIDStarSearchNode*) (*it))->y;
      alfa_r=atan2((ny-y),(nx-x));
      if (alfa_r<0){
      	alfa_r=alfa_r+2*M_PI;
      }
      alfa=floor(alfa_r/angularResolution);
      if (!OrientationIntervals::containsOrientation(ninterval,alfa)){
//      	alfa=(alfa+(maxOri+1)/2)%(maxOri+1); //rikverc
        if (!OrientationIntervals::containsOrientation(ninterval,alfa)){
   	   	continue;
        }
      }

      OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, alfa, closestOrientation, distance);
      minD = distance;
      middleOri = closestOrientation;
      OrientationIntervals::computeOrientationDistanceInInterval(ninterval, alfa, middleOri, distance);
      if (distance>delta){
//        alfa=(alfa+(maxOri+1)/2)%(maxOri+1);//rikverc
        if (!OrientationIntervals::containsOrientation(ninterval,alfa)){
   	   	continue;
        }
        OrientationIntervals::computeClosestOrientationInSubinterval(ninterval, intersection, alfa, closestOrientation, distance);
        minD = distance;
        middleOri = closestOrientation;
        OrientationIntervals::computeOrientationDistanceInInterval(ninterval, alfa, middleOri, distance);
        if (distance>delta){
          continue;
        }
      }
      if (x==nx || y==ny){
        travcost=COSTSTRAIGHT;
      }else{
        travcost=COSTDIAGONAL;
      }
      if (abs(x-nx)+abs(y-ny)>2){
        printf("far neighbor (%d,%d) of (%d,%d)!!!!!\n",x,y,nx,ny);
        continue;
      }
      if (cost<OBSTACLE)
      c = cellcost*(travcost) + (COSTROTATION*minD); //1 + minD; //c + minD; //c * (1+minD); 

      minLo=0;
      int hc=c;
#if 0
      if ((*it)->getDesiredOrientation()!=middleOri){
        if ((*it)->getNext()!=NULL){
//for optimal path calculation      
//          OrientationIntervals::computeClosestOrientationInSubinterval((*it)->getOrientationInterval(), intersection, (*it)->getDesiredOrientation(), closestOrientation, distance);
//          minLo=distance; //cost to the intersection interval

//for middle orientation path calculation
      OrientationIntervals::computeOrientationDistanceInInterval((*it)->getOrientationInterval(),(*it)->getDesiredOrientation(),middleOri,distance);

      if (cost<OBSTACLE)
          hc=cellcost*(travcost)+(COSTROTATION*distance);

          

//          if (h_val > h_point + hc){
//            printf("h_val=%d h_point=%d c=%d\n",h_val, h_point, hc);
//            printf("hc=%d, minUp=%d next cost=%d, next des ori=%d\n",hc,minUp, ((*it)->getNext())->h, ((*it)->getNext())->getDesiredOrientation());
//          }

//          printf("c=%d, minUp=%d next cost=%d, next des ori=%d\n",c,minUp, ((*it)->getNext())->h, ((*it)->getNext())->getDesiredOrientation());
 
        }
      }
#endif      		
//  printf("p=(%d,%d) intersection=[%d,%d], h_point=%d desiredorientation=%d, newdesired=%d, minD=%d, c=%d\n", x, y, intersection.lower, intersection.upper, h_point, (*it)->getDesiredOrientation(), middleOri, minD,c);

    	if ( (((*it)->tag==NEW)&&(1||c<OBSTACLE)&&(1||h_val<OBSTACLE)) || ( ((*it)->getNext() == n) && (h_point != h_val + c) && (1||(h_point<OBSTACLE)||(h_val<OBSTACLE)) && (1||(h_point<OBSTACLE)||(c<OBSTACLE))) )
		  {

		    (*it)->setNext(n);
 		    (*it)->setDesiredOrientation(middleOri);//here c hasn't changed because it is for new or if n is next node
      OrientationIntervals::computeOrientationDistanceInInterval(ninterval, alfa, middleOri, distance);
      if (distance>delta){
      printf("2. raise: neighbor (%d,%d) of (%d,%d) angularRes=%f, alfa=%d, alfar=%f, middleOri=%d\n",x,y,nx,ny,angularResolution,alfa,alfa_r,middleOri);
      }

 		    if (!OrientationIntervals::containsOrientation((*it)->getOrientationInterval(),middleOri))
		      printf("PANICraise2 interval=[%d,%d] middleOri=%d, intersection=[%d,%d], ninterval=[%d,%d], node n (%d,%d), node (*it) (%d,%d)\n",(*it)->getOrientationInterval().lower, (*it)->getOrientationInterval().upper, middleOri, intersection.lower, intersection.upper, ninterval.lower, ninterval.upper, nx,ny, x,y);

 			  insertNode((*it), h_val + c);
//  			printf("insert (%d,%d) new cost %d\n",x,y,h_val+c);

		  }
		  else
      {
//		    if ( ((*it)->getNext() != n) && (h_point > h_val + c || h_point > h_val + travcost) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (n->tag == CLOSED))
		    if ( ((*it)->getNext() != n) && (h_point+0*minLo > h_val + c) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && (n->tag == CLOSED)) //with minLo we have more wrong costs...
		    {
  			  insertNode(n, h_val);
//  			printf("insert n (%d,%d)\n",nx,ny);
		    }
		    else
        {
//		      if ( ((*it)->getNext() != n) && (h_val > h_point + c || h_val > h_point + travcost + COSTROTATION*minD) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((*it)->tag == CLOSED) && (k_val<h_point))
		      if ( ((*it)->getNext() != n) && (h_val > h_point + hc) && (c<OBSTACLE) && ((h_point<OBSTACLE)||(h_val<OBSTACLE)) && ((*it)->tag == CLOSED) && (k_val<h_point))
          {
  		      insertNode((*it), h_point);
//  			printf("insert p with h_point (%d,%d)\n",x,y);
  	      }
			  }
		  }

 		  if (((*it)->getNext()==n) && (std::isinf(n->getCosts()))){
		    (*it)->setNext(NULL);
		  }

    }   
  }
}

std::vector<DStarSearchNode*>* DStarSearch::getPath(DStarSearchNode* Start){

  std::vector<DStarSearchNode*>* result = new std::vector<DStarSearchNode*>();
  DStarSearchNode* s = Start;
//  int cnt=0;
  int h;
  int x,y,nx,ny;
	if (s->h>=OBSTACLE){
	  printf("current Start has h %d\n",s->h);
	  return NULL;
	}
//  if (currentGoal->tag==NEW){
//    printf("new goal node!\n");
//    return NULL;
//  }

  while(s!=NULL){
   	result->push_back(s);
   	if (s==currentGoal || std::isinf(s->getCosts()) || s->h>=OBSTACLE) break;
//   	printf("(%d,%d,%d) k=%d h=%d\n",((OIDStarSearchNode*) s)->x, ((OIDStarSearchNode*) s)->y, s->getDesiredOrientation(), s->k, s->h);
    h=s->h;
    x = ((OIDStarSearchNode*) (s))->x;
    y = ((OIDStarSearchNode*) (s))->y;
   	s = s->getNext();
   	if (s!=NULL){

      nx = ((OIDStarSearchNode*) (s))->x;
      ny = ((OIDStarSearchNode*) (s))->y;

      if (abs(x-nx)+abs(y-ny)>2){
        printf("getPath: far next node (%d,%d) -> (%d,%d)!!!!!\n",x,y,nx,ny);
         	  for(unsigned int j=0; j<result->size(); j++){
         	    printf("%d %d %d %d %d %d %d\n",((OIDStarSearchNode*) (result->at(j)))->x, ((OIDStarSearchNode*) (result->at(j)))->y, (result->at(j))->getDesiredOrientation(), (result->at(j))->getOrientationInterval().lower, (result->at(j))->getOrientationInterval().upper, (result->at(j))->k, (result->at(j))->h);
         	  }
         	  return NULL;      		
      }

   	  if (s->h > h){
   	  printf("increase in cost h %d k=%d (%d,%d)\n",s->h,s->k,nx,ny);
//   	cnt++;
//   	if (cnt>1000){
//   	  cnt=0;
//   	  for(unsigned int i=0; i<result->size(); i++){
//      		if ((result->at(i))==s){
//         	  printf("loop in the path!\n");
         	  for(unsigned int j=0; j<result->size(); j++){
         	    printf("%d %d %d %d %d %d %d\n",((OIDStarSearchNode*) (result->at(j)))->x, ((OIDStarSearchNode*) (result->at(j)))->y, (result->at(j))->getDesiredOrientation(), (result->at(j))->getOrientationInterval().lower, (result->at(j))->getOrientationInterval().upper, (result->at(j))->k, (result->at(j))->h);
         	  }
         	  return NULL;      		
//      		}
      }
   	}
  }
  if (s!=NULL){
  if (std::isinf(s->getCosts()) || s->h>=OBSTACLE || s!= currentGoal)
  {
    printf("infinite cost %f h %d\n", s->getCosts(),s->h);
   	printf("(%d,%d,%d) k=%d h=%d\n",((OIDStarSearchNode*) s)->x, ((OIDStarSearchNode*) s)->y, s->getDesiredOrientation(), s->k, s->h);
    return NULL;
  }
  }
  return result;
}

bool DStarSearch::update(const std::vector<DStarSearchNode*>& nodesWithChangedCosts){

//if (std::isinf(currentGoal->getCosts())){
//  printf("currentGoal has infinity\n");
//  return false;
//} 

//  queue.clear();

  if (currentGoal->tag==NEW){
    printf("new goal node!\n");
//    return false;
    insertNode(currentGoal,0);
  }

  for(unsigned int i=0; i<nodesWithChangedCosts.size(); i++)
  {
    std::vector<DStarSearchNode*>* successors = nodesWithChangedCosts[i]->getSuccessors();
    if (nodesWithChangedCosts[i]->tag == CLOSED)
      insertNode(nodesWithChangedCosts[i], nodesWithChangedCosts[i]->h);
    for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
      if ((*it)->tag == CLOSED)
        insertNode((*it), (*it)->h);
//        if (((*it)->getCosts()==0)&&((nodesWithChangedCosts[i])->getCosts()==0)) printf("insert clear k=%d\n",(*it)->k);
    }  
//    if ((nodesWithChangedCosts[i])->getCosts()==0) printf("element cost %f tag %d k %d h %d\n", nodesWithChangedCosts[i]->getCosts(), nodesWithChangedCosts[i]->tag, nodesWithChangedCosts[i]->k, nodesWithChangedCosts[i]->h);
  }

  struct timeval timeStart;
  struct timeval timeNow;
  int mySecStart, myMSecStart,mySecNow, myMSecNow;
  int time_difference;
  if (gettimeofday(&timeStart, NULL) == 0)
  {
	  mySecStart = timeStart.tv_sec;
	  myMSecStart = timeStart.tv_usec / 1000;
  }

  nmbExplored=0;
  int watchdog_counter=0;
  int maxOri = OrientationIntervals::getMaxOrientation();
  int farneighbor,x,y,nx,ny;
  
  while (queue.empty()==false)
  {
  int topPrio = queue.getTopPriority();
    DStarSearchNode* n = queue.pop(); 
    if(topPrio != n->k){
        continue;
    }
    if(n->tag != OPEN){
      continue;
    }else{
//      std::vector<DStarSearchNode*>* successors = n->getSuccessors();
//      nx = ((OIDStarSearchNode*) (n))->x;
//      ny = ((OIDStarSearchNode*) (n))->y;
//      farneighbor=0;
//      for(std::vector<DStarSearchNode*>::iterator it=successors->begin(), end=successors->end(); it!=end; ++it){
//        x = ((OIDStarSearchNode*) (*it))->x;
//        y = ((OIDStarSearchNode*) (*it))->y;
//        if (abs(x-nx)+abs(y-ny)>1){
//          printf("far neighbor (%d,%d) of (%d,%d)!!!!!\n",x,y,nx,ny);
//          farneighbor=1;
//          break;
//        }
//      }
//      if (farneighbor==1) continue;
	    processState(n);
	    ++watchdog_counter;
      if (n->k>=OBSTACLE) {
        printf("lowest node on the queue has k>=OBSTACLE!\n");
        break;
      }
	    //for exhaustive put 0 instead of 1
   	  if (1&& (n->k > currentStart->h + 1*maxOri*COSTSTRAIGHT))// && (currentStart->h < OBSTACLE) && (currentStart->h==currentStart->k))
      {
        break;
      }
      if (watchdog_counter>OBSTACLE/100) {
        printf("too long!\n");
        break;
      }
    }
  }

//  while (queue.empty()==false)
//  {
//    DStarSearchNode* n = queue.pop(); 
//    n->tag=CLOSED;
//  }
  
	if (gettimeofday(&timeNow, NULL) == 0)
	{
		mySecNow = timeNow.tv_sec;
		myMSecNow = timeNow.tv_usec / 1000;
	}
	time_difference=(mySecNow-mySecStart)*1000+(myMSecNow-myMSecStart);
	printf("DStarSearch replanning> search = %d ms, watchdog=%d, No. explored nodes=%d\n",time_difference, watchdog_counter, nmbExplored);

//	DStarSearchNode* s = currentStart;
//	if (currentStart->h>=OBSTACLE){
//	  printf("current Start has h %d\n",currentStart->h);
//	  return false;
//	}
//  while(s!=NULL){
//   	if (s==currentGoal || std::isinf(s->getCosts()) || s->h>=OBSTACLE) break;
////   	printf("(%d,%d,%d) k=%d h=%d\n",((OIDStarSearchNode*) s)->x, ((OIDStarSearchNode*) s)->y, s->getDesiredOrientation(), s->k, s->h);
//   	s = s->getNext();
//  }
//  if (s!=NULL){
//  if (std::isinf(s->getCosts()) || s->h>=OBSTACLE || s!= currentGoal)
//  {
//    printf("infinite cost %f h %d\n", s->getCosts(),s->h);
//   	printf("(%d,%d,%d) k=%d h=%d\n",((OIDStarSearchNode*) s)->x, ((OIDStarSearchNode*) s)->y, s->getDesiredOrientation(), s->k, s->h);
//    return false;
//  }
//  }
	return true;
}

void DStarSearch::setCurrentStart(DStarSearchNode* Start){
  currentStart=Start;
}

void DStarSearch::setCurrentGoal(DStarSearchNode* Goal){
  currentGoal=Goal;
}

