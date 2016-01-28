/*
 * OrientationIntervals.cpp
 *
 *  Created on: Nov 25, 2012
 *      Author: sprunkc
 */

#include "OrientationIntervals.h"
#include <assert.h>
#include <sstream>
#include <iostream>

int OrientationIntervals::minOrientation = 0;
int OrientationIntervals::maxOrientation = 0;
int OrientationIntervals::numOrientations = 0;
double OrientationIntervals::angularRes = 0;

OrientationIntervals::OrientationIntervals() {
  containsAllOrientations = false;
}

OrientationIntervals::~OrientationIntervals() {
}

void OrientationIntervals::addOrientation(int orientation) {
	assert(orientation >= minOrientation && orientation <= maxOrientation);

	OrientationInterval newInt;
	newInt.lower = orientation;
	newInt.upper = orientation;


	if(intervals.empty()){
		intervals.push_back(newInt);
	} else {
		//check if already contained in an interval, at the same time, search for right position to insert
		std::vector<OrientationInterval>::iterator it = intervals.begin();
		std::vector<OrientationInterval>::iterator end = intervals.end();
		while(it!=end && it->lower <= orientation){
			if(it->upper >= orientation){
				std::cerr<<"orientation "<<orientation<<" already contained in "<<print()<<std::endl;
				return;
			}
			++it;
		}

//		if(intervals.back().upper < intervals.back().lower && orientation < intervals.back().upper){
//			std::cerr<<"orientation "<<orientation<<" already contained in "<<print()<<std::endl;
//			return;
//		}

		it = intervals.insert(it, newInt);

		std::vector<OrientationInterval>::iterator previous = it;
		if(previous != intervals.begin()){
			previous --;
		} else {
			previous = intervals.end() - 1;
		}

		//std::cerr<<"previous "<<previous->lower<<" current "<<it->lower<<std::endl;
		it = checkMergeIntervalPrevious(previous, it);

		std::vector<OrientationInterval>::iterator next = it;
		if(it+1 != intervals.end()){
			next = it+1;
		} else {
			next = intervals.begin();
		}
		//std::cerr<<"current "<<it->lower<<" next "<<next->lower<<std::endl;
		it = checkMergeIntervalNext(it, next);

		checkFull();
	}
}

void OrientationIntervals::removeOrientation(int orientation) {
	assert(orientation >= minOrientation && orientation <= maxOrientation);


	//find interval that contains the orientation
//	std::vector<OrientationInterval>::iterator it = intervals.end();
//	if(intervals.empty()==false && orientation < intervals.front().lower){
//		//orientation can now only be in a wraparound interval at the end
//		if((intervals.back().lower > intervals.back().upper) && (intervals.back().upper >= orientation)){
//			it = intervals.end() - 1;
//		} else {
//			std::cerr<<"orientation "<<orientation<<" not contained in "<<print()<<std::endl;
//			return;
//		}
//	} else {
	  std::vector<OrientationInterval>::iterator it = intervals.begin();
	  std::vector<OrientationInterval>::iterator end = intervals.end();
		bool found=false;
		while(it!=end && it->lower <= orientation){
			if(it->upper >= orientation || it->upper < it->lower){
				found=true;
				break;
			}
			++it;
		}
		if(found==false){
			std::cerr<<"orientation "<<orientation<<" not contained in "<<print()<<std::endl;
			return;
		}
//	}

	//now it points to the interval that contains the orientation that is to be removed
		if(orientation == it->lower){
		  if(it->lower != it->upper){
		    it->lower = it->lower + 1;
		  } else {
		    intervals.erase(it);
		  }
		} else if (orientation == it->upper){
          if(it->lower != it->upper){
            it->upper = it->upper - 1;
          } else {
            intervals.erase(it);
          }
		} else {

		  OrientationInterval int_pre;
		  int_pre.lower = it->lower;
		  int_pre.upper = orientation - 1;;
		  it->lower = orientation + 1;
		  intervals.insert(it,int_pre);
		}

	containsAllOrientations = false;
}

bool OrientationIntervals::containsOrientation(int orientation){
	if(containsAllOrientations==true){
		return true;
	}

	if(intervals.empty()){
		return false;
	}

//	std::vector<OrientationInterval>::iterator it = intervals.end();
//	if(orientation < intervals.front().lower){
//		//orientation can now only be in a wraparound interval at the end
//		if((intervals.back().lower > intervals.back().upper) && (intervals.back().upper >= orientation)){
//			return true;
//		} else {
//			return false;
//		}
//	} else {
	std::vector<OrientationInterval>::iterator it = intervals.begin();
	std::vector<OrientationInterval>::iterator end = intervals.end();
		bool found=false;
		while(it!=end && it->lower <= orientation){
			if(it->upper >= orientation || it->upper < it->lower){
				found=true;
				break;
			}
			++it;
		}
		return found;
//	}
}

std::string OrientationIntervals::printOrientationIntervals(const std::vector<OrientationInterval> &orientation_intervals){
  std::stringstream result;
  for(unsigned int i=0; i<orientation_intervals.size(); i++){
      if(i>0)
          result<<", ";
      result<<"["<<orientation_intervals.at(i).lower<<","<<orientation_intervals.at(i).upper<<"]";
  }
  return result.str();

}

std::string OrientationIntervals::print() const {
  std::vector<OrientationInterval> orientations = getOrientationIntervals();
	return printOrientationIntervals(orientations);
}

std::string OrientationIntervals::printInternalRepresentation() const {
    return printOrientationIntervals(intervals);
}

bool OrientationIntervals::empty() {
	return intervals.empty();
}

bool OrientationIntervals::full() {
	return containsAllOrientations;
}

std::vector<OrientationInterval>::iterator OrientationIntervals::checkMergeIntervalPrevious(
		std::vector<OrientationInterval>::iterator previous,
		std::vector<OrientationInterval>::iterator current) {
	if(previous == current)
		return current;

	//if(previous->upper == current->lower-1 || (previous->upper==maxOrientation && current->lower==minOrientation)){
	if(previous->upper == current->lower-1){
		previous->upper = current->upper;

		if(previous == intervals.end() -1){
			//need to do this because previous iterator gets invalidated by erase in this case
			intervals.erase(current);
			return (intervals.end() - 1);
		} else {
			intervals.erase(current);
			return previous;
		}
	} else {
		return current;
	}
}

void OrientationIntervals::checkFull(){
//	if(intervals.size()==1){
//		if(intervals.front().lower == intervals.front().upper+1 || (intervals.front().lower==minOrientation && intervals.front().upper==maxOrientation)){
//			containsAllOrientations = true;
//			intervals.front().lower = minOrientation;
//			intervals.front().upper = maxOrientation;
//		} else {
//			containsAllOrientations = false;
//		}
//	} else {
//		containsAllOrientations = false;
//	}
  if(intervals.size()==1 && intervals.front().lower == minOrientation && intervals.front().upper == maxOrientation){
    containsAllOrientations = true;
  } else {
    containsAllOrientations = false;
  }
}

std::vector<OrientationInterval>::iterator OrientationIntervals::checkMergeIntervalNext(
		std::vector<OrientationInterval>::iterator current,
		std::vector<OrientationInterval>::iterator next) {
	if(current == next)
		return current;

	//if(current->upper == next->lower-1 || (current->upper == maxOrientation && next->lower == minOrientation)){
	if(current->upper == next->lower-1){
		current->upper = next->upper;
		if(current == intervals.end() - 1){
			//need to do this because current iterator gets invalidated by erase in this case
			intervals.erase(next);
			return (intervals.end() - 1);
		} else {
			intervals.erase(next);
			return current;
		}
	} else {
		return current;
	}
}

void OrientationIntervals::setToFull(){
  intervals.clear();
  OrientationInterval full;
  full.lower = minOrientation;
  full.upper = maxOrientation;
  intervals.push_back(full);
  containsAllOrientations = true;
}

std::vector<OrientationInterval> OrientationIntervals::getOrientationIntervals() const{
  if(!intervals.empty() && containsAllOrientations== false && intervals.front().lower==minOrientation && intervals.back().upper==maxOrientation){
    std::vector<OrientationInterval> result(intervals.begin()+1, intervals.end());
    result.back().upper = intervals.front().upper;
    return result;
  } else {
    return intervals;
  }

}

std::vector<OrientationInterval> OrientationIntervals::getIntersectionWithInterval(OrientationInterval interval) const{
	std::vector<OrientationInterval> result;

    if(containsAllOrientations){
        result.push_back(interval);
        return result;
    }

    if(intervals.empty()){
        return result;
    }

    if(interval.upper < interval.lower){
      //interval contains a wraparound

      OrientationInterval int_first;
      int_first.lower = minOrientation;
      int_first.upper = interval.upper;
      result = getIntersectionWithInterval(int_first);
      //std::cerr<<"first "<<printOrientationIntervals(result)<<std::endl;

      OrientationInterval int_second;
      int_second.lower = interval.lower;
      int_second.upper = maxOrientation;
      std::vector<OrientationInterval> intersect_two = getIntersectionWithInterval(int_second);
      //std::cerr<<"second "<<printOrientationIntervals(intersect_two)<<std::endl;

      if(result.empty()==false && intersect_two.empty()==false && result.back().upper +1 == intersect_two.front().lower){
        result.back().upper = intersect_two.front().upper;
        intersect_two.erase(intersect_two.begin());
      }

      result.insert(result.end(), intersect_two.begin(), intersect_two.end());
      //std::cerr<<"merge "<<printOrientationIntervals(result)<<std::endl;
    } else {
      for(std::vector<OrientationInterval>::const_iterator it=intervals.begin(), end=intervals.end(); it!= end; ++it){

        if(it->upper < interval.lower){
          continue;
        } else if(it->lower > interval.upper ){
          break;
        } else {
          OrientationInterval intersect;
          intersect.lower = (std::max)(it->lower, interval.lower);
          intersect.upper = (std::min)(it->upper, interval.upper);
          result.push_back(intersect);
        }
      }
    }


    //check if we need to establish a wraparound
    if(result.size() > 1 && result.front().lower==minOrientation && result.back().upper==maxOrientation){
      result.back().upper = result.front().upper;
      result.erase(result.begin());
    }
    //std::cerr<<"after wrapcheck "<<printOrientationIntervals(result)<<std::endl;

    return result;
}

void OrientationIntervals::getIntervalIndicesWithNonEmptyIntersectionWithInterval(const OrientationInterval interval, std::vector<unsigned int>* interval_indices) const{
  //basically a copy of the method above
  interval_indices->clear();

   if(containsAllOrientations){
     interval_indices->push_back(0);
     return;
   }

   if(intervals.empty()){
       return;
   }

   getInternalIntervalIndicesWithNonEmptyIntersectionWithInterval(interval, interval_indices);

   if(interval_indices->empty()==false){

     //check if we have a wraparound and therefore need to adapt indices
     if(intervals.size() > 1 && intervals.front().lower==minOrientation && intervals.back().upper==maxOrientation){
       //we have a wraparound in our intervals

       if(interval_indices->front()==0){
         //we have an intersection with the [0, whatever] part of the wraparound

         if(interval_indices->back()!=intervals.size()-1){
           //there is no intersection with the [something, maxorientation] part of the wraparound
           //therefore we will insert one as the two parts will be merged
           interval_indices->push_back(intervals.size()-1);
         }

         //->we need to delete the intersection with [0, whatever]
         interval_indices->erase(interval_indices->begin());
       }

       if(interval_indices->size() == 1 && interval_indices->front() == 0){
         //attention: intervals.size()-1 could be 0!
         //check for the case that interval_indices now contains only 0
         //if this is the case, we should not adapt the indices as done below!
         return;
       }

       //now we have to adapt the indices, as we do not count the wraparound interval as two intervals
       for(std::vector<unsigned int>::iterator it=interval_indices->begin(); it!=interval_indices->end(); ++it){
         (*it)--;
       }
     }
   }

//
//   std::cerr<<"indices: ";
//   for(unsigned int i=0; i<result.size(); i++){
//     std::cerr<<i<<",";
//   }
//   std::cerr<<std::endl;
   return;
}


void OrientationIntervals::getIntervalIndicesAndIntersectionSizeWithInterval(OrientationInterval interval, std::vector<std::pair<unsigned int, unsigned int> >* interval_indices) const {
  interval_indices->clear();

   if(containsAllOrientations){
     interval_indices->push_back(std::make_pair(0,getIntervalSize(interval)));
     return;
   }

   //TODO if interval contains all orientation, we could make our life easier here, then cut out some special stuff below
   //introduce a static method that checks if interval is full
   if(interval.lower == interval.upper + 1){
     interval.lower = minOrientation;
     interval.upper = maxOrientation;
   }

   if(intervals.empty()){
       return;
   }

   getInternalIntervalIndicesAndIntersectionSizeWithInterval(interval, interval_indices);

   if(interval_indices->empty()==false){
     //check if we have a wraparound and therefore need to adapt indices
     if(intervals.size() > 1 && intervals.front().lower==minOrientation && intervals.back().upper==maxOrientation){
       //we have a wraparound in our intervals

       if(interval_indices->front().first==0){
         //we have an intersection with the [0, whatever] part of the wraparound

         if(interval_indices->back().first!=intervals.size()-1){
           //there is no intersection with the [something, maxorientation] part of the wraparound
           //therefore we will insert one as the two parts will be merged
           interval_indices->push_back(std::make_pair(intervals.size()-1, interval_indices->front().second));
         } else {

        	 if(interval.lower==minOrientation && interval.upper==maxOrientation){
        		 interval_indices->back().second = interval_indices->front().second + interval_indices->back().second;
        	 } else {
        		 //interval is not a wraparound
        		 interval_indices->back().second = std::max(interval_indices->front().second, interval_indices->back().second);
        	 }
         }

         //->we need to delete the intersection with [0, whatever]
         interval_indices->erase(interval_indices->begin());
       }

       if(interval_indices->size() == 1 && interval_indices->front().first == 0){
         //attention: intervals.size()-1 could be 0!
         //check for the case that interval_indices now contains only 0
         //if this is the case, we should not adapt the indices as done below!
         return;
       }

       //now we have to adapt the indices, as we do not count the wraparound interval as two intervals
       for(std::vector<std::pair<unsigned int, unsigned int> >::iterator it=interval_indices->begin(); it!=interval_indices->end(); ++it){
         (*it).first--;
       }
     }
   }

   return;
}

void OrientationIntervals::getInternalIntervalIndicesWithNonEmptyIntersectionWithInterval(const OrientationInterval& interval, std::vector<unsigned int>* internal_interval_indices) const{
  if(interval.upper < interval.lower){
      //interval contains a wraparound

      OrientationInterval int_first;
      int_first.lower = minOrientation;
      int_first.upper = interval.upper;
      getInternalIntervalIndicesWithNonEmptyIntersectionWithInterval(int_first, internal_interval_indices);

      OrientationInterval int_second;
      int_second.lower = interval.lower;
      int_second.upper = maxOrientation;
      std::vector<unsigned int> intersect_two;
      getInternalIntervalIndicesWithNonEmptyIntersectionWithInterval(int_second, &intersect_two);

      if(intersect_two.empty() == false){
        //we need no concatenate the indices. Attention: the last of internal_interval_indices could be
        //identical to the first of intersect_two, so this extra effort here is needed to remove that

        std::vector<unsigned int>::iterator it = intersect_two.begin();
        if(internal_interval_indices->empty()==false){
          while(it != intersect_two.end() && *it == internal_interval_indices->back()){
            ++it;
          }
        }

        internal_interval_indices->insert(internal_interval_indices->end(), it, intersect_two.end());
      }

    } else {
      for(std::vector<OrientationInterval>::const_iterator it=intervals.begin(), end=intervals.end(); it!= end; ++it){

        if(it->upper < interval.lower){
          continue;
        } else if(it->lower > interval.upper ){
          break;
        } else {
          internal_interval_indices->push_back(it-intervals.begin());
        }
      }
    }

}

void OrientationIntervals::getInternalIntervalIndicesAndIntersectionSizeWithInterval(
    const OrientationInterval& interval,
    std::vector<std::pair<unsigned int, unsigned int> >* internal_interval_indices) const {
  if(interval.upper < interval.lower){
      //interval contains a wraparound

      OrientationInterval int_first;
      int_first.lower = minOrientation;
      int_first.upper = interval.upper;
      getInternalIntervalIndicesAndIntersectionSizeWithInterval(int_first, internal_interval_indices);

      OrientationInterval int_second;
      int_second.lower = interval.lower;
      int_second.upper = maxOrientation;
      std::vector<std::pair<unsigned int, unsigned int> > intersect_two;
      getInternalIntervalIndicesAndIntersectionSizeWithInterval(int_second, &intersect_two);

      //merge intersection that was cut apart due to wraparound
      if(internal_interval_indices->empty()==false && intersect_two.empty()==false && intervals.front().lower==minOrientation && intervals.back().upper==maxOrientation){
      	if(intersect_two.back().first == intervals.size()-1 && internal_interval_indices->front().first==0){
      		intersect_two.back().second = intersect_two.back().second + internal_interval_indices->front().second;
      		internal_interval_indices->erase(internal_interval_indices->begin());
      	}
      }

      if(intersect_two.empty() == false){
        //we need no concatenate the indices. Attention: the last of internal_interval_indices could be
        //identical to the first of intersect_two, so this extra effort here is needed to remove that

      	//also: we want to keep the width of the bigger intersection

        if(internal_interval_indices->empty()==false){
        	if(intersect_two.empty()==false && intersect_two.front().first == internal_interval_indices->back().first){
        		internal_interval_indices->back().second = std::max(internal_interval_indices->back().second, intersect_two.front().second);
        		intersect_two.erase(intersect_two.begin());
        	}
        }

        internal_interval_indices->insert(internal_interval_indices->end(), intersect_two.begin(), intersect_two.end());
      }

    } else {
      for(std::vector<OrientationInterval>::const_iterator it=intervals.begin(), end=intervals.end(); it!= end; ++it){

        if(it->upper < interval.lower){
          continue;
        } else if(it->lower > interval.upper ){
          break;
        } else {
          internal_interval_indices->push_back(std::make_pair(it-intervals.begin(), std::min(it->upper, interval.upper) - std::max(it->lower, interval.lower) + 1));
        }
      }
    }

}


void OrientationIntervals::getIntersectionsWithInterval(OrientationInterval interval, std::vector<std::pair<unsigned int, OrientationInterval> >* interval_indices) const{
  interval_indices->clear();

  if(containsAllOrientations){
    interval_indices->push_back(std::make_pair(0, interval));
    return;
  }

   //TODO if interval contains all orientation, we could make our life easier here, then cut out some special stuff below
   //introduce a static method that checks if interval is full
   if(interval.lower == interval.upper + 1){
     interval.lower = minOrientation;
     interval.upper = maxOrientation;
   }

   if(intervals.empty()){
     return;
   }

   getInternalIntersectionsWithInterval(interval, interval_indices);

   if(interval_indices->empty()==false){
     //check if we have a wraparound and therefore need to adapt indices
     if(intervals.size() > 1 && intervals.front().lower==minOrientation && intervals.back().upper==maxOrientation){
       //we have a wraparound in our intervals

       if(interval_indices->front().first==0){

         if(interval_indices->back().first ==intervals.size()-1 && interval_indices->front().second.lower == minOrientation && interval_indices->back().second.upper==maxOrientation){
           interval_indices->back().second.upper = interval_indices->front().second.upper;
           interval_indices->erase(interval_indices->begin());
         } else {
           interval_indices->front().first=intervals.size()-1;
         }

       }


/*       if(interval_indices->front().first==0){
         //we have an intersection with the [0, whatever] part of the wraparound

         //it needs to be moved to the end of the list
         interval_indices->push_back(std::make_pair(intervals.size()-1, interval_indices->front().second));

         //->we need to delete the intersection with [0, whatever]
         interval_indices->erase(interval_indices->begin());
       }*/

       if(interval_indices->size() == 1 && interval_indices->front().first == 0){
         //attention: intervals.size()-1 could be 0!
         //check for the case that interval_indices now contains only 0
         //if this is the case, we should not adapt the indices as done below!
         return;
       }

       //now we have to adapt the indices, as we do not count the wraparound interval as two intervals
       for(std::vector<std::pair<unsigned int, OrientationInterval> >::iterator it=interval_indices->begin(); it!=interval_indices->end(); ++it){
         (*it).first--;
       }
     }
   }

   return;

}

void OrientationIntervals::getInternalIntersectionsWithInterval(
    const OrientationInterval& interval,
    std::vector<std::pair<unsigned int, OrientationInterval> >* internal_interval_indices) const{

  if(interval.upper < interval.lower){
      //interval contains a wraparound

      OrientationInterval int_first;
      int_first.lower = minOrientation;
      int_first.upper = interval.upper;
      getInternalIntersectionsWithInterval(int_first, internal_interval_indices);

      OrientationInterval int_second;
      int_second.lower = interval.lower;
      int_second.upper = maxOrientation;
      std::vector<std::pair<unsigned int, OrientationInterval> > intersect_two;
      getInternalIntersectionsWithInterval(int_second, &intersect_two);

      //merge intersection that was cut apart due to wraparound
      if(internal_interval_indices->empty()==false && intersect_two.empty()==false && intervals.front().lower==minOrientation && intervals.back().upper==maxOrientation){
        if(intersect_two.back().first == intervals.size()-1 && internal_interval_indices->front().first==0){
          intersect_two.back().second.upper = internal_interval_indices->front().second.upper;
          internal_interval_indices->erase(internal_interval_indices->begin());
        }
      }

      internal_interval_indices->insert(internal_interval_indices->end(), intersect_two.begin(), intersect_two.end());

    } else {
      for(std::vector<OrientationInterval>::const_iterator it=intervals.begin(), end=intervals.end(); it!= end; ++it){

        if(it->upper < interval.lower){
          continue;
        } else if(it->lower > interval.upper ){
          break;
        } else {
          OrientationInterval tmp;
          tmp.lower = std::max(it->lower, interval.lower);
          tmp.upper = std::min(it->upper, interval.upper);
          internal_interval_indices->push_back(std::make_pair(it-intervals.begin(), tmp));
        }
      }
    }




}

void OrientationIntervals::setMinMaxOrientation(int minimumOrientation,
		int maximumOrientation, double angularResolution) {
	minOrientation = minimumOrientation;
	maxOrientation = maximumOrientation;
	numOrientations = maximumOrientation - minimumOrientation + 1;
        angularRes = angularResolution;
}

unsigned int OrientationIntervals::getIntervalSize(const OrientationInterval interval){
  if(interval.upper < interval.lower) {
  	//interval contains wraparound
  	return (maxOrientation - interval.lower + 1 + interval.upper - minOrientation + 1);
  } else {
  	return (interval.upper - interval.lower + 1);
  }
}

bool OrientationIntervals::containsOrientation(const OrientationInterval& interval, const int orientation){
  if(interval.upper < interval.lower) {
  	//interval contains wraparound
  	return (orientation >=interval.lower || orientation <= interval.upper);
  } else {
  	return (interval.lower<= orientation && interval.upper >= orientation);
  }

}

void OrientationIntervals::computeIntersection(const OrientationInterval& int_a, const OrientationInterval& int_b, std::vector<OrientationInterval>& intersection){
  intersection.clear();

  if(int_a.lower == int_a.upper+1 || (int_a.lower == minOrientation && int_a.upper == maxOrientation)){
    intersection.push_back(int_b);
    return;
  }

  if(int_b.lower == int_b.upper+1 || (int_b.lower == minOrientation && int_b.upper == maxOrientation)){
    intersection.push_back(int_a);
    return;
  }


  if(int_a.lower <= int_a.upper){
    if(int_b.lower <= int_b.upper){
      OrientationInterval tmp;
      tmp.lower = std::max(int_a.lower, int_b.lower);
      tmp.upper = std::min(int_a.upper, int_b.upper);
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }
    } else {
      //int b has a wraparound
      OrientationInterval tmp;
      tmp.lower = std::max(int_a.lower, int_b.lower);
      tmp.upper = int_a.upper;
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }
      tmp.lower = int_a.lower;
      tmp.upper = std::min(int_a.upper, int_b.upper);
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }
    }
  } else {
    //int a has a wraparound
    if(int_b.lower <= int_b.upper){
      OrientationInterval tmp;
      tmp.lower = std::max(int_a.lower, int_b.lower);
      tmp.upper = int_b.upper;
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }
      tmp.lower = int_b.lower;
      tmp.upper = std::min(int_a.upper, int_b.upper);
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }
    } else {
      //int b has a wraparound
      OrientationInterval tmp;
      tmp.lower = std::max(int_a.lower, int_b.lower);
      tmp.upper = std::min(int_a.upper, int_b.upper);
      //should always be an intersection
      intersection.push_back(tmp);

      tmp.lower = int_a.lower;
      tmp.upper = int_b.upper;
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }

      tmp.lower = int_b.lower;
      tmp.upper = int_a.upper;
      if(tmp.lower <= tmp.upper){
        intersection.push_back(tmp);
      }

    }
  }

  return;
}


void OrientationIntervals::computeClosestOrientationInSubinterval(const OrientationInterval& interval, const OrientationInterval& sub_interval, const int startOrientation, int& closestOrientation, int& abs_distance){
  if(containsOrientation(sub_interval, startOrientation)){
    closestOrientation = startOrientation;
    abs_distance = 0;
  } else if(OrientationIntervals::getIntervalSize(interval) == maxOrientation + 1){
    //interval contains all orientations.

    int dist_lower = std::min(abs(startOrientation - sub_interval.lower), std::min(abs(startOrientation + maxOrientation + 1 - sub_interval.lower), abs(startOrientation - (sub_interval.lower + maxOrientation + 1))));
    int dist_upper = std::min(abs(startOrientation - sub_interval.upper), std::min(abs(startOrientation + maxOrientation + 1 - sub_interval.upper), abs(startOrientation - (sub_interval.upper + maxOrientation + 1))));

    if(dist_lower < dist_upper){
      closestOrientation = sub_interval.lower;
      abs_distance = dist_lower;
    } else {
      closestOrientation = sub_interval.upper;
      abs_distance = dist_upper;
    }

  } else {
    //now: we want to go to one of the borders of the subinterval, question is: which one is the closest, watch out for wraparound intervals


    if(interval.lower > interval.upper){
      //we have a wraparound in interval
      int temp_startOrientation = startOrientation;
      if(startOrientation <= interval.upper){
        temp_startOrientation += maxOrientation + 1;
      }

      int temp_sub_interval_lower = sub_interval.lower;
      int temp_sub_interval_upper = sub_interval.upper;
      if(sub_interval.lower <= interval.upper){
        temp_sub_interval_lower += maxOrientation + 1;
      }
      if(sub_interval.upper <= interval.upper){
        temp_sub_interval_upper += maxOrientation + 1;
      }

      int dist_lower = abs(temp_startOrientation - temp_sub_interval_lower);
      int dist_upper = abs(temp_startOrientation - temp_sub_interval_upper);

      if(dist_lower < dist_upper){
        closestOrientation = sub_interval.lower;
        abs_distance = dist_lower;
      } else {
        closestOrientation = sub_interval.upper;
        abs_distance = dist_upper;
      }

    } else {
      //now wraparound in interval
      int dist_lower = abs(startOrientation - sub_interval.lower);
      int dist_upper = abs(startOrientation - sub_interval.upper);

      if(dist_lower < dist_upper){
        closestOrientation = sub_interval.lower;
        abs_distance = dist_lower;
      } else {
        closestOrientation = sub_interval.upper;
        abs_distance = dist_upper;
      }
    }
  }

  return;

}

void OrientationIntervals::computeOrientationDistanceInInterval(const OrientationInterval& interval, const int firstOrientation, const int secondOrientation, int& abs_distance){

  int tempfirst = firstOrientation;
  int tempsecond = secondOrientation;
  
  if(OrientationIntervals::getIntervalSize(interval) == maxOrientation + 1){
    //interval contains all orientations.

    abs_distance = std::min(abs(tempfirst - tempsecond), std::min(abs(tempfirst + maxOrientation + 1 - tempsecond), abs(tempfirst - (tempsecond + maxOrientation + 1))));
  } else {
  
    if (tempfirst<interval.lower){
      tempfirst+=maxOrientation+1;
    }
    if (tempsecond<interval.lower){
      tempsecond+=maxOrientation+1;
    }
    abs_distance = abs(tempfirst-tempsecond);
  
  }

  return;

}

