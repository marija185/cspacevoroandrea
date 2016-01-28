/*
 * test_orientationintervals.cpp
 *
 *  Created on: Nov 25, 2012
 *      Author: sprunkc
 */

#include "OrientationIntervals.h"
#include <iostream>
#include <limits>

OrientationIntervals getRandomOrientationIntervals(){
	int maxOri = OrientationIntervals::getMaxOrientation();
	OrientationIntervals ints;

		bool setToFull = (rand()%100) < 2;

		if(setToFull){
			ints.setToFull();
		} else {

			int numOris = rand()%(maxOri+1);
			for(int i=0; i<numOris; i++){
				int ori = rand()%(maxOri+1);
				ints.addOrientation(ori);
			}
		}
		return ints;
}

OrientationInterval getRandomOrientationInterval(bool normalizeFullInterval=true){
	int maxOri = OrientationIntervals::getMaxOrientation();
	OrientationInterval interval;
	interval.lower = rand()%(maxOri+1);
	interval.upper = rand()%(maxOri+1);

/*	if(interval.lower==7 && interval.upper==6){
		interval.lower=0;
		interval.upper=maxOri;
	}*/

	if(normalizeFullInterval){
		if(interval.lower == interval.upper + 1){
			interval.lower = 0;
			interval.upper = maxOri;
		}
	}
	return interval;
}

int getRandomOrientationFromInterval(OrientationInterval interval){

  int size = OrientationIntervals::getIntervalSize(interval);
  int offset = rand()%size;
  int maxOri = OrientationIntervals::getMaxOrientation();

  int result = interval.lower + offset;
  if(result > maxOri){
    result -= maxOri+1;
  }
  return result;
}

bool checkOrientationIntervals(OrientationIntervals& ints){
	std::vector<OrientationInterval> myintervals = ints.getOrientationIntervals();

	if(myintervals.empty() == false){
		for(unsigned int i=0; i<myintervals.size()-1; i++){
			if(myintervals[i].upper == myintervals[i+1].lower){
				std::cerr<<"Intervals are corrupted."<<std::endl;
				std::cerr<<"Intervals: "<<OrientationIntervals::printOrientationIntervals(myintervals)<<std::endl;
				return false;
			}
		}
	}
	return true;
}

bool checkIntervalIntersection(OrientationInterval& interval, OrientationIntervals& ints){
	std::vector<OrientationInterval> intersects = ints.getIntersectionWithInterval(interval);

	std::cerr<<"Intersections: "<<OrientationIntervals::printOrientationIntervals(intersects)<<"\n";

	int maxOri = OrientationIntervals::getMaxOrientation();

	for(int o=OrientationIntervals::getMinOrientation(); o<=maxOri; o++){

		bool containedInIntersects = false;
		for(unsigned int i=0; i<intersects.size(); i++){
			if(OrientationIntervals::containsOrientation(intersects[i], o)){
				containedInIntersects = true;
				break;
			}
		}

		bool containedInInterval = OrientationIntervals::containsOrientation(interval, o);

		bool containedInIntervals = ints.containsOrientation(o);

		if((containedInInterval && containedInIntervals) != containedInIntersects){
			std::cerr<<"Orientation "<<o<<" is not correctly contained in intersections: "<<OrientationIntervals::printOrientationIntervals(intersects)<<"\n";
			std::cerr<<"containedInIntersects: "<<containedInIntersects<<" containedInIntervals: "<<containedInIntervals<<" containedInInterval: "<<containedInInterval<<std::endl;
			return false;
		}


	}
	return true;
}

bool checkIntervalIntersectionIndices(OrientationInterval& interval, OrientationIntervals& ints){
	std::vector<unsigned int> intersect_indices;
	ints.getIntervalIndicesWithNonEmptyIntersectionWithInterval(interval, &intersect_indices);

	std::cerr<<"Intersection indices:";
	for(unsigned int i=0; i<intersect_indices.size(); i++){
		std::cerr<<" "<<intersect_indices[i];
	}
	std::cerr<<"\n";

	int maxOri = OrientationIntervals::getMaxOrientation();

	std::vector<unsigned int> brute_indices;
	std::vector<OrientationInterval> myintervals = ints.getOrientationIntervals();

	for(unsigned int idx=0; idx<myintervals.size(); idx++){
		OrientationInterval myinterval = myintervals[idx];

		bool cont = false;
		for(int o=0; o<=maxOri; o++){
			if(OrientationIntervals::containsOrientation(myinterval, o) && OrientationIntervals::containsOrientation(interval, o)){
				brute_indices.push_back(idx);
				cont = true;
				break;
			}
		}

		if(cont)
			continue;
	}

	if(brute_indices.size() == intersect_indices.size()){
		for(unsigned int i=0; i<brute_indices.size(); i++){
			if(brute_indices[i] != intersect_indices[i]){
				std::cerr<<"Indices do not match"<<std::cerr;
				return false;
			}
		}

	} else {
		std::cerr<<"Indices do not match (size)"<<std::cerr;
		return false;
	}

	return true;
}

bool checkIntervalIntersectionIndicesWithSize(OrientationInterval& interval, OrientationIntervals& ints){
	std::vector<std::pair<unsigned int, unsigned int> > intersect_indices;
	ints.getIntervalIndicesAndIntersectionSizeWithInterval(interval, &intersect_indices);

	std::cerr<<"Intersection indices w/size:";
	for(unsigned int i=0; i<intersect_indices.size(); i++){
		std::cerr<<" "<<intersect_indices[i].first<<":"<<intersect_indices[i].second;
	}
	std::cerr<<"\n";


	std::vector<OrientationInterval> myintervals = ints.getOrientationIntervals();
	std::vector<std::pair<unsigned int, unsigned int> > brute_indices;
	int maxOri = OrientationIntervals::getMaxOrientation();
	for(unsigned int idx=0; idx<myintervals.size(); idx++){

		OrientationInterval myint = myintervals[idx];

		OrientationIntervals temp_ints;
		int o = myint.lower;

		while(true){
			temp_ints.addOrientation(o);
			if(o == myint.upper){
				break;
			}
			o++;
			if(o > maxOri){
				o = 0;
			}
		}

		std::vector<OrientationInterval> local_intersects = temp_ints.getIntersectionWithInterval(interval);
		int size=0;
		for(unsigned int i=0; i<local_intersects.size(); i++){
			size = std::max(size, (int) OrientationIntervals::getIntervalSize(local_intersects[i]));
		}
		if(local_intersects.size() > 0){
			brute_indices.push_back(std::make_pair(idx,size));
		}
	}

	if(brute_indices.size() == intersect_indices.size()){
		for(unsigned int i=0; i<brute_indices.size(); i++){
			if(brute_indices[i].first != intersect_indices[i].first || brute_indices[i].second != intersect_indices[i].second){
				std::cerr<<"indices mismatch (content).\n"<<std::endl;
				std::cerr<<"reference indices:\n";
				for(unsigned int i=0; i<brute_indices.size(); i++){
						std::cerr<<" "<<brute_indices[i].first<<":"<<brute_indices[i].second;
					}
					std::cerr<<"\n";
				return false;
			}
		}
	} else {
		std::cerr<<"indices mismatch (size)\n"<<std::endl;
        std::cerr<<"reference indices:\n";
        for(unsigned int i=0; i<brute_indices.size(); i++){
          std::cerr<<" "<<brute_indices[i].first<<":"<<brute_indices[i].second;
        }
        std::cerr<<"\n";

		return false;
	}

	return true;
}


bool checkIntervalIntersectionIndicesAndIntervals(OrientationInterval& interval, OrientationIntervals& ints){
    std::vector<std::pair<unsigned int, OrientationInterval> > intersect_indices;
    ints.getIntersectionsWithInterval(interval, &intersect_indices);

    std::cerr<<"Intersection indices w/interval:";
    for(unsigned int i=0; i<intersect_indices.size(); i++){
        std::cerr<<" "<<intersect_indices[i].first<<":["<<intersect_indices[i].second.lower<<","<<intersect_indices[i].second.upper<<"]";
    }
    std::cerr<<"\n";


    std::vector<OrientationInterval> myintervals = ints.getOrientationIntervals();
    std::vector<std::pair<unsigned int, OrientationInterval> > brute_indices;
    int maxOri = OrientationIntervals::getMaxOrientation();
    for(unsigned int idx=0; idx<myintervals.size(); idx++){

        OrientationInterval myint = myintervals[idx];

        OrientationIntervals temp_ints;
        int o = myint.lower;

        while(true){
            temp_ints.addOrientation(o);
            if(o == myint.upper){
                break;
            }
            o++;
            if(o > maxOri){
                o = 0;
            }
        }

        std::vector<OrientationInterval> local_intersects = temp_ints.getIntersectionWithInterval(interval);
        for(unsigned int i=0; i<local_intersects.size(); i++){
          brute_indices.push_back(std::make_pair(idx,local_intersects[i]));
        }
    }

    if(brute_indices.size() == intersect_indices.size()){
        for(unsigned int i=0; i<brute_indices.size(); i++){
          bool found_intersect=false;
          for(unsigned int j=0; j<intersect_indices.size(); j++){
            if(brute_indices[i].first == intersect_indices[j].first && brute_indices[i].second.lower == intersect_indices[j].second.lower && brute_indices[i].second.upper == intersect_indices[j].second.upper){
              found_intersect=true;
              break;
            }
          }

          if(found_intersect==false){
            std::cerr<<"indices comparison mismatch (content).\n"<<std::endl;
            std::cerr<<"reference indices:\n";
            for(unsigned int idx=0; idx<brute_indices.size(); idx++){
              std::cerr<<" "<<brute_indices[idx].first<<":["<<brute_indices[idx].second.lower<<","<<brute_indices[idx].second.upper<<"]";
            }
            std::cerr<<"\n";
            return false;
          }
        }
    } else {
        std::cerr<<"indices mismatch (size)\n"<<std::endl;
        std::cerr<<"reference indices:\n";
        for(unsigned int idx=0; idx<brute_indices.size(); idx++){
          std::cerr<<" "<<brute_indices[idx].first<<":["<<brute_indices[idx].second.lower<<","<<brute_indices[idx].second.upper<<"]";
        }
        std::cerr<<"\n";
        return false;
    }

    return true;
}

bool checkPureIntervalIntersection(const OrientationInterval& int_a, const OrientationInterval& int_b){

  int maxOri = OrientationIntervals::getMaxOrientation();

  std::vector<OrientationInterval> intersections;
  OrientationIntervals::computeIntersection(int_a, int_b, intersections);

  //now compute it another way;
  OrientationIntervals temp_ints;
  int o = int_a.lower;
  while(true){
    temp_ints.addOrientation(o);
    if(o == int_a.upper){
      break;
    }
    o++;
    if(o > maxOri){
      o = 0;
    }
  }

  std::vector<OrientationInterval> intersections_brute = temp_ints.getIntersectionWithInterval(int_b);

  //now compare
  if(intersections.size() != intersections_brute.size()){
    std::cerr<<"Error computing intersection between two pure intervals: size mismatch."<<std::endl;
    std::cerr<<"int_a ["<<int_a.lower<<","<<int_a.upper<<"] int_b ["<<int_b.lower<<","<<int_b.upper<<"]"<<std::endl;
    std::cerr<<"Intersections: "<<OrientationIntervals::printOrientationIntervals(intersections)<<std::endl;
    std::cerr<<"Intersections brute: "<<OrientationIntervals::printOrientationIntervals(intersections_brute)<<std::endl;
    return false;
  } else {
    for(unsigned int i=0; i<intersections.size(); i++){
      if(intersections[i].lower != intersections[i].lower || intersections[i].upper != intersections[i].upper){
        std::cerr<<"Error computing intersection between two pure intervals: interval mismatch."<<std::endl;
        std::cerr<<"int_a ["<<int_a.lower<<","<<int_a.upper<<"] int_b ["<<int_b.lower<<","<<int_b.upper<<"]"<<std::endl;
        std::cerr<<"Intersections: "<<OrientationIntervals::printOrientationIntervals(intersections)<<std::endl;
        std::cerr<<"Intersections brute: "<<OrientationIntervals::printOrientationIntervals(intersections_brute)<<std::endl;
        return false;
      }
    }
  }

  return true;
}

bool checkClosestOrientationInSubinterval(const OrientationInterval& interval, const OrientationInterval& subinterval, const int startOrientation){

  int closestOrientation;
  int distance;
  OrientationIntervals::computeClosestOrientationInSubinterval(interval, subinterval, startOrientation, closestOrientation, distance);

  std::cerr<<"computed closest ori "<<closestOrientation<<" dist "<<distance<<"\n";

  int maxOri = OrientationIntervals::getMaxOrientation();

  if(OrientationIntervals::containsOrientation(subinterval, startOrientation)){
    if(closestOrientation!= startOrientation || distance != 0){
      std::cerr<<"Containing case wrongly handled"<<std::endl;
      return false;
    } else {
      return true;
    }
  }

  //walk to the right
  int refdist_right = 0;
  int refclosest_right;
  bool found_right = false;

  bool interval_complete = (OrientationIntervals::getIntervalSize(interval) == maxOri + 1);

  for(int o=startOrientation; interval_complete || o!=interval.upper+1; o++){
    if(o == maxOri+1){
      o = 0;
    }

    if(o==subinterval.lower || o==subinterval.upper){
      refclosest_right = o;
      found_right=true;
      break;
    }

    refdist_right++;
  }

  if(found_right==false){
    refdist_right = std::numeric_limits<int>::max()-2; //-2 to avoid a tie break
  }

  //walk to the left
  int refdist_left = 0;
  int refclosest_left;
  bool found_left = false;
  for(int o=startOrientation; interval_complete || o!= interval.lower-1; o--){
    if(o == -1){
      o = maxOri;
    }

    if(o==subinterval.lower || o==subinterval.upper){
      refclosest_left = o;
      found_left = true;
      break;
    }

    refdist_left++;
  }

  if(found_left == false){
    refdist_left = std::numeric_limits<int>::max();
  }

  int refdist;
  int refclosest;

  bool tie = false;

  if(refdist_left < refdist_right){
    refdist = refdist_left;
    refclosest = refclosest_left;
  } else if(refdist_left == refdist_right){
    refdist = refdist_left;
    refclosest = refclosest_left;
    tie = true;
  } else {
    refdist = refdist_right;
    refclosest = refclosest_right;
  }

  if(refdist != distance || ((!tie && refclosest != closestOrientation) || (tie && (closestOrientation != refclosest_left && closestOrientation != refclosest_right)))){
    std::cerr<<"ERROR in compute closest stuff.\n";
    std::cerr<<"reference closest is "<<refclosest<<" dist "<<refdist<<"\n";
    std::cerr<<"reference closest left is "<<refclosest_left<<" dist "<<refdist_left<<" found "<<found_left<<"\n";
    std::cerr<<"reference closest right is "<<refclosest_right<<" dist "<<refdist_right<<" found "<<found_right<<"\n";
    return false;
  } else {
    return true;
  }

}

bool testRandomIntervalIntersection(){
	int maxOri = OrientationIntervals::getMaxOrientation();
	OrientationIntervals ints;

	bool setToFull = (rand()%100) < 2;

	if(setToFull){
		ints.setToFull();
	} else {

		int numOris = rand()%(maxOri+1);
		std::cerr<<"numOris "<<numOris<<std::endl;
		for(int i=0; i<numOris; i++){
			int ori = rand()%(maxOri+1);
			std::cerr<<"try pushing "<<ori<<std::endl;
			ints.addOrientation(ori);
			std::cerr<<"pushed "<<ori<<std::endl;
			std::cerr<<"current intervals "<<OrientationIntervals::printOrientationIntervals(ints.getOrientationIntervals())<<std::endl;
		}
	}

	//first check the intervals
	std::vector<OrientationInterval> myintervals = ints.getOrientationIntervals();

	if(myintervals.empty() == false){
		for(unsigned int i=0; i<myintervals.size()-1; i++){
			if(myintervals[i].upper == myintervals[i+1].lower){
				std::cerr<<"Intervals are corrupted."<<std::endl;
				return false;
			}
		}
	}

	std::cerr<<"Intervals "<<OrientationIntervals::printOrientationIntervals(ints.getOrientationIntervals())<<std::endl;

	OrientationInterval interval;
	interval.lower = rand()%(maxOri+1);
	interval.upper = rand()%(maxOri+1);

/*	if(interval.lower==7 && interval.upper==6){
		interval.lower=0;
		interval.upper=maxOri;
	}*/

	if(interval.lower == interval.upper + 1){
		interval.lower = 0;
		interval.upper = maxOri;
	}

	std::cerr<<"Interval ["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;

  std::vector<std::pair<unsigned int, unsigned int> > indices_intersect;
  ints.getIntervalIndicesAndIntersectionSizeWithInterval(interval, &indices_intersect);

  std::cerr<<"calculated intersection with size:"<<std::endl;
  for(unsigned int i=0; i<indices_intersect.size(); i++){
        std::cerr<<" "<<indices_intersect.at(i).first<<":"<<indices_intersect.at(i).second;
  }
  std::cerr<<std::endl;


  //now check this.
  std::vector<OrientationInterval> intervals = ints.getOrientationIntervals();
  for(unsigned int i=0; i<intervals.size(); i++){
  	//check if intersection of interval and intervals[i] is correct
  	OrientationIntervals temp_ints;
  	int o = intervals[i].lower;

  	while(true){
  		temp_ints.addOrientation(o);
  		if(o == intervals[i].upper){
  			break;
  		}
  		o++;
  		if(o > maxOri){
  			o = 0;
  		}
  	}

  	//is the current index found in the intersections?
		bool found = false;
		int expected_size = 0;
		for(unsigned int j=0; j<indices_intersect.size(); j++){
			if(indices_intersect.at(j).first == i){
				found = true;
				expected_size = indices_intersect.at(j).second;
				break;
			}
		}

  	std::vector<OrientationInterval> temp_intersects = temp_ints.getIntersectionWithInterval(interval);
  	if(temp_intersects.empty() == false) {
  		//we must be contained in indices_intersect
  		if(found==false){
  			std::cerr<<"could not find"<<std::endl;
  			return false;
  		}

  		//now check that the size is ok
  		int size=0;
  		if(temp_intersects.empty()==false){
  			if(temp_intersects.size() > 1){
  				size = std::max(OrientationIntervals::getIntervalSize(temp_intersects.front()), OrientationIntervals::getIntervalSize(temp_intersects.back()));
  				//size = std::max(temp_intersects.front().upper - temp_intersects.front().lower + 1, temp_intersects.back().upper - temp_intersects.back().lower + 1);
  			} else {
  				if(temp_intersects.front().upper < temp_intersects.front().lower){
  					size = temp_intersects.front().upper - 0 + 1 + maxOri - temp_intersects.front().lower + 1;
  				} else {
  					size = temp_intersects.front().upper - temp_intersects.front().lower + 1;
  				}
  			}
  		}

  		if(size != expected_size){
  			std::cerr<<"temp intervals: "<<OrientationIntervals::printOrientationIntervals(temp_ints.getOrientationIntervals())<<std::endl;
  			std::cerr<<"temp intersects: "<<OrientationIntervals::printOrientationIntervals(temp_intersects)<<std::endl;
  			std::cerr<<"size mismatch: size "<<size<<" expected "<<expected_size<<std::endl;
  			return false;
  		}

  	} else {
  		//intersection found to be empty
  		//we should not be found in the intersections
  		if(found==true){
  			std::cerr<<"found but should not have been. "<<std::endl;
  			return false;
  		}
  	}
  }

  if(intervals.empty()){
  	if(indices_intersect.empty()==false){
  		std::cerr<<"there should be an empty intersection"<<std::endl;
  		return false;
  	} else
  		return true;
  } else {
  	return true;
  }

}




int main(int argc, char** argv)
{
	OrientationIntervals::setMinMaxOrientation(0,7,0.1);
	OrientationIntervals ints;

	std::cerr<<ints.print()<<" contains ori 3? "<<ints.containsOrientation(3)<<std::endl;
	std::cerr<<ints.print()<<" contains ori 0? "<<ints.containsOrientation(0)<<std::endl;

	ints.addOrientation(3);
	ints.addOrientation(5);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(2);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(3);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(6);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(4);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(0);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(7);
	std::cerr<<ints.print()<<std::endl;

	std::cerr<<ints.print()<<" contains ori 1? "<<ints.containsOrientation(1)<<std::endl;
	std::cerr<<ints.print()<<" contains ori 0? "<<ints.containsOrientation(0)<<std::endl;

	ints.addOrientation(1);
	std::cerr<<ints.print()<<std::endl;


	ints.removeOrientation(3);
	std::cerr<<ints.print()<<std::endl;
	ints.removeOrientation(5);
	std::cerr<<ints.print()<<std::endl;
	ints.removeOrientation(4);
	std::cerr<<ints.print()<<std::endl;

	std::cerr<<ints.print()<<" contains ori 7? "<<ints.containsOrientation(7)<<std::endl;
	std::cerr<<ints.print()<<" contains ori 1? "<<ints.containsOrientation(1)<<std::endl;

//	ints.addOrientation(5);
//    ints.addOrientation(3);
//    ints.removeOrientation(6);
//    ints.removeOrientation(7);
    //ints.removeOrientation(1);
    //ints.removeOrientation(0);
    std::cerr<<ints.print()<<std::endl;
    OrientationInterval interval;
    interval.lower = 1;
    interval.upper = 6;
    std::cerr<<"Intersection with ["<<interval.lower<<","<<interval.upper<<"]: "<<OrientationIntervals::printOrientationIntervals(ints.getIntersectionWithInterval(interval))<<std::endl;
    std::cerr<<"My intervals "<<OrientationIntervals::printOrientationIntervals(ints.getOrientationIntervals())<<std::endl;
    std::cerr<<"Indices: "<<std::endl;
    std::vector<unsigned int> tmp_indices;
    ints.getIntervalIndicesWithNonEmptyIntersectionWithInterval(interval, &tmp_indices);
    for(unsigned int i=0; i<tmp_indices.size(); i++){
      std::cerr<<" "<<tmp_indices.at(i);
    }
    std::cerr<<std::endl;
    std::cerr<<"Indices w/ size: "<<std::endl;
    std::vector<std::pair<unsigned int, unsigned int> > tmp_indices_w_size;
    ints.getIntervalIndicesAndIntersectionSizeWithInterval(interval, &tmp_indices_w_size);
    for(unsigned int i=0; i<tmp_indices_w_size.size(); i++){
      std::cerr<<" "<<tmp_indices_w_size.at(i).first<<":"<<tmp_indices_w_size.at(i).second;
    }
    std::cerr<<std::endl;


	ints.removeOrientation(4);
	std::cerr<<ints.print()<<std::endl;
	ints.removeOrientation(2);
	std::cerr<<ints.print()<<std::endl;

	ints.removeOrientation(0);
	std::cerr<<ints.print()<<std::endl;

	std::cerr<<ints.print()<<" contains ori 1? "<<ints.containsOrientation(1)<<std::endl;
	std::cerr<<ints.print()<<" contains ori 0? "<<ints.containsOrientation(0)<<std::endl;

	ints.removeOrientation(1);
	std::cerr<<ints.print()<<std::endl;

	ints.addOrientation(0);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(1);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(2);
	std::cerr<<ints.print()<<std::endl;


	ints.removeOrientation(7);
	std::cerr<<ints.print()<<std::endl;


	ints.removeOrientation(3);
	std::cerr<<ints.print()<<std::endl;

	ints.addOrientation(7);
	std::cerr<<ints.print()<<std::endl;
	//std::cerr<<"internal "<<ints.printInternalRepresentation()<<std::endl;


	ints.addOrientation(4);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(5);
	std::cerr<<ints.print()<<std::endl;
	ints.addOrientation(3);
	std::cerr<<ints.print()<<std::endl;

	std::cerr<<ints.print()<<" contains ori 3? "<<ints.containsOrientation(3)<<std::endl;
	std::cerr<<ints.print()<<" contains ori 0? "<<ints.containsOrientation(0)<<std::endl;



//	ints.addOrientation(6);
//	std::cerr<<ints.print()<<std::endl;
//	ints.addOrientation(7);
//	std::cerr<<ints.print()<<std::endl;
//	ints.addOrientation(0);
//	std::cerr<<ints.print()<<std::endl;
//	ints.addOrientation(1);
//	std::cerr<<ints.print()<<std::endl;
//	ints.addOrientation(2);
//	std::cerr<<ints.print()<<std::endl;
//	ints.addOrientation(1);
//	std::cerr<<ints.print()<<std::endl;



//
//		ints.addOrientation(0);
//		std::cerr<<ints.print()<<std::endl;
//		ints.addOrientation(3);
//		std::cerr<<ints.print()<<std::endl;
//		ints.addOrientation(6);
//		std::cerr<<ints.print()<<std::endl;
//		ints.addOrientation(4);
//		std::cerr<<ints.print()<<std::endl;
//		ints.addOrientation(5);
//		std::cerr<<ints.print()<<std::endl;
//		ints.addOrientation(7);
//		std::cerr<<ints.print()<<std::endl;

	srand(1234567);
    int nChecks = 0;
	for(unsigned int i=0; i<100000; i++){
		std::cerr<<"frame "<<i<<std::endl;

		OrientationIntervals ints = getRandomOrientationIntervals();
		OrientationInterval interval = getRandomOrientationInterval(false);

		std::cerr<<"Intervals "<<OrientationIntervals::printOrientationIntervals(ints.getOrientationIntervals())<<std::endl;
		std::cerr<<"Interval ["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;

		bool status = checkOrientationIntervals(ints);
		status = status && checkIntervalIntersection(interval, ints);
		status = status && checkIntervalIntersectionIndices(interval, ints);
		status = status && checkIntervalIntersectionIndicesWithSize(interval, ints);
		status = status && checkIntervalIntersectionIndicesAndIntervals(interval, ints);

		OrientationInterval interval_b = getRandomOrientationInterval(true);
		std::cerr<<"Interval b ["<<interval_b.lower<<","<<interval_b.upper<<"]"<<std::endl;

		status = status && checkPureIntervalIntersection(interval, interval_b);

		std::vector<OrientationInterval> intersects;
		OrientationIntervals::computeIntersection(interval, interval_b, intersects);
		if(intersects.empty() == false){
		  OrientationInterval int_sub = intersects[0];
		  int startOri = getRandomOrientationFromInterval(interval);
		  std::cerr<<"Interval sub ["<<int_sub.lower<<","<<int_sub.upper<<"]  startOri "<<startOri<<std::endl;
		  status = status && checkClosestOrientationInSubinterval(interval, int_sub,  startOri);
		  nChecks++;
		}

		if(status==false){
			std::cerr<<"Intervals "<<OrientationIntervals::printOrientationIntervals(ints.getOrientationIntervals())<<std::endl;
			std::cerr<<"Interval ["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
			std::cerr<<"Interval b ["<<interval_b.lower<<","<<interval_b.upper<<"]"<<std::endl;
			std::cerr<<">> E R R O R !"<<std::endl;
			break;
		} else {
			std::cerr<<"=>OK. n intersection tests"<<nChecks<<"\n";
		}
	}


	/*
	for(unsigned int i=0; i<10000; i++){
		std::cerr<<"frame "<<i<<std::endl;
		bool result = testRandomIntervalIntersection();
		if(result==false){
			break;
		} else {
			std::cerr<<"=>OK\n\n";
		}
	}

*/

}



