/*
 * OrientationIntervals.h
 *
 *  Created on: Nov 25, 2012
 *      Author: sprunkc
 */

#ifndef ORIENTATIONINTERVALS_H_
#define ORIENTATIONINTERVALS_H_

#include <string>
#include <vector>

typedef struct {
	int lower;
	int upper;
} OrientationInterval;

class OrientationIntervals {
public:
	OrientationIntervals();
	virtual ~OrientationIntervals();

	void addOrientation(int orientation);
	void removeOrientation(int orientation);
	bool containsOrientation(int orientation);

	std::vector<OrientationInterval> getOrientationIntervals() const;
	std::vector<OrientationInterval> getIntersectionWithInterval(OrientationInterval interval) const ;
	void getIntervalIndicesWithNonEmptyIntersectionWithInterval(const OrientationInterval interval, std::vector<unsigned int>* interval_indices) const;
	void getIntervalIndicesAndIntersectionSizeWithInterval(OrientationInterval interval, std::vector<std::pair<unsigned int, unsigned int> >* interval_indices) const;
	void getIntersectionsWithInterval(OrientationInterval interval, std::vector<std::pair<unsigned int, OrientationInterval> >* interval_indices) const;


	//This is wrong, need to subtract 1 if we have a wraparound
//	unsigned int getNumIntervals(){
//		return intervals.size();
//	}

	void setToFull();

	bool empty();
	bool full();

	std::string print() const;
	std::string printInternalRepresentation() const;

	static void setMinMaxOrientation(int minimumOrientation, int maximumOrientation, double angularResolution);
	static int getMinOrientation(){
	  return minOrientation;
	}
    static int getMaxOrientation(){
      return maxOrientation;
    }
  static double getAngularResolution(){
      return angularRes;
  }

    static std::string printOrientationIntervals(const std::vector<OrientationInterval> &orientation_intervals);
    static unsigned int getIntervalSize(const OrientationInterval interval);
    static bool containsOrientation(const OrientationInterval& interval, const int orientation);
    static void computeIntersection(const OrientationInterval& int_a, const OrientationInterval& int_b, std::vector<OrientationInterval>& intersection);
    static void computeClosestOrientationInSubinterval(const OrientationInterval& interval, const OrientationInterval& sub_interval, const int startOrientation, int& closestOrientation, int& abs_distance);
    static void computeOrientationDistanceInInterval(const OrientationInterval& interval, const int firstOrientation, const int secondOrientation, int& abs_distance);
private:

	std::vector<OrientationInterval>::iterator checkMergeIntervalPrevious(
			std::vector<OrientationInterval>::iterator previous,
			std::vector<OrientationInterval>::iterator current);
	std::vector<OrientationInterval>::iterator checkMergeIntervalNext(
			std::vector<OrientationInterval>::iterator current,
			std::vector<OrientationInterval>::iterator next);
	void checkFull();
	void getInternalIntervalIndicesWithNonEmptyIntersectionWithInterval(
	    const OrientationInterval& interval,
	    std::vector<unsigned int>* internal_interval_indices) const;
	void getInternalIntervalIndicesAndIntersectionSizeWithInterval(
	    const OrientationInterval& interval,
	    std::vector<std::pair<unsigned int, unsigned int> >* internal_interval_indices) const;
	void getInternalIntersectionsWithInterval(
	    const OrientationInterval& interval,
	    std::vector<std::pair<unsigned int, OrientationInterval> >* interval_indices) const;


	static int minOrientation;
	static int maxOrientation;
	static int numOrientations;
        static double angularRes;

	std::vector<OrientationInterval> intervals;
	bool containsAllOrientations;
};

#endif /* ORIENTATIONINTERVALS_H_ */
