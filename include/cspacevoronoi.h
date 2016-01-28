#ifndef _CSPACEVORONOI_H_
#define _CSPACEVORONOI_H_

#include "cspacevoronoilayer.h"
#include <list>
#include "point.h"
#include <map>
#include <set>

#include "node.h"
#include "simplemap.h"
#include "simplisticmap.h"
#include "OrientationIntervals.h"
#include "OINodeConnector.h"
#include "OISearchNode.h"
#include "OIDStarSearchNode.h"
#include "OIDStar.h"

typedef std::unordered_map< IntPose, Node*, IntPoseHash, intposeequality> NodeMapType;
typedef std::unordered_set< IntPose, IntPoseHash, intposeequality> PlannerMapType;
typedef std::unordered_set< IntPoint, IntPointHash, intpointequality> PlannerPointMapType;

#define HEURISTIC(p1, p2) (abs(p1.x-p2.x)+abs(p1.y-p2.y)+ANGULARDISTANCE(p1.theta-p2.theta))
#define NORMALIZEANGLE(a) (((a)<0) ? ((a)+maxAngle) : ((a)>=maxAngle ? ((a)-maxAngle) : (a)))
#define ANGULARDISTANCE(a) ((a)>maxAngle/2 ? (maxAngle-(a)) : (a))


class CSpaceVoronoi {
public:

  CSpaceVoronoi(std::vector<RobotColumn> &columns, SimpleMap<int> *map, int maxDist_squared=INT_MAX);
  ~CSpaceVoronoi();

  bool checkCollision(const Pose &p);
  bool checkCollision(const IntPose &p);
  float getDistanceInCells(const IntPose &p);
  int getSquaredDistanceInCells(const IntPose &p);
  bool collidesInAllOrientations(const int x, const int y);
  std::vector<OrientationInterval> getAdmissibleOrientations(const int x, const int y);
  OrientationIntervals& getAdmissibleOrientationStructure(const int x, const int y);

  void initializeMap(SimpleMap<int> *_gridMap);
  void updateObstacles(std::vector<IntPose> *points, bool updateVoronoi);
  int updateClearance(int x, int y, int clearance, bool updateVoronoi=true);

  void update(bool updateRealDist=true);
  void initOriNodeMap();
  void createOIGraph();
  void updateOrientationIntervalGraph();

  void initializeDStar();
  void incorporateUpdatesDStar(IntPose start, IntPose goal);

  void setDStar(DStarSearch* dstar){
    this->dstar = dstar;
  };

  DStarSearchNode* getDStarSearchNode(int x, int y, int theta);
  SearchNode* getSearchNode(int x, int y, int theta);

  void prune();
  virtual std::list<IntPose>* computeShortestPath(IntPose start, IntPose goal);
  virtual void cleanup();

  int worldToMapTheta(double theta); //expects theta to be in [0, 2pi)
  double mapToWorldTheta(int theta);

  void brushfireExpansion(IntPose pose, int minSqrDist, NodeMapType *nodeMap);
  void brushfireExpandLayer(int x, int y, int theta, bool makeGoalBubble, IntPose goal, int minSqrDist, NodeMapType *nodeMap);
  void expandNode(int nx, int ny, int nt, IntPose &goal, Node *s, NodeMapType &nodeMap, BucketPrioQueue<Node*> &openSet);

  int sizeX, sizeY;
  std::vector<CSpaceVoronoiLayer*> layers;
  double angularResolution;
  double time_search;
  int explored_nodes;
  double sqrt2;
  int maxAngle;
  SimpleMap<int> gridMap;
  SimpleMap<int> newGridMap;

  std::list<IntPose> path;
  NodeMapType nodeMap;
  BucketPrioQueue<Node*> openSet;

  typedef std::set<IntPoint, intpointComparison> LastObstacleType;
  LastObstacleType *lastObstacles;
  std::vector<RobotColumn> columns;

  SimpleMap<int> nCollidingOrientations;
  SimplisticMap<OrientationIntervals> admissibleOrientations;

  SimplisticMap<OINodeConnector<OISearchNode> > oriNodeMap;

  SimplisticMap<OINodeConnector<OIDStarSearchNode> > oriNodeMap_dstar;

  void plotOrientations();
  void saveFootprints(std::string basefilename);

  std::vector<DStarSearchNode*>* initialPlanningDStar(IntPose start, IntPose goal);
  std::vector<DStarSearchNode*>* replanningDStar(IntPose start, IntPose goal);
  void testPlanning();
  void testPlanningDStar(IntPose start, IntPose goal);
  void testRePlanningDStar(IntPose start, IntPose goal);

private:

  DStarSearch* dstar;
  void incorporateLayerCollisionChanges();
  void rewireOIGraph();
  void updateOriNodeMap(int x, int y);

  void initOriNodeMap_dstar();
  void createOIGraph_dstar();

  int getOrientationIntervalIndex(int x, int y, int theta);


  void plotPath(std::string filename, std::vector<SearchNode*>* path);
  void plotPath_dstar(std::string filename, std::vector<DStarSearchNode*>* path);

  std::set<std::pair<int, int> > collisionChangeLocations;

  std::vector<DStarSearchNode*> nodesToBeDeleted;
};


#endif
