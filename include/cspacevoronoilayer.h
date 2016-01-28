#ifndef _CSPACEVORONOI_LAYER_H_
#define _CSPACEVORONOI_LAYER_H_


#include "dynamicvoronoi.h"
#include "simplemap.h"
#include <math.h>

class CSpaceVoronoiLayer : public DynamicVoronoi {
public:
  CSpaceVoronoiLayer(std::vector<RobotColumn> &columns, double t, int maxDist_squared=INT_MAX);
  ~CSpaceVoronoiLayer();
  void initializeMap(SimpleMap<int> *_gridMap);
  void updateClearance(int x, int y, int col, bool newlyOccupied, bool updateVoronoi=true);
  void saveCountMap(const char* filename);
  void drawRobot(int x, int y, SimpleMap<int> &map);

  SimpleMap<int> countMap;

  class CollisionChange {
  public:
    CollisionChange(int x, int y, bool collides)
    :x(x), y(y), collides(collides)
    {};
    int x;
    int y;
    bool collides;
  };

  void overlayFootprint(const CSpaceVoronoiLayer* other);

  std::vector<CollisionChange> collisionChanges;

  const std::vector< std::vector<ScanLine> >& getColumnBoundaries(){
    return columnBoundaries;
  }

  const std::vector<RobotColumn>& getColumns(){
    return columns;
  }

private:

  std::vector< std::vector<ScanLine> > columnBoundaries; // maps a y-offset to x-offsets, one std::vector<ScanLine> per robotColumn
  std::vector<RobotColumn> columns;
};

#endif
