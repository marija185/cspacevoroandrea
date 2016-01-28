#include "cspacevoronoi.h"

#include <math.h>
#include <omp.h>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cmath>

#include "helper.h"

#define MINSQRDIST 1

template<> std::vector<Node*> MemoryManager<Node>::freeObjects = std::vector<Node*>();
template<> unsigned int MemoryManager<Node>::objectsInUse = 0;


#include "AStar.h"

CSpaceVoronoi::CSpaceVoronoi(std::vector<RobotColumn> &_columns, SimpleMap<int> *map, int maxDist_squared) {
  //MemoryManager<Node>::reserve(1000000);

  columns = _columns;

  double radius = 0;
  for (unsigned int i=0; i<columns.size(); i++) {

    for(unsigned int j=0; j<columns[i].vertices.size(); j++){
      double x = columns[i].vertices[j].first;
      double y = columns[i].vertices[j].second;
      double r = sqrt(x*x+y*y);
      if (r>radius) radius = r;
    }
  }
  sqrt2 = sqrt(2);  
  angularResolution = 1/radius;
  fprintf(stderr, "Angular resolution: %f deg,\n", angularResolution/2/M_PI*360);
  assert(angularResolution>0);

  printf("Generating layers\n");
  for (double theta = 0; theta<2*M_PI; theta+=angularResolution) {
    layers.push_back(new CSpaceVoronoiLayer(columns, theta, maxDist_squared));
  }

  //now we overlay each parts with the one for the succeeding discrete orientation to account for the flooring that takes place when checking a pose
  //(incoming theta will be mapped to a bin, robot can have all the orientations in the bin)
  for(unsigned int i=0; i<layers.size()-1; i++){
    layers[i]->overlayFootprint(layers[i+1]);
  }
  //the first layer has already been overlayed but we need a clean version to do the overlay with the last layer
  CSpaceVoronoiLayer temp(columns, 0.0, maxDist_squared);
  layers[layers.size()-1]->overlayFootprint(&temp);  

  maxAngle = layers.size(); //maxAngle needs to be one bigger than maxOrientation, look into definition of macro NORMALIZEANGLE
  OrientationIntervals::setMinMaxOrientation(0, layers.size()-1,angularResolution);

  initializeMap(map);
  map->writeToPGM("test.pgm");
  lastObstacles = new LastObstacleType();
  
  //initializing the map lists every occupied cell as a collision change, we reset this here
  collisionChangeLocations.clear();
  dstar = NULL;

}


CSpaceVoronoi::~CSpaceVoronoi() {
  delete lastObstacles;
  for (unsigned int i=0; i<layers.size(); i++) {
    delete layers[i];
  }

  for(int x=0; x<oriNodeMap.getMapSizeX(); x++){
    for(int y=0; y<oriNodeMap.getMapSizeY(); y++){
      std::vector<OISearchNode*> &nodes = oriNodeMap.getCellReference(x,y).nodes;
      for(unsigned int i=0; i<nodes.size(); i++){
        if(nodes[i] != NULL)
          MemoryManager<OISearchNode>::destroy(nodes[i]);
      }
    }
  }

  for(int x=0; x<oriNodeMap_dstar.getMapSizeX(); x++){
    for(int y=0; y<oriNodeMap_dstar.getMapSizeY(); y++){
      std::vector<OIDStarSearchNode*> &nodes = oriNodeMap_dstar.getCellReference(x,y).nodes;
      for(unsigned int i=0; i<nodes.size(); i++){
        if(nodes[i] != NULL)
          MemoryManager<OIDStarSearchNode>::destroy(nodes[i]);
      }
    }
  }

}

void CSpaceVoronoi::initializeMap(SimpleMap<int> *_gridMap) {
  sizeX = _gridMap->getMapSizeX();
  sizeY = _gridMap->getMapSizeY();
  gridMap.copyFrom(_gridMap);
  newGridMap.copyFrom(_gridMap);


  nCollidingOrientations.resize(sizeX, sizeY);
  nCollidingOrientations.fill(0);

  admissibleOrientations.resize(sizeX, sizeY);
  for(unsigned int x=0; x<sizeX; x++){
    for(unsigned int y=0; y<sizeY; y++){
      admissibleOrientations.getCellReference(x,y).setToFull(); //all Orientations are ok, corresponds to empty map
    }
  }

  printf("INITIALIZING LAYERS\n");

#pragma omp parallel for num_threads(6)
  for (unsigned int i=0; i<layers.size(); i++) {
    layers[i]->initializeMap(_gridMap);
  }

  incorporateLayerCollisionChanges();
}


void CSpaceVoronoi::incorporateLayerCollisionChanges(){
  for (unsigned int l=0; l<layers.size(); l++){
    for(unsigned int i=0; i<layers[l]->collisionChanges.size(); i++){

      const CSpaceVoronoiLayer::CollisionChange& collchange = layers[l]->collisionChanges[i];
      if(collchange.collides){
        nCollidingOrientations.preIncrement(collchange.x, collchange.y);
        //std::cerr<<"remove at "<<layers[l]->collisionChanges[i].x<<","<<layers[l]->collisionChanges[i].y<<std::endl;
        admissibleOrientations.getCellReference(collchange.x,collchange.y).removeOrientation(l);
      } else {
        nCollidingOrientations.preDecrement(collchange.x,collchange.y);
        //std::cerr<<"add at "<<layers[l]->collisionChanges[i].x<<","<<layers[l]->collisionChanges[i].y<<std::endl;
        admissibleOrientations.getCellReference(collchange.x,collchange.y).addOrientation(l);
      }

      collisionChangeLocations.insert(std::make_pair(collchange.x, collchange.y));
    }
    layers[l]->collisionChanges.clear();
  }
}

void CSpaceVoronoi::rewireOIGraph(){

  //first we mark all the guys that will be completely rebuilt
  //this makes it possible to save some computation when looking at the neighborhood later
  //In short: if a node is to be rebuilt completely anyway, there is no need to carefully update successors
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    oriNodeMap.getCellReference(it->first,it->second).dirty = true;
  }

  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){

    //from all neighbors that are not dirty, we need to remove ourselves from their successors
    int xstart = (std::max)(0,it->first-1);
    int xend = (std::min)(it->first+1, oriNodeMap.getMapSizeX()-1);
    int ystart = (std::max)(0,it->second-1);
    int yend = (std::min)(it->second+1, oriNodeMap.getMapSizeY()-1);

    OINodeConnector<OISearchNode>& currentConnector = oriNodeMap.getCellReference(it->first,it->second);
    for(int ny=ystart; ny<=yend; ny++){
      for(int nx=xstart; nx<=xend; nx++){

        if((nx==it->first && ny==it->second) || (nx!=it->first && ny!=it->second)){
          continue;
        } else {

          OINodeConnector<OISearchNode>& neighborConnector = oriNodeMap.getCellReference(nx,ny);
          if(neighborConnector.dirty){
            continue;
          } else {

            for(std::vector<OISearchNode*>::iterator it_neighborNodes = neighborConnector.nodes.begin(); it_neighborNodes!=neighborConnector.nodes.end(); ++it_neighborNodes){
              for(std::vector<OISearchNode*>::iterator it_deleteNodes = currentConnector.nodes.begin(); it_deleteNodes!=currentConnector.nodes.end(); ++it_deleteNodes){
                for(std::vector<SearchNode*>::iterator it_NNsuccessors = (*it_neighborNodes)->successors.begin(); it_NNsuccessors != (*it_neighborNodes)->successors.end(); ++it_NNsuccessors){
                  if(*it_deleteNodes == *it_NNsuccessors){
                    it_NNsuccessors = (*it_neighborNodes)->successors.erase(it_NNsuccessors);
                    //TODO can make more efficient?
                    (*it_neighborNodes)->successor_costs.erase((*it_neighborNodes)->successor_costs.begin() + (it_NNsuccessors-(*it_neighborNodes)->successors.begin()));
                    it_NNsuccessors--; //loop will increment again
                  }
                }
              }
            }
          }
        }
      }
    }

    //now we can kill ourselves and rebuild
    currentConnector.initFromOrientationIntervals(admissibleOrientations.getCellReference(it->first, it->second));
    //now the currentConnector.nodes is filled with NULL pointers
    //we instantiate the new nodes
    for(unsigned int o=0; o<currentConnector.orientationIntervals.size(); o++){
      currentConnector.nodes[o] = MemoryManager<OISearchNode>::getNew();
      currentConnector.nodes[o]->x = it->first;
      currentConnector.nodes[o]->y = it->second;
      currentConnector.nodes[o]->o = o;
    }
  }


  //now we compute the successors for all newly created nodes
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    OINodeConnector<OISearchNode>& currentConnector = oriNodeMap.getCellReference(it->first,it->second);
    for(std::vector<OISearchNode*>::iterator it=currentConnector.nodes.begin(); it!=currentConnector.nodes.end(); ++it){
      (*it)->computeSuccessors(&oriNodeMap, &admissibleOrientations);
    }
  }

  //now we add the newly created nodes to the successors list of their neighbors
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){

    int xstart = (std::max)(0,it->first-1);
    int xend = (std::min)(it->first+1, oriNodeMap.getMapSizeX()-1);
    int ystart = (std::max)(0,it->second-1);
    int yend = (std::min)(it->second+1, oriNodeMap.getMapSizeY()-1);

    OINodeConnector<OISearchNode>& currentConnector = oriNodeMap.getCellReference(it->first,it->second);
    for(int ny=ystart; ny<=yend; ny++){
      for(int nx=xstart; nx<=xend; nx++){

        if((nx==it->first && ny==it->second) || (nx!=it->first && ny!=it->second)){
          continue;
        } else {

          OINodeConnector<OISearchNode>& neighborConnector = oriNodeMap.getCellReference(nx,ny);
          if(neighborConnector.dirty){
            //these nodes have been completely rebuilt, we are already in their successors list
            continue;
          } else {

            for(std::vector<OISearchNode*>::iterator it_neighbornodes =neighborConnector.nodes.begin(); it_neighbornodes!= neighborConnector.nodes.end(); ++it_neighbornodes){
              //TODO maybe we can give this method more info/ cache some of the things that it is goinf to look up again in the maps
              (*it_neighbornodes)->addSuccessorsToSpecificNeighbor(it->first, it->second, &oriNodeMap, &admissibleOrientations);
            }
          }
        }
      }
    }
  }

  //finally, we can remove the dirty flag
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    oriNodeMap.getCellReference(it->first,it->second).dirty = false;
  }
  collisionChangeLocations.clear();
}

void CSpaceVoronoi::updateObstacles(std::vector<IntPose> *points, bool updateVoronoi) {
  LastObstacleType *newObstacles = new LastObstacleType();
  LastObstacleType::iterator it;

  LastObstacleType::const_iterator endit = lastObstacles->end();
  for (it = lastObstacles->begin(); it!=endit; ++it) {
    newGridMap.setCell(it->x, it->y, INT_MAX);
  }  

  unsigned int size = points->size();
  for (unsigned int i=0; i<size; i++) {
    IntPose pose = points->at(i);
    INTPOINT p(pose.x, pose.y);
    if (newGridMap.getCell(p)<=pose.theta) continue;
    it = lastObstacles->find(p);
    if (it != lastObstacles->end()) {
      lastObstacles->erase(it);
    }
    newGridMap.setCell(p.x, p.y, pose.theta);
    newObstacles->insert(p);
  }  

  endit = lastObstacles->end();
  for (it = lastObstacles->begin(); it!=endit; ++it) {
    updateClearance(it->x,it->y,INT_MAX);
  }  

  endit = newObstacles->end();

  for (it = newObstacles->begin(); it!=endit; ++it) {
    updateClearance(it->x, it->y, newGridMap.getCell(it->x, it->y));
  }  

  lastObstacles->clear();
  delete lastObstacles;
  lastObstacles = newObstacles;
}

int CSpaceVoronoi::updateClearance(int x, int y, int clearance, bool updateVoronoi) {
  int oldClearance = gridMap.getCell(x,y);
  gridMap.setCell(x,y,clearance);

  for (int i=0; i<(int)columns.size(); i++) {
    bool collidesNow = (clearance<=columns[i].upper);
    bool collidesBefore = (oldClearance<=columns[i].upper);
    if (collidesNow == collidesBefore) continue;
#pragma omp parallel for num_threads(6)
    for (unsigned int l=0; l<layers.size(); l++) 
      layers[l]->updateClearance(x,y,i,collidesNow,updateVoronoi);
  }

  //TODO measure time for this
  if(updateVoronoi){
    incorporateLayerCollisionChanges();
  }

  return oldClearance;
}


void CSpaceVoronoi::update(bool updateRealDist) {
#pragma omp parallel for num_threads(4)
  for (unsigned int i=0; i<layers.size(); i++) {
    layers[i]->update(updateRealDist);
  }
}

void CSpaceVoronoi::updateOrientationIntervalGraph(){
  rewireOIGraph();
}

void CSpaceVoronoi::prune() {

#pragma omp parallel for num_threads(4)
  for (unsigned int i=0; i<layers.size(); i++) layers[i]->prune();
}


std::list<IntPose>* CSpaceVoronoi::computeShortestPath(IntPose start, IntPose goal) {
  if (start==goal) {
    path.push_back(start);
    path.push_back(goal);
    fprintf(stderr, "Start and goal are identical!\n");
    return &path;
  }
  
  if (layers[start.theta]->getSqrDistance(start.x, start.y) < MINSQRDIST) {
    fprintf(stderr, "Start is occupied!\n");
    return &path;
  }
  if (layers[goal.theta]->getSqrDistance(goal.x, goal.y) < MINSQRDIST) {
    fprintf(stderr, "Goal is occupied!\n");
    return &path;
  }


  //add virtual obstacle to voronoi
  for (unsigned int l=0; l<layers.size(); l++) {
    layers[l]->setObstacle(start.x,start.y);
    layers[l]->setObstacle(goal.x,goal.y);
  }
  update(false);


  Node *startNode = MemoryManager<Node>::getNew();
  startNode->g = 0;
  startNode->h = HEURISTIC(start, goal);
  startNode->f = startNode->h;
  startNode->prev = NULL;
  startNode->pos = start;
  startNode->phase = Node::starting;
  startNode->state = Node::none;
  nodeMap[start] = startNode;

  Node* goalNode = MemoryManager<Node>::getNew();
  goalNode->g = INT_MAX;
  goalNode->h = 0;
  goalNode->f = goalNode->g + goalNode->h;
  goalNode->prev = NULL;
  goalNode->pos = goal;
  goalNode->phase = Node::goaling;
  goalNode->state = Node::none;
  nodeMap[goal] = goalNode;	

  for (unsigned int i=0; i<layers.size(); i++) {
    brushfireExpandLayer(goal.x, goal.y, i, true, goal, MINSQRDIST, &nodeMap);
    brushfireExpandLayer(start.x, start.y, i, false, goal, MINSQRDIST, &nodeMap);
  }


  startNode->state = Node::open;
  openSet.push(startNode->f, startNode);

  int steps = 0;
  while (!openSet.empty()) {
    Node* s = openSet.pop();
    if(s->state == Node::closed) continue;

    steps++;

    //We are expanding the goal, i.e., we have found a path
    if (s->pos == goal) { 
      while (s!=startNode) {
        path.push_front(s->pos);
        s = s->prev;
      }
      path.push_front(s->pos);

      //remove the virtual obstacle again from voronoi
      for (unsigned int l=0; l<layers.size(); l++) {
        layers[l]->removeObstacle(start.x,start.y);
        layers[l]->removeObstacle(goal.x,goal.y);
      }
      update(false);

      return &path;
    }

    //Expand current node s
    s->state = Node::closed;

    int x = s->pos.x;
    int y = s->pos.y;
    int theta = s->pos.theta;
        
    if (x>0)     expandNode(x-1, y  , theta, goal,s,nodeMap,openSet);
    if (x<sizeX) expandNode(x+1, y  , theta, goal,s,nodeMap,openSet);
    if (y>0)     expandNode(x  , y-1, theta, goal,s,nodeMap,openSet);
    if (y<sizeY) expandNode(x  , y+1, theta, goal,s,nodeMap,openSet);
    expandNode(x,y,NORMALIZEANGLE(theta-1),goal,s,nodeMap,openSet);
    expandNode(x,y,NORMALIZEANGLE(theta+1),goal,s,nodeMap,openSet);
  }

  //remove the virtual obstacle again from voronoi
  for (unsigned int l=0; l<layers.size(); l++) {
    layers[l]->removeObstacle(start.x,start.y);
    layers[l]->removeObstacle(goal.x,goal.y);
  }
  update(false);

  return &path;
}

void CSpaceVoronoi::expandNode(int nx, int ny, int nt, IntPose &goal, Node *s, NodeMapType &nodeMap, BucketPrioQueue<Node*> &openSet) {
  static const double EPSILON = 0.00001;

          //skip the parent node
          IntPose newPos(nx,ny,nt);
          if (s->prev && newPos==s->prev->pos) return;
        

          int lt = nt;

          //once we are on the voronoi graph, skip cells that leave it
          bool isVoronoi = layers[lt]->isVoronoi(nx,ny);
          if (s->phase==Node::voro && !isVoronoi) return;

          //compute costs from parent to here
          int gNew = s->g + 1;
	  
          //find out if node already exists
          NodeMapType::iterator it = nodeMap.find(newPos);
          Node *n;
          if (it!=nodeMap.end()) n = it->second;
          else n = NULL;
          
          //these phases can be substituted by simple boolean markers
          if (s->phase==Node::goaling && !n && !isVoronoi) return;
          if (s->phase==Node::starting && !n && !isVoronoi) return;

          if (n && gNew >= n->g - EPSILON) return;

          if (!n) {
            n = MemoryManager<Node>::getNew();	
            n->pos = newPos;
            assert(isVoronoi);
            n->phase = Node::voro;
            n->h = HEURISTIC(newPos, goal);
            n->state = Node::open;
            nodeMap[newPos] = n;
          }

          n->prev = s;	  
          n->g = gNew;	  
          n->f = n->g + n->h;

          openSet.push(n->f, n);

}

void CSpaceVoronoi::brushfireExpandLayer(int x, int y, int theta, bool makeGoalBubble, IntPose goal, int minSqrDist, NodeMapType *nodeMap) {
  std::queue<IntPoint> q;
  q.push(IntPoint(x,y));
  
  CSpaceVoronoiLayer *layer = layers[theta];

  while(!q.empty()) {
    IntPoint p = q.front();
    q.pop();
    int x = p.x;
    int y = p.y;
    
    for (int dx=-1; dx<=1; dx++) {
      int nx = x+dx;
      if (nx<0 || nx>=sizeX) continue;
      for (int dy=-1; dy<=1; dy++) {
        int ny = y+dy;
        if (dx && dy) continue;
        if (ny<0 || ny>=sizeY) continue;
        IntPoint n = IntPoint(nx, ny);
          
        if (layer->getSqrDistance(nx,ny)<1) continue;
        IntPose nPose = IntPose(nx, ny, theta);
        if (nodeMap->count(nPose)>0) continue;
        //        if (nd) continue;


        Node *nd = MemoryManager<Node>::getNew();
        nd->g = INT_MAX;
        nd->h = HEURISTIC(nPose, goal);
        nd->f = INT_MAX;
        nd->prev = NULL;
        nd->pos = nPose;
        nd->state = Node::none;

        bool isVoronoi = layer->isVoronoi(nx,ny);

        if (makeGoalBubble) nd->phase = Node::goaling;
        else {
          if (isVoronoi) nd->phase = Node::voro;
          else nd->phase = Node::starting;
        }


        (*nodeMap)[nPose] = nd;
        if (!isVoronoi) q.push(n);
      }
    }      
  }
}

void CSpaceVoronoi::cleanup() {
  for(NodeMapType::iterator it=nodeMap.begin(); it !=nodeMap.end(); ++it){
		MemoryManager<Node>::destroy(it->second);
  }
  nodeMap.clear();

  openSet.clear(); //TODO should be a local variable in computePath, not a member?

  path.clear();

  //char *dumbo = new char[1024*1024*100];
  //delete[] dumbo;
}

bool CSpaceVoronoi::checkCollision(const Pose &p) {
  int lt = floor(p.theta/angularResolution);
  return (layers[lt]->countMap.getCell(p.x, p.y)>0);
}

bool CSpaceVoronoi::checkCollision(const IntPose &p){
  return (layers[p.theta]->countMap.getCell(p.x, p.y)>0);
}

float CSpaceVoronoi::getDistanceInCells(const IntPose &p){
  return (layers[p.theta]->getDistance(p.x, p.y));
}

int CSpaceVoronoi::getSquaredDistanceInCells(const IntPose &p){
  return (layers[p.theta]->getSqrDistance(p.x, p.y));
}


bool CSpaceVoronoi::collidesInAllOrientations(const int x, const int y){

  //this is the new, more efficient version of the code below, we now have a incrementally updated countmap
  return (nCollidingOrientations.getCell(x,y) == (int) layers.size());

  /*
  for(unsigned int i=0; i<layers.size(); i++){
    if(layers[i]->countMap.getCell(x, y) == 0)
      return false;
  }
  return true;
  */
}

std::vector<OrientationInterval> CSpaceVoronoi::getAdmissibleOrientations(const int x, const int y){
  return admissibleOrientations.getCellReference(x,y).getOrientationIntervals();
}

OrientationIntervals& CSpaceVoronoi::getAdmissibleOrientationStructure(const int x, const int y){
  return admissibleOrientations.getCellReference(x,y);
}

int CSpaceVoronoi::worldToMapTheta(double theta){
  return floor(theta/angularResolution);
}

double CSpaceVoronoi::mapToWorldTheta(int theta){
  return (theta*angularResolution);
}

void CSpaceVoronoi::plotOrientations(){
  FILE* gnuplot = fopen("oriplot.txt", "w");

  fprintf(gnuplot, "set xrange [%d:%d]\n", -1, sizeX);
  fprintf(gnuplot, "set yrange [%d:%d]\n", -1, sizeY);
  fprintf(gnuplot, "set zrange [%d:%d]\n", -1, (int) layers.size());

  fprintf(gnuplot, "splot '-' w l\n");

  for(int x=0; x<sizeX; x=x+4){
    for(int y=0; y<sizeY; y=y+4){
      std::vector<OrientationInterval> intervals = getAdmissibleOrientations(x,y);
      for(unsigned int i=0; i<intervals.size(); i++){
        if(intervals[i].lower <= intervals[i].upper){
          fprintf(gnuplot, "%d %d %d\n%d %d %d\n\n\n", x,y,intervals[i].lower, x,y,intervals[i].upper);
        } else {
          //warparound
          fprintf(gnuplot, "%d %d %d\n%d %d %d\n\n\n", x,y,OrientationIntervals::getMinOrientation(), x,y,intervals[i].upper);
          fprintf(gnuplot, "%d %d %d\n%d %d %d\n\n\n", x,y,intervals[i].upper, x,y,OrientationIntervals::getMaxOrientation());
        }

      }
    }
  }


  fprintf(gnuplot, "e\n");

  fclose(gnuplot);

  std::stringstream outfile;
  outfile<<"\\documentclass{article}\n";
  outfile<<"\\usepackage{pgf,tikz}\n";
  outfile<<"\\begin{document}\n";
  outfile<<"\n";
  outfile<<"\\newcommand{\\oriinterval}[4]{\n";
  outfile<<"  \\begin{scope}\n";
  outfile<<"    \\clip (#1,#2) rectangle +(1,1);\n";
  outfile<<"    \\draw[green!50!black,fill=green!50!black] (#1+0.5,#2+0.5) -- +(#3:0.35) arc[start angle=#3, delta angle=#4, radius=0.35] -- cycle;\n";
  outfile<<"  \\end{scope}\n";
  outfile<<"}\n";
  outfile<<"\\begin{tikzpicture}[scale=0.1]\n";
  //\pagestyle{empty}
  //outfile<<"\\fill[fill=black] (0,0) rectangle +("<<sizeX<<","<<sizeY<<");\n";
  //\node[inner sep=0pt, anchor=south west] at(0,0) {\includegraphics[width=6.4cm]{../../testmap3.png}};

  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      std::vector<OrientationInterval> intervals = getAdmissibleOrientations(x,y);
      for(unsigned int i=0; i<intervals.size(); i++){
        int delta;

        if(intervals[i].lower <= intervals[i].upper){
          delta = intervals[i].upper-intervals[i].lower+1;
        } else {
          delta = OrientationIntervals::getMaxOrientation() - intervals[i].lower + 1 + intervals[i].upper - OrientationIntervals::getMinOrientation() + 1;
        }
        outfile<<"\\oriinterval{"<<x<<"}{"<<y<<"}{"<<intervals[i].lower*angularResolution*180/M_PI<<"}{"<<delta*angularResolution*180/M_PI<<"}\n";
      }
    }
  }

  outfile<<"\\end{tikzpicture}\n";
  outfile<<"\\end{document}\n";

  FILE* tikzfile = fopen("oriplot.tex", "w");
  fprintf(tikzfile, "%s\n", outfile.str().c_str());
  fclose(tikzfile);

  nCollidingOrientations.writeToPGM("ncollidingoris.pgm", true);
}

void CSpaceVoronoi::saveFootprints(std::string basefilename){
  unsigned int nOrientations = layers.size();
  unsigned int nParts = layers.front()->getColumnBoundaries().size();

  std::vector<std::string> numbers;
  numbers.push_back("one");
  numbers.push_back("two");
  numbers.push_back("three");
  numbers.push_back("four");
  numbers.push_back("five");
  numbers.push_back("six");
  numbers.push_back("seven");
  numbers.push_back("eight");
  numbers.push_back("nine");
  numbers.push_back("ten");
  numbers.push_back("eleven");
  numbers.push_back("twelve");
  numbers.push_back("thirteen");
  numbers.push_back("fourteen");
  numbers.push_back("fifteen");

  for(unsigned int p=0; p<nParts; p++){

    int x_min = INT_MAX;
    int x_max = INT_MIN;
    int y_min = INT_MAX;
    int y_max = INT_MIN;
    for(unsigned int oIdx=0; oIdx<nOrientations; oIdx++){
      for(unsigned int line=0; line<layers[oIdx]->getColumnBoundaries()[p].size(); line++){
        x_min = (std::min)(x_min, layers[oIdx]->getColumnBoundaries()[p][line].x1);
        x_max = (std::max)(x_max, layers[oIdx]->getColumnBoundaries()[p][line].x2);
        y_min = (std::min)(y_min, layers[oIdx]->getColumnBoundaries()[p][line].dy);
        y_max = (std::max)(y_max, layers[oIdx]->getColumnBoundaries()[p][line].dy);
      }
    }

    int padding=3;
    x_min -=padding;
    x_max +=padding;
    y_min -=padding;
    y_max +=padding;


    std::ostringstream identifier;
    identifier << std::setw( 2 ) << std::setfill( '0' ) << std::setw( 2 ) << std::setfill( '0' ) << p;
    std::string plotoverview_filename = basefilename;
    plotoverview_filename.append("_");
    plotoverview_filename.append(identifier.str());
    plotoverview_filename.append("_overview.tex");
    FILE* plotoverview = fopen(plotoverview_filename.c_str(), "w");
    fprintf(plotoverview, "\\documentclass{article}\n");
    fprintf(plotoverview, "\\usepackage{pgf,tikz}\n");
    fprintf(plotoverview, "\\newcommand{\\footprintcell}[2]{\n");
    fprintf(plotoverview, "\\fill[draw=none, fill=gray!50!white] (#1,#2) rectangle +(1,1);\n");
    fprintf(plotoverview, "}\n");
    fprintf(plotoverview, "\\newcommand{\\dimensions}[4]{\n");
    fprintf(plotoverview, "\\draw (#1,#3) grid (#2+1,#4+1);\n");
    fprintf(plotoverview, "\\draw[thick] (-0.25,0) -- (0.25,0);\n");
    fprintf(plotoverview, "\\draw[thick] (0,-0.25) -- (0,0.25);\n");
    fprintf(plotoverview, "}\n");
    fprintf(plotoverview, "\\newcommand{\\footprintpart}[1]{\n");
    fprintf(plotoverview, "\\draw[rotate=#1, thick, miter limit=3] ");
    for(unsigned int i=0; i<layers.front()->getColumns()[p].vertices.size(); i++){
      fprintf(plotoverview, "(%f,%f) -- ", layers.front()->getColumns()[p].vertices[i].first, layers.front()->getColumns()[p].vertices[i].second);
    }
    fprintf(plotoverview, "(%f,%f);\n", layers.front()->getColumns()[p].vertices.front().first, layers.front()->getColumns()[p].vertices.front().second);
    fprintf(plotoverview, "}\n");
    fprintf(plotoverview, "\n\\begin{document}\n");

    for(unsigned int oIdx=0; oIdx<nOrientations; oIdx++){
      //SimpleMap<int> countMap(x_max-x_min+1, y_max-y_min+1);
      //countMap.fill(0);

      std::stringstream tikzcommands;
      //tikzcommands<<"\\begin{tikzpicture}[scale=0.5]\n";

        for(unsigned int line=0; line<layers[oIdx]->getColumnBoundaries()[p].size(); line++){
          int minx = layers[oIdx]->getColumnBoundaries()[p][line].x1-x_min;
          int maxx = layers[oIdx]->getColumnBoundaries()[p][line].x2-x_min;
          int y = layers[oIdx]->getColumnBoundaries()[p][line].dy-y_min;

          if(minx > maxx){
            std::swap(minx, maxx);
          }
          for(int x=minx; x <= maxx; x++){
            //countMap.setCell(x, y, 1);
            tikzcommands<<"\\footprintcell{"<<x+x_min<<"}{"<<y+y_min<<"}\n";
          }
        }
        tikzcommands<<"\\dimensions{"<<x_min+padding<<"}{"<<x_max-padding<<"}{"<<y_min+padding<<"}{"<<y_max-padding<<"}\n";

        tikzcommands<<"\\footprintpart{"<<oIdx*angularResolution*180/M_PI<<"}\n";

        //temp
        if(oIdx + 1 < layers.size()){
          tikzcommands<<"\\footprintpart{"<<(oIdx+1)*angularResolution*180/M_PI<<"}\n";
        } else {
          tikzcommands<<"\\footprintpart{"<<0<<"}\n";
        }
        //tikzcommands<<"\\end{tikzpicture}\n";


        std::string filename = basefilename;
        filename.append("_");
        std::ostringstream local_identifier;
        local_identifier << "_" << std::setw( 2 ) << std::setfill( '0' )<< oIdx;
        filename.append(identifier.str());
        filename.append(local_identifier.str());
        std::string filename_tikz = filename;
        filename.append(".pgm");
        filename_tikz.append(".tikz");

        FILE* tikzfile=fopen(filename_tikz.c_str(), "w");
        fprintf(tikzfile, "%s", tikzcommands.str().c_str());
        fclose(tikzfile);
        fprintf(plotoverview, "\\begin{tikzpicture}[scale=0.1]\n");
        fprintf(plotoverview, "\\input{%s}\n", filename_tikz.c_str());
        fprintf(plotoverview, "\\end{tikzpicture}\n");

        //countMap.writeToPGM(filename.c_str(), true);
      }

      fprintf(plotoverview, "\\end{document}\n");
      fclose(plotoverview);

    }

  }

void CSpaceVoronoi::initOriNodeMap(){
  oriNodeMap.resize(sizeX, sizeY);
  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      updateOriNodeMap(x, y);
    }
  }
}

void CSpaceVoronoi::updateOriNodeMap(int x, int y){
  oriNodeMap.getCellReference(x,y).initFromOrientationIntervals(admissibleOrientations.getCellReference(x,y));
}

void CSpaceVoronoi::createOIGraph(){

  //instantiate a node for every orientation interval
  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      for(unsigned int o=0; o<oriNodeMap.getCellReference(x,y).orientationIntervals.size(); o++){
        oriNodeMap.getCellReference(x,y).nodes[o] = MemoryManager<OISearchNode>::getNew();
        oriNodeMap.getCellReference(x,y).nodes[o]->x = x;
        oriNodeMap.getCellReference(x,y).nodes[o]->y = y;
        oriNodeMap.getCellReference(x,y).nodes[o]->o = o;
      }
    }
  }

  //now connect the intervals by filling the successors field appropriately
  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      for(unsigned int o=0; o<oriNodeMap.getCellReference(x,y).orientationIntervals.size(); o++){
        oriNodeMap.getCellReference(x,y).nodes[o]->computeSuccessors(&oriNodeMap, &admissibleOrientations);
      }
    }
  }

}


void CSpaceVoronoi::initializeDStar(){
  if(dstar == NULL){
//    MemoryManager<OIDStarSearchNode>::reserve(800000);
    dstar=new DStarSearch();
    //std::cerr<<"WARNING: you did not set a dstar instance but are preparing dstar data structures!"<<std::endl;
  }
  collisionChangeLocations.clear();
  initOriNodeMap_dstar();
  createOIGraph_dstar();
}

void CSpaceVoronoi::initOriNodeMap_dstar(){
  oriNodeMap_dstar.resize(sizeX, sizeY);
  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
    	oriNodeMap_dstar.getCellReference(x,y).initFromOrientationIntervals(admissibleOrientations.getCellReference(x,y));
    }
  }
}

void CSpaceVoronoi::createOIGraph_dstar(){
	//instantiate a node for every orientation interval
	  for(int x=0; x<sizeX; x++){
	    for(int y=0; y<sizeY; y++){
	      for(unsigned int o=0; o<oriNodeMap_dstar.getCellReference(x,y).orientationIntervals.size(); o++){
	        oriNodeMap_dstar.getCellReference(x,y).nodes[o] = MemoryManager<OIDStarSearchNode>::getNew();
	        oriNodeMap_dstar.getCellReference(x,y).nodes[o]->x = x;
	        oriNodeMap_dstar.getCellReference(x,y).nodes[o]->y = y;
	        oriNodeMap_dstar.getCellReference(x,y).nodes[o]->o = o;
	        oriNodeMap_dstar.getCellReference(x,y).nodes[o]->costs = 0; //TODO correct costs
	      }
	    }
	  }

	  //now connect the intervals by filling the successors field appropriately
	  for(int x=0; x<sizeX; x++){
	    for(int y=0; y<sizeY; y++){
	      for(unsigned int o=0; o<oriNodeMap_dstar.getCellReference(x,y).orientationIntervals.size(); o++){
	      	oriNodeMap_dstar.getCellReference(x,y).nodes[o]->computeSuccessors(&oriNodeMap_dstar, &admissibleOrientations);
	      }
	    }
	  }
}


void CSpaceVoronoi::incorporateUpdatesDStar(IntPose start, IntPose goal){

  std::vector<DStarSearchNode*> changedNodesforDStar;
  std::vector<DStarSearchNode*> nodesInvalidThisRound;
  changedNodesforDStar.reserve(collisionChangeLocations.size()); //these are still too few but at least this removes some of the copying when enlarging the vector
  std::cout<<"size for update collisions "<<collisionChangeLocations.size()<<std::endl;
  std::cout<<"size of nodes to be deleted "<<nodesToBeDeleted.size()<<std::endl;
  //first we mark all the locations that have changes in their orientation intervals
  //we also set the cost of all nodes at these places to infinity
  //this includes setting their transitions to neighbors to infinity
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    OINodeConnector<OIDStarSearchNode>& currentConnector = oriNodeMap_dstar.getCellReference(it->first,it->second);
    currentConnector.dirty = true;

    for(unsigned int o=0; o<currentConnector.nodes.size(); o++){
      OIDStarSearchNode* node = currentConnector.nodes[o];
      node->costs = INFINITY;
      node->successor_costs.assign(node->successors.size(), INFINITY);
      changedNodesforDStar.push_back(node);
      nodesInvalidThisRound.push_back(node);
      nodesToBeDeleted.push_back(node);
    }
  }


  //TODO THIS LOOP HERE: maybe do not go via x/y neighbors but ask the infinity nodes for their successors and search for them again in their successor list
  //could be more robust for multiple iterations
  for(std::vector<DStarSearchNode*>::iterator it = nodesInvalidThisRound.begin(); it != nodesInvalidThisRound.end(); it++){

  	std::vector<DStarSearchNode*>* neighbors = (*it)->getSuccessors();
  	for(std::vector<DStarSearchNode*>::iterator it_neighbors = neighbors->begin(); it_neighbors!= neighbors->end(); ++it_neighbors){
  		std::vector<DStarSearchNode*>* successors = (*it_neighbors)->getSuccessors();
  		std::vector<double>* successor_costs = (*it_neighbors)->getSuccessorCosts();
  		for(std::vector<DStarSearchNode*>::iterator it_succ = successors->begin(); it_succ!= successors->end(); ++it_succ){
  			if(*it_succ == *it){
  				successor_costs->at(it_succ - successors->begin()) = INFINITY;
  			}
  		}
  	}
  }

  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
  	oriNodeMap_dstar.getCellReference(it->first,it->second).nodes.clear();
  }


/*
  //and make sure that all nodes that point to them as their successors now point to them with infinity costs
  //and remove them from the connector (only pointer to them is now in the list for D*)
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){

    int xstart = (std::max)(0,it->first-1);
    int xend = (std::min)(it->first+1, oriNodeMap_dstar.getMapSizeX()-1);
    int ystart = (std::max)(0,it->second-1);
    int yend = (std::min)(it->second+1, oriNodeMap_dstar.getMapSizeY()-1);

    OINodeConnector<OIDStarSearchNode>& currentConnector = oriNodeMap_dstar.getCellReference(it->first,it->second);
    for(int ny=ystart; ny<=yend; ny++){
      for(int nx=xstart; nx<=xend; nx++){

#ifdef DSTAR_EIGHT_CONNECTED
        if((nx==it->first && ny==it->second)){
#else
        if((nx==it->first && ny==it->second) || (nx!=it->first && ny!=it->second)){
#endif
          continue;
        } else {

          OINodeConnector<OIDStarSearchNode>& neighborConnector = oriNodeMap_dstar.getCellReference(nx,ny);
          if(neighborConnector.dirty){
            continue;
          } else {

            for(std::vector<OIDStarSearchNode*>::iterator it_neighborNodes = neighborConnector.nodes.begin(); it_neighborNodes!=neighborConnector.nodes.end(); ++it_neighborNodes){
              for(std::vector<OIDStarSearchNode*>::iterator it_deleteNodes = currentConnector.nodes.begin(); it_deleteNodes!=currentConnector.nodes.end(); ++it_deleteNodes){
                for(std::vector<DStarSearchNode*>::iterator it_NNsuccessors = (*it_neighborNodes)->successors.begin(); it_NNsuccessors != (*it_neighborNodes)->successors.end(); ++it_NNsuccessors){
                  if(*it_deleteNodes == *it_NNsuccessors){
                    //TODO can make more efficient?
                    (*it_neighborNodes)->successor_costs.at(it_NNsuccessors-(*it_neighborNodes)->successors.begin()) = INFINITY;
                  }
                }
              }
            }
          }
        }
      }
    }

    //remove the nodes from the connector
    currentConnector.nodes.clear();
  }
*/

  //now, we create new nodes for the updated orientation intervals
  //TODO maybe have a cached connector vector
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    OINodeConnector<OIDStarSearchNode>& currentConnector = oriNodeMap_dstar.getCellReference(it->first,it->second);

    currentConnector.initFromOrientationIntervals(admissibleOrientations.getCellReference(it->first, it->second));
    //now the currentConnector.nodes is filled with NULL pointers
    //we instantiate the new nodes
    for(unsigned int o=0; o<currentConnector.orientationIntervals.size(); o++){
      currentConnector.nodes[o] = MemoryManager<OIDStarSearchNode>::getNew();
      currentConnector.nodes[o]->x = it->first;
      currentConnector.nodes[o]->y = it->second;
      currentConnector.nodes[o]->o = o;
      currentConnector.nodes[o]->costs = 0; //TODO correct costs
      
      changedNodesforDStar.push_back(currentConnector.nodes[o]);
    }

  }

  //now we compute the successors for all newly created nodes
  //(they will not connect to any marked nodes since these are no longer in the node map
  //TODO maybe have a cached connector vector
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    OINodeConnector<OIDStarSearchNode>& currentConnector = oriNodeMap_dstar.getCellReference(it->first,it->second);
    for(std::vector<OIDStarSearchNode*>::iterator it=currentConnector.nodes.begin(); it!=currentConnector.nodes.end(); ++it){
      (*it)->computeSuccessors(&oriNodeMap_dstar, &admissibleOrientations);
    }
  }


  //now we add the newly created nodes to the successors list of their neighbors
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){

    int xstart = (std::max)(0,it->first-1);
    int xend = (std::min)(it->first+1, oriNodeMap_dstar.getMapSizeX()-1);
    int ystart = (std::max)(0,it->second-1);
    int yend = (std::min)(it->second+1, oriNodeMap_dstar.getMapSizeY()-1);

    //OINodeConnector<OIDStarSearchNode>& currentConnector = oriNodeMap_dstar.getCellReference(it->first,it->second);
    for(int ny=ystart; ny<=yend; ny++){
      for(int nx=xstart; nx<=xend; nx++){

#ifdef DSTAR_EIGHT_CONNECTED
        if(nx==it->first && ny==it->second){
#else
        if((nx==it->first && ny==it->second) || (nx!=it->first && ny!=it->second)){
#endif
          continue;
        } else {

          OINodeConnector<OIDStarSearchNode>& neighborConnector = oriNodeMap_dstar.getCellReference(nx,ny);
          if(neighborConnector.dirty){
            //these nodes have been completely rebuilt, we are already in their successors list
            continue;
          } else {

            for(std::vector<OIDStarSearchNode*>::iterator it_neighbornodes =neighborConnector.nodes.begin(); it_neighbornodes!= neighborConnector.nodes.end(); ++it_neighbornodes){
              //TODO maybe we can give this method more info/ cache some of the things that it is goinf to look up again in the maps
              (*it_neighbornodes)->addSuccessorsToSpecificNeighbor(it->first, it->second, &oriNodeMap_dstar, &admissibleOrientations);
            }
          }
        }
      }
    }
  }


    if(dstar!=NULL){
    	DStarSearchNode* startNode = getDStarSearchNode(start.x,start.y,start.theta);
    	dstar->setCurrentStart(startNode);
    	DStarSearchNode* goalNode = getDStarSearchNode(goal.x,goal.y,goal.theta);
    	dstar->setCurrentGoal(goalNode);
    	goalNode->setDesiredOrientation(goal.theta);
    }


  //TODO for later: update costs of nodes if necessary (compile list during distmap update) and add these nodes to the list for d*
  //TODO now tell the d* what happened, provide the list and let it update
  if(dstar != NULL){
    if (!dstar->update(changedNodesforDStar))
      std::cerr<<"NO PATH!"<<std::endl;
  } else {
    std::cerr<<"WARNING: no dstar instance set, cannot call dstar with updates! setting tag to closed for changed nodes"<<std::endl;
    for(std::vector<DStarSearchNode*>::iterator it=changedNodesforDStar.begin(); it!=changedNodesforDStar.end(); ++it){
    	(*it)->tag = CLOSED;
    }
  }


  //TODO change this loop like above todo?
  std::vector<DStarSearchNode*> nodesToBeDeletedInFuture;
  std::cout<<"size of nodes to be deleted in the future "<<nodesToBeDeletedInFuture.size()<<std::endl;
  for(std::vector<DStarSearchNode*>::iterator it = nodesInvalidThisRound.begin(); it != nodesInvalidThisRound.end(); it++){

  	if((*it)->tag != OPEN){
    	std::vector<DStarSearchNode*>* neighbors = (*it)->getSuccessors();
    	for(std::vector<DStarSearchNode*>::iterator it_neighbors = neighbors->begin(); it_neighbors!= neighbors->end(); ++it_neighbors){
    		std::vector<DStarSearchNode*>* successors = (*it_neighbors)->getSuccessors();
    		std::vector<double>* successor_costs = (*it_neighbors)->getSuccessorCosts();
    		for(std::vector<DStarSearchNode*>::iterator it_succ = successors->begin(); it_succ!= successors->end(); ++it_succ){
    			if(*it_succ == *it){

    				successor_costs->erase(successor_costs->begin() + (it_succ - successors->begin()));
						((OIDStarSearchNode*) (*it_neighbors))->successor_intersections.erase(((OIDStarSearchNode*) (*it_neighbors))->successor_intersections.begin() + (it_succ - successors->begin()));
						it_succ = successors->erase(it_succ);
						it_succ--; //loop will increment again

    			}
    		}
    	}
    	MemoryManager<OIDStarSearchNode>::destroy((OIDStarSearchNode*) *it);
  	} else {
//  	  printf("not deleted element: cost %f tag %d k %d h %d\n", (*it)->getCosts(), (*it)->tag, (*it)->k, (*it)->h);
  		nodesToBeDeletedInFuture.push_back(*it);
  	}
  }
  std::cout<<"size of nodes to be deleted in the future "<<nodesToBeDeletedInFuture.size()<<std::endl;



/*
  //now delete all the nodes that we set to infinity and remove them from the successor list of their (unmarked) neighbors
  for(std::vector<DStarSearchNode*>::iterator it=nodesToBeDeleted.begin(); it!=nodesToBeDeleted.end(); ++it){

    OIDStarSearchNode* currentNode = (OIDStarSearchNode*) *it;
    //this is a new condition. let the nodes live until d star has actually processed them
    if(currentNode->tag == CLOSED){
    	//from all neighbors that are not dirty, we need to remove ourselves from their successors
    	int xstart = (std::max)(0,currentNode->x-1);
    	int xend = (std::min)(currentNode->x+1, oriNodeMap_dstar.getMapSizeX()-1);
    	int ystart = (std::max)(0,currentNode->y-1);
    	int yend = (std::min)(currentNode->y+1, oriNodeMap_dstar.getMapSizeY()-1);

    	//OINodeConnector<OIDStarSearchNode>& currentConnector = oriNodeMap_dstar.getCellReference(currentNode->x,currentNode->y);
    	for(int ny=ystart; ny<=yend; ny++){
    		for(int nx=xstart; nx<=xend; nx++){

#ifdef DSTAR_EIGHT_CONNECTED
    			if(nx==currentNode->x && ny==currentNode->y){
#else
    			if((nx==currentNode->x && ny==currentNode->y) || (nx!=currentNode->x && ny!=currentNode->y)){
#endif
    				continue;
    			} else {

    				OINodeConnector<OIDStarSearchNode>& neighborConnector = oriNodeMap_dstar.getCellReference(nx,ny);
    				if(neighborConnector.dirty){
    					//these nodes in the connector have been rebuilt, cannot contain any link to us
    					continue;
    				} else {

    					for(std::vector<OIDStarSearchNode*>::iterator it_neighborNodes = neighborConnector.nodes.begin(); it_neighborNodes!=neighborConnector.nodes.end(); ++it_neighborNodes){
    						for(std::vector<DStarSearchNode*>::iterator it_NNsuccessors = (*it_neighborNodes)->successors.begin(); it_NNsuccessors != (*it_neighborNodes)->successors.end(); ++it_NNsuccessors){
    							if(*it_NNsuccessors == currentNode){
    								it_NNsuccessors = (*it_neighborNodes)->successors.erase(it_NNsuccessors);
    								(*it_neighborNodes)->successor_costs.erase((*it_neighborNodes)->successor_costs.begin() + (it_NNsuccessors-(*it_neighborNodes)->successors.begin()));
    								(*it_neighborNodes)->successor_intersections.erase((*it_neighborNodes)->successor_intersections.begin() + (it_NNsuccessors-(*it_neighborNodes)->successors.begin()));
    								it_NNsuccessors--; //loop will increment again
    							}
    						}
    					}
    				}
    			}
   			}
   		}
   	}
  }


  //Now we can finally kill the infinity nodes
  for(std::vector<DStarSearchNode*>::iterator it=nodesInvalidThisRound.begin(); it!=nodesInvalidThisRound.end(); ++it){
    OIDStarSearchNode* currentNode = (OIDStarSearchNode*) *it;
    //this is a new condition. let the nodes live until d star has actually processed them
    if(currentNode->tag == CLOSED){
    	MemoryManager<OIDStarSearchNode>::destroy(currentNode);
    } else {
    	nodesToBeDeleted.push_back(currentNode);
    }
  }
*/

  //todo what to do with old scheduled deletions, they are still in nodes to be deleted
  nodesToBeDeleted = nodesToBeDeletedInFuture;
  std::cout<<"size of nodes to be deleted "<<nodesToBeDeleted.size()<<std::endl;

  //finally, we can remove the dirty flag
  for(std::set<std::pair<int, int> >::const_iterator it=collisionChangeLocations.begin(); it!=collisionChangeLocations.end(); ++it){
    oriNodeMap_dstar.getCellReference(it->first,it->second).dirty = false;
  }
  collisionChangeLocations.clear();


}

int CSpaceVoronoi::getOrientationIntervalIndex(int x, int y, int theta){
  OrientationInterval tmp;
  tmp.lower = theta;
  tmp.upper = theta;
  std::vector<unsigned int> indices;
  admissibleOrientations.getCellReference(x,y).getIntervalIndicesWithNonEmptyIntersectionWithInterval(tmp, &indices);

  if(indices.empty()){
    return -1;
  } else {
    return indices.front();
  }
}

DStarSearchNode* CSpaceVoronoi::getDStarSearchNode(int x, int y, int theta){
  int idx = getOrientationIntervalIndex(x,y,theta);
  if(idx != -1){
    return oriNodeMap_dstar.getCellReference(x,y).nodes[idx];
  } else {
    return NULL;
  }
}


SearchNode* CSpaceVoronoi::getSearchNode(int x, int y, int theta){
  int idx = getOrientationIntervalIndex(x,y,theta);
  if(idx != -1){
    return oriNodeMap.getCellReference(x,y).nodes[idx];
  } else {
    return NULL;
  }
}


void CSpaceVoronoi::testPlanning(){

  MemoryManager<OISearchNode>::reserve(80000);

  std::cerr<<"initOriNodeMap"<<std::endl;
  double t_start = Helper::getTime();
  initOriNodeMap();
//  std::cerr<<"create Graph"<<std::endl;
//  OISearchNode::nodeMap = &oriNodeMap;
//  OISearchNode::admissibleOrientations= &admissibleOrientations;
  createOIGraph();
  double t_init = Helper::getTime()-t_start;
  std::cerr<<"done"<<std::endl;

  //now instantiate a search class
  AStar astar;

  //OISearchNode* startNode = oriNodeMap.getCellReference(30,30).nodes[0];
  SearchNode* startNode = getSearchNode(30,30,0);

  //OISearchNode* goalNode = oriNodeMap.getCellReference(30,220).nodes[0];
  SearchNode* goalNode = getSearchNode(30,220,0);

  t_start = Helper::getTime();
  std::vector<SearchNode*>* path = astar.searchPath(startNode, goalNode);
  double t_search = Helper::getTime()-t_start;


  if(path!=NULL){
  	for(unsigned int i=0; i<path->size(); i++){
  		int x = ((OISearchNode*) path->at(i))->x;
  		int y = ((OISearchNode*) path->at(i))->y;
  		OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[((OISearchNode*) path->at(i))->o];
  		std::cerr<<x<<","<<y<<",["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
  	}
  }

  std::cerr<<"init node map took "<<t_init<<std::endl;
  std::cerr<<"search path took "<<t_search<<std::endl;

  std::cerr<<"objects in use "<<MemoryManager<OISearchNode>::getNumObjectsInUse()<<std::endl;

  if(path!=NULL){
    plotPath("testpath", path);
  }

  std::cerr<<" startNode is "<<startNode<<" goalNode is "<<goalNode<<std::endl;
  t_start = Helper::getTime();
  path = astar.searchPath(startNode, goalNode);
  t_search = Helper::getTime()-t_start;


  if(path!=NULL){
    std::cerr<<"path has "<<path->size()<<" states"<<std::endl;
    /*for(unsigned int i=0; i<path->size(); i++){
        int x = ((OISearchNode*) path->at(i))->x;
        int y = ((OISearchNode*) path->at(i))->y;
        OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[((OISearchNode*) path->at(i))->o];
        std::cerr<<x<<","<<y<<",["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
    }*/
  }

  std::cerr<<"search path took "<<t_search<<std::endl;
  std::cerr<<"objects in use "<<MemoryManager<OISearchNode>::getNumObjectsInUse()<<std::endl;


  delete path;

}

void CSpaceVoronoi::testPlanningDStar(IntPose start, IntPose goal){

  MemoryManager<OIDStarSearchNode>::reserve(80000);

  std::cerr<<"initOriNodeMap"<<std::endl;
  double t_start = Helper::getTime();
  //now instantiate a search class
  if (dstar==NULL)
    dstar=new DStarSearch();
  initializeDStar();
//  std::cerr<<"create Graph"<<std::endl;
//  OISearchNode::nodeMap = &oriNodeMap;
//  OISearchNode::admissibleOrientations= &admissibleOrientations;
  double t_init = Helper::getTime()-t_start;
  std::cerr<<"done"<<std::endl;


  //OISearchNode* startNode = oriNodeMap.getCellReference(30,30).nodes[0];
//  DStarSearchNode* startNode = getDStarSearchNode(30,30,0);
  DStarSearchNode* startNode = getDStarSearchNode(start.x,start.y,start.theta);
  DStarSearchNode* goalNode = getDStarSearchNode(goal.x,goal.y,goal.theta);

  //OISearchNode* goalNode = oriNodeMap.getCellReference(30,220).nodes[0];
//  DStarSearchNode* goalNode = getDStarSearchNode(30,220,0);
  goalNode->setDesiredOrientation(goal.theta);

  std::cerr<<"start cost "<<startNode->getCosts()<<std::endl;

  t_start = Helper::getTime();
  if (dstar->initialComputations(startNode, goalNode)){
  std::vector<DStarSearchNode*>* path = dstar->getPath(startNode);
  double t_search = Helper::getTime()-t_start;

  std::cerr<<"init node map took "<<t_init<<std::endl;
  std::cerr<<"search path took "<<t_search<<std::endl;

//  startNode = getDStarSearchNode(30,130,0);
//  t_start = Helper::getTime();
//  path = dstar->getPath(startNode);
//  t_search = Helper::getTime()-t_start;

//  std::cerr<<"search path took "<<t_search<<std::endl;

  if(path!=NULL){
  	for(unsigned int i=0; i<path->size(); i++){
  		int x = ((OIDStarSearchNode*) path->at(i))->x;
  		int y = ((OIDStarSearchNode*) path->at(i))->y;
        int o = ((OIDStarSearchNode*) path->at(i))->o;
//        std::cerr<<x<<","<<y<<","<<o<<std::endl;
        std::vector<OrientationInterval> intervals = admissibleOrientations.getCellReference(x,y).getOrientationIntervals();
//        std::cerr<<"size "<<intervals.size()<<std::endl;
//        std::cerr<<"costs: "<<((OIDStarSearchNode*) path->at(i))->costs<<std::endl;
        OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[o];
     		std::cerr<<x<<","<<y<<","<<(path->at(i))->getDesiredOrientation()<<", ["<<interval.lower<<","<<interval.upper<<"]=["<<(path->at(i))->getOrientationInterval().lower<<","<<(path->at(i))->getOrientationInterval().upper<<"]"<<std::endl;
  DStarSearchNode* xNode = getDStarSearchNode(x,y,(path->at(i))->getDesiredOrientation());
        std::cerr<<xNode->h<<std::endl;
        assert(std::isinf(((OIDStarSearchNode*) path->at(i))->costs)==false);
  	}
  }


  std::cerr<<"objects in use "<<MemoryManager<OIDStarSearchNode>::getNumObjectsInUse()<<std::endl;

  std::cerr<<" startNode is "<<startNode<<" goalNode is "<<goalNode<<std::endl;

  if(path!=NULL){
    plotPath_dstar("testpathdstar", path);
  }


  if(path!=NULL){
    std::cerr<<"path has "<<path->size()<<" states"<<std::endl;
    /*for(unsigned int i=0; i<path->size(); i++){
        int x = ((OISearchNode*) path->at(i))->x;
        int y = ((OISearchNode*) path->at(i))->y;
        OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[((OISearchNode*) path->at(i))->o];
        std::cerr<<x<<","<<y<<",["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
    }*/
  }


  delete path;
  }else{
  std::cerr<<"no path!"<<std::endl;
  }

}

void CSpaceVoronoi::testRePlanningDStar(IntPose start, IntPose goal){

  MemoryManager<OIDStarSearchNode>::reserve(80000);

  double t_start = Helper::getTime();

  incorporateUpdatesDStar(start, goal);
  DStarSearchNode* startNode = getDStarSearchNode(start.x,start.y,start.theta);

  std::vector<DStarSearchNode*>* path = dstar->getPath(startNode);
  double t_search = Helper::getTime()-t_start;

  std::cerr<<"search path took "<<t_search<<std::endl;

//  startNode = getDStarSearchNode(30,130,0);
//  t_start = Helper::getTime();
//  path = dstar->getPath(startNode);
//  t_search = Helper::getTime()-t_start;

//  std::cerr<<"search path took "<<t_search<<std::endl;

  if(path!=NULL){
  	for(unsigned int i=0; i<path->size(); i++){
  		int x = ((OIDStarSearchNode*) path->at(i))->x;
  		int y = ((OIDStarSearchNode*) path->at(i))->y;
  		int o = ((OIDStarSearchNode*) path->at(i))->o;
//  		std::cerr<<x<<","<<y<<","<<o<<std::endl;
  		std::vector<OrientationInterval> intervals = admissibleOrientations.getCellReference(x,y).getOrientationIntervals();
//  		std::cerr<<"size "<<intervals.size()<<std::endl;
//  		std::cerr<<"costs: "<<((OIDStarSearchNode*) path->at(i))->costs<<std::endl;
  		OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[o];
  		std::cerr<<x<<","<<y<<",["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
  		assert(std::isinf(((OIDStarSearchNode*) path->at(i))->costs)==false);
  	}
  }else{
    std::cerr<<"no path!"<<std::endl;  
  }


  std::cerr<<"objects in use "<<MemoryManager<OIDStarSearchNode>::getNumObjectsInUse()<<std::endl;

  if(path!=NULL){
    plotPath_dstar("testpathdstarre", path);
  }


  if(path!=NULL){
    std::cerr<<"path has "<<path->size()<<" states"<<std::endl;
    /*for(unsigned int i=0; i<path->size(); i++){
        int x = ((OISearchNode*) path->at(i))->x;
        int y = ((OISearchNode*) path->at(i))->y;
        OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[((OISearchNode*) path->at(i))->o];
        std::cerr<<x<<","<<y<<",["<<interval.lower<<","<<interval.upper<<"]"<<std::endl;
    }*/
  }


  delete path;

}

std::vector<DStarSearchNode*>* CSpaceVoronoi::initialPlanningDStar(IntPose start, IntPose goal){

//  MemoryManager<OIDStarSearchNode>::reserve(80000);

  double t_start = Helper::getTime();
  DStarSearchNode* startNode = getDStarSearchNode(start.x,start.y,start.theta);
  DStarSearchNode* goalNode = getDStarSearchNode(goal.x,goal.y,goal.theta);
//  startNode->setDesiredOrientation(start.theta);
  goalNode->setDesiredOrientation(goal.theta);
  dstar->initialComputations(startNode, goalNode);
  std::vector<DStarSearchNode*>* path = dstar->getPath(startNode);
  double t_search = Helper::getTime()-t_start;
  std::cout<<"search path took "<<t_search<<std::endl;
  std::cout<<"objects in use "<<MemoryManager<OIDStarSearchNode>::getNumObjectsInUse()<<std::endl;
  std::cout<<" startNode is "<<startNode<<" goalNode is "<<goalNode<<std::endl;
  if(path!=NULL){
    std::cout<<"path has "<<path->size()<<" states"<<std::endl;
  }else{
    std::cerr<<"no path!"<<std::endl;  
  }
  t_search = Helper::getTime()-t_start;
  time_search=t_search;
  explored_nodes=dstar->nmbExplored;
  std::cout<<"print of path took "<<t_search<<std::endl;
  int cnt=0;
  for(int x=0; x<oriNodeMap_dstar.getMapSizeX(); x++){
    for(int y=0; y<oriNodeMap_dstar.getMapSizeY(); y++){
      cnt+=(oriNodeMap_dstar.getCellReference(x,y).nodes.size());
    }
  }
  std::cout<<"number of ori nodes "<<cnt<<std::endl;

  return path;

}

std::vector<DStarSearchNode*>* CSpaceVoronoi::replanningDStar(IntPose start, IntPose goal){

//  MemoryManager<OIDStarSearchNode>::reserve(80000);
  double t_start = Helper::getTime();
  incorporateUpdatesDStar(start, goal);
  DStarSearchNode* startNode = getDStarSearchNode(start.x,start.y,start.theta);
  std::vector<DStarSearchNode*>* path = dstar->getPath(startNode);
  double t_search = Helper::getTime()-t_start;
  std::cout<<"search path took "<<t_search<<std::endl;
  if(path!=NULL){
    std::cout<<"path has "<<path->size()<<" states"<<std::endl;
  }else{
    std::cerr<<"no path!"<<std::endl;  
  }
  time_search=t_search;
  explored_nodes=dstar->nmbExplored;
  std::cout<<"objects in use "<<MemoryManager<OIDStarSearchNode>::getNumObjectsInUse()<<std::endl;
  return path;

}

void CSpaceVoronoi::plotPath(std::string filename, std::vector<SearchNode*>* path){

	std::stringstream outfile;

	std::string mapfile_base = filename;
	mapfile_base.append("_map");
	std::string mapfile_pgm = mapfile_base;
	mapfile_pgm.append(".pgm");

	gridMap.writeToPGM(mapfile_pgm.c_str(), true);

    std::string mapfile_png = mapfile_base;
    mapfile_png.append(".png");


	std::string convert_cmd;
	convert_cmd = "convert ";
	convert_cmd.append(mapfile_pgm);
	convert_cmd.append(" ");
	convert_cmd.append(mapfile_png);

	system(convert_cmd.c_str());

  outfile<<"\\documentclass{article}\n";
  outfile<<"\\usepackage{pgf,tikz,pgfplots}\n";
  outfile<<"\\usetikzlibrary{calc}\n";
  outfile<<"\\begin{document}\n";
  outfile<<"\n";
  outfile<<"\\newcommand{\\pathelement}[4]{\n";
  outfile<<"%  \\draw[green] (axis cs:#1,#2) circle (0.5pt);\n";


  for(unsigned int p=0; p<columns.size(); p++){
    outfile<<"\\draw[green, miter limit=3, rotate around={#3*180/3.1416:(axis cs:#1,#2)}] ";
    for(unsigned int i=0; i<columns.front().vertices.size(); i++){
      outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns[p].vertices[i].first<<","<<columns[p].vertices[i].second<<")$) -- ";
    }
    outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns[p].vertices.front().first<<","<<columns[p].vertices.front().second<<")$);\n";
  }

  outfile<<"\\draw[red, miter limit=3, rotate around={#4*180/3.1416:(axis cs:#1,#2)}] ";
  for(unsigned int i=0; i<columns.front().vertices.size(); i++){
    outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns.front().vertices[i].first<<","<<columns.front().vertices[i].second<<")$) -- ";
  }
  outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns.front().vertices.front().first<<","<<columns.front().vertices.front().second<<")$);";

  outfile<<"}\n";
  outfile<<"\\begin{tikzpicture}\n";
  outfile<<"\\begin{axis}[axis equal]\n";
  outfile<<"  \\addplot graphics[xmin=0, ymin=0, xmax="<<sizeX<<", ymax="<<sizeY<<"]{"<<mapfile_png<<"};\n";

  for(unsigned int i=0; i<path->size(); i++){
		int x = ((OISearchNode*) path->at(i))->x;
		int y = ((OISearchNode*) path->at(i))->y;
		OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[((OISearchNode*) path->at(i))->o];
		outfile<<"  \\pathelement{"<<x<<"}{"<<y<<"}{"<<mapToWorldTheta(interval.lower)<<"}{"<<mapToWorldTheta(interval.upper)<<"}\n";
  }

  outfile<<"\\end{axis}\n";
  outfile<<"\\end{tikzpicture}\n";
  outfile<<"\\end{document}\n";


  std::string texfile = filename;
  texfile.append(".tex");
  FILE* output = fopen(texfile.c_str(), "w");
  fprintf(output, "%s", outfile.str().c_str());
  fclose(output);
}

void CSpaceVoronoi::plotPath_dstar(std::string filename, std::vector<DStarSearchNode*>* path){

    std::stringstream outfile;

    std::string mapfile_base = filename;
    mapfile_base.append("_map");
    std::string mapfile_pgm = mapfile_base;
    mapfile_pgm.append(".pgm");

    gridMap.writeToPGM(mapfile_pgm.c_str(), true);

    std::string mapfile_png = mapfile_base;
    mapfile_png.append(".png");


    std::string convert_cmd;
    convert_cmd = "convert ";
    convert_cmd.append(mapfile_pgm);
    convert_cmd.append(" ");
    convert_cmd.append(mapfile_png);

    system(convert_cmd.c_str());

  outfile<<"\\documentclass{article}\n";
  outfile<<"\\usepackage{pgf,tikz,pgfplots}\n";
  outfile<<"\\usetikzlibrary{calc}\n";
  outfile<<"\\begin{document}\n";
  outfile<<"\n";
  outfile<<"\\newcommand{\\pathelement}[4]{\n";
  outfile<<"%  \\draw[green, miter limit=3] (axis cs:#1,#2) circle (0.5pt);\n";

  for(unsigned int p=0; p<columns.size(); p++){
    outfile<<"\\draw[green, miter limit=3, rotate around={#3*180/3.1416:(axis cs:#1,#2)}] ";
    for(unsigned int i=0; i<columns.front().vertices.size(); i++){
      outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns[p].vertices[i].first<<","<<columns[p].vertices[i].second<<")$) -- ";
    }
    outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns[p].vertices.front().first<<","<<columns[p].vertices.front().second<<")$);\n";
  }

  outfile<<"\\draw[red, miter limit=3, rotate around={#4*180/3.1416:(axis cs:#1,#2)}] ";
  for(unsigned int i=0; i<columns.front().vertices.size(); i++){
    outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns.front().vertices[i].first<<","<<columns.front().vertices[i].second<<")$) -- ";
  }
  outfile<<"($(axis cs:#1,#2)+(axis cs:"<<columns.front().vertices.front().first<<","<<columns.front().vertices.front().second<<")$);";

  outfile<<"}\n";
  outfile<<"\\begin{tikzpicture}\n";
  outfile<<"\\begin{axis}[axis equal]\n";
  outfile<<"  \\addplot graphics[xmin=0, ymin=0, xmax="<<sizeX<<", ymax="<<sizeY<<"]{"<<mapfile_png<<"};\n";

  for(unsigned int i=0; i<path->size(); i++){
        int x = ((OIDStarSearchNode*) path->at(i))->x;
        int y = ((OIDStarSearchNode*) path->at(i))->y;
        OrientationInterval interval = admissibleOrientations.getCellReference(x,y).getOrientationIntervals()[((OIDStarSearchNode*) path->at(i))->o];
        outfile<<"  \\pathelement{"<<x<<"}{"<<y<<"}{"<<mapToWorldTheta(interval.lower)<<"}{"<<mapToWorldTheta(interval.upper)<<"}\n";
  }

  outfile<<"\\end{axis}\n";
  outfile<<"\\end{tikzpicture}\n";
  outfile<<"\\end{document}\n";


  std::string texfile = filename;
  texfile.append(".tex");
  FILE* output = fopen(texfile.c_str(), "w");
  fprintf(output, "%s", outfile.str().c_str());
  fclose(output);
}
