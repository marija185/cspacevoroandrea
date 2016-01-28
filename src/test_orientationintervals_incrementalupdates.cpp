/*
 * test_orientationintervals_incrementalupdates.cpp
 *
 *  Created on: Nov 25, 2012
 *      Author: sprunkc
 */

#include "cspacevoronoi.h"
#include "OrientationIntervals.h"
#include <iostream>
#include <fstream>
#include <stdio.h>

#define TESTDSTAR 1

bool compareOrientationIntervals(OrientationIntervals& incremental, OrientationIntervals &brute){
  std::vector<OrientationInterval> oris_inc = incremental.getOrientationIntervals();
  std::vector<OrientationInterval> oris_brt = brute.getOrientationIntervals();

  if(oris_inc.size() != oris_brt.size()){
    std::cerr<<"Orientation interval mismatch (size)"<<std::endl;
    std::cerr<<"inc: "<<OrientationIntervals::printOrientationIntervals(oris_inc)<<std::endl;
    std::cerr<<"brt: "<<OrientationIntervals::printOrientationIntervals(oris_brt)<<std::endl;
    return false;
  } else {
    for(unsigned int i=0; i<oris_inc.size(); i++){
      if((oris_inc[i].lower != oris_brt[i].lower) || (oris_inc[i].upper != oris_brt[i].upper)){
        std::cerr<<"Orientation interval mismatch (interval borders)"<<std::endl;
        std::cerr<<"inc: "<<OrientationIntervals::printOrientationIntervals(oris_inc)<<std::endl;
        std::cerr<<"brt: "<<OrientationIntervals::printOrientationIntervals(oris_brt)<<std::endl;
        return false;
      }
    }
  }
  return true;
}

bool compareAdmissibleOrientations(CSpaceVoronoi* incremental, CSpaceVoronoi* brute){

  int sizeX = incremental->sizeX;
  int sizeY = incremental->sizeY;

  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      bool result = compareOrientationIntervals(
          incremental->admissibleOrientations.getCellReference(x,y),
          brute->admissibleOrientations.getCellReference(x,y));
      if(result==false){
        std::cerr<<"This happened for cell "<<x<<","<<y<<std::endl;
        return false;
      }
    }
  }
  return true;
}


bool identicalOINodes(OISearchNode* node_incremental, OISearchNode* node_brute){
  return ((node_incremental->x == node_brute->x) && (node_incremental->y == node_brute->y) && (node_incremental->o == node_brute->o));
}

bool identicalOINodes_dstar(OIDStarSearchNode* node_incremental, OIDStarSearchNode* node_brute){
  return ((node_incremental->x == node_brute->x) && (node_incremental->y == node_brute->y) && (node_incremental->o == node_brute->o));
}


bool compareOINodes(OISearchNode* node_incremental, OISearchNode* node_brute){

  if(node_incremental == NULL){
    std::cerr<<"node_incremental is a NULL pointer."<<std::endl;
    return false;
  }

  if(node_brute == NULL){
    std::cerr<<"node_brute is a NULL pointer."<<std::endl;
    return false;
  }

  std::vector<SearchNode*>& successors_inc = *node_incremental->getSuccessors();
  std::vector<SearchNode*>& successors_brt = *node_brute->getSuccessors();

  if(successors_inc.size() != successors_brt.size()){
    std::cerr<<"Number of successors does not match for node. #incremental "<<successors_inc.size()<<" #brute "<<successors_brt.size()<<std::endl;
    return false;
  } else {
    for(unsigned int idxSuccInc=0; idxSuccInc<successors_inc.size(); idxSuccInc++){
      bool found = false;
      for(unsigned int idxSuccBrt=0; idxSuccBrt<successors_brt.size(); idxSuccBrt++){
        if(identicalOINodes((OISearchNode*) successors_inc[idxSuccInc], (OISearchNode*) successors_brt[idxSuccBrt])){
          found = true;

          std::vector<double>& successor_costs_inc = *node_incremental->getSuccessorCosts();
          std::vector<double>& successor_costs_brt = *node_brute->getSuccessorCosts();
          if(fabs(successor_costs_inc.at(idxSuccInc) - successor_costs_brt.at(idxSuccBrt)) > 0.00001){
          	std::cerr<<"successor costs mismatch! inc: "<<successor_costs_inc.at(idxSuccInc)<<" brt: "<<successor_costs_brt.at(idxSuccBrt)<<std::endl;
          	std::cerr<<"all costs inc:";
          	for(unsigned int j=0; j<successor_costs_inc.size(); j++){
          		std::cerr<<" "<<successor_costs_inc[j];
          	}
          	std::cerr<<"\n all costs brt:";
          	for(unsigned int j=0; j<successor_costs_brt.size(); j++){
          		std::cerr<<" "<<successor_costs_brt[j];
          	}
          	std::cerr<<std::endl;
          	std::cerr<<"maxori "<<OrientationIntervals::getMaxOrientation()<<std::endl;

          	return false;
          }
          break;
        }
      }
      if(found == false){
        std::cerr<<"Cannot find successor of incremental node in the successors of the brute node."<<std::endl;
        return false;
      }
    }
  }

  std::vector<double>& successor_costs_inc = *node_incremental->getSuccessorCosts();
  std::vector<double>& successor_costs_brt = *node_brute->getSuccessorCosts();
  if(successor_costs_inc.size() != successor_costs_brt.size()){
  	std::cerr<<"Number of successor costs does not match for node. #incremental "<<successor_costs_inc.size()<<" #brute "<<successor_costs_brt.size()<<std::endl;
  	return false;
  }

  return true;
}

bool compareOINodes_dstar(OIDStarSearchNode* node_incremental, OIDStarSearchNode* node_brute){

  if(node_incremental == NULL){
    std::cerr<<"node_incremental is a NULL pointer."<<std::endl;
    return false;
  }

  if(node_brute == NULL){
    std::cerr<<"node_brute is a NULL pointer."<<std::endl;
    return false;
  }

  std::vector<DStarSearchNode*>& successors_inc = *node_incremental->getSuccessors();
  std::vector<DStarSearchNode*>& successors_brt = *node_brute->getSuccessors();
  std::vector<double>& successor_costs_inc = *node_incremental->getSuccessorCosts();
  std::vector<double>& successor_costs_brt = *node_brute->getSuccessorCosts();

  std::vector<OrientationInterval>& successor_intersects_inc = node_incremental->successor_intersections;
  std::vector<OrientationInterval>& successor_intersects_brt = node_brute->successor_intersections;


  if(successors_inc.size() != successor_costs_inc.size()){
    std::cerr<<"successors do not match size of succesor costs: incremental."<<std::endl;
    return false;
  }

  if(successors_brt.size() != successor_costs_brt.size()){
    std::cerr<<"successors do not match size of succesor costs: brute."<<std::endl;
    return false;
  }

  if(successor_intersects_inc.size() != successor_intersects_brt.size()){
    std::cerr<<"successor intersects sizes do not match: inc "<<successor_intersects_inc.size()<<" brt "<<successor_intersects_brt.size()<<std::endl;
    return false;
  }

  if(successor_intersects_inc.size() != successors_inc.size() ){
    std::cerr<<"successor intersects and successor sizes do not match: intersects"<<successor_intersects_inc.size()<<" successors "<<successors_inc.size()<<std::endl;
    return false;
  }

  if(successor_intersects_brt.size() != successors_brt.size() ){
    std::cerr<<"successor intersects and successor sizes do not match: intersects"<<successor_intersects_brt.size()<<" successors "<<successors_brt.size()<<std::endl;
    return false;
  }



  if(successors_inc.size() != successors_brt.size()){
    std::cerr<<"Number of successors does not match for node. #incremental "<<successors_inc.size()<<" #brute "<<successors_brt.size()<<std::endl;
    return false;
  } else {
    for(unsigned int idxSuccInc=0; idxSuccInc<successors_inc.size(); idxSuccInc++){
      bool found = false;
      for(unsigned int idxSuccBrt=0; idxSuccBrt<successors_brt.size(); idxSuccBrt++){
        if(   identicalOINodes_dstar((OIDStarSearchNode*) successors_inc[idxSuccInc], (OIDStarSearchNode*) successors_brt[idxSuccBrt])
           && (successor_intersects_inc[idxSuccInc].lower == successor_intersects_brt[idxSuccBrt].lower)
           && (successor_intersects_inc[idxSuccInc].upper == successor_intersects_brt[idxSuccBrt].upper)
        ){
          found = true;

          if(fabs(successor_costs_inc.at(idxSuccInc) - successor_costs_brt.at(idxSuccBrt)) > 0.00001){
            std::cerr<<"successor costs mismatch! inc: "<<successor_costs_inc.at(idxSuccInc)<<" brt: "<<successor_costs_brt.at(idxSuccBrt)<<std::endl;
            std::cerr<<"all costs inc:";
            for(unsigned int j=0; j<successor_costs_inc.size(); j++){
                std::cerr<<" "<<successor_costs_inc[j];
            }
            std::cerr<<"\n all costs brt:";
            for(unsigned int j=0; j<successor_costs_brt.size(); j++){
                std::cerr<<" "<<successor_costs_brt[j];
            }
            std::cerr<<std::endl;
            std::cerr<<"maxori "<<OrientationIntervals::getMaxOrientation()<<std::endl;

            return false;
          }

          break;
        }
      }
      if(found == false){
        std::cerr<<"Cannot find successor of incremental node in the successors of the brute node."<<std::endl;
        std::cerr<<"The incremental node is: "<<node_incremental->x<<","<<node_incremental->y<<","<<node_incremental->o<<" cost "<<node_incremental->costs<<std::endl;
        std::cerr<<"The brute node is: "<<node_brute->x<<","<<node_brute->y<<","<<node_brute->o<<" cost "<<node_brute->costs<<std::endl;
        std::cerr<<"Successors of incremental"<<std::endl;
        for(unsigned int i=0; i<successors_inc.size(); i++){
          OIDStarSearchNode* node = (OIDStarSearchNode*) successors_inc[i];
          std::cerr<<node->x<<","<<node->y<<","<<node->o<<" cost "<<node->costs<<" mem "<<node<<std::endl;
        }
        std::cerr<<"Successors of brute"<<std::endl;
        for(unsigned int i=0; i<successors_brt.size(); i++){
          OIDStarSearchNode* node = (OIDStarSearchNode*) successors_brt[i];
          std::cerr<<node->x<<","<<node->y<<","<<node->o<<" cost "<<node->costs<<" mem "<<node<<std::endl;
        }

        return false;
      }
    }
  }
  return true;
}

bool compareOIGraphs(CSpaceVoronoi* incremental, CSpaceVoronoi* brute){

  //attention: the ori graph must have been built for this
  //and for the incremental the rewire method needs to be called.

  //for each x,y get all the nodes
  //and ask them for their successors
  //then: we want the successor list to be the same size
  //and: we want to be able to find all of the successors of the one in the successor list of the other

  int sizeX = incremental->sizeX;
  int sizeY = incremental->sizeY;

  SimplisticMap<OINodeConnector<OISearchNode> >& oriNodeMap_incremental = incremental->oriNodeMap;
  SimplisticMap<OINodeConnector<OISearchNode> >& oriNodeMap_brute = brute->oriNodeMap;

  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      if(oriNodeMap_incremental.getCellReference(x,y).nodes.size() != oriNodeMap_brute.getCellReference(x,y).nodes.size()){
        std::cerr<<"Number of nodes mismatch at location "<<x<<","<<y<<" #incremental "<<oriNodeMap_incremental.getCellReference(x,y).nodes.size()<<" #brute "<<oriNodeMap_brute.getCellReference(x,y).nodes.size()<<std::endl;
        return false;
      } else {
        for(unsigned int o=0; o<oriNodeMap_incremental.getCellReference(x,y).nodes.size(); o++){
          bool result = compareOINodes(oriNodeMap_incremental.getCellReference(x,y).nodes[o], oriNodeMap_brute.getCellReference(x,y).nodes[o]);
          if(result == false){
            std::cerr<<"This happened at location "<<x<<","<<y<<" for orientation interval index "<<o<<std::endl;
            return false;
          }
        }
      }
    }
  }

  return true;
}

bool compareOIGraphs_dstar(CSpaceVoronoi* incremental, CSpaceVoronoi* brute){

  //attention: the ori graph must have been built for this
  //and for the incremental the rewire method needs to be called.

  //for each x,y get all the nodes
  //and ask them for their successors
  //then: we want the successor list to be the same size
  //and: we want to be able to find all of the successors of the one in the successor list of the other

  int sizeX = incremental->sizeX;
  int sizeY = incremental->sizeY;

  SimplisticMap<OINodeConnector<OIDStarSearchNode> >& oriNodeMap_incremental = incremental->oriNodeMap_dstar;
  SimplisticMap<OINodeConnector<OIDStarSearchNode> >& oriNodeMap_brute = brute->oriNodeMap_dstar;

  for(int x=0; x<sizeX; x++){
    for(int y=0; y<sizeY; y++){
      if(oriNodeMap_incremental.getCellReference(x,y).nodes.size() != oriNodeMap_brute.getCellReference(x,y).nodes.size()){
        std::cerr<<"Number of nodes mismatch at location "<<x<<","<<y<<" #incremental "<<oriNodeMap_incremental.getCellReference(x,y).nodes.size()<<" #brute "<<oriNodeMap_brute.getCellReference(x,y).nodes.size()<<std::endl;
        return false;
      } else {
        for(unsigned int o=0; o<oriNodeMap_incremental.getCellReference(x,y).nodes.size(); o++){
          bool result = compareOINodes_dstar(oriNodeMap_incremental.getCellReference(x,y).nodes[o], oriNodeMap_brute.getCellReference(x,y).nodes[o]);
          if(result == false){
            std::cerr<<"This happened at location "<<x<<","<<y<<" for orientation interval index "<<o<<std::endl;
            return false;
          }
        }
      }
    }
  }

  return true;
}

int main(int argc, char** argv)
{

  if(argc!=4) {
    std::cerr<<"usage: "<<argv[0]<<" <pgm map> <length> <width>\n";
    exit(-1);
  }
  std::ifstream is(argv[1]);
  if (!is) {
    std::cerr << "Could not open map file for reading.\n";
    exit(-1);
  }


  //MemoryManager<Node>::reserve(1000000);

  {
    srand(0);
    SimpleMap<int> gridmap;
    gridmap.loadFromPGM(is);
    is.close();

    int sizeX = gridmap.getMapSizeX();
    int sizeY = gridmap.getMapSizeY();

    // turn map into a height map
    for (int x=0; x<sizeX; x++) {
      for (int y=0; y<sizeY; y++) {
        if (gridmap.getCell(x,y)>0) gridmap.setCell(x,y,0); // totally occupied
        else gridmap.setCell(x,y,INT_MAX);
      }
    }

    fprintf(stderr, "Map loaded (%dx%d).\n",  gridmap.getMapSizeX(), gridmap.getMapSizeY());


    //robot model
    int length = atoi(argv[2]);
    int width = atoi(argv[3]);
    std::vector<RobotColumn> columns;
    RobotColumn col;
    col.lower = 0;
    col.upper = 1;

    col.vertices.push_back(std::make_pair(-(double)length/2.0, -(double)width/2.0));
    col.vertices.push_back(std::make_pair( (double)length/2.0, -(double)width/2.0));
    col.vertices.push_back(std::make_pair( (double)length/2.0,  (double)width/2.0));
    col.vertices.push_back(std::make_pair(-(double)length/2.0,  (double)width/2.0));

//    col.x1 = -(double)length/2.0;
//    col.x2 = (double)length/2.0;
//    col.y1 = -(double)width/2.0;
//    col.y2 = (double)width/2.0;
    columns.push_back(col);

    SimpleMap<int> map_incremental;
    map_incremental.copyFrom(&gridmap);

    CSpaceVoronoi cspace_incremental(columns, &gridmap);
#ifdef TESTDSTAR
    cspace_incremental.initializeDStar();
#else
    cspace_incremental.initOriNodeMap();
    cspace_incremental.createOIGraph();
#endif


    bool allGood = true;
    unsigned int numFrames = 150;
    for(unsigned int idxFrame = 0; idxFrame < numFrames; idxFrame++){


      //generate some flips
      unsigned int numFlips = 2000;
      std::vector<int> flip_x, flip_y;
      std::vector<bool> flip_occupied;
      flip_x.reserve(numFlips);
      flip_y.reserve(numFlips);
      flip_occupied.reserve(numFlips);

      for(unsigned int i=0; i<numFlips; i++){
        int x = rand() % sizeX;
        int y = rand() % sizeY;
        bool occ = rand() % 2;
        flip_x.push_back(x);
        flip_y.push_back(y);
        flip_occupied.push_back(occ);
        //std::cerr<<flip_x.back()<<" "<<flip_y.back()<<" "<<flip_occupied.back()<<std::endl;
      }

      //apply the flips
      for(unsigned int i=0; i<flip_x.size(); i++){
        if(flip_occupied[i] == true){
          cspace_incremental.updateClearance(flip_x[i], flip_y[i], 0); //0 is occupied
          map_incremental.setCell(flip_x[i], flip_y[i], 0); //0 is occupied
        } else {
          cspace_incremental.updateClearance(flip_x[i], flip_y[i], INT_MAX); //INT_MAX is free
          map_incremental.setCell(flip_x[i], flip_y[i], INT_MAX); //INT_MAX is free
        }
      }

#ifdef TESTDSTAR
	IntPose start;
	start.x = 30;
	start.y = 30;
	start.theta = 0;
	IntPose goal;
	goal.x = 30;
	goal.y = 220;
	goal.theta = 0;
      cspace_incremental.incorporateUpdatesDStar(start,goal);
#else
      cspace_incremental.updateOrientationIntervalGraph();
#endif


      //compare with brute force computation
      CSpaceVoronoi cspace_brute(columns, &map_incremental);
#ifdef TESTDSTAR
      cspace_brute.initializeDStar();
#else
      cspace_brute.initOriNodeMap();
      cspace_brute.createOIGraph();
#endif



      std::cerr<<"Frame "<<idxFrame<<": doing comparison."<<std::endl;
      bool result = compareAdmissibleOrientations(&cspace_incremental, &cspace_brute);
#ifdef TESTDSTAR
      result = result && compareOIGraphs_dstar(&cspace_incremental, &cspace_brute);
#else
      result = result && compareOIGraphs(&cspace_incremental, &cspace_brute);
#endif
      if(result==true){
        std::cerr<<"ALL IS GOOD. CONGRATS. OR DID YOU NOT TEST ENOUGH?"<<std::endl;
      } else {
        std::cerr<<"BRACE. BRACE. WE HAVE AN ISSUE. (or not?)"<<std::endl;
      }

      allGood = allGood && result;
      if(allGood==false){
        break;
      }

      //undo the flips
      for(unsigned int i=0; i<flip_x.size(); i++){
        int original_value = gridmap.getCell(flip_x[i], flip_y[i]);
        cspace_incremental.updateClearance(flip_x[i], flip_y[i], original_value); //INT_MAX is free
        map_incremental.setCell(flip_x[i], flip_y[i], original_value); //INT_MAX is free
      }

//      for(int x=0; x<sizeX; x++){
//        for(int y=0; y<sizeY; y++){
//          if(gridmap.getCell(x,y) != map_incremental.getCell(x,y)){
//            std::cerr<<"Difference in maps at "<<x<<","<<y<<" "<<"gridmap: "<<gridmap.getCell(x,y)<<" incremenal: "<<map_incremental.getCell(x,y)<<std::endl;
//          }
//        }
//      }

      std::cerr<<"#used OISearchNode "<<MemoryManager<OISearchNode>::getNumObjectsInUse()<<std::endl;
      std::cerr<<"#used OIDStarSearchNode "<<MemoryManager<OIDStarSearchNode>::getNumObjectsInUse()<<std::endl;
    }

    if(allGood==false){
      std::cerr<<"** TEST ABORTED WITH ERROR **"<<std::endl;
    } else {
      std::cerr<<"-> Test successful."<<std::endl;
    }

  }

  std::cerr<<"#used OISearchNode "<<MemoryManager<OISearchNode>::getNumObjectsInUse()<<std::endl;
  std::cerr<<"#used OIDStarSearchNode "<<MemoryManager<OIDStarSearchNode>::getNumObjectsInUse()<<std::endl;

  MemoryManager<OIDStarSearchNode>::releaseFreeObjects();
  MemoryManager<OISearchNode>::releaseFreeObjects();
  MemoryManager<Node>::releaseFreeObjects();



}



