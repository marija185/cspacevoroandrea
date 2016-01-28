#include "dynamicvoronoi.h"
#include "simplemap.h"
#include <string>
#include <iostream>
#include <fstream>

int main( int argc, char *argv[] ) {
  if (argc != 2) {
    fprintf(stdout, "Usage: %s <map-file>\n", argv[0]);
    exit(1);
  }

  std::string input_map(argv[1]);

  if(input_map.find_last_of(".") != std::string::npos){
    std::cerr<<"converting input map to pgm"<<std::endl;

    std::string input_map_basename = input_map.substr(0, input_map.find_last_of("."));
    std::string input_map_pgm = input_map_basename;
    input_map_pgm.append(".pgm");

    std::string cmd_convert_input = "convert ";
    cmd_convert_input.append(input_map);
    cmd_convert_input.append(" ");
    cmd_convert_input.append(input_map_pgm);

    system(cmd_convert_input.c_str());

    SimpleMap<int> map;
    std::ifstream is(input_map_pgm.c_str());
    if (!is) {
       std::cerr << "Could not open map file for reading.\n";
       exit(-1);
     } else {
       map.loadFromPGMPure(is);
       is.close();
       fprintf(stderr, "Map loaded (%dx%d).\n",  map.getMapSizeX(), map.getMapSizeY());
     }

     DynamicVoronoi voronoi;

     voronoi.initializeEmpty(map.getMapSizeX(), map.getMapSizeY(), true);
     for(unsigned int x=0; x<voronoi.getSizeX(); x++){
       for(unsigned int y=0; y<voronoi.getSizeY(); y++){

         float occupancy = (float) (255 - map.getCell(x,y))/ 255.0;

         if(occupancy > 0.15)
           voronoi.setObstacle(x,y);
       }
     }


     voronoi.update(true);

     //now extract the EDT
     SimpleMap<float> distmap;
     distmap.resize(voronoi.getSizeX(), voronoi.getSizeY());
     for(unsigned int x=0; x<voronoi.getSizeX(); x++){
       for(unsigned int y=0; y<voronoi.getSizeY(); y++){
         distmap.setCell(x,y, voronoi.getDistance(x,y));
       }
     }

     //now save the EDT
     std::string distmap_name = input_map_basename;
     distmap_name.append("_EDT_normalized");

     std::string distmap_name_pgm = distmap_name;
     distmap_name_pgm.append(".pgm");
     distmap.writeToPGM(distmap_name_pgm.c_str(), true);

     std::cerr<<"Saved EDT to "<<distmap_name_pgm<<std::endl;

     std::cerr<<"converting to png "<<std::endl;
     std::string distmap_name_png = distmap_name;
     distmap_name_png.append(".png");
     std::string cmd_convert_output = "convert ";
     cmd_convert_output.append(distmap_name_pgm);
     cmd_convert_output.append(" ");
     cmd_convert_output.append(distmap_name_png);

     system(cmd_convert_output.c_str());

  } else {
    std::cerr<<"input map file seems not to have a file extension"<<std::endl;
    exit(0);
  }






   return 0;
}
