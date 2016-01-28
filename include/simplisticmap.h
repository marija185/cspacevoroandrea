#ifndef SIMPLISTICMAP_H_
#define SIMPLISTICMAP_H_

#include <stdio.h>
#include <iostream>
#include <assert.h>
#include <string.h>

template <class T>
class SimplisticMap {
public:
  SimplisticMap() {
    sizeX = -1;
    sizeY = -1;
    data = NULL;
  }
  SimplisticMap(int w, int h) {
    data = NULL;
    sizeX = -1;
    sizeY = -1;
    resize(w,h);
  }
  ~SimplisticMap() {
    if (data) {
/*
      for (int x=0; x<sizeX; x++) {
        delete[] data[x];
      }
*/
      delete[] data;
    }
  }
  int getMapSizeX() { return sizeX; }
  int getMapSizeY() { return sizeY; }

  
//  inline T getCell(int x, int y) { return data[x][y]; }
  inline T getCell(int x, int y) { return data[x+y*sizeX]; }
//  inline T& getCellReference(int x, int y) { return data[x][y]; }
  inline T& getCellReference(int x, int y) { return data[x+y*sizeX]; }

  void resize(int width, int height) {
    if (data && (width!=sizeX || height!=sizeY)) {
  /*
      for (int x=0; x<sizeX; x++) {
        delete[] data[x];
      }
  */
      delete[] data;
      data = NULL;
    }

    if (!data) {
//      data = new T*[width];
      data = new T[width*height];
      sizeX = width;
      sizeY = height;
      /*
      for (int x=0; x<sizeX; x++) {
        data[x] = new T[height];
      }
      */
    }
  }

private:
  //T **data;
  T *data;

private:
  int sizeX;
  int sizeY;
};


#endif
