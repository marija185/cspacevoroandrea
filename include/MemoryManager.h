#ifndef _MEMORYMANAGER_H_
#define _MEMORYMANAGER_H_

class MemoryManagerObject {
public:
  virtual void destroy()=0;
  virtual void recycle()=0;
};

#define USERECYCLING

template <class T>
class MemoryManager {
public:

  static void reserve(int count) {
    freeObjects.reserve(count);
    for (int i=0; i<count; i++) {
        T* t = new T();
        freeObjects.push_back(t);
    }
  }

  static T* getNew() {
#ifdef USERECYCLING
    if (freeObjects.size()==0) {
      for (unsigned int i=0; i<objectsInUse+1; i++) {
        T* t = new T();
        freeObjects.push_back(t);
      }
    }
    T* t = freeObjects.back();
    t->recycle();

    freeObjects.pop_back();
    objectsInUse++;
    return t;
#else
    T* t = new T();
    t->recycle();
    return t;
#endif
  }
  static void destroy(T* t) {
    objectsInUse--;
#ifdef USERECYCLING
    freeObjects.push_back(t);
    assert(objectsInUse>=0);
#else 
    delete t;
#endif
    
  }

  static void releaseFreeObjects(){
  	for(unsigned int i=0; i<freeObjects.size(); i++){
  		delete freeObjects.at(i);
  	}
  	freeObjects.clear();
  }

  static unsigned int getNumObjectsInUse(){
    return objectsInUse;
  }

private:
  static std::vector<T*> freeObjects;
  static unsigned int objectsInUse;

};

#endif
