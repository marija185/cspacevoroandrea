/*
 * OINodeConnector.h
 *
 *  Created on: Dec 14, 2012
 *      Author: sprunkc
 */

#ifndef OINODECONNECTOR_H_
#define OINODECONNECTOR_H_

#include "OrientationIntervals.h"
#include "MemoryManager.h"
#include <vector>

template <class OISearchNodeType>
class OINodeConnector {
public:
  OINodeConnector();
  virtual ~OINodeConnector();

  void initFromOrientationIntervals(const OrientationIntervals &ois);

  std::vector<OrientationInterval> orientationIntervals;
  std::vector<OISearchNodeType*> nodes;
  bool dirty;

};

template <class OISearchNodeType>
OINodeConnector<OISearchNodeType>::OINodeConnector()
:dirty(false){
}

template <class OISearchNodeType>
OINodeConnector<OISearchNodeType>::~OINodeConnector() {
}

template <class OISearchNodeType>
void OINodeConnector<OISearchNodeType>::initFromOrientationIntervals(const OrientationIntervals & ois)
{
  orientationIntervals = ois.getOrientationIntervals();
  for(unsigned int i=0; i<nodes.size(); ++i){
    if(nodes[i]!=NULL){
      //delete nodes[i];
      MemoryManager<OISearchNodeType>::destroy(nodes[i]);
    }
  }
  nodes.assign(orientationIntervals.size(), NULL);
}

#endif /* OINODECONNECTOR_H_ */
