#ifndef _HELPER_H_
#define _HELPER_H_

#include <sys/time.h>

class Helper {
public:
  static double getTime() {
      struct timeval tv;
      double t;

      if (gettimeofday(&tv, NULL) < 0) printf("COULD NOT MEASURE TIME!\n");
      t = tv.tv_sec + tv.tv_usec/1000000.0;
      return t;
    }

  };


#endif
