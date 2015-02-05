/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/
#include "../include/wrappers.hpp"

/*
 * Insert Piotr Dollar's wrappers.hpp here. But please remove the part
 * for MatLab and the "inline" part of each declaration
 */
/*******************************************************************************
* Piotr's Image&Video Toolbox      Version 3.00
* Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

// wrapper functions if compiling from C/C++
void wrError(const char *errormsg) { throw errormsg; }
void* wrCalloc( size_t num, size_t size ) { return calloc(num,size); }
void* wrMalloc( size_t size ) { return malloc(size); }
void wrFree( void * ptr ) { free(ptr); }

// platform independent aligned memory allocation (see also alFree)
void* alMalloc( size_t size, int alignment ) {
  const size_t pSize = sizeof(void*), a = alignment-1;
  void *raw = wrMalloc(size + a + pSize);
  void *aligned = (void*) (((size_t) raw + pSize + a) & ~a);
  *(void**) ((size_t) aligned-pSize) = raw;
  return aligned;
}

// platform independent alignned memory de-allocation (see also alMalloc)
void alFree(void* aligned) {
  void* raw = *(void**)((char*)aligned-sizeof(void*));
  wrFree(raw);
}
