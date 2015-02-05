/*******************************************************************************
* Pedestrian Detector v0.1    2013-08
*
* Fabio Reis
* [freis-at-isr.ist.utl.pt]
*
* Please email me if you find bugs, or have suggestions or questions!
* Licensed under the Simplified BSD License [see external/bsd.txt]
*******************************************************************************/

/*
 * This is just an header file for the wrappers.cpp created by Piotr Dollar.
 * Take attention when mergin with other versions of the Piotr's toolbox, as 
 * stated in the README.
 */
#ifndef _WRAPPERS_HPP_
#define _WRAPPERS_HPP_

#include <cstdlib>

// wrapper functions if compiling from C/C++
void wrError(const char *errormsg);
void* wrCalloc( size_t num, size_t size );
void* wrMalloc( size_t size );
void wrFree( void * ptr );

// platform independent aligned memory allocation (see also alFree)
void* alMalloc( size_t size, int alignment );

// platform independent alignned memory de-allocation (see also alMalloc)
void alFree(void* aligned);

#endif
