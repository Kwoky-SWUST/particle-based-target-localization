#ifndef CROCO__UTILS__GRAPHICS__NETPBM_H
#define CROCO__UTILS__GRAPHICS__NETPBM_H

#include <string>

/** @author Philipp Vorst
 *  @see http://en.wikipedia.org/wiki/Netpbm_format
 */

/** Create a graymap file
 *  @param  data An array of gray values coded as floating point numbers between 0.0 (black) and 1.0 (white). Its indices (x,y)
 *               are given by [y*w + x]
 *  @return 0 on success, -1 on error
 */
int createPGM( std::string fileName, double* data, int w, int h );

/** Create a graymap file
 *  @param  data An array of gray values coded as floating point numbers between 0.0 (black) and 1.0 (white). Its indices (x,y)
 *               are given by [y*w + x]
 *  @return 0 on success, -1 on error
 */
int createPGM( std::string fileName, float* data, int w, int h );


#endif // CROCO__UTILS__GRAPHICS__NETPBM_H
