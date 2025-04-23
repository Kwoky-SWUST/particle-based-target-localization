#include "netpbm.h"
//
#include <iostream>
#include <fstream>

using namespace std;


// --------------------------------------------------------------------------
int createPGM( std::string fileName, double* data, int w, int h )
// --------------------------------------------------------------------------
{
  ofstream out( fileName.c_str(), ios::out );
  if ( !out )
    return -1;

  out << "P5\n#Binary PGM image\n" << flush;  // P5 = binary-coded portable graymap
  out << w << ' ' << h << "\n255\n" << flush; // w, h, max colors (255)
  char c;
  for ( int y = 0; y < h; ++y )
  {
    for ( int x = 0; x < w; ++x )
    {
      c = (char) ( data[ y*w + x ] * 255 );
      out << c;
    }
  }
  out.close();

  return 0;
}


// --------------------------------------------------------------------------
int createPGM( std::string fileName, float* data, int w, int h )
// --------------------------------------------------------------------------
{
  ofstream out( fileName.c_str(), ios::out );
  if ( !out )
    return -1;

  out << "P5\n#Binary PGM image\n" << flush;  // P5 = binary-coded portable graymap
  out << w << ' ' << h << "\n255\n" << flush; // w, h, max colors (255)
  char c;
  for ( int y = 0; y < h; ++y )
  {
    for ( int x = 0; x < w; ++x )
    {
      c = (char) ( data[ y*w + x ] * 255 );
      out << c;
    }
  }
  out.close();

  return 0;
}
