#include <string>
#include <iostream>
#include <stdio.h>
#include "opencv2\\imgproc\\imgproc.hpp"
#include "opencv2\\highgui\\highgui.hpp"
#include "opencv2\\calib3d\\calib3d.hpp"
#include "opencv2\\opencv.hpp"
#include "png.h"
#include "pngconf.h"
#include "pngdebug.h"
#include "pngstruct.h"
#include "pnginfo.h"
#include "pnglibconf.h"
#include "pngpriv.h"
#include "pngstruct.h"
#include "png.hpp"
#include "io_disp.h"

using std::string;
using namespace cv;
using namespace std;

extern int CTBLKSIZE;  // census transform block size.
extern int maxDisp  ;  // maximum Disparity map.
extern int AREAWINDOW; // window size for area-based matching cost.

typedef struct cfgPara{
  string leftImg;
  string rightImg;
  string outDisparity;
  float  lambda;

  unsigned sgmP1;
  unsigned sgmP2;
}CFGPARA;

typedef struct stereo{
  Mat leftImg;
  Mat rightImg;

  Mat leftCTImg;
  Mat rightCTImg;

  Mat fullPelDis;
}STEREO;

typedef struct predDisp{
  float disp;
  bool  valid;
}PREDDISP;