#include "stereoVision.h"

#define MAX_UNSIGNED 0xFFFFFFFF
#define MAX_SIGNED   0x7FFFFFFF

int CTBLKSIZE = 5;  // census transform block size.
int maxDisp   = 128; // maximum Disparity map.
int AREAWINDOW= 7;  // window size for area-based matching cost.
float LAMBDA  = 24.0; // Test KITTI Benchmark, this is the best result.
unsigned P1   = 200;
unsigned P2   = 800;

void parseCmd(int argc, char **argv, CFGPARA &cfgPara)
{
  cfgPara.leftImg  = argv[1];
  cfgPara.rightImg = argv[2];
  cfgPara.outDisparity = argv[3];
  cfgPara.lambda   = atof(argv[4]);
  cfgPara.sgmP1    = (unsigned)atoi(argv[5]);
  cfgPara.sgmP2    = (unsigned)atoi(argv[6]);
}

void inline getPadPelLoc(int &curX, int &curY, int width, int height)
{
  if (curX<0)
  {
    curX = 0;
  }
  else if (curX>=width)
  {
    curX = width-1;
  }

  if (curY<0)
  {
    curY = 0;
  }
  else if (curY>=height)
  {
    curY = height-1;
  }
}

bool inline isInsidePicture(int &curX, int &curY, int width, int height)
{
  if (curX<0 || curX>=width)
    return false;
  
  if (curY<0 || curY>=width)
    return false;

  return true;
}

// Census Transform
void censusTransform(Mat inImg, Mat &ctImg)
{
  int width = inImg.cols;
  int height= inImg.rows;

  ctImg.create(inImg.rows, inImg.cols, CV_32SC1);
    
  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      unsigned long census = 0;

      // Census Transform.
      int ctBlk = CTBLKSIZE/2;
      int centerPel = inImg.at<uchar>(i, j);

      for (int m=-ctBlk; m<ctBlk+1; m++)
      {
        for (int n=-ctBlk; n<ctBlk+1; n++)
        {
          if (m==0&&n==0) // center is not considered.
            continue;

          census = census<<1;

          int curX = j+n;
          int curY = i+m;

          getPadPelLoc(curX, curY, width, height);
          int curPel = inImg.at<uchar>(curY, curX);
          
          if (curPel<centerPel) census+=1;
        }
      }

      ctImg.at<int>(i,j) = (int)census;
    }
  }
}

unsigned hammingDistance(int x, int y)
{
  unsigned  dist = 0;
  unsigned  val = x^y;

  while (val != 0) {
    dist++;
    val &= val - 1;
  }

  return dist;
}

void getPredDisp(int curLocX, int curLocY, Mat bestDispMat, PREDDISP* predDisp)
{
  if (curLocX==0 && curLocY==0) // The first pixel. No predict disparity.
  {
    memset(predDisp, 0, 4*sizeof(PREDDISP));
    return ;
  }

  int neigX[4], neigY[4];
  int step[4][2] = {{-1, 0}, {-1, -1}, {0, -1}, {1, -1}}; // Neighboring 4 Pixels' step.
  int picWidth=bestDispMat.cols, picHeight=bestDispMat.rows;
  bool vld;

  for (int i=0; i<4; i++)
  {
    neigX[i] = curLocX+step[i][0];
    neigY[i] = curLocY+step[i][1];

    vld = isInsidePicture(neigX[i], neigY[i], picWidth, picHeight);

    if (vld)
    {
      predDisp[i].disp  = bestDispMat.at<float>(neigY[i], neigX[i]);
      predDisp[i].valid = true;
    }
    else
    {
      predDisp[i].disp  = 0.0f;
      predDisp[i].valid = false;
    }
  }

  return ;
}

void getBestPredDisp(const PREDDISP* neigPredDisp, int curX, int curY, const Mat leftCT, const Mat rightCT, PREDDISP& bestPredDisp)
{
  unsigned minCost = 0xFFFFFFFF;
  int width = leftCT.cols, height = leftCT.rows;
  int i = curY, j = curX;

  for (int predIdx=0; predIdx<4; predIdx++)
  {
    if (neigPredDisp[predIdx].valid==false) continue;

    int m = neigPredDisp[predIdx].disp;

    // using HD cost to decide the best disparity prediction. Like H265.
#if 1 // calc cost.
    {
      int  centerCur = j;
      int  centerRef = j-m;

      unsigned cost = 0;

      // area window-based matching cost. 
      for (int y=-AREAWINDOW/2; y<=AREAWINDOW/2; y++)
      {
        for (int x=-AREAWINDOW/2; x<=AREAWINDOW/2; x++)
        {
          int curX = centerCur+x;
          int curY = i+y;

          int refX = centerRef+x;
          int refY = i+y;

          getPadPelLoc(curX, curY, width, height);
          getPadPelLoc(refX, refY, width, height);

          int val = leftCT.at<int>(curY, curX);
          int refVal = rightCT.at<int>(refY, refX);

          cost += hammingDistance(val, refVal);
        }
      }

      if (cost<minCost)
      {
        minCost = cost;
        bestPredDisp.disp = m;
        bestPredDisp.valid = true;
      }
    }
#endif 
  }

  if (minCost==0xFFFFFFFF)
  {
    bestPredDisp.disp = 0.0f;
    bestPredDisp.valid = true;
  }
}

void fullPelSearchArea(Mat leftCT, Mat rightCT, Mat& outDisp)
{
  int width = leftCT.cols;
  int height= leftCT.rows;

  unsigned *costBuffer = NULL;
  costBuffer = (unsigned *) malloc(maxDisp*width*height*sizeof(unsigned));

  PREDDISP neigPredDisp[4];
  PREDDISP bestPredDisp;

  outDisp.create(height, width, CV_32FC1);

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      unsigned minCost = 0xFFFFFFFF;
      int  bestDisp = 0;

      // getPredDisp(j, i, outDisp, neigPredDisp);
      // getBestPredDisp(neigPredDisp, j, i, leftCT, rightCT, bestPredDisp);
      
      for (int m=0; m<maxDisp; m++)
      {
        int  centerCur = j;
        int  centerRef = j-m;

        unsigned cost = 0;
        unsigned predDisp = 0;

        // area window-based matching cost. 
        for (int y=-AREAWINDOW/2; y<=AREAWINDOW/2; y++)
        {
          for (int x=-AREAWINDOW/2; x<=AREAWINDOW/2; x++)
          {
            int curX = centerCur+x;
            int curY = i+y;

            int refX = centerRef+x;
            int refY = i+y;

            getPadPelLoc(curX, curY, width, height);
            getPadPelLoc(refX, refY, width, height);

            int val = leftCT.at<int>(curY, curX);
            int refVal = rightCT.at<int>(refY, refX);

            cost += hammingDistance(val, refVal); // HD cost.
          }
        }

        // add Predicted Disparity cost. (Using SGM Method; this MVD cost is not considered.)
        // predDisp = (unsigned)LAMBDA*abs(m-bestPredDisp.disp);
        // cost += predDisp;

        costBuffer[m+(i*width+j)*maxDisp] = cost;
        
        if (cost<minCost)
        {
          minCost = cost;
          bestDisp = m;
        }
      }

#if 0
      unsigned *Spd = costBuffer+(i*width+j)*maxDisp;

      bool isUniquenessRatio = false;
      for(int d = 0; d < maxDisp; d++ )
      {
        if( Spd[d]*(100 - 10) < minCost*100 && std::abs(bestDisp - d) > 1 )
        { 
          isUniquenessRatio = true;
          break;
        }
      }
      if (isUniquenessRatio==true)
      {
        bestDisp = 0;
      }

      float bestDispFullPel = (float)bestDisp;
      // do sub-pixel quadratic interpolation:
      // fit parabola into (x1=d-1, y1=Sp[d-1]), (x2=d, y2=Sp[d]), (x3=d+1, y3=Sp[d+1])
      // then find minimum of the parabola.
      int subPelScale16 = bestDisp;
      if( 0 < bestDisp && bestDisp < maxDisp-1 )
      {
        int denom2 = (int)std::max<unsigned>(Spd[bestDisp-1] + Spd[bestDisp+1] - 2*Spd[bestDisp], 1);
        subPelScale16 = (int)bestDisp*16 + (((int)Spd[bestDisp-1] - (int)Spd[bestDisp+1])*16 + denom2)/(denom2*2);
        if (Spd[bestDisp-1]==Spd[bestDisp+1])
        {
          subPelScale16 = (int)bestDisp*16;
        }
      }
      else
      {
        subPelScale16 *= 16;
      }

      float bestDispSubPel = (float)subPelScale16/16.0;

      if (abs(bestDispFullPel-bestDispSubPel)>1.0f)
      {
        int tt=0;
      }
#endif

      outDisp.at<float>(i,j)=(float)bestDisp; // assigned for disparity prediction.
    }
  }

  // return ;

  // Semi-Global Matching Refine.
  // above Lr Line buffer & above minLr Line buffer. 
  unsigned *LrAboveLineBuffer = NULL;
  LrAboveLineBuffer = (unsigned *)malloc(width*4*maxDisp*sizeof(unsigned)); // 4 Directions.
  unsigned *minLrAboveLineBuffer = NULL;
  minLrAboveLineBuffer = (unsigned *)malloc(width*4*sizeof(unsigned)); // 4 Directions.

  memset(LrAboveLineBuffer, 0, width*4*maxDisp*sizeof(unsigned)); 
  memset(minLrAboveLineBuffer, 0, width*4*sizeof(unsigned));

  // cur Lr Line buffer & cur minLr Line buffer.
  unsigned *LrCurLineBuffer = NULL;
  LrCurLineBuffer = (unsigned *)malloc(width*4*maxDisp*sizeof(unsigned)); // 4 Directions.
  unsigned *minLrCurLineBuffer = NULL;
  minLrCurLineBuffer = (unsigned *)malloc(width*4*sizeof(unsigned)); // 4 Directions.

  // right LR Pixel buffer & right minLr Line buffer.
  unsigned *LrRightPelBuffer = NULL; // (1, 0) Direction. 
  LrRightPelBuffer = (unsigned *)malloc(maxDisp*sizeof(unsigned));
  unsigned minLrRightPelBuffer;

  unsigned *LrCurDir4PelBuffer = NULL; // (1, 0) Direction. 
  LrCurDir4PelBuffer = (unsigned *)malloc(maxDisp*sizeof(unsigned));

  // total sgm cost; Formula 14 in HH-TPAMI.
  unsigned *sgmCostLineBuffer = NULL;
  sgmCostLineBuffer = (unsigned *)malloc(width*maxDisp*sizeof(unsigned));

  // zero cost just for boundary.
  unsigned *zeroLrCost;
  zeroLrCost = (unsigned *)malloc(maxDisp*sizeof(unsigned));
  memset(zeroLrCost, 0, maxDisp*sizeof(unsigned));
  unsigned zerominLrCost = 0;

  unsigned *LrDir0=NULL, *LrDir1=NULL, *LrDir2=NULL, *LrDir3=NULL;
  unsigned  minLrDir0,    minLrDir1,    minLrDir2,    minLrDir3;
  unsigned *Cpd, *Spd, *CurLr, *minCurLr;

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++) // 4 Directions: (-1,0):0; (-1,-1):1; (0,-1):2; (1,-1):3;
    {
      if (j-1>=0)            { LrDir0 = LrCurLineBuffer+(j-1)*4*maxDisp;              minLrDir0 = minLrCurLineBuffer[(j-1)*4];     } // Dir 0.
      else                   { LrDir0 = zeroLrCost;                                   minLrDir0 = zerominLrCost;                   } 
      if (i-1>=0&&j-1>=0)    { LrDir1 = LrAboveLineBuffer+(j-1)*4*maxDisp+maxDisp;    minLrDir1 = minLrAboveLineBuffer[(j-1)*4+1]; } // Dir 1.
      else                   { LrDir1 = zeroLrCost;                                   minLrDir1 = zerominLrCost;                   }
      if (i-1>=0)            { LrDir2 = LrAboveLineBuffer+j*4*maxDisp+2*maxDisp;      minLrDir2 = minLrAboveLineBuffer[j*4+2];     } // Dir 2.
      else                   { LrDir2 = zeroLrCost;                                   minLrDir2 = zerominLrCost;                   }
      if (i-1>=0&&j+1<width) { LrDir3 = LrAboveLineBuffer+(j+1)*4*maxDisp+3*maxDisp;  minLrDir3 = minLrAboveLineBuffer[(j+1)*4+3]; } // Dir 3.
      else                   { LrDir3 = zeroLrCost;                                   minLrDir3 = zerominLrCost;                   }

      Cpd = costBuffer+(i*width+j)*maxDisp;
      Spd = sgmCostLineBuffer+j*maxDisp;

      CurLr = LrCurLineBuffer+j*4*maxDisp;
      minCurLr = minLrCurLineBuffer+j*4;

      unsigned minDir0=MAX_UNSIGNED, minDir1=MAX_UNSIGNED, minDir2=MAX_UNSIGNED, minDir3=MAX_UNSIGNED;

      for (int disp=0; disp<maxDisp; disp++)
      {
        // Formula 13 in HH-TPAMI.
        unsigned dir0 = Cpd[disp] + min(LrDir0[disp], min((disp-1<0?MAX_SIGNED:LrDir0[disp-1])+P1, min((disp+1>=maxDisp?MAX_SIGNED:LrDir0[disp+1])+P1, minLrDir0+P2)))-minLrDir0;
        unsigned dir1 = Cpd[disp] + min(LrDir1[disp], min((disp-1<0?MAX_SIGNED:LrDir1[disp-1])+P1, min((disp+1>=maxDisp?MAX_SIGNED:LrDir1[disp+1])+P1, minLrDir1+P2)))-minLrDir1;
        unsigned dir2 = Cpd[disp] + min(LrDir2[disp], min((disp-1<0?MAX_SIGNED:LrDir2[disp-1])+P1, min((disp+1>=maxDisp?MAX_SIGNED:LrDir2[disp+1])+P1, minLrDir2+P2)))-minLrDir2;
        unsigned dir3 = Cpd[disp] + min(LrDir3[disp], min((disp-1<0?MAX_SIGNED:LrDir3[disp-1])+P1, min((disp+1>=maxDisp?MAX_SIGNED:LrDir3[disp+1])+P1, minLrDir3+P2)))-minLrDir3;
        
        Spd[disp] = dir0+dir1+dir2+dir3;

        CurLr[disp]           = dir0;
        CurLr[maxDisp+disp]   = dir1;
        CurLr[2*maxDisp+disp] = dir2;
        CurLr[3*maxDisp+disp] = dir3;

        minDir0 = min(minDir0, dir0);
        minDir1 = min(minDir1, dir1);
        minDir2 = min(minDir2, dir2);
        minDir3 = min(minDir3, dir3);
      }

      minCurLr[0] = minDir0;
      minCurLr[1] = minDir1;
      minCurLr[2] = minDir2;
      minCurLr[3] = minDir3;
    }

    unsigned *LrDir4;
    unsigned minLrDir4;
    for (int j=width-1; j>=0; j--) // (1, 0) Direction. Right Direction.
    {
      LrDir4    = j+1<width?LrRightPelBuffer:zeroLrCost;
      minLrDir4 = j+1<width?minLrRightPelBuffer:zerominLrCost;
      
      Cpd = costBuffer+(i*width+j)*maxDisp;
      Spd = sgmCostLineBuffer+j*maxDisp;

      unsigned minDir4 = MAX_UNSIGNED;
      unsigned minSpd  = MAX_UNSIGNED;
      int      bestDisp;
      for (int disp=0; disp<maxDisp; disp++)
      {
        unsigned dir4 = Cpd[disp] + min(LrDir4[disp], min((disp-1<0?MAX_SIGNED:LrDir4[disp-1])+P1, min((disp+1>=maxDisp?MAX_SIGNED:LrDir4[disp+1])+P1, minLrDir4+P2)))-minLrDir4;

        LrCurDir4PelBuffer[disp] = dir4;
        minDir4 = min(minDir4, dir4);

        Spd[disp] += dir4; 

        if (Spd[disp]<minSpd)
        {
          minSpd   = Spd[disp];
          bestDisp = disp;
        } 
      }

      bool isUniquenessRatio = false;
      for(int d = 0; d < maxDisp; d++ )
      {
        if( Spd[d]*(100 - 10) < minSpd*100 && std::abs(bestDisp - d) > 1 )
        { 
          isUniquenessRatio = true;
          break;
        }
      }
      if (isUniquenessRatio==true)
      {
        bestDisp = 0;
      }

      float bestDispFullPel = (float)bestDisp;
      // do sub-pixel quadratic interpolation:
      // fit parabola into (x1=d-1, y1=Sp[d-1]), (x2=d, y2=Sp[d]), (x3=d+1, y3=Sp[d+1])
      // then find minimum of the parabola.
      int subPelScale16 = bestDisp;
      if( 0 < bestDisp && bestDisp < maxDisp-1 )
      {
        int denom2 = (int)std::max<unsigned>(Spd[bestDisp-1] + Spd[bestDisp+1] - 2*Spd[bestDisp], 1);
        subPelScale16 = (int)bestDisp*16 + (((int)Spd[bestDisp-1] - (int)Spd[bestDisp+1])*16 + denom2)/(denom2*2);
        if (Spd[bestDisp-1]==Spd[bestDisp+1])
        {
          subPelScale16 = (int)bestDisp*16;
        }
      }
      else
      {
        subPelScale16 *= 16;
      }

      float bestDispSubPel = (float)subPelScale16/16.0;

      if (abs(bestDispFullPel-bestDispSubPel)>1.0f)
      {
        int tt=0;
      }

      // get the best disparity.
      outDisp.at<float>(i,j)=(float)(bestDispSubPel);

      // SWAP, for next pixel.
      minLrRightPelBuffer = minDir4;
      memcpy(LrRightPelBuffer, LrCurDir4PelBuffer, maxDisp*sizeof(unsigned));
    }

    // SWAP, for next line buffer.
    memcpy(LrAboveLineBuffer,    LrCurLineBuffer,    width*4*maxDisp*sizeof(unsigned));
    memcpy(minLrAboveLineBuffer, minLrCurLineBuffer, width*4*sizeof(unsigned));
  }

  free(costBuffer);
  free(LrAboveLineBuffer);
  free(minLrAboveLineBuffer);
  free(LrCurLineBuffer);
  free(minLrCurLineBuffer);
  free(LrRightPelBuffer);
  free(sgmCostLineBuffer);
}

// This is a simple method.
// Each pixel searching a best idx using HD cost.
void fullPelSearch(Mat leftCT, Mat rightCT, Mat& outDisp)
{
  int width = leftCT.cols;
  int height= leftCT.rows;
  
  outDisp.create(height, width, CV_32FC1);

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      // target sample value
      int value = leftCT.at<int>(i,j);
      unsigned minCost = 0xFFFFFFFF;
      int bestDisp = 0;

      // winner takes all.
      for (int m=0; m<maxDisp; m++)
      {
        int refLocX = j-m;
        getPadPelLoc(refLocX, i, width, height);

        int refValue = rightCT.at<int>(i, refLocX);
        unsigned cost = hammingDistance(value, refValue);

        if (cost<minCost)
        {
          bestDisp = m;
          minCost = cost;
        }
      }
      outDisp.at<float>(i,j)=(float)bestDisp;
    }
  }
}

void dispMapGen(Mat disp, string rawFileName, string colorFileName)
{
  int width = disp.cols;
  int height= disp.rows;

  png::image<png::gray_pixel_16> image(width, height);

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      float mvx = disp.at<float>(i,j);
      png::gray_pixel_16 val;
      // val = mvx;
      val = (uint16_t)(mvx*256.0f);
      image.set_pixel(j, i, val);
    }
  }

  image.write(rawFileName);
  
  DisparityImage dispMap;
  dispMap.readDisparityMap(image);

  // compute maximum disparity
  float max_disp = dispMap.maxDisp();

  // save original flow image
  dispMap.writeColor(colorFileName, max_disp);
}

void dispMapGenScale256(Mat disp, string colorFileName)
{
  int width = disp.cols;
  int height= disp.rows;

  png::image<png::gray_pixel_16> image(width, height);

  for (int i=0; i<height; i++)
  {
    for (int j=0; j<width; j++)
    {
      unsigned short mvx = disp.at<unsigned short>(i,j);
      png::gray_pixel_16 val;
      // val = mvx;
      val = mvx;
      image.set_pixel(j, i, val);
    }
  }

  DisparityImage dispMap;
  dispMap.readDisparityMap(image);

  // compute maximum disparity
  float max_disp = dispMap.maxDisp();

  // save original flow image
  dispMap.writeColor(colorFileName, max_disp);
}

void dumpDisp(Mat inDisp, string outFileName)
{
  const char* name = outFileName.data();
  FILE *fp = fopen(name, "wb");
  float val;

  for (int i=0; i<inDisp.rows; i++)
  {
    for (int j=0; j<inDisp.cols; j++)
    {
      val = inDisp.at<float>(i,j);
      fwrite(&val, sizeof(float), 1, fp);
    }
  }
  fclose(fp);
}

void pixelBasedStereoMatching(STEREO &stereoData)
{
  // census transform pre-processing.
  censusTransform(stereoData.leftImg,  stereoData.leftCTImg); 
  censusTransform(stereoData.rightImg, stereoData.rightCTImg); 

  fullPelSearchArea(stereoData.leftCTImg, stereoData.rightCTImg, stereoData.fullPelDis);

  // medianBlur(stereoData.fullPelDis, stereoData.fullPelDis, 3);

  dumpDisp(stereoData.fullPelDis, "mv1.bin");

  dispMapGen(stereoData.fullPelDis, "1.png", "2.png");
}

void openCVBM(STEREO stereoData)
{
  StereoBM bm;

  cv::setUseOptimized(false);
  // bm.state->roi1 = roi1;
  // bm.state->roi2 = roi2;
  bm.state->preFilterCap = 31;
  bm.state->SADWindowSize = 5;
  bm.state->minDisparity = 0;
  bm.state->numberOfDisparities = 64;
  bm.state->textureThreshold = 10;
  bm.state->uniquenessRatio = 15;
  bm.state->speckleWindowSize = 100;
  bm.state->speckleRange = 32;
  bm.state->disp12MaxDiff = 1;

  Mat disp;
  bm(stereoData.leftImg, stereoData.rightImg, disp);
  
  Mat t0;
  disp.convertTo(t0, CV_16U, 16.0);
  imwrite("1.png", t0);
}

void openCVSGBM(STEREO stereoData)
{
  StereoSGBM sgbm;

  cv::setUseOptimized(false);

  sgbm.preFilterCap = 63;
  
  int SADWindowSize = 7;
  sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

  int cn = stereoData.leftImg.channels();

  sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
  sgbm.minDisparity = 0;
  sgbm.numberOfDisparities = 128;
  sgbm.uniquenessRatio = 0;
  sgbm.speckleWindowSize = 100;
  sgbm.speckleRange = 32;
  sgbm.disp12MaxDiff = 1;
  sgbm.fullDP = false;

  Mat disp;
  sgbm(stereoData.leftImg, stereoData.rightImg, disp);

  Mat t0;
  disp.convertTo(t0, CV_16U, 16.0);
  imwrite("1.png", t0);
}

// Not Good! Omit Currently!
void openCVVar(STEREO stereoData)
{
  StereoVar var;

  var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
  var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
  var.nIt = 25;
  var.minDisp = -64;
  var.maxDisp = 0;
  var.poly_n = 3;
  var.poly_sigma = 0.0;
  var.fi = 15.0f;
  var.lambda = 0.03f;
  var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
  var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
  var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

  Mat disp;
  var(stereoData.leftImg, stereoData.rightImg, disp);

  Mat t0;
  disp.convertTo(t0, CV_16U, 256.0);
  imwrite("1.png", t0);
}

int main(int argc, char **argv)
{
  CFGPARA cfgPara;
  STEREO  stereoData;

  // Parse Command Line.
  parseCmd(argc, argv, cfgPara);

  // assign lambda
  LAMBDA = cfgPara.lambda;
  P1 = cfgPara.sgmP1;
  P2 = cfgPara.sgmP2;

  stereoData.leftImg  = imread(cfgPara.leftImg, CV_LOAD_IMAGE_GRAYSCALE);
  stereoData.rightImg = imread(cfgPara.rightImg,CV_LOAD_IMAGE_GRAYSCALE);

  // pixelBasedStereoMatching(stereoData);

  // openCVBM(stereoData); 

  openCVSGBM(stereoData);

  return 0;
}