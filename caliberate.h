/************************************************************************/
/* Common includes for main.cpp and caliberate.cpp                      */
/************************************************************************/
#ifndef CALIBERATE_H_
#define CALIBERATE_H_
#include  <iostream>
#include  <cmath>
using namespace std;
#include  <cv.h>
#include  <cxcore.h>
#include  <highgui.h>
#define NCHANNELS 3
#define WIDTH_SMALL 320
#define HEIGHT_SMALL 240
#define WIDTH_STEP_SMALL WIDTH_SMALL*NCHANNELS
#define MAX_CORNERS 2500
#define RED   cvScalar(0,0,255)
#define GREEN cvScalar(0,255,0)
#define BLUE  cvScalar(255,0,0)
#define MAXCARS	50
// common variables used in both cpp files
extern CvCapture* capture;
extern IplImage * frameImg;
extern IplImage * polygonImg;
extern uchar* frameData;
extern CvPoint pts[4];
extern double polyArea;
#endif

IplImage* findRoadImage(void);
void calibPolygon(void);
void calibIntensity(void);
