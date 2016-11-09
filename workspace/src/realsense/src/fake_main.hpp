#ifndef _MAIN_HEADER_ 
#define _MAIN_HEADER_ 

#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h"
#include "opencv2/highgui/highgui_c.h"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "myImage.hpp"
#include "roi.hpp"
#include "handGesture.hpp"
#include <vector>
#include <cmath>

using namespace cv;
using namespace std;


#define ORIGCOL2COL CV_BGR2HLS
#define COL2ORIGCOL CV_HLS2BGR
#define NSAMPLES 7
#define PI 3.14159



void init(MyImage *m);
void col2origCol(int hsv[3], int bgr[3], Mat src);
void printText(Mat src, string text);
void waitForPalmCover(MyImage* m, int& count);
int getMedian(vector<int> val);
void getAvgColor(MyImage *m,My_ROI roi,int avg[3]);
void average(MyImage *m, int& count);
void initTrackbars();
void normalizeColors(MyImage * myImage);
void produceBinaries(MyImage *m);
void initWindows(MyImage m);
void showWindows(MyImage m);
int findBiggestContour(vector<vector<Point> > contours);
void init_saved_bounds();

//void myDrawContours(MyImage *m,HandGesture *hg);
//void makeContours(MyImage *m, HandGesture* hg);

#endif
