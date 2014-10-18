#include"caliberate.h"
IplImage* avgImage;	//Road Image
uchar* avgImageData; //data of road image
float table[WIDTH_SMALL][HEIGHT_SMALL][NCHANNELS];
bool isFixed[WIDTH_SMALL][HEIGHT_SMALL];
IplImage * polygonImg;
int polyPts[4][2]; //four points of polygon
CvPoint pts[4]; //four points of polygon
int counter=0 ;
double polyArea = 0;
bool firstTime = true;
void my_mouse_callback(int event, int x, int y, int flags, void* param )
{
	if(counter==4)	counter=0;
	switch( event ) 
	{
		case CV_EVENT_LBUTTONDOWN: 
		{
			polyPts[counter][0] = x ; polyPts[counter][1] = y;
			cout<<polyPts[counter][0]<<"  "<<polyPts[counter][1]<<endl;
			if(!firstTime)
			{
				int prevCounter = (counter==0?3:counter-1);
				cvLine(avgImage, cvPoint(polyPts[prevCounter][0], polyPts[prevCounter][1]), cvPoint(polyPts[counter][0], polyPts[counter][1]), CV_RGB(255,0,0),1,CV_AA);
				cvShowImage("photo_road", avgImage);
			}
			firstTime = false;
		}
		break;
		case CV_EVENT_LBUTTONUP: 
		{
			++counter;
		}
		break; 
	}
}
void calibPolygon(void)
{
	//while caliberating polygon, click on four points to select polygon.
	//If any pixel is chosen wrong keep clicking circularly clockwise to update polygon points
	polygonImg	 = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL),IPL_DEPTH_8U,1);	cvZero(polygonImg);	//blackout area out of polygon
	cvShowImage("photo_road", avgImage);
	cvSetMouseCallback("photo_road",my_mouse_callback,(void*) 0);
	cvWaitKey(0);

	pts[0] = cvPoint( polyPts[0][0], polyPts[0][1] );
	pts[1] = cvPoint( polyPts[1][0], polyPts[1][1]);
	pts[2] = cvPoint( polyPts[2][0], polyPts[2][1]  );
	pts[3] = cvPoint( polyPts[3][0], polyPts[3][1]);
	cvFillConvexPoly( polygonImg, pts, 4, cvScalar( 255, 255, 255 ), 4);
	float s,a,b,c,d,e;
	a = (double)pow((double)(pts[0].x-pts[1].x)*(pts[0].x-pts[1].x)+(pts[0].y-pts[1].y)*(pts[0].y-pts[1].y),.5);
	b = (double)pow((double)(pts[1].x-pts[2].x)*(pts[1].x-pts[2].x)+(pts[1].y-pts[2].y)*(pts[1].y-pts[2].y),.5);
	c = (double)pow((double)(pts[0].x-pts[2].x)*(pts[0].x-pts[2].x)+(pts[0].y-pts[2].y)*(pts[0].y-pts[2].y),.5);
	d = (double)pow((double)(pts[2].x-pts[3].x)*(pts[2].x-pts[3].x)+(pts[2].y-pts[3].y)*(pts[2].y-pts[3].y),.5);
	e = (double)pow((double)(pts[3].x-pts[0].x)*(pts[3].x-pts[0].x)+(pts[3].y-pts[0].y)*(pts[3].y-pts[0].y),.5);
	s = (a+b+c)/2;
	polyArea+= fabs(sqrt(s*(s-a)*(s-b)*(s-c)));
	s = (c+d+e)/2;
	polyArea+= fabs((double)pow((double)s*(s-c)*(s-d)*(s-e),.5));
	cvDestroyWindow("photo_road");
}

void calibIntensity(void)
{
	//Find average Image Intensity (just for information)
	float intensity = 0;
	int temp;
	for(int i=0; i<HEIGHT_SMALL; i++)
	{
		for(int j=0; j<WIDTH_SMALL; j++)
		{
			temp = i*WIDTH_STEP_SMALL+j*NCHANNELS;
			intensity += ( frameData[temp] + frameData[temp+1] + frameData[temp+2] )/3;
		}
	}
	intensity/= WIDTH_SMALL * HEIGHT_SMALL;
	cout<<"Intensity = "<<intensity<<endl;
}

IplImage* findRoadImage(void)
{	
	avgImage = cvCreateImage( cvSize(WIDTH_SMALL, HEIGHT_SMALL), 8, 3);	//averaged over 100 gray image frames to get gray photo of road only
	avgImageData = (uchar*)avgImage->imageData;	cvZero(avgImage);
	IplImage* img1_origSize;
	IplImage* img1 = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL),8,3);	cvZero(img1);
	IplImage* img2 = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL),8,3);	//previous frame of img1
	IplImage* img3 = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL),8,3);	//previous frame of img2
	IplImage* img4 = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL),8,3);	//previous frame of img3
	for(int i=0; i<HEIGHT_SMALL; ++i)
	{
		for(int j=0; j<WIDTH_SMALL; ++j)
		{
			isFixed[j][i] = false;
		}
	}
	img1_origSize = cvQueryFrame(capture);	cvResize(img1_origSize, img1); cvCopyImage(img1, img4);// cvShowImage("4",img4); cvWaitKey(0);
	img1_origSize = cvQueryFrame(capture);	cvResize(img1_origSize, img1); cvCopyImage(img1, img3);
	img1_origSize = cvQueryFrame(capture);	cvResize(img1_origSize, img1); cvCopyImage(img1, img2);// cvShowImage("2",img2); cvShowImage("4",img4); cvWaitKey(0);
	uchar* img1data = (uchar*)img1->imageData;
	uchar* img2data = (uchar*)img2->imageData;
	uchar* img3data = (uchar*)img3->imageData;
	uchar* img4data = (uchar*)img4->imageData;
	int xSamples = 100;
	int thresh = 3;
	cvCreateTrackbar("road_thresh", "trackbar", &thresh, 50, 0);
	cvCreateTrackbar("road_xSamples", "trackbar", &xSamples, 200, 0);
	for(int i=0; i<xSamples; ++i)
	{
		img1_origSize = cvQueryFrame(capture);
		cvResize(img1_origSize, img1);
		int index;
		for(int h=0; h<HEIGHT_SMALL; ++h)
		{
			for(int w=0; w<WIDTH_SMALL; ++w)
			{
				index = h*WIDTH_STEP_SMALL + w*NCHANNELS;
				if( isFixed[w][h] == false &&
					abs(img1data[index+0]-img2data[index+0])  < thresh &&
					abs(img1data[index+1]-img2data[index+1])  < thresh &&
					abs(img1data[index+2]-img2data[index+2])  < thresh &&
					abs(img2data[index+0]-img3data[index+0])  < thresh &&
					abs(img2data[index+1]-img3data[index+1])  < thresh &&
					abs(img2data[index+2]-img3data[index+2])  < thresh &&
					abs(img3data[index+0]-img4data[index+0])  < thresh &&
					abs(img3data[index+1]-img4data[index+1])  < thresh &&
					abs(img3data[index+2]-img4data[index+2])  < thresh )
				{
					isFixed[w][h] = true;
					avgImageData[index] = img1data[index];
					avgImageData[index+1] = img1data[index+1];
					avgImageData[index+2] = img1data[index+2];
				}
			}
		}
		cvCopyImage(img3, img4);
		cvCopyImage(img2, img3);
		cvCopyImage(img1, img2);
		cvShowImage("road_image_formation", avgImage);
		cvWaitKey(33);
	}
	return avgImage;
}
