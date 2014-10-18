#include  <ctime>	//To calculate FPS using time(0) - returns current time
#include  "caliberate.h"	//contains functions which are called only once - 1.average brightness 2.Polygon Caliberation 3.Road Image finding
CvCapture	* capture;	//The capture class captures video either from harddisk(.avi) or from camera
IplImage 	* frameImg;	//contains every frame of the video
uchar		* frameData;	//data of frameImg image
IplImage	* g_image;		//gray image of frameImg, passed into cvFindContour function
CvMemStorage* g_storage = cvCreateMemStorage(0);	//Memory storage for contours. Each contour corresponds to one vehicle.
double dist(CvPoint2D32f x1, CvPoint2D32f x2)	//distance between two points x1 and x2
{	return pow((double) (x1.x-x2.x)*(x1.x-x2.x) + (x1.y-x2.y)*(x1.y-x2.y) ,(double) .5); }
int main() 
{
	capture = cvCaptureFromAVI( "F:\cam1.avi" );	//Video capture from harddisk(.avi) or from camera
	IplImage* frameImg_origSize;	//image taken from camera feed in original size
	frameImg = cvCreateImage(cvSize(WIDTH_SMALL, HEIGHT_SMALL),8,3);	//same image from camera feed but in smaller size for faster calculation
	cvNamedWindow ( "out"	  , CV_WINDOW_AUTOSIZE);	//window to show output
	cvNamedWindow ( "trackbar", CV_WINDOW_AUTOSIZE);	//Trackbars to change value of parameters
	cvResizeWindow(	"trackbar", 300, 600		  );	//Resizing trackbar window for proper view of all the parameters
	frameImg_origSize 	= cvQueryFrame( capture );	//Just to know original size of video
	cvResize(frameImg_origSize, frameImg);	//Resize original frame into smaller frame for faster calculations
	CvSize origSize = cvGetSize(frameImg_origSize);	//original size
	cout<<"ORIG: size = "<<frameImg_origSize->width
		<<" X "<<frameImg_origSize->height
		<<" step "<<frameImg_origSize->widthStep
		<<" nchannels "<<frameImg_origSize->nChannels<<endl;	//print original size: width, height, widthStep, no of channels.

	g_image = cvCreateImage(cvSize(WIDTH_SMALL, HEIGHT_SMALL),IPL_DEPTH_8U,1);	cvZero(g_image);	//Gray image of frameImg
	frameData  = (uchar*)frameImg ->imageData;	//Data of frameImg
	calibIntensity();	//Average Intensity of all pixels in the image
	IplImage* roadImage = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL), IPL_DEPTH_8U, 3);	//Image of the road (without vehicles)
	roadImage = findRoadImage();	//Image of the road
	
	uchar* roadImageData = (uchar*)roadImage->imageData;	//Data of roadImage
	calibPolygon();	//Polygon caliberation: Select four points of polygon clockwise and press enter
	cout<<"polyArea = "<<polyArea;	//Area of selected polygon
	IplImage* binImage = cvCreateImage(cvSize(WIDTH_SMALL, HEIGHT_SMALL),IPL_DEPTH_8U,1);	//white pixel = cars, black pixel = other than cars
	uchar* binImageData = (uchar*)binImage->imageData;	//data of binImage
	IplImage* finalImage = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL), IPL_DEPTH_8U, 3);	//final image to show output

	double T = time(0);	//Current time
	float fps = 0, lastCount = 0;	//frames per second
	int thresh_r = 43, thresh_g = 43, thresh_b = 49;	//Threshold parameters for Red, Green, Blue colors
	cvCreateTrackbar( "Red Threshold", "trackbar", &thresh_r, 255, 0 );	//Threshold for Red color
	cvCreateTrackbar( "Green Threshold", "trackbar", &thresh_g, 255, 0 );	//Threshold for Green color
	cvCreateTrackbar( "Blue Threshold", "trackbar", &thresh_b, 255, 0 );//Threshold for Blue color
	int dilate1=1, erode1=2, dilate2=5;	//Dilate and Erode parameters
	IplImage* imgA = cvCreateImage(cvSize(WIDTH_SMALL,HEIGHT_SMALL),8,3);//Used for opticalFlow
	CvPoint2D32f* cornersA = new CvPoint2D32f[ MAX_CORNERS ];	//Input points for opticalFlow
	CvPoint2D32f* cornersB = new CvPoint2D32f[ MAX_CORNERS ];	//Output points from opticalFlow
	cvCopyImage(frameImg,imgA);	//copy from frameImg to imgA

	int win_size = 20;	//parameter for opticalFlow
	int corner_count = MAX_CORNERS;	//no of points tracked in opticalFlow
	IplImage* pyrA = cvCreateImage( cvSize(WIDTH_SMALL,HEIGHT_SMALL), IPL_DEPTH_32F, 1 );	//Temp image (opticalFlow)
	IplImage* pyrB = cvCreateImage( cvSize(WIDTH_SMALL,HEIGHT_SMALL), IPL_DEPTH_32F, 1 );	//Temp image (opticalFlow)
	double distance;	//Length of lines tracked by opticalFlow
	int maxArrowLength = 100, minArrowLength = 0;	//div by 10 //Max and Min length of the tracked lines
	int arrowGap = 5;	//distance between consecutive tracking points (opticalFlow)
	cvCreateTrackbar("max arrow length", "trackbar", &maxArrowLength, 100, 0);	//Manually change max length of tracked lines
	cvCreateTrackbar("min arrow length", "trackbar", &minArrowLength, 100, 0);	//Manually change min length of tracked lines
	cvCreateTrackbar("dilate 1","trackbar", &dilate1, 15, 0);	//first dilate
	cvCreateTrackbar("erode 1","trackbar", &erode1, 15, 0);		//first erode
	cvCreateTrackbar("dilate 2","trackbar", &dilate2, 15, 0);	//second dilate
	char features_found[ MAX_CORNERS ];	//temp data (opticalFlow)
	float feature_errors[ MAX_CORNERS ];//temp data (opticalFlow)
	//////////////////////////////////////////////////////////////////////////
	while(true) //Loops till video buffers
	{
		cout<<endl;
		++fps;	//calculation of Frames Per Second
		frameImg_origSize = cvQueryFrame( capture ); //Store image in original size
		if( !frameImg_origSize ) break; //if there is no frame available (end of buffer); stop.
		cvResize(frameImg_origSize, frameImg); //resize original image into smaller image for fast calculation
		cvShowImage("video", frameImg);
		
		register int X; //temp variable
		for( int i=0; i<HEIGHT_SMALL; ++i) //iter through whole frame and compare it with image of road; if greater than threshold, it must be a vehicle
		{
			for(int j=0; j<WIDTH_SMALL; ++j)
			{
				X = i*WIDTH_STEP_SMALL+j*NCHANNELS;
				if(	abs(roadImageData[X+0]-frameData[X+0])<thresh_r &&
					abs(roadImageData[X+1]-frameData[X+1])<thresh_g &&
					abs(roadImageData[X+2]-frameData[X+2])<thresh_b ) //comparing frame image against road image using threshold of Red, Green and Blue
				{	binImageData[i*binImage->widthStep+j] = 0;	}	//other than vehicle (black)
				else
				{	binImageData[i*binImage->widthStep+j] = 255;	}	//vehicle (white)
		    }
		}
		cvCopyImage(frameImg, finalImage); //final image to show output in it
		cvAnd(binImage, polygonImg, binImage, 0);	//Quadrilateral Cropping
		cvShowImage("bin image", binImage);
		cvDilate(binImage, binImage, 0, dilate1);	//dilate and erode removes noise. cvBlur also removes noise but is slow.
		cvErode(binImage, binImage, 0, erode1);
		cvDilate(binImage, binImage, 0, dilate2);
		cvShowImage("noise removed", binImage);
		//////////////////////////////////////////////////////////////////////////
		cvCopyImage(binImage,g_image); //g_image is passed in cvFindContours
		CvSeq* contours = NULL; //contour information
		cvFindContours( g_image, g_storage, &contours, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); 
		/// finds contours. g_image = imput image, g_storage = temp storage, &contours = location where contour info is saved
		/// CV_RETR_CCOMP = contours are stored as connected component, CV_CHAIN_APPROX_SIMPLE = contour finding method
		double contourArea = 0, percentArea = 0; // % of area occupied by vehicles from the area of polygon
		if( contours )
		{
			int xContours=1; // no of contours (vehicles)
			for(CvSeq* tempSeq = contours ;xContours<MAXCARS && tempSeq!=NULL; tempSeq = tempSeq->h_next, xContours++)
			{ //iterate through all the contours by using ->h_next to access next contour
				contourArea+=fabs(cvContourArea(tempSeq)); // finding  total contour area
				cvDrawContours(finalImage,contours,RED,BLUE,3,1, CV_AA); //draw contour boundary around all vehicles; RED=contours, BLUE=holes
			}	--xContours; // xContours = No of cars
			cout<<"("<<xContours<<") ";
		}
		cvShowImage("contour drawing", finalImage);
		percentArea = (contourArea*100)/polyArea; // May be >100% sometimes due to Dilate which expands the contours
		cout<<int(percentArea)<<"% ";
		//////////////////////////////////////////////////////////////////////////
		int xCorners = 0; //No of points to be tracked by opticalFlow
		for(int i=0; i<HEIGHT_SMALL; i+=arrowGap) //preparing input points to be tracked
		{
			for(int j=0; j<WIDTH_SMALL; j+=arrowGap)
			{
				if( xCorners >= MAX_CORNERS-1 ) break; //no of points must not exceed MAX_CORNERS
				if( binImageData[i*WIDTH_SMALL + j] == 255 ) //points must be chosen only on the vehicles (white pixels)
				{
					cornersA[xCorners].x = j;
					cornersA[xCorners].y = i;
					++xCorners;
				}
			}
		}
		if( percentArea>80.0 || fps<=4 )	arrowGap=15; //reduce point density if processor is loaded
		else if( percentArea>40.0 || fps<=7 )	arrowGap=10;
		else	arrowGap=5;
		corner_count = xCorners; //no of points to be tracked

		cvCalcOpticalFlowPyrLK(imgA,frameImg,pyrA,pyrB,cornersA,cornersB,corner_count,cvSize( win_size,win_size ),5,features_found,feature_errors,
			cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3 ),0); //calculates opticalFlow
		/// imgA = previous image; frameImg = current image; pyrA, pyrB = temp images; cornersA = input points; cornersB = output points; Rest is not important
			
		int xCornersInRange = 1; // No of points which satisfies Min and Max length criteria
		double avgDist = 0; //average length of tracked lines = average movement of vehicles.
		for( int i=0; i<corner_count; i++ ) //iterate through all tracking points
		{
			distance = dist(cornersA[i], cornersB[i]); //length of tracked lines = magnitude of movement of vehicle
			if( distance < maxArrowLength/10 && distance > minArrowLength/10) //only accept points which lies in Min-Max range
			{
				++xCornersInRange;
				avgDist += distance; //add length of all lines
				cvLine( finalImage, cvPoint(cornersA[i].x,cornersA[i].y), cvPoint(cornersB[i].x,cornersB[i].y) , CV_RGB(0,0,255),1 , CV_AA); //draw all tracking  lines
			}
		}
		avgDist /= xCornersInRange; //average length of lines
		cout<<avgDist;
		cvCopyImage(frameImg,imgA); //current image frameImg will be previous image imgA for the next frame
		//////////////////////////////////////////////////////////////////////////
		cvLine(finalImage, pts[0], pts[1], CV_RGB(0,255,0),1,CV_AA); //draw polygon in final image (Green)
		cvLine(finalImage, pts[1], pts[2], CV_RGB(0,255,0),1,CV_AA);
		cvLine(finalImage, pts[2], pts[3], CV_RGB(0,255,0),1,CV_AA);
		cvLine(finalImage, pts[3], pts[0], CV_RGB(0,255,0),1,CV_AA);
		cvShowImage( "out", finalImage); // show final output image
		char c = cvWaitKey(33); //waits 33msec before processing next frame
		if( c == 27 ) break; // if "esc" pressed; exits.
		if( time(0) >= T+1 )
		{
			cout<<'['<<fps<<"] ";
			//lastCount = fps;
			fps=0;
			T = time(0);
		}
	}
}
