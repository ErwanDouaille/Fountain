// SourceControl.cpp : Defines the entry point for the console application.
//



#include <stdio.h>      /* printf */
#include <signal.h>     /* signal, raise, sig_atomic_t */
#include <iostream> 
#include <stdlib.h> 
#include <map>
#include <sstream>
#include <string>
#include <Windows.h>
#include <math.h>  
#include <fstream>
#include <string>
#include <vector>

#include "Kinect.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define TOTAL_PIXELS_THRESHOLD 1000
#define BLOB_PIXELS_THRESHOLD 500
#define HAND_PIXELS_NUMBER 50


// Augmented stack reserve size

using namespace cv;
using namespace std;


int hauteurCamera;
int fountainXPosition,fountainYPosition,fountainWidth,blasterWidth;
vector<int> blasterXPosition,blasterYPosition;

int fountainHeight = 1000;
Mat frame;
Mat centers;

/*
vector<bool> _done;
vector<int> _ids;

vector<bool> _firstFireDone;
vector<bool> _secondFireDone;


map<int, int> _xIds;
map<int, int> _yIds;
map<int, int> _cptIds;*/


int height = 240;
int width = 320;
/*
int farestX,farestY;
float farestDist;
int cntPix = 0;
*/

unsigned int bufferSize = 0;
unsigned short* buffer = nullptr;

int counter = 1;
/*
bool secondFire(int x,int y, int xCenter,int yCenter, int ray){
	if(!_secondFireDone[w*y+x]){
		_secondFireDone[w*y+x] = true;
		if(sqrt(pow(x-xCenter,2)+pow(y-yCenter,2)) > ray){
			cntPix++;
			float distt = sqrt(pow(x-xCenter,2)+pow(y-yCenter,2));
			if(distt > farestDist){
				farestX = x;
				farestY = y;
				farestDist = distt;
			}
			//cout << "test" << endl;
			Vec3b intensity = frame.at<Vec3b>(y, x);
			intensity.val[0] = 255*(counter%2);
			intensity.val[1]= 255*(counter%3)/2;
			intensity.val[2] = 255*(counter%5)/4;
			frame.at<Vec3b>(y, x) = intensity;
			if(x>0){
				if(y>0) secondFire(x-1,y-1,xCenter,yCenter,ray);
				secondFire(x-1,y,xCenter,yCenter,ray);
				if(y<h-1) secondFire(x-1,y+1,xCenter,yCenter,ray);
			}
			if(y>0) secondFire(x,y-1,xCenter,yCenter,ray);
			if(y<h-1) secondFire(x,y+1,xCenter,yCenter,ray);
			if(x<w-1){
				if(y>0) secondFire(x+1,y-1,xCenter,yCenter,ray);
				secondFire(x+1,y,xCenter,yCenter,ray);
				if(y<h-1) secondFire(x+1,y+1,xCenter,yCenter,ray);
			}
		}
		return true;
	}
	else
		return false;
}


void firstFire(int x,int y, int xCenter,int yCenter, int ray){
	if(!_firstFireDone[w*y+x]){
		_firstFireDone[w*y+x] = true;
		if(sqrt(pow(x-xCenter,2)+pow(y-yCenter,2)) < ray){
			//cout << "test" << endl;
			//Vec3b intensity = frame.at<Vec3b>(y, x);
			//intensity.val[0] = 0;
			//intensity.val[1]= 200;
			//intensity.val[2] = 0;
			//frame.at<Vec3b>(y, x) = intensity;
			if(x>0){
				if(y>0) firstFire(x-1,y-1,xCenter,yCenter,ray);
				firstFire(x-1,y,xCenter,yCenter,ray);
				if(y<h-1) firstFire(x-1,y+1,xCenter,yCenter,ray);
			}
			if(y>0) firstFire(x,y-1,xCenter,yCenter,ray);
			if(y<h-1) firstFire(x,y+1,xCenter,yCenter,ray);
			if(x<w-1){
				if(y>0) firstFire(x+1,y-1,xCenter,yCenter,ray);
				firstFire(x+1,y,xCenter,yCenter,ray);
				if(y<h-1) firstFire(x+1,y+1,xCenter,yCenter,ray);
			}
		}
		else
		{
			farestX = x;
			farestY = y;
			farestDist = ray;
			cntPix = 0;
			if(secondFire(x,y,xCenter,yCenter,ray)){
				// something with the computed blob from secondFire
				if(cntPix>BLOB_PIXELS_THRESHOLD){
					circle(frame,Point(farestX,farestY),20,Scalar(0,255,255),2);
					// TODO sauvegarde de la main dans une liste de main a partir de la "liste" de HAND_PIXELS_NUMBER pixels

				}
				counter++;
			}
		}
		_secondFireDone[w*y+x] = true;
	}
}


bool labelling(int x,int y,int id){
	bool processed = false;
	if(!_done[w*y+x]){
		_done[w*y+x] = true;
		Vec3b intensity = frame.at<Vec3b>(y, x);
		if ((buffer[w*y+x] > 0)&&(buffer[w*y+x] < hauteurCamera-fountainHeight)){
			_firstFireDone[w*y+x] = false;
			_secondFireDone[w*y+x] = false;
			processed = true;
			_ids[w*y+x] = id;		
			_xIds[id]+=x;
			_yIds[id]+=y;
			_cptIds[id]++;
			//intensity.val[0] = 255*(_ids[w*y+x]%2);
			//intensity.val[1]= 255*(_ids[w*y+x]%3)/2;
			//intensity.val[2] = 255*(_ids[w*y+x]%5)/4;
			intensity.val[0] = 255;
			intensity.val[1]= 255;
			intensity.val[2] = 255;


			if(x>0){
				if(y>0) labelling(x-1,y-1,id);
				labelling(x-1,y,id);
				if(y<h-1) labelling(x-1,y+1,id);
			}
			if(y>0) labelling(x,y-1,id);
			if(y<h-1) labelling(x,y+1,id);
			if(x<w-1){
				if(y>0) labelling(x+1,y-1,id);
				labelling(x+1,y,id);
				if(y<h-1) labelling(x+1,y+1,id);
			}
		}
		else
		{
			_firstFireDone[w*y+x] = true;
			_secondFireDone[w*y+x] = true;
			_ids[w*y+x] = 0;				
			intensity.val[0] = 0;
			intensity.val[1]= 0;
			intensity.val[2] = 0;
		}
		frame.at<Vec3b>(y, x) = intensity;
	}
	return processed;
}
*/



int main(int argc, char* argv[])
{


	string line;
	ifstream myfile ("source_config.txt");
	if (myfile.is_open())
	{
		getline (myfile,line);
		hauteurCamera = stoi(line);
		getline (myfile,line);
		fountainXPosition = stoi(line);
		getline (myfile,line);
		fountainYPosition = stoi(line);
		getline (myfile,line);
		fountainWidth = stoi(line);

		getline (myfile,line);
		blasterWidth = stoi(line);

		getline (myfile,line);
		int numberOfBlaster = stoi(line);

		for(int i = 0;i<numberOfBlaster;i++){
			getline (myfile,line);
			blasterXPosition.push_back(stoi(line));
			getline (myfile,line);
			blasterYPosition.push_back(stoi(line));
		}

		myfile.close();
	}

	else
	{
		cout << "Unable to open file"; 
		exit(-1);
	}



	IKinectSensor* m_pKinectSensor;
	IDepthFrameReader* pDepthReader;
	IDepthFrameSource* pDepthFrameSource = NULL; 

	
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : GetDefaultKinectSensor failed." << endl;
		return false;
	}

	hr = m_pKinectSensor->Open();
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : Open failed." << endl;
		return false;
	}

	hr = m_pKinectSensor->get_DepthFrameSource( &pDepthFrameSource );
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : get_DepthFrameSource failed." << endl;
		return false;
	}
	hr = pDepthFrameSource->OpenReader( &pDepthReader );
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : OpenReader failed." << endl;
		return false;
	}

	IFrameDescription* pDepthDescription;
	hr = pDepthFrameSource->get_FrameDescription( &pDepthDescription );
	if( FAILED( hr ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	pDepthDescription->get_Width( &width ); 
	pDepthDescription->get_Height( &height ); 

	//cout << h << " * " << w << " = " << h*w << endl;
	//424 * 512

	/*for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			_done.push_back(false);
			_ids.push_back(0);
			_firstFireDone.push_back(false);
			_secondFireDone.push_back(false);
		}
	}*/

	cv::namedWindow("source_control", CV_WINDOW_AUTOSIZE );

	//frame.create(h,w, CV_8UC3);
	frame.create(height,width, CV_8UC1);
	centers.create(height,width, CV_8UC1);

	char k = 0;
	while(k!=27){
		IDepthFrame* pDepthFrame = nullptr;
		hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
		while(!SUCCEEDED(hr)){
			Sleep(5);
			hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
		}


		hr = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );

		int currentId = 1;
		for (int y = 0; y < height; ++y)
		{
			for (int x = 0; x < width; ++x)
			{
				/*if(!_done[w*y+x]){
					if(labelling(x,y,currentId))
						currentId++;
				}*/

				// put inensity in frame
				if ((buffer[width*y+x] > 0)&&(buffer[width*y+x] < hauteurCamera-fountainHeight))
					frame.at<uchar>(y, x) = 255;
				else
					frame.at<uchar>(y, x) = 0;
			}	
		}
		
		// jouer sur l erosion 
		// forte -> recupere le centre
		// Faible pour les bras

		int erosionBigSize = 50;
		erode(frame, centers, getStructuringElement(MORPH_ELLIPSE, Size(erosionBigSize, erosionBigSize)) );
		dilate( centers, centers, getStructuringElement(MORPH_ELLIPSE, Size(erosionBigSize, erosionBigSize)) ); 
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		vector<RotatedRect> ellipses;

		findContours( centers, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
		for( int i = 0; i< contours.size(); i++ )
		 {
		   //drawContours( centers, contours, i, Scalar(255), 1, 8,  vector<Vec4i>(), 0, Point() );
		   //	RotatedRect r = minAreaRect(contours[i]);
			//ellipse(centers, r.center, r.size*0.6f, r.angle, 0, 360, Scalar(255), 1, LINE_AA);
			ellipses.push_back(minAreaRect(contours[i]));
			
		   //  check if a Point is in an ellipse
			//Point2f p;


			//(pow(((cos(r.angle)*(r.center.x - p.x)) + (sin(r.angle)*(r.center.y - p.y))),2)/(pow(r.size.width,2)) + pow(((sin(r.angle)*(r.center.x - p.x)) - (cos(r.angle)*(r.center.y - p.y))),2)/(pow(r.size.height,2))) <= 1.0f;

		 }






		int erosionSmallSize = 10;
		//inRange(frame, 100,255, frame); 
		//morphological opening (removes small objects from the foreground)
		erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(erosionSmallSize, erosionSmallSize)) );
		dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(erosionSmallSize, erosionSmallSize)) ); 
		//morphological closing (removes small holes from the foreground)
		//dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		//erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		int nbPixelsHand = 25;
		vector<Point> hands;

		//  contours
		findContours( frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		for( int i = 0; i< contours.size(); i++ )
		 {
		   drawContours( frame, contours, i, Scalar(255), 1, 8,  vector<Vec4i>(), 0, Point() );


		   vector<Point> res;
		   approxPolyDP( contours[i], res, 70,true);
		   for(int c = 1; c < res.size();c++)
			   cv::line(frame,res[c-1],res[c],255,1);
		    cv::line(frame,res[res.size()-1],res[0],255,1);

			//cout << "res0 " << res[0] << endl;
			//cout << "Center " << ellipses[0].center << endl;


			for(int e = 0; e < ellipses.size();e++){
				vector<Point> handsTemp;
				RotatedRect r = ellipses[e];
				int nbP = 0;
				 for(int c = 0; c < res.size();c++){
					Point p = res[c];
					float a = r.angle;
					if(((pow(((cos(a)*(p.x - r.center.x)) + (sin(a)*(p.y - r.center.y))),2.0f)/(pow(r.size.width*0.6f,2.0f))) + (pow(((sin(a)*(p.x - r.center.x)) - (cos(a)*(p.y - r.center.y))),2.0f)/(pow(r.size.height*0.6f,2.0f)))) <= 1.0f){
						cv::circle(frame,p,10,200,1);
						nbP++;
					}
					else{
						handsTemp.push_back(p);
						cv::circle(frame,p,20,200,1);
					}
				} 
				// check if the ellipse is the one associated with contours ( check centers ?)
				// Plus de La moitié des points est a l interieur de l ellipse
				 if(nbP > res.size()/2){
					ellipse(frame, r.center, r.size*0.6f, r.angle, 0, 360, Scalar(255), 1, LINE_AA); // ellipse is the one linked with contours[i] or res

					for(int h = 0; h < handsTemp.size();h++)
						hands.push_back(handsTemp[h]);

				 }
			 }

			 




		 }

		
		// TODO hands get all depth data close to the hand estimated position
		for(int h = 0; h < hands.size();h++){
			int minX = hands[h].x - nbPixelsHand;
			if(minX < 0) minX = 0;
			int maxX = hands[h].x - nbPixelsHand;
			if(maxX > width) maxX = width;
			int minY = hands[h].y - nbPixelsHand;
			if(minY < 0) minY = 0;
			int maxY = hands[h].y - nbPixelsHand;
			if(maxY > height) maxY = height;

			int xCpt = 0, yCpt = 0 , dCpt = 0, nbCpt = 0;
			for(int x = minX; x < maxX;x++)
				for(int y = minY; y < maxY;y++)
					if ((buffer[width*y+x] > 0)&&(buffer[width*y+x] < hauteurCamera-fountainHeight)){
						xCpt += x;
						yCpt += y;
						dCpt += buffer[width*y+x];
						nbCpt++;
					}
			xCpt = (float)xCpt / (float)nbCpt;
			yCpt = (float)yCpt / (float)nbCpt;
			dCpt = (float)dCpt / (float)nbCpt;

			// TODO convert / associate with previous hand...
		}

		

		//  convex hull 
		/*vector<vector<Point> >hull( contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		  {  convexHull( Mat(contours[i]), hull[i], false ); }

		for( int i = 0; i< hull.size(); i++ )
		 {
		   drawContours( frame, hull, i, Scalar(255), 1, 8,  vector<Vec4i>(), 0, Point() );
			//RotatedRect rect =  minAreaRect(hull[i]);
			
		 }*/



		/*for( int i = 0; i< contours.size(); i++ )
		 {
			vector<Point> res;
		   approxPolyDP( contours[i], res, 70,true);
		  for(int c = 1; c < res.size();c++)
			   cv::line(frame,res[c-1],res[c],255,1);
		   cv::line(frame,res[res.size()-1],res[0],255,1);

		   //Moments mu = moments( res, true );
			//cv::circle(frame,Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 ),50,200,1);

		   //cv::Point2f center;
		   //float radius;
		   //minEnclosingCircle(res,center,radius);
		   //cv::circle(frame,center,radius,200,1);

			//RotatedRect r = minAreaRect(contours[i]);
			//ellipse(frame, r, Scalar(255), 1, LINE_AA);
			//ellipse(frame, r.center, r.size*0.5f, r.angle, 0, 360, Scalar(255), 1, LINE_AA);
			//Point2f vertices[4];
			//r.points(vertices);
			//for (int i = 0; i < 4; i++)
			//	cv::line(frame, vertices[i], vertices[(i+1)%4], Scalar(255));




		 }*/



		//  blob center ? / centroid
		/// Get the moments
		/*vector<Moments> mu(contours.size() );
		for( int i = 0; i < contours.size(); i++ )
			{ mu[i] = moments( contours[i], true ); }

		///  Get the mass centers:
		vector<Point2f> mc( contours.size() );
		for( int i = 0; i < contours.size(); i++ )
		{ mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); 
			cv::circle(frame,mc[i],20,200,2);
		} */











		// find centers
		/*map<int,int>::iterator xit = _xIds.begin();
		map<int,int>::iterator yit = _yIds.begin();
		for(map<int,int>::iterator cit = _cptIds.begin();cit != _cptIds.end();cit++,xit++,yit++){
			int xCenter = (float)xit->second/(float)cit->second;
			int yCenter = (float)yit->second/(float)cit->second;
			float ray = cit->second/250; // TODO write a function that estimate a ray from the number of pixels in this blob
			// draw a circle at that position with the ray
			//circle(frame,Point(xCenter,yCenter),ray,Scalar(255,0,0),2);

			if(cit->second>TOTAL_PIXELS_THRESHOLD){
				// replace if necessary center of blob (deplace toward exterior)
				if(_ids[w*yCenter+xCenter] == 0){
					int tempCenterX = xCenter;
					int tempCenterY = yCenter;
					//cout << "Correcting center" << _ids[w*yCenter+xCenter] << endl; 
					float dist = sqrt(pow(tempCenterX-(w/2),2)+pow(tempCenterY-(h/2),2));
					int xInc = (int)(((float)xCenter-(float)w/2)/(float)dist);
					if(xInc==0) {
						if(xCenter<w/2) xInc = -1;
						else xInc = 1;
					}
					int yInc = (int)(((float)yCenter-(float)h/2)/(float)dist);
					if(yInc==0) {
						if(yCenter<h/2) yInc = -1;
						else yInc = 1;
					}
					while((tempCenterX<w)&&(tempCenterX>=0)&&(tempCenterY<h)&&(tempCenterY>=0)&&(_ids[tempCenterY*w+tempCenterX] == 0)){
						tempCenterX += xInc;
						tempCenterY += yInc;
					}
					if((tempCenterX<w)&&(tempCenterX>0)&&(tempCenterY<h)&&(tempCenterY>0)&&(_ids[tempCenterY*w+tempCenterX] != 0)){
						xCenter = tempCenterX;
						yCenter = tempCenterY;
					}
				}

				if(_ids[w*yCenter+xCenter] == 0){
					int tempCenterX = xCenter;
					int tempCenterY = yCenter;
					//cout << "Correcting center" << _ids[w*yCenter+xCenter] << endl; 
					float dist = sqrt(pow(tempCenterX-(w/2),2)+pow(tempCenterY-(h/2),2));
					int xInc = (int)(((float)xCenter-(float)w/2)/(float)dist);
					if(xInc==0) {
						if(xCenter<w/2) xInc = -1;
						else xInc = 1;
					}
					int yInc = (int)(((float)yCenter-(float)h/2)/(float)dist);
					if(yInc==0) {
						if(yCenter<h/2) yInc = -1;
						else yInc = 1;
					}
					while((tempCenterX<w)&&(tempCenterX>=0)&&(tempCenterY<h)&&(tempCenterY>=0)&&(_ids[tempCenterY*w+tempCenterX] == 0)){
						tempCenterX -= xInc;
						tempCenterY -= yInc;
					}
					if((tempCenterX<w)&&(tempCenterX>0)&&(tempCenterY<h)&&(tempCenterY>0)&&(_ids[tempCenterY*w+tempCenterX] != 0)){
						//cout << "Corrected center" << _ids[w*tempCenterY+tempCenterX] << endl; 
						xCenter = tempCenterX;
						yCenter = tempCenterY;
					}
				}

				counter = 1;

				if(_ids[w*yCenter+xCenter] != 0){
					circle(frame,Point(xCenter,yCenter),ray,Scalar(0,0,255),2);
					firstFire(xCenter,yCenter,xCenter,yCenter,ray);

				}
			}

		}*/




		/*for(int i=0;i<w*h;i++) _done[i] = false;
		_xIds.clear();
		_yIds.clear();
		_cptIds.clear();*/

		//cv::imshow("centers", centers);
		cv::imshow("source_control", frame);

		k=cv::waitKey(1);	

		if( pDepthFrame != NULL ){
			pDepthFrame->Release();
			pDepthFrame = NULL;
		}

	}



	if (pDepthReader)
	{
		pDepthReader->Release();
		pDepthReader = NULL;
	}

	m_pKinectSensor->Close();
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Release();
		m_pKinectSensor = NULL;
	}

	return 0;
}

