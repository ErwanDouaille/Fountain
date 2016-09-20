// SourceCalibration.cpp : Defines the entry point for the console application.
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
#include <math.h>       /* isinf */

#include "Kinect.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;


int hauteurCamera, w, h;
int fountainXPosition,fountainYPosition,fountainWidth,blasterWidth;
vector<int> blasterXPosition,blasterYPosition;

int attached = -2;

int dx=0,dy=0;

int displaySize =2;

bool displayColor = true;

void MouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	int fx = x/displaySize;
	int fy = y/displaySize;
	if  ( event == EVENT_LBUTTONDOWN )
	{
		// cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
		// check if a square is attached, if yes attach it unless, check if the square is attached
		for(int i = 0;i<blasterXPosition.size();i++){
			if((fx>=blasterXPosition[i]-blasterWidth/2.0)&&(fx<=blasterXPosition[i]+blasterWidth/2.0)&&(fy>=blasterYPosition[i]-blasterWidth/2.0)&&(fy<=blasterYPosition[i]+blasterWidth/2.0))
				attached = i;
		}
		if(attached == -2){
			if((fx>=fountainXPosition-fountainWidth/2.0)&&(fx<=fountainXPosition+fountainWidth/2.0)&&(fy>=fountainYPosition-fountainWidth/2.0)&&(fy<=fountainYPosition+fountainWidth/2.0))
				attached = -1;
		}
		dx=fx;
		dy=fy;


	}
	if  ( event == EVENT_LBUTTONUP )
	{
		attached = -2;
	}
	/*else if  ( event == EVENT_RBUTTONDOWN )
	{
	cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else if  ( event == EVENT_MBUTTONDOWN )
	{
	cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
	}
	else */
	if ( event == EVENT_MOUSEMOVE )
	{
		//cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
		if(attached!=-2){
			if(attached == -1){
				fountainXPosition += (fx-dx);
				fountainYPosition += (fy-dy);
			}
			else
			{
				blasterXPosition[attached] += (fx-dx);
				blasterYPosition[attached] += (fy-dy);
			}
			dx = fx;
			dy = fy;
		}

	}
}

void alignBuseFromLayout()
{	
	int cpt = 4;
	for(int i = -1; i <= 1; i++)
	{
		for(int j = -2; j <= 2; j++)
		{
			blasterXPosition[cpt-1] = fountainXPosition + (j*blasterWidth);
			blasterYPosition[cpt-1] = fountainYPosition + (i*blasterWidth);
			cpt++;
		}
	}
	cpt = 1;
	for(int i = -1; i <= 1; i++)
	{
		blasterXPosition[cpt-1] = fountainXPosition + (i*blasterWidth);
		blasterYPosition[cpt-1] = fountainYPosition + (-2*blasterWidth);
		blasterXPosition[cpt+17] = fountainXPosition + (i*blasterWidth);
		blasterYPosition[cpt+17] = fountainYPosition + (2*blasterWidth);
		cpt++;
	}
}

void resetFountainPosition()
{	
	fountainXPosition = 512/2;		
	fountainYPosition = 424/2;
}

int main(int argc, char* argv[])
{


	int similarity = 0;

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
	IDepthFrameSource* pDepthFrameSource = NULL; // Depth image

	IColorFrameReader* pColorReader;
	IColorFrameSource* pColorFrameSource = NULL;


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
	w = 0,h = 0;
	pDepthDescription->get_Width( &w ); // 512
	pDepthDescription->get_Height( &h ); // 424
	//unsigned int irBufferSize = w * h * sizeof( unsigned short );


	hr = m_pKinectSensor->get_ColorFrameSource( &pColorFrameSource );
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : get_ColorFrameSource failed." << endl;
		return false;
	}
	hr = pColorFrameSource->OpenReader( &pColorReader );
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : OpenReader failed." << endl;
		return false;
	}

	// Description
	IFrameDescription* pColorDescription;
	hr = pColorFrameSource->get_FrameDescription( &pColorDescription );
	if( FAILED( hr ) ){
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		return -1;
	}
	int colorWidth = 0;
	int colorHeight = 0;
	pColorDescription->get_Width( &colorWidth ); // 1920
	pColorDescription->get_Height( &colorHeight ); // 1080
	unsigned int colorBufferSize = colorWidth * colorHeight * 4 * sizeof( unsigned char );

	Mat colorBufferMat = Mat::zeros( cvSize(colorWidth,colorHeight), CV_8UC4   );
	//colorMat = Mat::zeros( cvSize(colorWidth/2,colorHeight/2), CV_8UC4   );



	ICoordinateMapper* pCoordinateMapper;
	hr = m_pKinectSensor->get_CoordinateMapper( &pCoordinateMapper );



	//  open a opencv window
	cv::namedWindow("source_config", CV_WINDOW_AUTOSIZE );
	setMouseCallback("source_config", MouseCallBackFunc, NULL);

	Mat frame(h,w, CV_8UC3, Scalar(255,255,255));
	Mat display;
	//Mat img;

	char k = 0;
	while(k!=27){

		HRESULT hResult = S_OK;

		if(displayColor){
			IColorFrame* pColorFrame = nullptr;
			hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
			while(!SUCCEEDED(hResult)){
				Sleep(50);
				hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
			}

			if( SUCCEEDED( hResult ) ){
				hResult = pColorFrame->CopyConvertedFrameDataToArray( colorBufferSize, reinterpret_cast<BYTE*>( colorBufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
				if( !SUCCEEDED( hResult ) ){
					return false;
				}

				resize(colorBufferMat,display,Size(displaySize*w,displaySize*h));

				flip(display,display,1);

				cv::line(display,Point(displaySize*w/2,0),Point(displaySize*w/2,displaySize*h),Scalar(0,0,255),2);
				cv::line(display,Point(0,displaySize*h/2),Point(displaySize*w,displaySize*h/2),Scalar(0,0,255),2);


				if (pColorFrame )
				{
					pColorFrame ->Release();
					pColorFrame  = NULL;
				}



			}
			else 
				return false;



		}
		else
		{

			IDepthFrame* pDepthFrame = nullptr;
			hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
			while(!SUCCEEDED(hResult)){
				Sleep(10);
				hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
			}

			if( SUCCEEDED( hResult ) ){
				unsigned int bufferSize = 0;
				unsigned short* buffer = nullptr;
				hResult = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );

				if( SUCCEEDED( hResult ) ){
					for( int y = 0; y < h; y++ ){
						for( int x = 0; x < w; x++ ){
							Vec3b intensity = frame.at<Vec3b>(y, x);
							if(buffer[ y * w + (w - x - 1) ]  < hauteurCamera){
								int d = buffer[ y * w + (w - x - 1) ];
								intensity.val[0] = 2.55*(d % 100);
								intensity.val[1] = 1.22*(d % 200);
								intensity.val[2] = 256.0*d/hauteurCamera;
							}
							else
							{
								intensity.val[0] = 255;
								intensity.val[1] = 255;
								intensity.val[2] = 255;
							}
							/*intensity.val[0] = buffer[ y * w + x ] >> 8;
							intensity.val[1] = buffer[ y * w + x ] >> 8;
							intensity.val[2] = buffer[ y * w + x ] >> 8;*/
							frame.at<Vec3b>(y, x) = intensity;
						}
					}

					// changer la couleur du rectangle en fonction de la hauteur des coins (similaire ou non) ( moins de 4cm)

					float d1 = buffer[(int)(fountainYPosition-fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition-fountainWidth/2.0))];
					float d2 = buffer[(int)(fountainYPosition-fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition+fountainWidth/2.0))];
					float d3 = buffer[(int)(fountainYPosition+fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition-fountainWidth/2.0))];
					float d4 = buffer[(int)(fountainYPosition+fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition+fountainWidth/2.0))];

					if((d1 < 0)||(d1>3500)||(d2 < 0)||(d2>3500)||(d3 < 0)||(d3>3500)||(d4 < 0)||(d4>3500)){
						similarity = 0;
					}
					else
					{
						int mn = 100;
						if((abs(d1-d2) < mn)
							&&(abs(d1-d3) < mn)
							&&(abs(d1-d4) < mn)
							&&(abs(d2-d3) < mn)
							&&(abs(d2-d4) < mn)
							&&(abs(d3-d4) < mn))
							similarity = 255;
						else{
							int md = abs(d1-d2);
							md = MAX(md,abs(d1-d3));
							md = MAX(md,abs(d1-d4));
							md = MAX(md,abs(d2-d3));
							md = MAX(md,abs(d2-d4));
							md = MAX(md,abs(d3-d4));
							if(md-mn>128)
								similarity = 0;
							else
								similarity = 128 - (md - mn);
						}
					}


					if(k=='s'){

						// get hauteur camera

						// Depthframe get 3D position of 1m20 jets et les enregistrer dans un autre fichier pour etre charger par un Observer


						CameraSpacePoint cameraPoint = { 0 };
						DepthSpacePoint depthPoint = { 0 };
						UINT16 depth;

						// Compute hauteur camera, by average of four points
						float h = 0;
						int cptH = 0;
						float d;

						d = buffer[(int)(fountainYPosition-fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition-fountainWidth/2.0))];
						if((d>500)&&(d<3500)){h+=d; cptH++;}
						d = buffer[(int)(fountainYPosition-fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition+fountainWidth/2.0))];
						if((d>500)&&(d<3500)){h+=d; cptH++;}
						d = buffer[(int)(fountainYPosition+fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition-fountainWidth/2.0))];
						if((d>500)&&(d<3500)){h+=d; cptH++;}
						d = buffer[(int)(fountainYPosition+fountainWidth/2.0)*w + (w-1-(int)(fountainXPosition+fountainWidth/2.0))];
						if((d>500)&&(d<3500)){h+=d; cptH++;}
						if(cptH>0){
							//hauteurCamera = h/cptH;
							cout << "H = " << hauteurCamera << endl;
						}

						// Compute real size of blaster
						/*depthPoint.X = static_cast<float>(blasterXPosition[0] - blasterWidth); 
						depthPoint.Y = static_cast<float>(blasterYPosition[0] - blasterWidth); 
						depth = hauteurCamera - 1000.0f; // TODO change
						pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint,depth,&cameraPoint);
						float corner1X = static_cast<float>(cameraPoint.X);
						float corner1Y = static_cast<float>(cameraPoint.Y);


						depthPoint.X = static_cast<float>(blasterXPosition[0] + blasterWidth); 
						depthPoint.Y = static_cast<float>(blasterYPosition[0] + blasterWidth); 
						pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint,depth,&cameraPoint);
						float corner2X = static_cast<float>(cameraPoint.X);
						float corner2Y = static_cast<float>(cameraPoint.Y);*/

						//float realBlasterWidth = 1000.0*(abs(corner2X-corner1X)+abs(corner2Y-corner1Y))/2.0;


						ofstream myfile;
						myfile.open ("source_config.txt");
						myfile << hauteurCamera << "\n";
						myfile << fountainXPosition << "\n";
						myfile << fountainYPosition << "\n";
						myfile << fountainWidth << "\n";
						myfile << blasterWidth << "\n";
						myfile << blasterXPosition.size() << "\n";
						for(int i = 0;i<blasterXPosition.size();i++){
							myfile << blasterXPosition[i] << "\n";
							myfile << blasterYPosition[i] << "\n";
						}
						myfile.close();

						//  save real positions to file

						myfile.open ("source3D.txt");
						myfile << hauteurCamera << "\n";
						myfile << blasterWidth << "\n";
						myfile << blasterXPosition.size() << "\n";
						for(int i = 0;i<blasterXPosition.size();i++){
							depthPoint.X = static_cast<float>(blasterXPosition[i]); 
							depthPoint.Y = static_cast<float>(blasterYPosition[i]); 
							depth = hauteurCamera;
							pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint,depth,&cameraPoint);
							cout << depthPoint.X << " - " << depthPoint.Y << endl;
							cout << static_cast<float>(cameraPoint.X) << " - " << static_cast<float>(cameraPoint.Y) << endl;
							pCoordinateMapper->MapCameraPointToDepthSpace(cameraPoint, &depthPoint);
							cout << depthPoint.X << " - " << depthPoint.Y << endl;
							cout << static_cast<float>(cameraPoint.X) << " - " << static_cast<float>(cameraPoint.Y) << endl;
							myfile << 1000.0*static_cast<float>(cameraPoint.X) << "\n";
							myfile << 1000.0*static_cast<float>(cameraPoint.Y) << "\n";
						}
						myfile.close();


					}


				}
				else{
					return false;
				}

			}
			else 
				return false;

			if( pDepthFrame != NULL ){
				pDepthFrame->Release();
				pDepthFrame = NULL;
			}

			rectangle(frame,
				Rect(fountainXPosition-fountainWidth/2.0, 
				fountainYPosition-fountainWidth/2.0, 
				fountainWidth	, 
				fountainWidth),
				Scalar(0,similarity,255-similarity),
				3);
			for(int i = -2; i <= 2; i++)
			{
				rectangle(frame,
					Rect(fountainXPosition - (i*blasterWidth) - (blasterWidth/2.0), 
					fountainYPosition-fountainWidth/2.0, 
					blasterWidth, 
					fountainWidth),
					Scalar(0,255,0),
					1);
				rectangle(frame,
					Rect(fountainXPosition-fountainWidth/2.0,
					fountainYPosition - (i*blasterWidth) - (blasterWidth/2.0),
					fountainWidth, 
					blasterWidth),
					Scalar(0,255,0),
					1);
			}

			char textbuffer [33];
			for(int i = 0;i<blasterXPosition.size();i++){
				sprintf(textbuffer,"%i",i+1);
				putText(frame,textbuffer, Point2f(blasterXPosition[i]-blasterWidth/2.0,blasterYPosition[i]), FONT_HERSHEY_PLAIN, 1,  Scalar(0,0,255,255), 2);
				//rectangle(frame,Rect(blasterXPosition[i]-blasterWidth/2.0,blasterYPosition[i]-blasterWidth/2.0,blasterWidth,blasterWidth),Scalar(255,0,0));
				circle(frame,Point(blasterXPosition[i],blasterYPosition[i]),blasterWidth/2.0,Scalar(255,0,0));
			}
			resize(frame,display,Size(displaySize*w,displaySize*h));
		}

		cv::imshow("source_config", display);


		k=cv::waitKey(1);	

		if(k == 32) // Space
			displayColor = ! displayColor;
		
		if(k=='a')
			alignBuseFromLayout();
		if(k=='r')
			resetFountainPosition();

	}


	if (pDepthReader)
	{
		pDepthReader->Release();
		pDepthReader = NULL;
	}


	if (pCoordinateMapper)
	{
		pCoordinateMapper->Release();
		pCoordinateMapper = NULL;
	}

	m_pKinectSensor->Close();
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Release();
		m_pKinectSensor = NULL;
	}


	//system("pause");

	return 0;
}

