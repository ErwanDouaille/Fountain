#include "OpenCVDrawObserver.h"


OpenCVDrawObserver::OpenCVDrawObserver(void) : Observer("BlasterObserver")
{
}

OpenCVDrawObserver::~OpenCVDrawObserver(void)
{
}

Node* OpenCVDrawObserver::clone(string cloneName) const
{
	return new OpenCVDrawObserver();
}

bool OpenCVDrawObserver::start()
{
	cout << "Start OpenCVDrawObserver " << endl;	
	return true;
}

bool OpenCVDrawObserver::stop()
{
	cout << "Stop OpenCVDrawObserver" << endl;
	return true;
}

set<string> OpenCVDrawObserver::need() const
{
	set<string> needed;
	return needed;
}

bool OpenCVDrawObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>)
{
	Mat frame = generator->getFrame();// Mat::zeros(generator->getHeight(), generator->getWidth(), CV_8UC3);
	vector<vector<Point> > contours = generator->getContours();
	vector<RotatedRect> ellipses = generator->getEllipses();
	vector<Point> contourPoints;	
	vector<Point> tempHands = generator->getTempHands();	

	int bodySize = generator->getBodySize();
		
	// Draw contours
	for( int i = 0; i< contours.size(); i++ )
	{
		approxPolyDP( contours[i], contourPoints, APPROX_PREC,true);
		for(int c = 1; c < contourPoints.size();c++)
			cv::line(frame, contourPoints[c-1], contourPoints[c], 255, 1);
		cv::line(frame, contourPoints[contourPoints.size()-1], contourPoints[0], 255, 1);
	}
	
	// Draw ellipses
	for(int e = 0; e < ellipses.size();e++)
	{
		RotatedRect r = ellipses[e];
		float a = r.angle*3.14159/180.0;
		ellipse(frame, r.center, r.size*(float)bodySize/100.0f, r.angle, 0, 360, Scalar(255,255,0), 1, LINE_AA); 
	}

	// Draw temp hands
	for(int e = 0; e < tempHands.size();e++)
	{
		Point r = tempHands[e];
		rectangle(frame,cv::Rect(r.x, r.y, 20, 20),255);
	}
	
	// Draw hands
	for(map<string,Group3D*>::iterator git = g3D.begin();git != g3D.end(); git++)
	{
		if(isObservedGroup(git->first,git->second->getType()))
		{
			Group3D* g = git->second;
			set<HOrientedPoint3D*> rhs = g->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
			for(set<HOrientedPoint3D*> ::iterator sit = rhs.begin();sit != rhs.end();sit++)
			{				
				HOrientedPoint3D* rh = *sit;
				if (rh->getLastTimestamp() == _timestamp)
				{
					Point3D position = rh->getLast()->getPosition();
					CameraSpacePoint cameraPoint = { 0 };
					DepthSpacePoint depthPoint = { 0 };	
					cameraPoint.X = position.getX()/1000.0;
					cameraPoint.Y = position.getY()/1000.0;
					cameraPoint.Z = position.getZ()/1000.0;
					generator->getCoordinateMapper()->MapCameraPointToDepthSpace(cameraPoint, &depthPoint);					
					cv::circle(frame,Point(depthPoint.X, depthPoint.Y),10,Scalar(255,255,255),1);
				}
			}
		}
	}
	
	// Display
	cv::imshow("openCVDrawObserver", frame);
	cv::waitKey(1);	

	return true;
}

