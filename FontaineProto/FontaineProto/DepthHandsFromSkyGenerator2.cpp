#include "DepthHandsFromSkyGenerator2.h"

DepthHandsFromSkyGenerator2::DepthHandsFromSkyGenerator2(string name) : Generator(name)
{
	fountainHeight = 100;
	bodySize = 80;
	height = 424;
	width = 512;
	bufferSize = 0;
	buffer = nullptr;
	_newId = 0;
	_removeNBFrames = 10;
	_removeBackgroundDirectory = "";
}

DepthHandsFromSkyGenerator2::~DepthHandsFromSkyGenerator2(void)
{
}

Node* DepthHandsFromSkyGenerator2::clone(string cloneName) const
{
	// A ColorTrackerv2 cannot be cloned, it is better to return the current instance 
	// Another ColorTrackerv2 can be created for openning another device or another file
	return (Node*)this;
}

bool isInsideEllipse(RotatedRect r, Point p, int bodySize)
{
	Point center = r.center;
	float a = r.angle*3.14159/180.0;	
	return ((pow(((cos(a)*(p.x - center.x)) + (sin(a)*(p.y - center.y))),2.0f)/(pow(r.size.width*(float)bodySize/100.0f,2.0f))) + (pow(((sin(a)*(p.x - center.x)) - (cos(a)*(p.y - center.y))),2.0f)/(pow(r.size.height*(float)bodySize/100.0f,2.0f)))) <= 1.0f;
}

bool DepthHandsFromSkyGenerator2::initKinect()
{
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

	hr = m_pKinectSensor->get_CoordinateMapper( &pCoordinateMapper );
	if (FAILED(hr)){
		cout << "ColorTrackerv2 Error : Mapper failed." << endl;
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

}

bool DepthHandsFromSkyGenerator2::setupVariables()
{
	string line;
	ifstream myfile ("source_config.txt");
	if (myfile.is_open())
	{
		getline (myfile,line);
		hauteurCamera = stoi(line);
		myfile.close();
	}
	else
	{
		cout << "Unable to open file"; 
		exit(-1);
	}

	myfile.open("source3D.txt");
	if (myfile.is_open())
	{
		getline (myfile,line);
		hauteurCamera = stoi(line);

		getline (myfile,line);
		blasterWidth = stof(line);

		getline (myfile,line);
		int blastersNumber = stoi(line);

		for(int i = 0;i<blastersNumber;i++)
		{
			getline (myfile,line);
			xBlaster.push_back(stof(line));

			getline (myfile,line);
			yBlaster.push_back(stof(line));
		}
		myfile.close();
	}
	else
	{
		cout << "Unable to open file"; 
		exit(-1);
	}
	return true;
}

bool DepthHandsFromSkyGenerator2::start()
{
	cout << "Start OpenCVDrawObserver " << endl;	
	setupVariables();
	initKinect();

	frame.create(height,width, CV_8UC1);
	_background.create(0, 0, CV_8UC1);
	centers.create(height,width, CV_8UC1);

	initRemoveBackground();
	return true;
}

void printProgress (double percentage)
{
	cout << "Removing background ..." << int(percentage * 100.0) << " %\r";
	cout.flush();
}

void DepthHandsFromSkyGenerator2::initRemoveBackground()
{
	IDepthFrame* pDepthFrame = nullptr;
	HRESULT hr;

	if(_removeBackgroundDirectory.compare("") != 0)
		_background = imread( _removeBackgroundDirectory, 1 );

	if(_background.rows != height || _background.cols != width)
	{
		_background = Mat::zeros(height, width, CV_8UC1);
		for (int i = 0; i != _removeNBFrames; i++)
		{
			printProgress((float)i/(float)_removeNBFrames);
			hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
			while(!SUCCEEDED(hr))
			{
				Sleep(50);
				hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
			}
			hr = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );
			for (int y = 0; y < height; ++y)
			{
				for (int x = 0; x < width; ++x)
				{
					if ((buffer[width*y+(width-1-x)] > 0)&&(buffer[width*y+(width-1-x)] < hauteurCamera-fountainHeight) && x > 100 && x < width-100 && y > 100 && y < height -100)
					{
						int value = (int)((1.0 - (float)buffer[width*y+(width-1-x)] / (float)(hauteurCamera-fountainHeight)) * 255.0);
						_background.at<uchar>(y, x) = (int)_background.at<uchar>(y, x) < value ? value : (int)_background.at<uchar>(y, x);
					}
				}
			}
			if( pDepthFrame != NULL )
			{
				pDepthFrame->Release();
				pDepthFrame = NULL;
			}
		}
		imwrite( "background.jpg", _background );

		if( pDepthFrame != NULL )
		{
			pDepthFrame->Release();
			pDepthFrame = NULL;
		}
		printProgress(1.0);
		cout << endl << "Saving backgroud as background.jpg" << endl;
	}
}

bool DepthHandsFromSkyGenerator2::stop()
{

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

	return true;
}

void DepthHandsFromSkyGenerator2::findHands()
{
	for( int i = 0; i< contours.size(); i++ )
	{
		vector<Point> contourPoints;
		approxPolyDP( contours[i], contourPoints, APPROX_PREC,true);
		if (contourArea(contours[i]) > MIN_BLOB_SIZE)
		{
			for(int e = 0; e < ellipses.size();e++)
			{
				vector<Point> handsTemp;
				RotatedRect r = ellipses[e];
				int nbP = 0;
				for(int c = 0; c < contourPoints.size();c++)
				{
					if(isInsideEllipse(r, contourPoints[c], bodySize))
						nbP++;
					else 
						handsTemp.push_back(contourPoints[c]);
				} 

				// check if the ellipse is the one associated with contours ( check centers ?)
				// Plus de La moitié des points est a l interieur de l ellipse
				if(nbP >= MIN_NB_CONT)
				{
					//cout << "Body found" << endl;
					for(int h = 0; h < handsTemp.size();h++)
						hands.push_back(handsTemp[h]);
				}
			}
		}
		else
		{
			//check if contours without attached ellipses can be hands			
			if(contourPoints.size() < 1)
				continue;
			Point bestPoint = contourPoints[0];
			float distanceBestPoint = Point2D(width/2.0, height/2.0).distanceTo(Point2D(bestPoint.x, bestPoint.y));
			for(int c = 0; c < contourPoints.size();c++)
			{
				Point p = contourPoints[c];
				float distance = Point2D(width/2.0, height/2.0).distanceTo(Point2D(p.x, p.y));
				if(distance < distanceBestPoint)
				{
					distanceBestPoint = distance;
					bestPoint = p;
				}
			}
			if (distanceBestPoint > 90)
				hands.push_back(bestPoint);
		}
	}
}

void DepthHandsFromSkyGenerator2::removeHandsInsideEllipses()
{
	// ensure ellipses does not contains hands
	vector<Point> handsTmp = hands;
	for(int h = 0; h < handsTmp.size();h++)
	{
		for(int e = 0; e < ellipses.size();e++)
		{
			RotatedRect r = ellipses[e];
			float a = r.angle*3.14159/180.0;	
			Point p = handsTmp[h];
			if(isInsideEllipse(r, p, bodySize))
			{
				auto it = std::find(hands.begin(), hands.end(), p);
				if(it != hands.end())
					hands.erase(it);	
			}
		}
	}
}

void DepthHandsFromSkyGenerator2::findContoursAndEllipses()
{
	vector<Vec4i> hierarchy;
	// premiere approche pour choper les gens autour de la fontaine
	// remplit le tableau ellipses avec les minAreaRect de chaque personne
	cv::erode(frame, centers, getStructuringElement(MORPH_ELLIPSE, Size(BIG_EROSION, BIG_EROSION)) );
	cv::dilate( centers, centers, getStructuringElement(MORPH_ELLIPSE, Size(BIG_EROSION, BIG_EROSION)) ); 

	// DO not process centers
	cv::circle(centers, Point(width/2.0, height/2.0), 100, 0, -1);

	cv::findContours( centers, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	for( int i = 0; i< contours.size(); i++ )
		ellipses.push_back(minAreaRect(contours[i]));

	// détection forme des gens pour distinguer mains
	// update : erodé seulement une fois ça suffit et on enlève la dilatation des 
	cv::erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );
	//cv::dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 

	//  contours
	cv::findContours( frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

}

void DepthHandsFromSkyGenerator2::convertDepthHandsToCameraHands()
{
	// hands get all depth data close to the hand estimated position
	for(int h = 0; h < hands.size();h++)
	{
		int minX = hands[h].x - HANDS_PIXELS_WIDTH;
		if(minX < 0) minX = 0;
		int maxX = hands[h].x + HANDS_PIXELS_WIDTH;
		if(maxX > width) maxX = width;
		int minY = hands[h].y - HANDS_PIXELS_WIDTH;
		if(minY < 0) minY = 0;
		int maxY = hands[h].y + HANDS_PIXELS_WIDTH;
		if(maxY > height) maxY = height;

		int xCpt = 0, yCpt = 0 , dCpt = 0, nbCpt = 0;
		for(int x = minX; x < maxX;x++)
		{
			for(int y = minY; y < maxY;y++)
			{
				if ((buffer[width*y+(width-1-x)] > 0)&&(buffer[width*y+(width-1-x)] < hauteurCamera-fountainHeight))
				{
					xCpt += x;
					yCpt += y;
					dCpt += buffer[width*y+(width-1-x)];
					nbCpt++;
				}
			}
		}

		xCpt = (float)xCpt / (float)nbCpt;
		yCpt = (float)yCpt / (float)nbCpt;
		dCpt = (float)dCpt / (float)nbCpt;

		// convert to real world and add to _hands
		CameraSpacePoint cameraPoint = { 0 };
		DepthSpacePoint depthPoint = { 0 };
		UINT16 depth = dCpt;

		depthPoint.X = static_cast<float>(xCpt); 
		depthPoint.Y = static_cast<float>(yCpt); 
		pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint,dCpt,&cameraPoint);
		
		Point3D point(cameraPoint.X, cameraPoint.Y, cameraPoint.Z);
		//cout << point.getX() << " - " << point.getY() << "\t" << point.distanceTo(Point2D(0.0, 0.0)) << endl;
		if(point.distanceTo(Point3D(0.0, 0.0, hauteurCamera/1000)) < 0.8) //meters
		{
			currentHands.push_back(Point3D(cameraPoint.X,cameraPoint.Y,cameraPoint.Z));
		}
	}
}

void DepthHandsFromSkyGenerator2::removeHandsProximity()
{
	// supprimer les mains trop proches (< HANDS_PROXIMITY)
	vector<int> toDel;
	int index = 0;
	for(vector<Point3D>::iterator pit = currentHands.begin();pit != currentHands.end();pit++)
	{
		bool toD = false;
		for(vector<Point3D>::iterator sit = pit+1;sit != currentHands.end();sit++)
		{
			if(pit->distanceTo(*sit) <= HANDS_PROXIMITY)
				toD = true;
		}
		if(toD) toDel.push_back(index);
		index++;
	}

	for(vector<int>::reverse_iterator rit = toDel.rbegin();rit != toDel.rend();rit++)
		currentHands.erase(currentHands.begin()+ *rit);
}

void DepthHandsFromSkyGenerator2::removeHandsBehindEllipses()
{
	// ensure hands are not behind ellipses
	vector<Point> handsTmp = hands;
	Point2D origin(width/2.0, height/2.0);
	for(int h = 0; h < handsTmp.size();h++)
	{
		Point2D p = Point2D(handsTmp[h].x, handsTmp[h].y);
		for(int e = 0; e < ellipses.size();e++)
		{
			RotatedRect r = ellipses[e];
			Point2D center = Point2D(r.center.x, r.center.y);
			if( center.distanceTo(origin) < p.distanceTo(origin))
			{
				auto it = std::find(hands.begin(), hands.end(), handsTmp[h]);
				if(it != hands.end())
					hands.erase(it);	
			}
		}
	}
}

bool DepthHandsFromSkyGenerator2::generate(map<string,Group3D*>& g3D,map<string,Group2D*>& g2D,map<string,Group1D*>&,map<string,GroupSwitch*>&)
{
	IDepthFrame* pDepthFrame = nullptr;
	HRESULT hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	hands.clear();
	currentHands.clear();
	ellipses.clear();
	contours.clear();	

	while(!SUCCEEDED(hr))
	{
		Sleep(50);
		hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	}

	hr = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );

	int currentId = 1;
	for (int y = 0; y < height; ++y)
		for (int x = 0; x < width; ++x)
			if ( buffer[width*y+(width-1-x)] > 0 && buffer[width*y+(width-1-x)] < hauteurCamera-fountainHeight && (int)((1.0 - (float)buffer[width*y+(width-1-x)] / (float)(hauteurCamera-fountainHeight)) * 255.0) > (int)_background.at<uchar>(y, x)+10)
				frame.at<uchar>(y, x) = 255;
			else
				frame.at<uchar>(y, x) = 0;

	findContoursAndEllipses();
	findHands();	
	removeHandsInsideEllipses();
	removeHandsBehindEllipses();
	convertDepthHandsToCameraHands();
	removeHandsProximity();

	// associer les mains a un id et d'une frame a l autre les reassocier avec cet id ( par distance ?)
	if(!_hands.empty())
	{
		set<int> ids;
		for(map<int,Point3D>::iterator mit = _hands.begin();mit != _hands.end();mit++)
			ids.insert(mit->first);

		for(vector<Point3D>::iterator pit = currentHands.begin();pit != currentHands.end();pit++)
		{
			int idToSyncWith = -1;
			float dist = HAND_DISTANCE_FOR_ASSOCIATION2;
			for(set<int>::iterator iit = ids.begin();iit != ids.end();iit++)
			{
				if(_hands[*iit].distanceTo(*pit) < dist)
				{
					idToSyncWith = *iit;
					dist = _hands[*iit].distanceTo(*pit);
				}
			}

			if(idToSyncWith == -1)
				_hands[_newId++] = *pit;
			else
			{
				_hands[idToSyncWith] = *pit;
				ids.erase(idToSyncWith);
			}
		}

		if(!ids.empty())
		{
			for(set<int>::iterator iit = ids.begin();iit != ids.end();iit++)
				_hands.erase(*iit);
		}

		//  mettre les mains de _hands dans g3D (et effacer ancienne main si necessaires)
		// rotation : mettre les mains dans le bon sens (X -> X ; Y -> Z ; Z -> Y)
		set<string> toDel;
		for(map<string,Group3D*>::iterator git = g3D.begin();git != g3D.end();git++)
			if((git->second->getType() == "Kinectv2_Up_RightHands")&&(_hands.count(stoi(git->first)-_id) < 1)) toDel.insert(git->first);

		for(set<string>::iterator dit = toDel.begin();dit != toDel.end();dit++)
			g3D.erase(*dit);

		for(map<int,Point3D>::iterator hit = _hands.begin();hit != _hands.end();hit++)
			updateData(_environment,g3D,to_string(_id+hit->first),"Kinectv2_Up_RightHands",LG_ORIENTEDPOINT3D_RIGHT_HAND,LG_ORIENTEDPOINT3D_RIGHT_HAND,_timestamp,OrientedPoint3D(Point3D(1000.0*hit->second.getX(),1000.0*hit->second.getY(),1000.0*hit->second.getZ()),Point3D(),1.0,1.0));
	}
	else
		for(vector<Point3D>::iterator pit = currentHands.begin();pit != currentHands.end();pit++)
			_hands[_newId++] = *pit;

	if( pDepthFrame != NULL )
	{
		pDepthFrame->Release();
		pDepthFrame = NULL;
	}

	return true;
}

set<string> DepthHandsFromSkyGenerator2::produce() const
{
	set<string> produce;
	produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);

	return produce;
}

