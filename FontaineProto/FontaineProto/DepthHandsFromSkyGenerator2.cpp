#include "DepthHandsFromSkyGenerator2.h"


DepthHandsFromSkyGenerator2::DepthHandsFromSkyGenerator2(string name) : Generator(name)
{
	fountainHeight = 800;

	height = 424;
	width = 512;

	bufferSize = 0;
	buffer = nullptr;

	_newId = 0;
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

bool DepthHandsFromSkyGenerator2::start()
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


	cv::namedWindow("source_control", CV_WINDOW_AUTOSIZE );

	frame.create(height,width, CV_8UC1);
	centers.create(height,width, CV_8UC1);
	//hellipse.create(height,width, CV_8UC1);


	return true;
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

bool DepthHandsFromSkyGenerator2::generate(map<string,Group3D*>& g3D,map<string,Group2D*>& g2D,map<string,Group1D*>&,map<string,GroupSwitch*>&)
{
	currentHands.clear();

	IDepthFrame* pDepthFrame = nullptr;
	HRESULT hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	while(!SUCCEEDED(hr)){
		//cout << " Error : Acquire failed." << endl;
		Sleep(50);
		hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	}


	hr = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );

	int currentId = 1;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x)
		{
			if ((buffer[width*y+(width-1-x)] > 0)&&(buffer[width*y+(width-1-x)] < hauteurCamera-fountainHeight))
				frame.at<uchar>(y, x) = 255;
			else
				frame.at<uchar>(y, x) = 0;
		}	
	}

	erode(frame, centers, getStructuringElement(MORPH_ELLIPSE, Size(BIG_EROSION, BIG_EROSION)) );
	dilate( centers, centers, getStructuringElement(MORPH_ELLIPSE, Size(BIG_EROSION, BIG_EROSION)) ); 
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<RotatedRect> ellipses;
	
	// DO not process center
	cv::circle(centers,Point(width/2.0,height/2.0),100,0,-1);

	findContours( centers, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE );
	for( int i = 0; i< contours.size(); i++ )
		ellipses.push_back(minAreaRect(contours[i]));



	//inRange(frame, 100,255, frame); 
	//morphological closing (removes small holes from the foreground)
	dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(SMALL_EROSION, SMALL_EROSION)) ); 
	erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(SMALL_EROSION, SMALL_EROSION)) );
	//morphological opening (removes small objects from the foreground)
	erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(SMALL_EROSION, SMALL_EROSION)) );
	dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(SMALL_EROSION, SMALL_EROSION)) ); 


	vector<Point> hands;

	//  contours
	findContours( frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	for( int i = 0; i< contours.size(); i++ )
	{
		drawContours( frame, contours, i, Scalar(255), 1, 8,  vector<Vec4i>(), 0, Point() );


		vector<Point> res;
		approxPolyDP( contours[i], res, APPROX_PREC,true);
		for(int c = 1; c < res.size();c++)
			cv::line(frame,res[c-1],res[c],255,1);
		cv::line(frame,res[res.size()-1],res[0],255,1);

		//cout << "res0 " << res[0] << endl;
		//cout << "Center " << ellipses[0].center << endl;

		//cout << "Nb body = " << ellipses.size() << endl;


		for(int e = 0; e < ellipses.size();e++){

			vector<Point> handsTemp;
			RotatedRect r = ellipses[e];
			float a = r.angle*3.14159/180.0;

			
			//  retirer hellipse
			/*for(int xx = 0;xx < hellipse.cols;xx++)
				for(int yy = 0;yy < hellipse.rows;yy++)
					if(((pow(((cos(a)*(xx - r.center.x)) + (sin(a)*(yy - r.center.y))),2.0f)/(pow(r.size.width*(float)BODY_SIZE/100.0f,2.0f))) + (pow(((sin(a)*(xx - r.center.x)) - (cos(a)*(yy - r.center.y))),2.0f)/(pow(r.size.height*(float)BODY_SIZE/100.0f,2.0f)))) <= 1.0f)
						hellipse.at<uchar>(yy, xx) = 255;
					else
						hellipse.at<uchar>(yy, xx) = 0;*/


			int nbP = 0;
			for(int c = 0; c < res.size();c++){
				Point p = res[c];
				if(((pow(((cos(a)*(p.x - r.center.x)) + (sin(a)*(p.y - r.center.y))),2.0f)/(pow(r.size.width*(float)BODY_SIZE/100.0f,2.0f))) + (pow(((sin(a)*(p.x - r.center.x)) - (cos(a)*(p.y - r.center.y))),2.0f)/(pow(r.size.height*(float)BODY_SIZE/100.0f,2.0f)))) <= 1.0f){
					cv::circle(frame,p,10,200,1);
					nbP++;
				}
				else{
					handsTemp.push_back(p);
					cv::circle(frame,p,20,200,1);
				}
			} 
			// check if the ellipse is the one associated with contours ( check centers ?)
			// Plus de La moiti� des points est a l interieur de l ellipse
			if(nbP >= MIN_NB_CONT){
				//cout << "Body found" << endl;

				ellipse(frame, r.center, r.size*(float)BODY_SIZE/100.0f, r.angle, 0, 360, Scalar(255), 1, LINE_AA); // ellipse is the one linked with contours[i] or res

				for(int h = 0; h < handsTemp.size();h++)
					hands.push_back(handsTemp[h]);

			}
		}

	}

		
	// hands get all depth data close to the hand estimated position
	for(int h = 0; h < hands.size();h++){
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
			for(int y = minY; y < maxY;y++)
				if ((buffer[width*y+(width-1-x)] > 0)&&(buffer[width*y+(width-1-x)] < hauteurCamera-fountainHeight)){
					xCpt += x;
					yCpt += y;
					dCpt += buffer[width*y+(width-1-x)];
					nbCpt++;
				}
		xCpt = (float)xCpt / (float)nbCpt;
		yCpt = (float)yCpt / (float)nbCpt;
		dCpt = (float)dCpt / (float)nbCpt;

		// convert to real world and add to _hands
		CameraSpacePoint cameraPoint = { 0 };
		DepthSpacePoint depthPoint = { 0 };
		depthPoint.X = static_cast<float>(xCpt); 
		depthPoint.Y = static_cast<float>(yCpt); 
		UINT16 depth = dCpt;

		pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint,dCpt,&cameraPoint);

		currentHands.push_back(Point3D(cameraPoint.X,cameraPoint.Y,cameraPoint.Z));
	}

	// supprimer les mains trop proches (< HANDS_PROXIMITY)
	vector<int> toDel;
	int index = 0;
	for(vector<Point3D>::iterator pit = currentHands.begin();pit != currentHands.end();pit++){
		bool toD = false;
		for(vector<Point3D>::iterator sit = pit+1;sit != currentHands.end();sit++){
			if(pit->distanceTo(*sit) <= HANDS_PROXIMITY)
				toD = true;
		}
		if(toD) toDel.push_back(index);
		index++;
	}

	for(vector<int>::reverse_iterator rit = toDel.rbegin();rit != toDel.rend();rit++)
		currentHands.erase(currentHands.begin()+ *rit);


	//cout << "Nb hands = " << currentHands.size() << endl;


	// associer les mains a un id et d'une frame a l autre les reassocier avec cet id ( par distance ?)
	if(_hands.empty()){
		for(vector<Point3D>::iterator pit = currentHands.begin();pit != currentHands.end();pit++)
			_hands[_newId++] = *pit;
	}
	else
	{
		set<int> ids;
		for(map<int,Point3D>::iterator mit = _hands.begin();mit != _hands.end();mit++)
			ids.insert(mit->first);

		for(vector<Point3D>::iterator pit = currentHands.begin();pit != currentHands.end();pit++){
			int idToSyncWith = -1;
			float dist = HAND_DISTANCE_FOR_ASSOCIATION2;
			for(set<int>::iterator iit = ids.begin();iit != ids.end();iit++){
				if(_hands[*iit].distanceTo(*pit) < dist){
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

		if(!ids.empty()){
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

	//cv::imshow("ellipses_test", hellipse);
	cv::imshow("source_control", frame);

	cv::waitKey(1);	

	if( pDepthFrame != NULL ){
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
