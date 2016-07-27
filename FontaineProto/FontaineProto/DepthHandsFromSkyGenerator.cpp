#include "DepthHandsFromSkyGenerator.h"


DepthHandsFromSkyGenerator::DepthHandsFromSkyGenerator(string name) : Generator(name)
{
	fountainHeight = 1000;

	h = 424;
	w = 512;

	cntPix = 0;


	bufferSize = 0;
	buffer = nullptr;

	counter = 1;

	_newId = 0;
}


DepthHandsFromSkyGenerator::~DepthHandsFromSkyGenerator(void)
{
}

Node* DepthHandsFromSkyGenerator::clone(string cloneName) const
{

	// A ColorTrackerv2 cannot be cloned, it is better to return the current instance 
	// Another ColorTrackerv2 can be created for openning another device or another file
	return (Node*)this;
}

bool DepthHandsFromSkyGenerator::start()
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
	pDepthDescription->get_Width( &w ); 
	pDepthDescription->get_Height( &h ); 

	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			_done.push_back(false);
			_ids.push_back(0);
			_firstFireDone.push_back(false);
			_secondFireDone.push_back(false);
		}
	}

	cv::namedWindow("source_control", CV_WINDOW_AUTOSIZE );

	frame.create(h,w, CV_8UC3);


	return true;
}

bool DepthHandsFromSkyGenerator::stop()
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

bool DepthHandsFromSkyGenerator::generate(map<string,Group3D*>& g3D,map<string,Group2D*>& g2D,map<string,Group1D*>&,map<string,GroupSwitch*>&)
{

	IDepthFrame* pDepthFrame = nullptr;
	HRESULT hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	while(!SUCCEEDED(hr)){
		//cout << " Error : Acquire failed." << endl;
		Sleep(50);
		hr = pDepthReader->AcquireLatestFrame( &pDepthFrame );
	}


	hr = pDepthFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );

	int currentId = 1;
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			if(!_done[w*y+x]){
				if(labelling(x,y,currentId))
					currentId++;
			}
		}	
	}

	//inRange(frame, 100,255, frame); 
	//morphological opening (removes small objects from the foreground)
	/*erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );
	dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
	//morphological closing (removes small holes from the foreground)
	dilate( frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) ); 
	erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)) );*/


	currentHands.clear();


	// find centers
	map<int,int>::iterator xit = _xIds.begin();
	map<int,int>::iterator yit = _yIds.begin();
	for(map<int,int>::iterator cit = _cptIds.begin();cit != _cptIds.end();cit++,xit++,yit++){
		int xCenter = (float)xit->second/(float)cit->second;
		int yCenter = (float)yit->second/(float)cit->second;
		float ray = cit->second/250; // TODO write a function that estimate a ray from the number of pixels in this blob
		// draw a circle at that position with the ray
		//circle(frame,Point(xCenter,yCenter),ray,Scalar(255,0,0),2);

		if(cit->second>TOTAL_PIXELS_THRESHOLD){

			// TODO convex hull ?sur vector

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

	}

	// associer les mains a un id et d'une frame a l autre les reassocier avec cet id ( par distance ?)
	currentHands;
	_hands;
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
			float dist = HAND_DISTANCE_FOR_ASSOCIATION;
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
			if(_hands.count(stoi(git->first)-_id) < 1) toDel.insert(git->first);
		for(set<string>::iterator dit = toDel.begin();dit != toDel.end();dit++)
			g3D.erase(*dit);

		for(map<int,Point3D>::iterator hit = _hands.begin();hit != _hands.end();hit++)
			updateData(_environment,g3D,to_string(_id+hit->first),"Kinectv2_Up_RightHands",LG_ORIENTEDPOINT3D_RIGHT_HAND,LG_ORIENTEDPOINT3D_RIGHT_HAND,_timestamp,OrientedPoint3D(Point3D(1000.0*hit->second.getX(),1000.0*hit->second.getY(),1000.0*hit->second.getZ()),Point3D(),1.0,1.0));
	}



	for(int i=0;i<w*h;i++) _done[i] = false;
	_xIds.clear();
	_yIds.clear();
	_cptIds.clear();

	cv::imshow("source_control", frame);

	cv::waitKey(1);	

	if( pDepthFrame != NULL ){
		pDepthFrame->Release();
		pDepthFrame = NULL;
	}












































	return true;
}

set<string> DepthHandsFromSkyGenerator::produce() const
{
	set<string> produce;
	produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);

	return produce;
}


bool DepthHandsFromSkyGenerator::secondFire(int x,int y, int xCenter,int yCenter, int ray){
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


void DepthHandsFromSkyGenerator::firstFire(int x,int y, int xCenter,int yCenter, int ray){
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


					// conversion vers l'espace reel
					CameraSpacePoint cameraPoint = { 0 };
					DepthSpacePoint depthPoint = { 0 };
					depthPoint.X = static_cast<float>(farestX); 
					depthPoint.Y = static_cast<float>(farestY); 
					UINT16 depth = buffer[farestY*w + farestX];

					pCoordinateMapper->MapDepthPointToCameraSpace(depthPoint,depth,&cameraPoint);

					currentHands.push_back(Point3D(cameraPoint.X,cameraPoint.Y,cameraPoint.Z));
				}
				counter++;
			}
		}
		_secondFireDone[w*y+x] = true;
	}
}


bool DepthHandsFromSkyGenerator::labelling(int x,int y,int id){
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
