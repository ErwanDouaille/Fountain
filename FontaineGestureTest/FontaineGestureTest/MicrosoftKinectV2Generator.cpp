#include "MicrosoftKinectV2Generator.h"

using namespace lg;

map<string,int> g3DCounts;

MicrosoftKinectV2Generator::MicrosoftKinectV2Generator(string name) : Generator(name)
{
	// Initialize attributes
	_xPosition = 0;
	_yPosition = 0;
	_zPosition = 0;

	_pitch = 0.0;
	_roll = 0.0;
	_yaw = 0.0;

	_inputMode = LG_KINECTGENERATORV2_INPUT_ANY;
	_outputMode = LG_KINECTGENERATORV2_OUTPUT_NONE;

	_3DMode = true;
	_3DGroupType = "KINECTV2_SKELETON";

	_2DMode = false;
	_2DGroupType = "KINECTV2_SKELETON";

	_mirrorMode = false;

	_confidenceMin = 0.0;
	_depthMax = INT_MAX;

	_width = 1920;
	_height = 1080;

	_frequency = 30;

	_useElevation = true;

	g3DCounts.clear();
}

MicrosoftKinectV2Generator::~MicrosoftKinectV2Generator(void)
{
}

bool MicrosoftKinectV2Generator::start()
{
	
	if(!_environment)
	{
		cout << "KinectGenerator Error : Parent environment has not been set." << endl;
		return false;
	}

	if (!initKinect()) return false;

	//WaitForSingleObject(h_skeleton, INFINITE);
	return true;
}

bool MicrosoftKinectV2Generator::stop()
{
	if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE) cout << "Stop Generator" << endl;
	return true;
}

Node* MicrosoftKinectV2Generator::clone(string cloneName) const
{
	// A KinectGenerator cannot be cloned, it is better to return the current instance 
	// Another KinectGenerator can be created for openning another device or another file
	return (Node*)this;
}

template <class T> void SafeRelease(T *ppT)
{
	if (ppT)
	{
		(ppT)->Release();
		ppT = NULL;
	}
}

bool MicrosoftKinectV2Generator::initKinect() 
{
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);

	if (FAILED(hr)) return false;

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader & color image
		IBodyFrameSource* pBodyFrameSource = NULL; // skeleton
		IColorFrameSource* pColorSource = NULL; // color image

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr)) hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		else return false;
		if (SUCCEEDED(hr)) hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		else return false;
		if (SUCCEEDED(hr)) hr = m_pKinectSensor->get_ColorFrameSource( &pColorSource );
		else return false;
		if (SUCCEEDED(hr)) hr = pColorSource->OpenReader( &pColorReader );
		else return false;

		SafeRelease(pColorSource);
		SafeRelease(pBodyFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		cout << " No Kinect Found!" << endl;;
		return false;
	}
	m_pKinectSensor->get_CoordinateMapper(&pCoordinateMapper );

	return SUCCEEDED(hr);
}

double computeAngleBetweenJoints(Point3D baseJoint, Point3D jointA, Point3D jointB)
{
	Point3D normalizeJointA = Point3D(jointA.getX() - baseJoint.getX(), 
		jointA.getY() - baseJoint.getY(), 
		jointA.getZ() - baseJoint.getZ());
	Point3D normalizeJointB = Point3D(jointB.getX() - baseJoint.getX(), 
		jointB.getY() - baseJoint.getY(), 
		jointB.getZ() - baseJoint.getZ());
	double dot = normalizeJointA.getX()*normalizeJointB.getX() + normalizeJointA.getY()*normalizeJointB.getY() + normalizeJointA.getZ()*normalizeJointB.getZ();
	double lenSq1 = normalizeJointA.getX()*normalizeJointA.getX() + normalizeJointA.getY()*normalizeJointA.getY() + normalizeJointA.getZ()*normalizeJointA.getZ();
	double lenSq2 = normalizeJointB.getX()*normalizeJointB.getX() + normalizeJointB.getY()*normalizeJointB.getY() + normalizeJointB.getZ()*normalizeJointB.getZ();
	double value = abs(acos(dot/sqrt(lenSq1 * lenSq2)));
	return value;
}

void MicrosoftKinectV2Generator::RegisterBody(INT64 nTime, int nBodyCount, IBody** ppBodies, map<string,Group3D*>& g3D,map<string,Group2D*>& g2D)
{
	// The skeleton is not tracked in every successive frame, so use g3DCounts to count the number of frames
	map<string,Group3D*> g3D_copy = g3D;
	// Delete when user leaves
	for(map<string,Group3D*>::iterator mit = g3D_copy.begin();mit != g3D_copy.end();mit++)
	{
		if(mit->second->getType() == _3DGroupType)
		{
			string groupID = mit->first;
			for (int i = 0; i < nBodyCount; i++)
			{
				IBody* pBody = ppBodies[i];
				char buffer [33];
				UINT64 trackingId;
				pBody->get_TrackingId(&trackingId);
				sprintf_s(buffer,"%lu", trackingId);
				string myID2 = buffer;
				if(myID2 == groupID){ g3DCounts[groupID] = 0;}
				else{ g3DCounts[groupID] += 1;}
			}

			if(g3DCounts[groupID] >  100)
			{
				delete g3D[groupID];
				g3D.erase(groupID);
				if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
					cout << "*************deleted successfully*******"  << groupID << endl;
			}
		}
	}

	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			HRESULT hr = pBody->get_IsTracked(&bTracked);
			char buffer [33];
			UINT64 trackingId;
			pBody->get_TrackingId(&trackingId);
			sprintf_s(buffer,"%lu", trackingId);
			string myID = buffer;

			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count]; 
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{	
					Point3D pointJoint;
					double angle = 0.0;

					// LEFT HAND
					if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HAND))) 
					{
						pointJoint = getRepositionedPoint(joints[JointType_HandLeft].Position);
						angle = computeAngleBetweenJoints(pointJoint, 
							getRepositionedPoint(joints[JointType_HandTipLeft].Position), 
							getRepositionedPoint(joints[JointType_WristLeft].Position));
						updateData(_environment, g3D, myID, _3DGroupType, "left_hand", LG_ORIENTEDPOINT3D_LEFT_HAND, _timestamp,
							OrientedPoint3D(Point3D(pointJoint.getX()*1000.0f,pointJoint.getY()*1000.0f,pointJoint.getZ()*1000.0f), Orientation3D(angle, 0.0, 0.0), 1.0, 0.0));
					}

					// RIGHT HAND
					if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HAND))) 
					{
						pointJoint = getRepositionedPoint(joints[JointType_HandRight].Position);
						angle = computeAngleBetweenJoints(pointJoint, 
							getRepositionedPoint(joints[JointType_WristRight].Position), 
							getRepositionedPoint(joints[JointType_HandTipRight].Position));
						updateData(_environment, g3D, myID, _3DGroupType, "right_hand", LG_ORIENTEDPOINT3D_RIGHT_HAND, _timestamp,
							OrientedPoint3D(Point3D(pointJoint.getX()*1000.0f,pointJoint.getY()*1000.0f,pointJoint.getZ()*1000.0f), Orientation3D(angle, 0.0, 0.0), 1.0, 0.0));
					}

				}
			}
		}
	}
}

Point3D MicrosoftKinectV2Generator::getRepositionedPoint(Point3D old)
{
	Point3D newPos;
	newPos.setX(_xPosition+old.getX());
	newPos.setY(_yPosition+(old.getY()*cos(-_roll*M_PI/180))-(old.getZ()*sin(-_roll*M_PI/180)));
	newPos.setZ(_zPosition+(old.getY()*sin(-_roll*M_PI/180))+(old.getZ()*cos(-_roll*M_PI/180)));
	return newPos;
}

Point3D MicrosoftKinectV2Generator::getRepositionedPoint(CameraSpacePoint old)
{
	Point3D newPos;
	newPos.setX(_xPosition+old.X);
	newPos.setY(_yPosition+(old.Y*cos(-_roll*M_PI/180))-(old.Z*sin(-_roll*M_PI/180)));
	newPos.setZ(_zPosition+(old.Y*sin(-_roll*M_PI/180))+(old.Z*cos(-_roll*M_PI/180)));
	return newPos;
}

set<string> MicrosoftKinectV2Generator::produce() const
{
	set<string> produce;

	if(_3DMode)
	{
		if(isGenerated(LG_ORIENTEDPOINT3D_HEAD)) produce.insert(LG_ORIENTEDPOINT3D_HEAD);
		if(isGenerated(LG_ORIENTEDPOINT3D_NECK)) produce.insert(LG_ORIENTEDPOINT3D_NECK);
		if(isGenerated("LG_ORIENTEDPOINT3D_SPINESHOULDER")) produce.insert("LG_ORIENTEDPOINT3D_SPINESHOULDER");
		if(isGenerated("LG_ORIENTEDPOINT3D_SPINEMID")) produce.insert("LG_ORIENTEDPOINT3D_SPINEMID");
		if(isGenerated("LG_ORIENTEDPOINT3D_SPINEBASE")) produce.insert("LG_ORIENTEDPOINT3D_SPINEBASE");
		if(isGenerated("LG_ORIENTEDPOINT3D_LEFT_HAND_TIP")) produce.insert("LG_ORIENTEDPOINT3D_LEFT_HAND_TIP");
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HAND)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_HAND);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_THUMB)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_THUMB);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_WRIST)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_WRIST);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_ELBOW)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_ELBOW);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_SHOULDER)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_SHOULDER);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_SHOULDER)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_SHOULDER);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_ELBOW)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_ELBOW);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_WRIST)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_WRIST);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HAND)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_THUMB)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_THUMB);
		if(isGenerated("LG_ORIENTEDPOINT3D_RIGHT_HAND_TIP")) produce.insert("LG_ORIENTEDPOINT3D_RIGHT_HAND_TIP");
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_FOOT)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_FOOT);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_ANKLE)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_ANKLE);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_KNEE)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_KNEE);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HIP)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_HIP);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HIP)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HIP);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_KNEE)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_KNEE);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_ANKLE)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_ANKLE);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_FOOT)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_FOOT);
	}

	if(_2DMode)
	{
		if(isGenerated("LG_ORIENTEDPOINT2D_HEAD")) produce.insert("LG_ORIENTEDPOINT2D_HEAD");
		if(isGenerated("LG_ORIENTEDPOINT2D_NECK")) produce.insert("LG_ORIENTEDPOINT2D_NECK");
		if(isGenerated("LG_ORIENTEDPOINT2D_SPINESHOULDER")) produce.insert("LG_ORIENTEDPOINT2D_SPINESHOULDER");
		if(isGenerated("LG_ORIENTEDPOINT2D_SPINEMID")) produce.insert("LG_ORIENTEDPOINT2D_SPINEMID");
		if(isGenerated("LG_ORIENTEDPOINT2D_SPINEBASE")) produce.insert("LG_ORIENTEDPOINT2D_SPINEBASE");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HAND_TIP")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_HAND_TIP");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HAND")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_HAND");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_THUMB")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_THUMB");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_WRIST")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_WRIST");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_ELBOW")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_ELBOW");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_SHOULDER")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_SHOULDER");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_SHOULDER")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_SHOULDER");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_ELBOW")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_ELBOW");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_WRIST")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_WRIST");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HAND")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_HAND");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_THUMB")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_THUMB");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HAND_TIP")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_HAND_TIP");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_FOOT")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_FOOT");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_ANKLE")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_ANKLE");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_KNEE")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_KNEE");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HIP")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_HIP");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HIP")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_HIP");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_KNEE")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_KNEE");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_ANKLE")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_ANKLE");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_FOOT")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_FOOT");
	}
	return produce;
}

bool MicrosoftKinectV2Generator::generate(map<string,Group3D*>& g3D,map<string,Group2D*>& g2D,map<string,Group1D*>&,map<string,GroupSwitch*>&)
{
	if (!m_pBodyFrameReader) return false;

	IBodyFrame* pBodyFrame = NULL;
	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
	while(!SUCCEEDED(hr)) hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	INT64 nTime = 0;
	hr = pBodyFrame->get_RelativeTime(&nTime);
	IBody* ppBodies[BODY_COUNT] = {0};

	if (SUCCEEDED(hr)) hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
	if (SUCCEEDED(hr)) RegisterBody(nTime, BODY_COUNT, ppBodies,g3D,g2D);

	for (int i = 0; i < _countof(ppBodies); ++i) SafeRelease(ppBodies[i]);
	SafeRelease(pBodyFrame);
	return SUCCEEDED(hr);
}