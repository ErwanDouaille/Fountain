#include "KinectGenerator.h"
#include <vector>

// Kinect variables
HANDLE depthStream;
HANDLE rgbStream;
INuiSensor* sensor;
HANDLE h_skeleton;

#define SKELETON_WIDTH 640
#define SKELETON_HEIGHT 480
#define CHANNEL 3

const int width = 640;
const int height = 480;

map<string,int> g3DCounts;

KinectGenerator::KinectGenerator(string name) : Generator(name)
{
	// Initialize attributes

	_xPosition = 0;
	_yPosition = 0;
	_zPosition = 0;

	_pitch = 0.0;
	_roll = 0.0;
	_yaw = 0.0;

	_inputMode = LG_KINECTGENERATOR_INPUT_ANY;
	_outputMode = LG_KINECTGENERATOR_OUTPUT_NONE;

	_3DMode = true;
	_3DGroupType = "KINECT_SKELETON";

	_2DMode = false;
	_2DGroupType = "KINECT_SKELETON";

	_mirrorMode = false;

	_confidenceMin = 0.0;
	_depthMax = INT_MAX;

	_width = 0;
	_height = 0;

	_frequency = 30;

	_waitTime = 100;
	_deleteUserFrameCount = 50;

	g3DCounts.clear();
}

Node* KinectGenerator::clone(string cloneName) const
{
	// A KinectGenerator cannot be cloned, it is better to return the current instance 
	// Another KinectGenerator can be created for openning another device or another file
	return (Node*)this;
}

bool KinectGenerator::start()
{
	if(!_environment){
		cout << "KinectGenerator Error : Parent environment has not be set." << endl;
		return false;
	}

	if (!initKinect()) return false;

	WaitForSingleObject(h_skeleton, INFINITE);
	return true;
}

bool KinectGenerator::initKinect() {

	// Get a working kinect sensor
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 1) return false;
	if (NuiCreateSensorByIndex(0, &sensor) < 0) return false;

	// Initialize sensor
	sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON);
	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
		0,        // Image stream flags, e.g. near mode
		2,        // Number of frames to buffer
		NULL,     // Event handle
		&depthStream);
									//COLOR IMAGE//
	/*	sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
	NUI_IMAGE_RESOLUTION_640x480,                // Image resolution
	0,      // Image stream flags, e.g. near mode
	2,      // Number of frames to buffer
	NULL,   // Event handle
	&rgbStream);*/
	h_skeleton = CreateEvent(NULL, TRUE, FALSE, NULL);
	sensor->NuiSkeletonTrackingEnable(h_skeleton, 0); // NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT for only upper body

	return sensor;
}

bool KinectGenerator::generate(map<string,Group3D*>& g3D,map<string,Group2D*>& g2D,map<string,Group1D*>&,map<string,GroupSwitch*>&)
{
	//system("cls");

	//cout << "Update Generator " << getName().c_str() << " with id " << getID() << " and type " << getType().c_str() << " at " << _timestamp << endl;

	NUI_SKELETON_FRAME SkeletonFrame;
	HRESULT hr = NuiSkeletonGetNextFrame(_waitTime, &SkeletonFrame);



	/*if(hr == S_OK )
		cout << "Yeah"  << endl;
	else
		cout << "No new frame"  << endl;*/
	if(hr != S_OK )
		return false;

	const NUI_TRANSFORM_SMOOTH_PARAMETERS DefaultParams = {0.5f, 0.1f, 0.5f, 0.1f, 0.1f};
	NuiTransformSmooth(&SkeletonFrame, &DefaultParams);

	int videoWidth = 640;
	int videoHeight = 320;

	// Check if a skeleton is found (crucial for Kinect SDK)
	bool bFoundSkeleton = false;
	for (int i = 0; i < NUI_SKELETON_COUNT; i++)
	{	

		if (SkeletonFrame.SkeletonData[i].eTrackingState
			== NUI_SKELETON_TRACKED)
		{
			bFoundSkeleton = true;
		}
	}
	
	// print all groups
	/*
	for(map<string,Group3D*>::iterator mit = g3D.begin();mit != g3D.end();mit++){
		string groupID = mit->first;
		cout << "*******Groups: " << groupID << endl; 
	}
	*/

	// handle user exit
	// The skeleton is not tracked in every successive frame, so use g3DCounts to count the number of frames
	map<string,Group3D*> g3D_copy = g3D;

	for(map<string,Group3D*>::iterator mit = g3D_copy.begin();mit != g3D_copy.end();mit++){

		if(mit->second->getType() == _3DGroupType)
		{
			string groupID = mit->first;

			for (int i = 0; i < NUI_SKELETON_COUNT; i++){
				char buffer [33];
				sprintf(buffer,"%lu",SkeletonFrame.SkeletonData[i].dwTrackingID);
				string myID2 = buffer;
				if(myID2 == groupID){ g3DCounts[groupID] = 0;}
				else{ g3DCounts[groupID] += 1;}
			}

			if(g3DCounts[groupID] >  _deleteUserFrameCount){
				delete g3D[groupID];
				g3D.erase(groupID);
				if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
					cout << "*************deleted successfully*******"  << groupID << endl;
			}
		}
	}

	// if a skeleton is found, create or update groups
	if(bFoundSkeleton){

		for (int i = 0; i < NUI_SKELETON_COUNT; i++){
			int userID = (int)SkeletonFrame.SkeletonData[i].dwTrackingID;
			char buffer [33];
			sprintf(buffer,"%lu",SkeletonFrame.SkeletonData[i].dwTrackingID);
			string myID = buffer;

			if (SkeletonFrame.SkeletonData[i].eTrackingState
				== NUI_SKELETON_TRACKED)
			{
				if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
					cout << " Create/Update User: " << myID << endl;
				
				NUI_SKELETON_BONE_ORIENTATION boneOrientations[NUI_SKELETON_POSITION_COUNT];
				NuiSkeletonCalculateBoneOrientations(&(SkeletonFrame.SkeletonData[i]), boneOrientations);

				Vector4 jointPos;
				NUI_SKELETON_BONE_ORIENTATION & orientation = boneOrientations[NUI_SKELETON_POSITION_HEAD];
				long fx, fy;
				USHORT fz;

				// HEAD
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HEAD];
				orientation = boneOrientations[NUI_SKELETON_POSITION_HEAD];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_HEAD))) 
					updateData(_environment,g3D,myID,_3DGroupType ,"head",LG_ORIENTEDPOINT3D_HEAD,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_HEAD"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","head","LG_ORIENTEDPOINT2D_HEAD",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}
				
				//cout << convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion).getX() << endl;
				//Vector4 q1 = orientation.absoluteRotation.rotationQuaternion;
			
				//cout << " Head Ori" << heading << ";" << attitude << ";" << bank  << endl;
				//cout << " Head Ori  " << heading  << endl;

				//cout << " Head Ori" << orientation.absoluteRotation.rotationQuaternion.x << ";" << orientation.absoluteRotation.rotationQuaternion.y << ";" << orientation.absoluteRotation.rotationQuaternion.z << ";" << orientation.absoluteRotation.rotationQuaternion.w  << endl;
				//cout << " Head " << getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)).getY() << endl;

				// LEFT SHOULDER
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_SHOULDER))) updateData(_environment,g3D,myID,_3DGroupType ,"left_shoulder",LG_ORIENTEDPOINT3D_LEFT_SHOULDER,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_SHOULDER"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_shoulder","LG_ORIENTEDPOINT2D_LEFT_SHOULDER",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT SHOULDER
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_SHOULDER))) updateData(_environment,g3D,myID,_3DGroupType ,"right_shoulder",LG_ORIENTEDPOINT3D_RIGHT_SHOULDER,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_SHOULDER"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_shoulder","LG_ORIENTEDPOINT2D_RIGHT_SHOULDER",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// NECK
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER];
				orientation = boneOrientations[NUI_SKELETON_POSITION_SHOULDER_CENTER];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_NECK))) updateData(_environment,g3D,myID,_3DGroupType ,"neck",LG_ORIENTEDPOINT3D_NECK,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_NECK"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","neck","LG_ORIENTEDPOINT2D_NECK",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT ELBOW
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_ELBOW_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_ELBOW))) updateData(_environment,g3D,myID,_3DGroupType ,"left_elbow",LG_ORIENTEDPOINT3D_LEFT_ELBOW,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_ELBOW"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_elbow","LG_ORIENTEDPOINT2D_LEFT_ELBOW",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT ELBOW
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ELBOW_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_ELBOW_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_ELBOW))) updateData(_environment,g3D,myID,_3DGroupType ,"right_elbow",LG_ORIENTEDPOINT3D_RIGHT_ELBOW,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_ELBOW"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_elbow","LG_ORIENTEDPOINT2D_RIGHT_ELBOW",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT WRIST
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_WRIST_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_WRIST))) updateData(_environment,g3D,myID,_3DGroupType ,"left_wrist",LG_ORIENTEDPOINT3D_LEFT_WRIST,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_WRIST"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_wrist","LG_ORIENTEDPOINT2D_LEFT_WRIST",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT WRIST
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_WRIST_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_WRIST_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_WRIST))) updateData(_environment,g3D,myID,_3DGroupType ,"right_wrist",LG_ORIENTEDPOINT3D_RIGHT_WRIST,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_WRIST"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_wrist","LG_ORIENTEDPOINT2D_RIGHT_WRIST",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT HAND
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_HAND_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HAND))) updateData(_environment,g3D,myID,_3DGroupType ,"left_hand",LG_ORIENTEDPOINT3D_LEFT_HAND,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HAND"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_hand","LG_ORIENTEDPOINT2D_LEFT_HAND",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT HAND
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HAND_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_HAND_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HAND))) updateData(_environment,g3D,myID,_3DGroupType ,"right_hand",LG_ORIENTEDPOINT3D_RIGHT_HAND,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HAND"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_hand","LG_ORIENTEDPOINT2D_RIGHT_HAND",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// TORSO
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SPINE];
				orientation = boneOrientations[NUI_SKELETON_POSITION_SPINE];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_TORSO))) updateData(_environment,g3D,myID,_3DGroupType ,"torso",LG_ORIENTEDPOINT3D_TORSO,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_TORSO"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","torso","LG_ORIENTEDPOINT2D_TORSO",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT ANKLE 
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_ANKLE_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_ANKLE))) updateData(_environment,g3D,myID,_3DGroupType ,"left_ankle",LG_ORIENTEDPOINT3D_LEFT_ANKLE,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_ANKLE"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_ankle","LG_ORIENTEDPOINT2D_LEFT_ANKLE",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT ANKLE
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_ANKLE_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_ANKLE_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_ANKLE))) updateData(_environment,g3D,myID,_3DGroupType ,"right_ankle",LG_ORIENTEDPOINT3D_RIGHT_ANKLE,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_ANKLE"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_ankle","LG_ORIENTEDPOINT2D_RIGHT_ANKLE",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT FOOT
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_FOOT_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_FOOT_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_FOOT))) updateData(_environment,g3D,myID,_3DGroupType ,"left_foot",LG_ORIENTEDPOINT3D_LEFT_FOOT,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_FOOT"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_foot","LG_ORIENTEDPOINT2D_LEFT_FOOT",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT FOOT
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_FOOT_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_FOOT_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_FOOT))) updateData(_environment,g3D,myID,_3DGroupType ,"right_foot",LG_ORIENTEDPOINT3D_RIGHT_FOOT,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_FOOT"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_foot","LG_ORIENTEDPOINT2D_RIGHT_FOOT",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// STOMACH/HIP CENTER
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HIP_CENTER];
				orientation = boneOrientations[NUI_SKELETON_POSITION_HIP_CENTER];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_STOMACH))) updateData(_environment,g3D,myID,_3DGroupType ,"stomach",LG_ORIENTEDPOINT3D_STOMACH,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_STOMACH"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","stomach","LG_ORIENTEDPOINT2D_STOMACH",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT HIP
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HIP_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_HIP_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HIP))) updateData(_environment,g3D,myID,_3DGroupType ,"left_hip",LG_ORIENTEDPOINT3D_LEFT_HIP,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HIP"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_hip","LG_ORIENTEDPOINT2D_LEFT_HIP",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT HIP
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_HIP_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_HIP_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HIP))) updateData(_environment,g3D,myID,_3DGroupType ,"right_hip",LG_ORIENTEDPOINT3D_RIGHT_HIP,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HIP"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_hip","LG_ORIENTEDPOINT2D_RIGHT_HIP",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// LEFT KNEE
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_KNEE_LEFT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_KNEE_LEFT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_LEFT_KNEE))) updateData(_environment,g3D,myID,_3DGroupType ,"left_knee",LG_ORIENTEDPOINT3D_LEFT_KNEE,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_LEFT_KNEE"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","left_knee","LG_ORIENTEDPOINT2D_LEFT_KNEE",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				// RIGHT KNEE
				jointPos = SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_KNEE_RIGHT];
				orientation = boneOrientations[NUI_SKELETON_POSITION_KNEE_RIGHT];
				if(_3DMode&&(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_KNEE))) updateData(_environment,g3D,myID,_3DGroupType ,"right_knee",LG_ORIENTEDPOINT3D_RIGHT_KNEE,_timestamp,OrientedPoint3D(getRepositionedPoint(Point3D(1000.0f*jointPos.x,1000.0f*jointPos.y,1000.0f*jointPos.z)),convertOrientationsToEuler(orientation.absoluteRotation.rotationQuaternion),1.0,0.0));
				if(_2DMode&&(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_KNEE"))){
					NuiTransformSkeletonToDepthImage(jointPos,&fx, &fy,&fz);
					updateData(_environment,g2D,myID,"KINECTV1_MICROSOFTSDK1.8_SKELETON_PROJECTION","right_knee","LG_ORIENTEDPOINT2D_RIGHT_KNEE",_timestamp,OrientedPoint2D(Point2D((int) (fx*2 ) / videoWidth,(int) (fy*2 )/ videoHeight),0.0,0.0,0.0));
				}

				g3DCounts[myID] = 0;

				/*if(g3D.count(myID) ==0){
					//  create g3D[groupID]       
					cout << " Create User: " << myID << endl;
					Group3D* g3 = new Group3D(_environment,myID,_3DGroupType);
					Group2D* g2 = new Group2D(_environment,myID,_3DGroupType);
					//HEAD
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_HEAD,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_HEAD);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"HEAD");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//LEFT SHOULDER
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_SHOULDER_LEFT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_SHOULDER);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_SHOULDER");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//RIGHT SHOULDER
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_SHOULDER_RIGHT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_SHOULDER);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_SHOULDER");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//NECK
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_SHOULDER_CENTER,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_NECK);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"NECK");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//LEFT ELBOW
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_ELBOW_LEFT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_ELBOW);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_ELBOW");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//RIGHT ELBOW
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_ELBOW_RIGHT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_ELBOW);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_ELBOW");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//LEFT WRIST
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_WRIST_LEFT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_WRIST);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_WRIST");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//RIGHT WRIST
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_WRIST_RIGHT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_WRIST);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_WRIST");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//LEFT HAND
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_HAND_LEFT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_LEFT_HAND);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"LEFT_HAND");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//RIGHT HAND
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_HAND_RIGHT,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_RIGHT_HAND);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"RIGHT_HAND");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					//TORSO
					{
					string myJoint;
					itoa (NUI_SKELETON_POSITION_SPINE,buffer,10);
					myJoint = buffer;
					HOrientedPoint3D* ho1 = new HOrientedPoint3D(_environment,myJoint,LG_ORIENTEDPOINT3D_TORSO);
					ho1->updateHistoric(_timestamp,OrientedPoint3D(Point3D(),Orientation3D(),0,0.0));
					g3->addElement(myJoint,ho1);
					HOrientedPoint2D* hv1 = new HOrientedPoint2D(_environment,myJoint,"TORSO");
					hv1->updateHistoric(_timestamp,OrientedPoint2D(Point2D(),0,0.0,0.0));
					g2->addElement(myJoint,hv1);
					}
					g3D[myID] = g3;
					g2D[myID] = g2;
					g3DCounts[myID] = 0;
				}
				else{
					//   update g3D[groupID]       
					Group3D* g3 = g3D.at(myID);
					if(!g3) return false;
					Group2D* g2 = g2D[myID];
					if(!g2) return false;
					cout << "Update user " << myID.c_str() << endl;
					//
				}*/
			}
		}
	}
		
	return true;
}

bool KinectGenerator::stop()
{
	if(_environment->getVerboseLevel() != LG_ENV_VERBOSE_MUTE)
		cout << "Stop Generator" << endl;

	return true;
}

set<string> KinectGenerator::produce() const
{
	set<string> produce;
	
	if(_3DMode){
		if(isGenerated(LG_ORIENTEDPOINT3D_HEAD)) produce.insert(LG_ORIENTEDPOINT3D_HEAD);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_ELBOW)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_ELBOW);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_WRIST)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_WRIST);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HAND)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_HAND);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_SHOULDER)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_SHOULDER);
		if(isGenerated(LG_ORIENTEDPOINT3D_NECK)) produce.insert(LG_ORIENTEDPOINT3D_NECK);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_ELBOW)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_ELBOW);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HAND)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_SHOULDER)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_SHOULDER);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_WRIST)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_WRIST);
		if(isGenerated(LG_ORIENTEDPOINT3D_TORSO)) produce.insert(LG_ORIENTEDPOINT3D_TORSO);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_ANKLE)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_ANKLE);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_ANKLE)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_ANKLE);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_FOOT)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_FOOT);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_FOOT)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_FOOT);
		if(isGenerated(LG_ORIENTEDPOINT3D_STOMACH)) produce.insert(LG_ORIENTEDPOINT3D_STOMACH);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_HIP)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_HIP);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_HIP)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_HIP);
		if(isGenerated(LG_ORIENTEDPOINT3D_LEFT_KNEE)) produce.insert(LG_ORIENTEDPOINT3D_LEFT_KNEE);
		if(isGenerated(LG_ORIENTEDPOINT3D_RIGHT_KNEE)) produce.insert(LG_ORIENTEDPOINT3D_RIGHT_KNEE);
	}

	
	if(_2DMode){
		if(isGenerated("LG_ORIENTEDPOINT2D_HEAD")) produce.insert("LG_ORIENTEDPOINT2D_HEAD");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_ELBOW")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_ELBOW");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_WRIST")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_WRIST");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HAND")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_HAND");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_SHOULDER")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_SHOULDER");
		if(isGenerated("LG_ORIENTEDPOINT2D_NECK")) produce.insert("LG_ORIENTEDPOINT2D_NECK");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_ELBOW")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_ELBOW");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HAND")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_HAND");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_SHOULDER")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_SHOULDER");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_WRIST")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_WRIST");
		if(isGenerated("LG_ORIENTEDPOINT2D_TORSO")) produce.insert("LG_ORIENTEDPOINT2D_TORSO");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_ANKLE")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_ANKLE");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_ANKLE")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_ANKLE");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_FOOT")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_FOOT");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_FOOT")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_FOOT");
		if(isGenerated("LG_ORIENTEDPOINT2D_STOMACH")) produce.insert("LG_ORIENTEDPOINT2D_STOMACH");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_HIP")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_HIP");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_HIP")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_HIP");
		if(isGenerated("LG_ORIENTEDPOINT2D_LEFT_KNEE")) produce.insert("LG_ORIENTEDPOINT2D_LEFT_KNEE");
		if(isGenerated("LG_ORIENTEDPOINT2D_RIGHT_KNEE")) produce.insert("LG_ORIENTEDPOINT2D_RIGHT_KNEE");
	}


	return produce;
}

KinectGenerator::~KinectGenerator(void)
{
}

Point3D KinectGenerator::getRepositionedPoint(Point3D old){
	Point3D newPos;

	newPos.setX(_xPosition+old.getX());
	newPos.setY(_yPosition+(old.getY()*cos(-_roll*M_PI/180))-(old.getZ()*sin(-_roll*M_PI/180)));
	newPos.setZ(_zPosition+(old.getY()*sin(-_roll*M_PI/180))+(old.getZ()*cos(-_roll*M_PI/180)));


	return newPos;
}

Orientation3D KinectGenerator::convertOrientationsToEuler(Vector4 q1){

	float heading,attitude,bank;
	float test = q1.x*q1.y + q1.z*q1.w;
	if (test > 0.499f) { // singularity at north pole
		heading = 2.0f * atan2(q1.x,q1.w);
		attitude = M_PI/2;
		bank = 0;
	}
	else{
		if (test < -0.499) { // singularity at south pole
			heading = -2 * atan2(q1.x,q1.w);
			attitude = - M_PI/2;
			bank = 0;
		}
		else{
			double sqx = q1.x*q1.x;
			double sqy = q1.y*q1.y;
			double sqz = q1.z*q1.z;
			heading = atan2((float)(2.0f*q1.y*q1.w-2.0f*q1.x*q1.z) ,(float) (1.0f - 2.0f*sqy - 2.0f*sqz));
			attitude = asin(2.0f*test);
			bank = atan2((float)(2.0f*q1.x*q1.w-2.0f*q1.y*q1.z) ,(float)( 1.0f - 2.0f*sqx - 2.0f*sqz));
		}
	}

	return Orientation3D(attitude,heading,bank);
}