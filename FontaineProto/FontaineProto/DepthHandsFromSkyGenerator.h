#pragma once

#define TOTAL_PIXELS_THRESHOLD 1000
#define BLOB_PIXELS_THRESHOLD 500
#define HAND_PIXELS_NUMBER 50

#define HAND_DISTANCE_FOR_ASSOCIATION 200


#include "Kinect.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include "LgGenerator.h"

using namespace cv;
using namespace std;
using namespace lg;

class DepthHandsFromSkyGenerator:
	public Generator
{
private:

	int hauteurCamera;
	int fountainXPosition,fountainYPosition,fountainWidth,blasterWidth;
	vector<int> blasterXPosition,blasterYPosition;

	int fountainHeight;
	Mat frame;

	vector<bool> _done;
	vector<int> _ids;

	vector<bool> _firstFireDone;
	vector<bool> _secondFireDone;


	map<int, int> _xIds;
	map<int, int> _yIds;
	map<int, int> _cptIds;


	int h;
	int w;

	int farestX,farestY;
	float farestDist;
	int cntPix;


	unsigned int bufferSize;
	unsigned short* buffer;

	int counter;

	IKinectSensor* m_pKinectSensor;
	IDepthFrameReader* pDepthReader;
	IDepthFrameSource* pDepthFrameSource; 
	ICoordinateMapper* pCoordinateMapper;

	bool secondFire(int x,int y, int xCenter,int yCenter, int ray);
	void firstFire(int x,int y, int xCenter,int yCenter, int ray);
	bool labelling(int x,int y,int id);





	map<int,Point3D> _hands;
	int _newId;

	vector<Point3D> currentHands;

public:
	DepthHandsFromSkyGenerator(string name);
	~DepthHandsFromSkyGenerator(void);

	/*!
	* \brief Get a pointer to a copy of this KinectGenerator
	* \param[in] cloneName : name for the clone
	* \return A pointer to a copy of this Node
	*/
	Node* clone(string cloneName) const;

		/*!
	* \brief Start the KinectGenerator by initializing streams
	* Initialize RGB, IR or Depth stream depending on the output mode.
	* \return true if success
	*/
	bool start();

	/*!
	* \brief Stop the KinectGenerator by closing streams
	* \return true if success
	*/
	bool stop();

	/*!
	* \brief Generate Groups and data from the device
	* Do not generate 1D and Switch.
	* \param[in] groups3D : Groups of HOrientedPoint3D (Skeletons/Users)
	* \param[in] groups2D : Groups of HOrientedPoint2D (Projections of 3D)
	* \return true if success
	*/
	bool generate(map<string,Group3D*>& groups3D,map<string,Group2D*>& groups2D,map<string,Group1D*>&,map<string,GroupSwitch*>&);

	/*!
	* \brief Return a set of string describing what kind of type this Node produces (adds to output).
	* LG_ORIENTEDPOINT3D_HEAD, LG_ORIENTEDPOINT3D_NECK, LG_ORIENTEDPOINT3D_RIGHT_HAND ... 
	* and/or LG_ORIENTEDPOINT2D_RIGHTHAND ...
	* \return A set of type
	*/
	set<string> produce() const; 

};

