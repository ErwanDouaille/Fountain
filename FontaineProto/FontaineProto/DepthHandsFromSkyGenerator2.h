#pragma once

// distance minimum d'association de mains
#define HAND_DISTANCE_FOR_ASSOCIATION2 150
#define BIG_EROSION 50
#define SMALL_EROSION 10
#define HANDS_PIXELS_WIDTH 35
#define MIN_NB_CONT 2
#define APPROX_PREC 40
#define HANDS_PROXIMITY 0
#define MIN_BLOB_SIZE 3000

#include "Kinect.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


#include "LgGenerator.h"

using namespace cv;
using namespace std;
using namespace lg;

class DepthHandsFromSkyGenerator2:
	public Generator
{
private:

	int hauteurCamera;
	int bodySize;
	int fountainHeight;
	int height;
	int width;
	int _newId;
	
	unsigned int bufferSize;
	
	unsigned short* buffer;

	float blasterWidth;
	
	vector<Point3D> currentHands;
	vector<vector<Point> > contours;
	vector<RotatedRect> ellipses;

	vector<float> xBlaster;
	vector<float> yBlaster;

	map<int,Point3D> _hands;
	
	Mat frame;
	Mat centers;

	IKinectSensor* m_pKinectSensor;
	IDepthFrameReader* pDepthReader;
	IDepthFrameSource* pDepthFrameSource; 
	ICoordinateMapper* pCoordinateMapper;

	bool setupVariables();
	bool initKinect();

public:
	DepthHandsFromSkyGenerator2(string name);
	~DepthHandsFromSkyGenerator2(void);

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

	void setFountainHeight(int value) {fountainHeight = value;}
	void setBodySize(int value) {bodySize = value;}
	
	int getWidth() {return width;}
	int getHeight() {return height;}
	int getBodySize() {return bodySize;}

	vector<vector<Point> > getContours() {return contours;}
	vector<RotatedRect> getEllipses() {return ellipses;}
	map<int,Point3D> getHands() { return _hands;}

	Mat getFrame() { return frame;}

	ICoordinateMapper* getCoordinateMapper() {return pCoordinateMapper;}
};

