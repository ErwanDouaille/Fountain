#pragma once

#define _USE_MATH_DEFINES

#include "LgGenerator.h"

#include <Windows.h>
#include <Ole2.h>

#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <string.h> 

using namespace lg;

//! Value for defining no output image as parameters
#define LG_KINECTGENERATOR_OUTPUT_NONE 0x00
//! Value for defining RGB image as parameter
#define LG_KINECTGENERATOR_OUTPUT_RGB 0x01
//! Value for defining IR image as parameter
#define LG_KINECTGENERATOR_OUTPUT_IR 0x02
//! Value for defining DEPTH image as parameter
#define LG_KINECTGENERATOR_OUTPUT_DEPTH 0x04
//! Value for defining users map as parameter
#define LG_KINECTGENERATOR_OUTPUT_USERS 0x08
//! Value for defining any device as input
#define LG_KINECTGENERATOR_INPUT_ANY "LG_KINECTGENERATOR_INPUT_ANY"

struct LgImage{
	int width;
	int height;
	// RGB IR ...
};

/*!
	* \class KinectGenerator
	* \brief Generator for connecting Microsoft Kinect through Microsoft Kinect SDK
	*
	* Generate millimeters
	*
	*
	* For using it : 
	* include C:\Program Files\Microsoft SDKs\Kinect\v1.8\inc
	* lib directory : C:\Program Files\Microsoft SDKs\Kinect\v1.8\lib\x86
	* lib : Kinect10.lib
	*
	*
	*
*/
class KinectGenerator : public Generator
{

private:
		//! Translation on the X axis to add to the data (position of the camera)
	int _xPosition;

	//! Translation on the Y axis to add to the data (position of the camera)
	int _yPosition;

	//! Translation on the Z axis to add to the data (position of the camera)
	int _zPosition;

	//! Rotation on the X axis to add to the data (orientation of the camera)
	float _pitch;
	
	//! Rotation on the Y axis to add to the data (orientation of the camera)
	float _roll;
	
	//! Rotation on the Z axis to add to the data (orientation of the camera)
	float _yaw;

	//! Number of frames to wait before deleting a user which doesn't appear in consecutive frames
	int _deleteUserFrameCount;

	//! Time (in ms) to wait a new frame
	int _waitTime;

	//! Define if the data are retrieved from a Kinect or a file
	string _inputMode;
	
	//! Define if one or several images must be put in parameters
	int _outputMode;

	//! Enable/Disable the generation of 3D data
	bool _3DMode;
	
	//! String representing the type for 3D data 
	string _3DGroupType;

	//! Enable/Disable the generation of 2D data
	bool _2DMode;
	
	//! String representing the type for 2D data
	string _2DGroupType;

	//! Enable for inverting right/left
	bool _mirrorMode;

	//! Threshold minimum for accepting the value of a joint 
	float _confidenceMin;
	
	//! Threshold maximum for accepting values of joints
	int _depthMax;

	//! Width of the output images
	int _width;

	//! Height of the output images
	int _height;

	//! Frequency of updates
	int _frequency;

	/*!
	* \brief Rotate and translate the point according to stored data
	* \param[in] Point3D : point3D in Kinect coordinates
	* \return a Point3D in global coordinates
	*/
	Point3D getRepositionedPoint(Point3D);

	/*!
	* \brief Convert a quaternion in a Vector4 to a Orientation3D
	* \param[in] Vector4 : quaternion 
	* \return orientation3D en radians
	*/
	Orientation3D convertOrientationsToEuler(Vector4);


public:
	KinectGenerator(string name);
	~KinectGenerator(void);

	bool initKinect(void);

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

	////defaults
	/*!
	* \brief Set the camera position or translations that should be applied to data before they are added in the Group
	* \param[in] x : translation on the X axis
	* \param[in] y : translation on the Y axis
	* \param[in] z : translation on the Z axis
	*/
	void setCameraPosition(int x, int y, int z) {_xPosition = x;_yPosition = y;_zPosition = z;}

	/*!
	* \brief Get the applied translation on the X axis
	* \return Translation on the X axis
	*/
	int getCameraX() {return _xPosition;}

	/*!
	* \brief Get the applied translation on the Y axis
	* \return Translation on the Z axis
	*/
	int getCameraY() {return _yPosition;}
	
	/*!
	* \brief Get the applied translation on the Z axis
	* \return Translation on the Z axis
	*/
	int getCameraZ() {return _zPosition;}

	/*!
	* \brief Set the camera orientations or rotations that should be applied to data before they are added in the Group
	* \param[in] pitch : rotations on the X axis
	* \param[in] roll : rotations on the Y axis
	* \param[in] yaw : rotations on the Z axis
	*/
	void setCameraOrientation(float pitch, float roll, float yaw) {_pitch = pitch;_roll = roll;_yaw = yaw;}


	/*!
	* \brief Get the applied rotation on the X axis (pitch)
	* \return Pitch
	*/
	float getCameraPitch() {return _pitch;}
	
	/*!
	* \brief Get the applied rotation on the Y axis (roll)
	* \return Roll
	*/
	float getCameraRoll() {return _roll;}
	
	/*!
	* \brief Get the applied rotation on the Z axis (yaw)
	* \return Yaw
	*/
	float getCameraYaw() {return _yaw;}

	/*!
	* \brief Set a input mode for OpenNI (device or file)
	* Use LG_KINECTGENERATOR_INPUT_ANY for any Kinect or other for a file
	* \param[in] iMode : a port of a device or a file path
	*/
	void setInputmode(string iMode) {_inputMode = iMode;}
	
	/*!
	* \brief Get the input mode
	* \return The input mode
	*/
	string getInputmode() {return _inputMode;}

	/*!
	* \brief Set a output mode by flags
	* Use LG_KINECTGENERATOR_OUTPUT_NONE or a combination of LG_KINECTGENERATOR_OUTPUT_RGB, LG_KINECTGENERATOR_OUTPUT_IR, LG_KINECTGENERATOR_OUTPUT_DEPTH and LG_KINECTGENERATOR_OUTPUT_USERS with | between
	* A LgImage containing the chosen images will be set as parameter
	* \param[in] oMode : the mode for parameter image
	*/
	void setOutputMode(int oMode) {_outputMode = oMode;}
	
	/*!
	* \brief Get the output mode
	* \return The output mode
	*/
	int getOutputMode() {return _outputMode;}

	/*!
	* \brief Activate the generation of 3D data
	* \param[in] mode3D : enable/disable generation of 3D data
	*/
	void set3DMode(bool mode3D) {_3DMode = mode3D;}
	
	/*!
	* \brief Get the 3D mode
	* \return State of the 3D mode
	*/
	bool get3DMode() {return _3DMode;}

	/*!
	* \brief Activate the generation of 2D data
	* \param[in] mode3D : enable/disable generation of 2D data
	*/
	void set2DMode(bool mode2D) {_2DMode = mode2D;}
	
	/*!
	* \brief Get the 2D mode
	* \return State of the 2D mode
	*/
	bool get2DMode() {return _2DMode;}

	/*!
	* \brief Invert right/left sides
	* \param[in] modeMirror : enable the inversion of left/right
	*/
	void setMirrorMode(bool modeMirror) {_mirrorMode = modeMirror;}
	
	/*!
	* \brief Get the mirror mode
	* \return State of the mirror mode
	*/
	bool getMirrorMode() {return _mirrorMode;}

	/*!
	* \brief Set a minimum for confidence of updated data 
	* If confidence is under the minimum, the joint will not be updated with the associated value
	* \param[in] confidenceMin : the minimum threshold for updating data
	*/
	void setConfidenceMin(float confidenceMin) {_confidenceMin = confidenceMin;}
	
	/*!
	* \brief Get the confidence threshold
	* \return The confidence threshold
	*/
	float getConfidenceMin() {return _confidenceMin;}

	/*!
	* \brief Points further than this threshold will not be updated
	* \param[in] depthMax : distance threshold
	*/
	void setDepthMax(int depthMax) {_depthMax = depthMax;}
	
	/*!
	* \brief Get the distance threshold
	* \return The distance threshold
	*/
	int getDepthMax() {return _depthMax;}

	/*!
	* \brief Set a string for the type of the 3D Groups
	* \param[in] groupType : string which will be used as type for 3D Groups
	*/
	void set3DGroupType(string groupType) {_3DGroupType = groupType;}
	
	/*!
	* \brief Get the string used as type for 3D Groups
	* \return The string used as type for 3D Groups
	*/
	string get3DGroupTypeMode() {return _3DGroupType;}

	/*!
	* \brief Set a string for the type of the 2D Groups
	* \param[in] groupType : string which will be used as type for 2D Groups
	*/
	void set2DGroupType(string groupType) {_2DGroupType = groupType;}
	
	/*!
	* \brief Get the string used as type for 2D Groups
	* \return The string used as type for 2D Groups
	*/
	string get2DGroupTypeMode() {return _2DGroupType;}

	/*!
	* \brief Set width of the output images
	* \param[in] width : the mode for parameter image
	*/
	void setOutputWidth(int width) {_width = width;}
	
	/*!
	* \brief Get the width of the output images
	* \return The width of the output images
	*/
	int getOutputWidth() {return _width;}


	/*!
	* \brief Set height of the output images
	* \param[in] height : the mode for parameter image
	*/
	void setOutputHeight(int height) {_height = height;}
	
	/*!
	* \brief Get the height of the output images
	* \return The height of the output images
	*/
	int getOutputHeight() {return _height;}


	/*!
	* \brief Set the update frequency
	* 
	* \param[in] frequency : the update frequency
	*/
	void setFrequency(int frequency) {_frequency = frequency;}
	
	/*!
	* \brief Get the update frequency
	* \return The update frequency
	*/
	int getFrequency() {return _frequency;}

	/*!
	* \brief Set the time to wait for a new frame from the Kinect
	* \param[in] newWaitTime : time to wait in milliseconds
	*/
	void setWaitingTime(int newWaitTime) {_waitTime = newWaitTime;}
	
	/*!
	* \brief Get the time to wait for a new frame from the Kinect
	* \return time to wait in milliseconds
	*/
	int getWaitingTime() {return _waitTime;}

	/*!
	* \brief Set the number of consecutives frame to wait before deleting a user that doesn't appear on it
	* \param[in] newFrameCount : Number of consecutives frame to wait before deleting a user that doesn't appear on it 
	*/
	void setDeleteUserFrameCount(int newFrameCount) {_deleteUserFrameCount = newFrameCount;}
	
	/*!
	* \brief Get the number of consecutives frame to wait before deleting a user that doesn't appear on it
	* \return Number of consecutives frame to wait before deleting a user that doesn't appear on it 
	*/
	int getDeleteUserFrameCount() {return _deleteUserFrameCount;}

};