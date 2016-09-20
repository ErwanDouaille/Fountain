#pragma once

#include "LgObserver.h"
#include "DepthHandsFromSkyGenerator2.h"
#include "BlasterObserver.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace lg;
using namespace cv;

class OpenCVDrawObserver :
	public Observer
{
private:
	DepthHandsFromSkyGenerator2* generator;
	BlasterObserver* _blasterObserver;

public:
	OpenCVDrawObserver(void);
	~OpenCVDrawObserver(void);

	Node* clone(string) const;

	bool start();
	bool stop();
	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void setDepthHandsFromSkyGenerator(DepthHandsFromSkyGenerator2* value) { generator = value;}
	void setBlasterObserver( BlasterObserver* value ) { _blasterObserver = value; }
};

