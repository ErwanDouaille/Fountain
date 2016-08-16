#pragma once
#include <stdio.h>
#include <iostream>

#include "LgPoint3D.h"

using namespace std;
using namespace lg;

class DualHandContainer
{
private: 
	string _idFirst, _idSecond;
	Point3D _positionFirst, _positionSecond;
	int _timestamp;

public:
	DualHandContainer(string idFirst, Point3D positionFirst, string idSecond, Point3D positionSecond, int timestamp);
	~DualHandContainer(void);
	string getIdFirst() const {return _idFirst;}
	string getIdSecond() const {return _idSecond;}
	Point3D getPositionFirst() const {return _positionFirst;}
	Point3D getPositionSecond() const {return _positionSecond;}
	void setPositionFirst(Point3D newPosition) { _positionFirst = newPosition;}
	void setPositionSecond(Point3D newPosition) { _positionSecond= newPosition;}
	int getTimestamp() {return _timestamp;}
	void setTimeStamp(int timeStamp) { _timestamp = timeStamp;}

};