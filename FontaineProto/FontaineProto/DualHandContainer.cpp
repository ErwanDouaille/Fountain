#include "DualHandContainer.h"


DualHandContainer::DualHandContainer(string idFirst, Point3D positionFirst, string idSecond, Point3D positionSecond, int timestamp)
{
	_idFirst = idFirst;
	_idSecond = idSecond;
	_positionFirst = positionFirst;
	_positionSecond = positionSecond;
	_timestamp = timestamp;
}

DualHandContainer::~DualHandContainer(void)
{
}
