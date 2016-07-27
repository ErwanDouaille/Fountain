/*
 * DirectionObserver.h
 *
 *      Author: nicolas
 */

#ifndef DIRECTIONOBSERVER_H_
#define DIRECTIONOBSERVER_H_

#include "LgObserver.h"

using namespace lg;

#define DIRECTION_AXIS_X 0
#define DIRECTION_AXIS_Y 1
#define DIRECTION_AXIS_Z 2


// MINIMUM : difference of position must stay below this value to get a proba 1
#define DIRECTION_TYPE_MINIMUM 0
// MINIMUM : difference of position above this value will get a proba 1, under will be mapped between 0 and 1
#define DIRECTION_TYPE_MAXIMUM 1
#define DIRECTION_TYPE_IGNORE 2

class DirectionObserver: public Observer {
private:
	int _typeX,_typeY,_typeZ;

	int _moveToX,_moveToY,_moveToZ;

	float goToDirection(int direction,float difference);
public:

	/*
	 * Observe if a hand move to a relative position to the user
	 * By example if you want the front of the user you can use [0,0,-1]
	 * or if the hand goes up [0,1,0]
	 * or to the right [1,0,0]
	 */
	DirectionObserver();
	virtual ~DirectionObserver();


	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	set<string> need() const; 

	void observeAxis(int axis,int type, int value);
};

#endif /* DIRECTIONOBSERVER_H_ */
