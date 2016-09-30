#pragma once

#include "LgObserver.h"

using namespace lg;

class CmdGlobObserver :
	public Observer
{
private:
	float _speed, _amplitude;
	string _cmdName;
	Point3D _direction;
	int controlPosIteration;
	int controlPosLastTimestamp;

	bool recognition(HOrientedPoint3D* rh, HOrientedPoint3D* srh);
	bool oneHandRecognition(HOrientedPoint3D* rh);

public:
	CmdGlobObserver(void);
	~CmdGlobObserver(void);
	
	Node* clone(string) const;

	bool start();
	bool stop();

	bool observe(map<string,Group3D*>,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>);

	void setSpeed(float value) {_speed = value;}
	void setAmplitude(float value) {_amplitude = value;}
	void setCmdName(string value) {_cmdName = value;}
	void setDirection(Point3D value) {_direction = value;}
	
	float getSpeed() {return _speed;}
	float getAmplitude() {return _amplitude;}
	
	Point3D getDirection() {return _direction;}

	string getCmdName() {return _cmdName;}
	
	set<string> need() const; 
};

