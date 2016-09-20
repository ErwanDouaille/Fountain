#include "OneEuroFilterProcessor.h"

OneEuroFilterProcessor::OneEuroFilterProcessor(string name) : Processor(name)
{
	_generatedGroupType = "Filtered";
}

OneEuroFilterProcessor::~OneEuroFilterProcessor(void)
{
	for(map<string,OneEuroFilterWrapper*>::iterator it = _filter.begin(); it!= _filter.end(); it++)
		delete it->second;
}

Node* OneEuroFilterProcessor::clone(string newName) const
{
	return new OneEuroFilterProcessor(newName);
}

bool OneEuroFilterProcessor::start()
{
	this->setFreq(100.0);
	this->setMincutoff(1.0);
	this->setBeta(0.0);
	this->setDcutoff(1.0);

	return true;
}

bool OneEuroFilterProcessor::stop()
{
	return true;
}

bool OneEuroFilterProcessor::update(map<string,Group3D*>& g3D, map<string,Group2D*>& g2D, map<string,Group1D*>& g1D, map<string,GroupSwitch*>& groupsSwitch)
{
	
	vector<string> updated;
	for(map<string,Group3D*>::iterator mit = g3D.begin();mit != g3D.end();mit++){
		if(isProcessedGroup(mit->first,mit->second->getType())){
			map<string,HOrientedPoint3D*> elements = mit->second->getAll();
			for(map<string,HOrientedPoint3D*>::iterator eit = elements.begin();eit != elements.end();eit++){
				if(isProcessedType(eit->second->getType())){
					OrientedPoint3D* tp = eit->second->getLast();
					if(tp){
						Point3D prevPos = tp->getPosition();
						if(g3D.find(mit->first+"Filtered") != g3D.end())
							if(HOrientedPoint3D* p = g3D[mit->first+"Filtered"]->getElementByID(eit->first))
								if(p->getHistoric().size()>=2)
									prevPos = (p->getHistoric().rbegin()++)->second.getPosition();
						// Filtering
						Point3D current;
						current = tp->getPosition();
						OneEuroFilterWrapper* filter;
						if(_filter.count(mit->first + eit->first))
							filter = _filter.at(mit->first + eit->first);
						else
						{
							filter = new OneEuroFilterWrapper(this->freq(), this->mincutoff(), this->beta(), this->dcutoff());
							this->_filter[mit->first + eit->first] = filter;
						}

						vector<double> filteredVector = filter->filter(current.getX(), current.getY(), current.getZ(), _timestamp);
						current.setX(filteredVector.at(0));
						current.setY(filteredVector.at(1));
						current.setZ(filteredVector.at(2)); 
						
						updateData(_environment,g3D,mit->first+"Filtered",_generatedGroupType,eit->first,eit->second->getType(),_timestamp,
							OrientedPoint3D(current,Orientation3D(),1.0,0));
					}
				}
			}
		}
		else
		{
			if(mit->second->getType() == _generatedGroupType)
				updated.push_back(mit->first);
		}
	}
	// check if all element in g2D are in g3D, or delete them
	vector<string> toDel;
	for(vector<string>::iterator mit = updated.begin();mit != updated.end();mit++){
		//cout << "Test substr " << mit->first << " - " << mit->first.substr(0,mit->first.length()-6) << endl;
		if(g3D.find(mit->substr(0,mit->length()-_generatedGroupType.length())) == g3D.end())
			toDel.push_back(*mit);
	}
	for(vector<string>::iterator mit = toDel.begin();mit != toDel.end();mit++)
		g3D.erase(*mit);
	return true;
}

set<string> OneEuroFilterProcessor::need() const
{
	return 	_observedPointType;
}

set<string> OneEuroFilterProcessor::consume() const
{
	set<string> consumed;
	return consumed;
}

set<string> OneEuroFilterProcessor::produce() const
{
	set<string> produce;
	produce.insert(_generatedGroupType);
	return produce;
}
