#include "FilteringProcessor.h"

FilteringProcessor::FilteringProcessor(string name) : Processor(name)
{
	_generatedGroupType = "Filtered";

	_filter = LowPassFilter<Vecteur3D>();
	_cutOff = 2.0;

}


FilteringProcessor::~FilteringProcessor(void)
{
}

Node* FilteringProcessor::clone(string newName) const
{
	return new FilteringProcessor(newName);
}

bool FilteringProcessor::start()
{
	//_filter.SetCutoffFrequency(0.25);
	//_filter.SetCutoffFrequency(0.6);
	_filter.SetCutoffFrequency(_cutOff);
	_filter.SetUpdateFrequency(60.0);

	//cout << "Start Processor" << endl;
	return true;
}

bool FilteringProcessor::stop()
{
	//cout << "Stop Processor" << endl;
	return true;
}

bool FilteringProcessor::update(map<string,Group3D*>& g3D, map<string,Group2D*>& g2D, map<string,Group1D*>& g1D, map<string,GroupSwitch*>& groupsSwitch)
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
						//current =Point3D(((2*tp->getPosition().getX())+prevPos.getX())/3.0,((2*tp->getPosition().getY())+prevPos.getY())/3.0,((2*tp->getPosition().getZ())+prevPos.getZ())/3.0); 

						current = tp->getPosition();

						Vecteur3D v = Vecteur3D(current.getX(),current.getY(),current.getZ());
						v = _filter.FilterValue(v);
						current.setX(v.x);
						current.setY(v.y);
						current.setZ(v.z);
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

	// delete filtered elements that have been deleted
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



set<string> FilteringProcessor::need() const
{
	return 	_observedPointType;
}

set<string> FilteringProcessor::consume() const
{
	set<string> consumed;
	return consumed;
}

set<string> FilteringProcessor::produce() const
{
	set<string> produce;
	produce.insert(_generatedGroupType);
	return produce;
}


