#include "BlasterObserver.h"


BlasterObserver::BlasterObserver(void) : Observer("BlasterObserver")
{
}


BlasterObserver::~BlasterObserver(void)
{
}

Node* BlasterObserver::clone(string cloneName) const
{
	return new BlasterObserver();
}

bool BlasterObserver::start()
{
	cout << "Start Observer" << endl;

	string line;
	ifstream myfile ("source3D.txt");
	if (myfile.is_open())
	{
		getline (myfile,line);
		hauteurCamera = stoi(line);

		getline (myfile,line);
		blasterWidth = stof(line);

		getline (myfile,line);
		int blastersNumber = stoi(line);
		//cout << "Nb blasters : " << stoi(line) << endl;
		

		for(int i = 0;i<blastersNumber;i++){
			//cout << "Loading ... ";
			getline (myfile,line);
			xBlaster.push_back(stof(line));
			//cout << stof(line);

			getline (myfile,line);
			yBlaster.push_back(stof(line));
			//cout << stof(line) << endl;
		}


		myfile.close();
	}

	else
	{
		cout << "Unable to open file"; 
		exit(-1);
	}

	return true;
}

bool BlasterObserver::stop()
{
	cout << "Stop Observer" << endl;
	return true;
}

bool BlasterObserver::observe(map<string,Group3D*> g3D,map<string,Group2D*>,map<string,Group1D*>, map<string,GroupSwitch*>)
{
	//cout << "Update Observer" << endl;
	// delete all proba
	map<string,float> probas = getProbabilities();
	for(map<string,float>::iterator pit = probas.begin();pit != probas.end();pit++)
		updateProbability(pit->first,0.0f);

	for(map<string,Group3D*>::iterator git = g3D.begin();git != g3D.end(); git++)
	{
		if(isObservedGroup(git->first,git->second->getType()))
		{
			Group3D* g = git->second;
			set<HOrientedPoint3D*> rhs = g->getElementsByType(LG_ORIENTEDPOINT3D_RIGHT_HAND);
			float proba = 0.0;
			for(set<HOrientedPoint3D*> ::iterator sit = rhs.begin();sit != rhs.end();sit++)
			{
				HOrientedPoint3D* rh = *sit;
				if(rh->getHistoric().size() < 3 )
					continue;

				Point3D pos = rh->getLast()->getPosition();
				bool wasAbove = (_jets[git->first] != -1);
				bool needUpdate = true;

				if(needUpdate)
				{
					vector<float>::iterator yit = yBlaster.begin();
					int cpt = 1;
					float dist = 1000000000.0;
					float h = -1.0;
					int jet = -1;
					for(vector<float>::iterator xit = xBlaster.begin();xit != xBlaster.end();xit++,yit++,cpt++)
					{
						Point2D p2 = Point2D(pos.getX(),pos.getY());
						float d = p2.distanceTo(Point2D(*xit,*yit));
						if(d <= (blasterWidth)/2.0)
						{
							if(d < dist)
							{
								dist = d;
								h = hauteurCamera - pos.getZ(); // - fountainHeight; ?? pourquoi fountainHeight ? h = hauteurCamera - pos.getZ() - fountainHeight;
								jet = cpt;
							}
						}
					}

					if(dist <= blasterWidth/2.0){
						proba = 1.0;
					}
					_hauteurs[git->first] = h;
					_jets[git->first] = jet;
				}
			}

			updateProbability(git->first,proba);
		}
		else
			updateProbability(git->first,0.0f);
	}
	return true;
}

set<string> BlasterObserver::need() const
{
	set<string> needed;
	needed.insert(LG_ORIENTEDPOINT3D_RIGHT_HAND);
	return needed;
}

float BlasterObserver::getHauteur(string group){
	return _hauteurs[group];
}

int BlasterObserver::getJet(string group){
	return _jets[group];
}