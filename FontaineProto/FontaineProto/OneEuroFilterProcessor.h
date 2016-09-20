#ifndef ONEEUROFILTERPROCESSOR_H
#define ONEEUROFILTERPROCESSOR_H

#include "LgProcessor.h"
#include "OneEuroFilter.cpp"

using namespace lg;

class OneEuroFilterProcessor : public Processor
{
private:
	string _generatedGroupType;
	map<string, OneEuroFilterWrapper*> _filter;
	double _freq;
	double _mincutoff;
	double _beta;
	double _dcutoff;

public:
	OneEuroFilterProcessor(string);
	~OneEuroFilterProcessor(void);

	Node* clone(string name) const;

	bool start();
	bool stop();
	bool update(map<string,Group3D*>& groups3D, map<string,Group2D*>& groups2D, map<string,Group1D*>& groups1D, map<string,GroupSwitch*>& groupsSwitch);

	set<string> need() const;
	set<string> consume() const;
	set<string> produce() const;

	double OneEuroFilterProcessor::freq() const {return _freq;}
	void OneEuroFilterProcessor::setFreq(double freq) {_freq = freq;}
	double OneEuroFilterProcessor::mincutoff() const {return _mincutoff;}
	void OneEuroFilterProcessor::setMincutoff(double mincutoff) {_mincutoff = mincutoff;}
	double OneEuroFilterProcessor::beta() const {return _beta;}
	void OneEuroFilterProcessor::setBeta(double beta) {_beta = beta;}
	double OneEuroFilterProcessor::dcutoff() const {return _dcutoff;}
	void OneEuroFilterProcessor::setDcutoff(double dcutoff) {_dcutoff = dcutoff;}
	string getGeneratedGroupType(){return _generatedGroupType;}
	void setGeneratedGroupType(string newType){_generatedGroupType = newType;}
};

#endif // ONEEUROFILTERPROCESSOR_H