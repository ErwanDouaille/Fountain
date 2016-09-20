/* -*- coding: utf-8 -*-
*
* OneEuroFilter.cc -
*
* Author: Nicolas Roussel (nicolas.roussel@inria.fr)
*
*/

#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace std;
// -----------------------------------------------------------------
// Utilities

//void
//randSeed(void) {
//  srand(time(0)) ;
//}
//
//double
//unifRand(void) {
//  return rand() / double(RAND_MAX) ;
//}

typedef double TimeStamp ; // in seconds

static const TimeStamp UndefinedTime = -1.0 ;

// -----------------------------------------------------------------

class LowPassFilter {

	double y, a, s ;
	bool initialized ;
	void setAlpha(double alpha) {
		if ( .0 >= alpha || alpha>1.0)
			throw std::range_error("alpha should be in (0.0., 1.0]") ;
		a = alpha ;
	}

public:

	LowPassFilter(double alpha, double initval=0.0) {
		y = s = initval ;
		setAlpha(alpha) ;
		initialized = false ;
	}

	double filter(double value) {
		double result ;
		if (initialized)
			result = a*value + (1.0-a)*s ;
		else {
			result = value ;
			initialized = true ;
		}
		y = value ;
		s = result ;
		return result ;
	}

	double filterWithAlpha(double value, double alpha) {
		setAlpha(alpha) ;
		return filter(value) ;
	}

	bool hasLastRawValue(void) {
		return initialized ;
	}

	double lastRawValue(void) {
		return y ;
	}

} ;

// -----------------------------------------------------------------

class OneEuroFilterWrapper {

	double freq ;
	double mincutoff ;
	double beta_ ;
	double dcutoff ;
	LowPassFilter *x ;
	LowPassFilter *dx ;
	LowPassFilter *y ;
	LowPassFilter *dy ;
	LowPassFilter *z ;
	LowPassFilter *dz ;
	TimeStamp lasttime ;

	double alpha(double cutoff) {
		double te = 1.0 / freq ;
		double tau = 1.0 / (2*3.14*cutoff) ;
		return 1.0 / (1.0 + tau/te) ;
	}

	void setFrequency(double f) {
		if (f<=0) throw std::range_error("freq should be >0") ;
		freq = f ;
	}

	void setMinCutoff(double mc) {
		if (mc<=0) throw std::range_error("mincutoff should be >0") ;
		mincutoff = mc ;
	}

	void setBeta(double b) {
		beta_ = b ;
	}

	void setDerivateCutoff(double dc) {
		if (dc<=0) throw std::range_error("dcutoff should be >0") ;
		dcutoff = dc ;
	}

public:

	OneEuroFilterWrapper(double freq,
		double mincutoff=1.0, double beta_=0.0, double dcutoff=1.0) {
			setFrequency(freq) ;
			setMinCutoff(mincutoff) ;
			setBeta(beta_) ;
			setDerivateCutoff(dcutoff) ;
			x = new LowPassFilter(alpha(mincutoff)) ;
			dx = new LowPassFilter(alpha(dcutoff)) ;
			y = new LowPassFilter(alpha(mincutoff)) ;
			dy = new LowPassFilter(alpha(dcutoff)) ;
			z = new LowPassFilter(alpha(mincutoff)) ;
			dz = new LowPassFilter(alpha(dcutoff)) ;
			lasttime = UndefinedTime ;
	}

	double filterX(double value, TimeStamp timestamp=UndefinedTime) {
		// update the sampling frequency based on timestamps
		// estimate the current variation per second
		double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
		double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff)) ;
		// use it to update the cutoff frequency
		double cutoff = mincutoff + beta_*fabs(edvalue) ;
		// filter the given value
		return x->filterWithAlpha(value, alpha(cutoff)) ;
	}

	double filterY(double value, TimeStamp timestamp=UndefinedTime) {
		double dvalue = y->hasLastRawValue() ? (value - y->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
		double edvalue = dy->filterWithAlpha(dvalue, alpha(dcutoff)) ;
		double cutoff = mincutoff + beta_*fabs(edvalue) ;
		return y->filterWithAlpha(value, alpha(cutoff)) ;
	}

	double filterZ(double value, TimeStamp timestamp=UndefinedTime) {
		double dvalue = z->hasLastRawValue() ? (value - z->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
		double edvalue = dz->filterWithAlpha(dvalue, alpha(dcutoff)) ;
		double cutoff = mincutoff + beta_*fabs(edvalue) ;
		return z->filterWithAlpha(value, alpha(cutoff)) ;
	}

	vector<double> filter(double valueX, double valueY, double valueZ, TimeStamp timestamp=UndefinedTime) {
		vector<double> values;
		if (lasttime!=UndefinedTime && timestamp!=UndefinedTime)
			freq = 1.0 / (timestamp-lasttime) ;
		lasttime = timestamp ;
		values.push_back(filterX(valueX, timestamp));
		values.push_back(filterY(valueY, timestamp));
		values.push_back(filterZ(valueZ, timestamp));
		return values;
	}

	~OneEuroFilterWrapper(void) {
		delete x ;
		delete dx ;
		delete y ;
		delete dy ;
		delete z ;
		delete dz ;
	}

} ;