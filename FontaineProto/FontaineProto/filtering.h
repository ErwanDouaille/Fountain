//////////////////////////////////////////////////////////////////////
//
//  Filter
//	Géry Casiez - February 2006
//
//////////////////////////////////////////////////////////////////////
// Template class for filtering
//

#pragma once

template <class X> class Filter
{
protected:
	// Update frequency of the signal to filter (Hz)
	float mUpdateFrequency;
	// To filter data
	X mPrevValue; //Previous filtered value
	bool mFirstTime;

	// To compute and filter the derivate of data
	X mPrevValuePos; //Previous not filtered value
	X mPrevValueVel; //Previous filtered velocity
	bool mFirstTime2;

public:
	Filter(float mUpdateFrequency);
	Filter() { };
	virtual ~Filter(void);

	void Clear() { mFirstTime = true; mFirstTime2 = true;};
	void SetUpdateFrequency(float updateFrequency) { mUpdateFrequency = updateFrequency; };
};

//mUpdateFrequency in Hz
template <class X> Filter<X>::Filter(float updateFrequency)
{
	mUpdateFrequency = updateFrequency;
	mFirstTime = true;
	mFirstTime2 = true;
}

template <class X> Filter<X>::~Filter(void)
{
}

//////////////////////////////////////////////////////////////////////
//
// Low Pass Filter
//
//////////////////////////////////////////////////////////////////////
// set the following before using:
//		mCutoffFrequency
//
template <class X> class LowPassFilter : public Filter<X>
{
protected:
	float mCutoffFrequency;
	float mCutoffFrequencyForDerivate;

public:
	LowPassFilter(float mUpdateFrequency) : Filter<X>(mUpdateFrequency) { };
	LowPassFilter() : Filter<X>() { };
	X FilterValue(X NewValue);
  X ComputeAndFilterDerivate(X NewValue);
	void SetCutoffFrequency(float f) { mCutoffFrequency = f; mCutoffFrequencyForDerivate = f;};
  void SetCutoffFrequencyForDerivate(float f) { mCutoffFrequencyForDerivate = f; };
	float GetCutoffFrequency(void) { return mCutoffFrequency; };
	float GetCutoffFrequencyForDerivate(void) { return mCutoffFrequencyForDerivate; };
};

template <class X>
X LowPassFilter<X>::FilterValue(X newValue)
{
	/*

	Let's say Pnf the filtered position, Pn the non filtered position and Pn-1f the previous filtered position,
	Te the sampling period (in second) and tau a time constant calculated from the cut-off frequency fc.

	tau = 1 / (2 * pi * fc)
	Pnf = ( Pn + tau/Te * Pn-1f ) * 1/(1+ tau/Te)

	Attention: tau >= 10 * Te
	*/

	if (this->mFirstTime)
	{
		this->mPrevValue = newValue;
		this->mFirstTime = false;
	}

	float updateFrequency = this->mUpdateFrequency;

	float Te = 1 / updateFrequency;		// the sampling period (in seconds)
	float Tau = 1 / (float)(2*3.14159265*mCutoffFrequency);	// a time constant calculated from the cut-off frequency

	X filteredValue = (newValue + this->mPrevValue * (Tau/Te) ) * (1.0 / (1.0 + Tau / Te));

	this->mPrevValue = filteredValue;
	return filteredValue;
}

template <class X>
X LowPassFilter<X>::ComputeAndFilterDerivate(X newValue)
{
	/*

	Let's say Vf the filtered velocity, Vn-1f the previous filtered velocity, Pn the non filtered position and
	Pn-1 the previous non-filtered position, Te the sampling period (in second) and tau a time constant calculated
	from the cut-off frequency fc.

	tau = 1 / (2 * pi * fc)
	Vf = ( (Pn - Pn-1) / Te + tau / Te * Vn-1f ) * 1/(1+ tau/Te)

	Attention: tau >= 10 * Te
	*/

	if (this->mFirstTime2)
	{
		this->mPrevValuePos = newValue;
		this-> mPrevValueVel = 0;
		this->mFirstTime2 = false;
	}

	float updateFrequency = this->mUpdateFrequency;

	float Te = 1 / updateFrequency;		// the sampling period (in seconds)
	float Tau = 1 / (2*3.14159265*mCutoffFrequencyForDerivate);	// a time constant calculated from the cut-off frequency

	X filteredValue = ( (newValue - this->mPrevValuePos) * (1.0 / Te) + this->mPrevValueVel * (Tau / Te) ) * (1.0 / (1.0 + Tau / Te));

	this->mPrevValuePos = newValue;
  this->mPrevValueVel = filteredValue;

	return filteredValue;
}
