/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#include "SimpleKalmanCHOP.h"
#include "SimpleKalmanFilter.h"

#include <stdio.h>
#include <string.h>
#include <cmath>
#include <assert.h>

//#include <vector> // to get the vector class definition
//using std::vector; // to 

//vector<SimpleKalmanFilter> simpleKalmanFilter;

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillCHOPPluginInfo(CHOP_PluginInfo *info)
{
	// Always set this to CHOPCPlusPlusAPIVersion.
	info->apiVersion = CHOPCPlusPlusAPIVersion;

	// The opType is the unique name for this CHOP. It must start with a 
	// capital A-Z character, and all the following characters must lower case
	// or numbers (a-z, 0-9)
	info->customOPInfo.opType->setString("Simplekalman");

	// The opLabel is the text that will show up in the OP Create Dialog
	info->customOPInfo.opLabel->setString("Simple Kalman filter");

	// Information about the author of this OP
	info->customOPInfo.authorName->setString("Yeataro");
	info->customOPInfo.authorEmail->setString("yeataro@gmail.com");

	// This CHOP can work with 0 inputs
	info->customOPInfo.minInputs = 0;

	// It can accept up to 1 input though, which changes it's behavior
	info->customOPInfo.maxInputs = 1;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per CHOP that is using the .dll
	return new SimpleKalmanCHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the CHOP using that instance is deleted, or
	// if the CHOP loads a different DLL
	delete (SimpleKalmanCHOP*)instance;
}

};


SimpleKalmanCHOP::SimpleKalmanCHOP(const OP_NodeInfo* info) : myNodeInfo(info)
{
	//myExecuteCount = 0;
	//myOffset = 0.0;
}

SimpleKalmanCHOP::~SimpleKalmanCHOP()
{

}

void
SimpleKalmanCHOP::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void* reserved1)
{
	// This will cause the node to cook every frame
	ginfo->cookEveryFrameIfAsked = true;

	// Note: To disable timeslicing you'll need to turn this off, as well as ensure that
	// getOutputInfo() returns true, and likely also set the info->numSamples to how many
	// samples you want to generate for this CHOP. Otherwise it'll take on length of the
	// input CHOP, which may be timesliced.
	ginfo->timeslice = true;

	ginfo->inputMatchIndex = 0;
}

bool
SimpleKalmanCHOP::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void* reserved1)
{
	// If there is an input connected, we are going to match it's channel names etc
	// otherwise we'll specify our own.
	//double	 scale = inputs->getParDouble("Scale");
	
	Measure = inputs->getParDouble("Measure");
	Estimate = inputs->getParDouble("Estimate");
	Q = inputs->getParDouble("Q");

	if (inputs->getNumInputs() > 0)
	{
		simpleKalmanFilter.reserve((inputs->getNumInputs())) ;

		for (int i = 0; i < inputs->getNumInputs(); ++i)
		{
			simpleKalmanFilter.push_back(SimpleKalmanFilter(Measure, Estimate, Q));
		}
		
		return false;
	}
	else
	{
		info->numChannels = 0;

		// Since we are outputting a timeslice, the system will dictate
		// the numSamples and startIndex of the CHOP data
		//info->numSamples = 1;
		//info->startIndex = 0

		// For illustration we are going to output 120hz data
		//info->sampleRate = 120;
		return true;
	}
}

void
SimpleKalmanCHOP::getChannelName(int32_t index, OP_String *name, const OP_Inputs* inputs, void* reserved1)
{
	name->setString("chan1");
}

void
SimpleKalmanCHOP::execute(CHOP_Output* output,
							  const OP_Inputs* inputs,
							  void* reserved)
{
	//myExecuteCount++;
	
	//double	 scale = inputs->getParDouble("Scale");


	// In this case we'll just take the first input and re-output it scaled.

	if (inputs->getNumInputs() > 0)
	{

		inputs->enablePar("Reset", 1);

		Measure = inputs->getParDouble("Measure");
		Estimate = inputs->getParDouble("Estimate");
		Q = inputs->getParDouble("Q");

		int ind = 0;
		const OP_CHOPInput	*cinput = inputs->getInputCHOP(0);

		for (int i = 0 ; i < output->numChannels; i++)
		{
			for (int j = 0; j < output->numSamples; j++)
			{		
				simpleKalmanFilter[i].setMeasurementError(Measure);
				simpleKalmanFilter[i].setProcessNoise(Q);
				output->channels[i][j] = simpleKalmanFilter[i].updateEstimate(float(cinput->getChannelData(i)[ind]));
				ind++;

				// Make sure we don't read past the end of the CHOP input
				ind = ind % cinput->numSamples;
			}
		}

	}
	else {
		inputs->enablePar("Reset", 0);	// not used
	}
}

int32_t
SimpleKalmanCHOP::getNumInfoCHOPChans(void * reserved1)
{
	// We return the number of channel we want to output to any Info CHOP
	// connected to the CHOP. In this example we are just going to send one channel.
	return 0;
}

void
SimpleKalmanCHOP::getInfoCHOPChan(int32_t index,
										OP_InfoCHOPChan* chan,
										void* reserved1)
{
	// This function will be called once for each channel we said we'd want to return
	// In this example it'll only be called once.

	if (index == 0)
	{
		chan->name->setString("EstimateError");
		chan->value = (float)EstimateError;
	}

}

bool		
SimpleKalmanCHOP::getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved1)
{
	infoSize->rows = 0;
	infoSize->cols = 0;
	// Setting this to false means we'll be assigning values to the table
	// one row at a time. True means we'll do it one column at a time.
	infoSize->byColumn = false;
	return true;
}

void
SimpleKalmanCHOP::getInfoDATEntries(int32_t index,
										int32_t nEntries,
										OP_InfoDATEntries* entries, 
										void* reserved1)
{
	char tempBuffer[4096];

	if (index == 0)
	{
		// Set the value for the first column
		entries->values[0]->setString("EstimateError");

		// Set the value for the second column
#ifdef _WIN32
		sprintf_s(tempBuffer, "%d", EstimateError);
#else // macOS
		snprintf(tempBuffer, sizeof(tempBuffer), "%d", EstimateError);
#endif
		entries->values[1]->setString(tempBuffer);
	}

}

void
SimpleKalmanCHOP::setupParameters(OP_ParameterManager* manager, void *reserved1)
{
	// Measurement Error
	{
		OP_NumericParameter	np;

		np.name = "Measure";
		np.label = "Measurement Error";
		np.defaultValues[0] = 0.1;
		np.minSliders[0] = 0.01;
		np.maxSliders[0] =  1;
		
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Estimate Error
	{
		OP_NumericParameter	np;

		np.name = "Estimate";
		np.label = "Estimate Error";
		np.defaultValues[0] = 0.1;
		np.minSliders[0] = 0.01;
		np.maxSliders[0] =  1;
		
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Estimate Error
	{
		OP_NumericParameter	np;

		np.name = "Q";
		np.label = "Process Noise";
		np.defaultValues[0] = 0.1;
		np.minSliders[0] = 0.01;
		np.maxSliders[0] = 1;

		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}


	// pulse
	{
		OP_NumericParameter	np;

		np.name = "Reset";
		np.label = "Reset Estimate Error";
		
		OP_ParAppendResult res = manager->appendPulse(np);
		assert(res == OP_ParAppendResult::Success);
	}

}

void 
SimpleKalmanCHOP::pulsePressed(const char* name, void* reserved1)
{
	if (!strcmp(name, "Reset"))
	{
		for (int i = 0; i < simpleKalmanFilter.size(); i++){
		simpleKalmanFilter[i].setEstimateError(Estimate);
		}
	}
}

