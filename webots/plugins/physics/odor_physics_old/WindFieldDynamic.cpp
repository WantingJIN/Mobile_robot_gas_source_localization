// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <sys/types.h>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <dirent.h>
#include "WindFieldDynamic.h"
#include "TextFileReaderOpenFOAMSamples.h"
#define	THISCLASS WindFieldDynamic

THISCLASS::WindFieldDynamic(Simulation *sim): WindField(sim) {
	mWindFieldSnapshot[0] = new WindFieldSnapshot();
	mWindFieldSnapshot[1] = new WindFieldSnapshot();
}

void THISCLASS::OnSimulationStart() {
	mFileHeapCount = 0;

	// Open folder
	DIR *d = opendir(mSamplesFolder.c_str());
	if (! d) {
		AddError("Unable to access samples folder! Is the path correct?");
		return;
	}

	// Reinitialize the heap
	for (int i = 0; i < mFileHeapMax; i++) {
		mFileHeap[i] = 0;
	}
	std::make_heap(mFileHeap, mFileHeap);

	// Read folder items
	while (struct dirent *entry = readdir(d)) {
		if ((entry->d_type == DT_DIR) && (entry->d_name[0] != '.')) {
			double val;
			std::string str(entry->d_name);
			std::istringstream istr(str);
			// TODO catch errors, i.e. ignore all subfolders that are not numbers
			istr >> val;

			mFileHeap[mFileHeapCount] = val;
			mFileHeapCount++;
			std::push_heap(mFileHeap, mFileHeap + mFileHeapCount);
		}
	}

	// Close folder
	closedir(d);

	// Read the two first wind fields
	ReadNextFile();
	ReadNextFile();
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
	// Read next file if necessary
	while (mSimulation->mSimulationTime > mWindFieldSnapshot[1]->mTime) {
		bool ok = ReadNextFile();
		if (! ok) {
			AddError("At end of wind simulation!");
			return;
		}
	}

	mTimeInterpolationFactor = (mSimulation->mSimulationTime - mWindFieldSnapshot[0]->mTime) / (mWindFieldSnapshot[1]->mTime - mWindFieldSnapshot[0]->mTime);
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	mWindFieldSnapshot[0]->WriteConfiguration(out);
	mWindFieldSnapshot[1]->WriteConfiguration(out);
}

Point3 THISCLASS::GetWindSpeed(const Point3 &preal) {
	Point3 w0 = mWindFieldSnapshot[0]->GetWindSpeed(preal);
	Point3 w1 = mWindFieldSnapshot[1]->GetWindSpeed(preal);
	return w0*mTimeInterpolationFactor + w1*(1 - mTimeInterpolationFactor);
}

bool THISCLASS::ReadNextFile() {
	// Switch the two wind fields
	WindFieldSnapshot *temp = mWindFieldSnapshot[1];
	mWindFieldSnapshot[1] = mWindFieldSnapshot[0];
	mWindFieldSnapshot[0] = temp;

	// If there are no more files on the heap, stop the simulation
	if (mFileHeapCount <= 0) {
		return false;
	}

	// Get the next file name
	std::pop_heap(mFileHeap, mFileHeap + mFileHeapCount);
	mFileHeapCount--;
	double time = mFileHeap[mFileHeapCount];
	mFileHeap[mFileHeapCount] = 0;

	std::ostringstream file;
	file << mSamplesFolder << "/" << time << "/velfield_xy";

	// Read the next file
	TextFileReaderOpenFOAMSamples tfr(std::string(file));
	//if (tfr.Error()) {return false;}
	//tfr.Read(mWindFieldSnapshot[1]);
	return true;
}
