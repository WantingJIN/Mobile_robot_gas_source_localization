// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "WindFieldStatic.h"
#include "TextFileReaderOpenFOAMSamples.h"
#define	THISCLASS WindFieldStatic

bool THISCLASS::SetFile(const std::string &file) {
	TextFileReaderOpenFOAMSamples tfr(file);
	if (tfr.Error()) {
		return false;
	}
	tfr.Read(&mWindFieldSnapshot);
	return true;
}

Point3 THISCLASS::GetWindSpeed(const Point3 &preal) {
	return mWindFieldSnapshot.GetWindSpeed(preal);
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	mWindFieldSnapshot.WriteConfiguration(out);
}
