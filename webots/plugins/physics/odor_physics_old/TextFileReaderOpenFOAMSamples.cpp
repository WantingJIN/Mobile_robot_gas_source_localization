// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include "TextFileReaderOpenFOAMSamples.h"
#define THISCLASS TextFileReaderOpenFOAMSamples

#include <sstream>

THISCLASS::TextFileReaderOpenFOAMSamples(const std::string &filename):
		TextFileReader(filename) {

}

void THISCLASS::Read(WindFieldSnapshot *wfs) {
	// Init variables
	int col;
	int row;
	std::string s;
	double val;

	// Go through the file and determine dimensions
	bool isfirstx = true;
	bool isfirsty = true;
	bool isfirstz = true;
	Point3 last;
	Point3 min;
	Point3 max;
	Point3 stepsize;
	Start();
	while (NextToken(&col, &row, &s)) {
		std::istringstream istr(s);
		istr >> val;
		if (col == 0) {
			if (isfirstx) {
				min.x = val;
				max.x = val;
			} else {
				double diff = fabs(last.x - val);
				if ((diff > 0) && ((stepsize.x == 0) || (diff < stepsize.x))) {
					stepsize.x = diff;
				}
				if (val < min.x) {
					min.x = val;
				}
				if (val > max.x) {
					max.x = val;
				}
			}
			last.x = val;
		} else if (col == 1) {
			if (isfirsty) {
				min.y = val;
				max.y = val;
			} else {
				double diff = fabs(last.y - val);
				if ((diff > 0) && ((stepsize.y == 0) || (diff < stepsize.y))) {
					stepsize.y = diff;
				}
				if (val < min.y) {
					min.y = val;
				}
				if (val > max.y) {
					max.y = val;
				}
			}
			last.y = val;
		} else if (col == 2) {
			if (isfirstz) {
				min.z = val;
				max.z = val;
			} else {
				double diff = fabs(last.z - val);
				if ((diff > 0) && ((stepsize.z == 0) || (diff < stepsize.z))) {
					stepsize.z = diff;
				}
				if (val < min.z) {
					min.z = val;
				}
				if (val > max.z) {
					max.z = val;
				}
			}
			last.z = val;
		} else if (col == 3) {
		} else if (col == 4) {
		} else if (col == 5) {
		}
	}

	// 2D workaround
	bool workaround2d = (stepsize.z == 0);
	if (workaround2d) {
		stepsize.z = 1;
	}

	// Calculate how many entries we need
	Point3 arraysize = (max - min).DotDivide(stepsize);
	if (workaround2d) {
		arraysize.z = 1;
	}

	// Initialize memory
	wfs->AllocateArray(arraysize);

	// Read
	Point3 cur;
	int curi=0;
	Start();
	while (NextToken(&col, &row, &s)) {
		std::istringstream istr(s);
		istr >> val;
		if (col == 0) {
			cur.x = val;
		} else if (col == 1) {
			cur.y = val;
		} else if (col == 2) {
			cur.z = val;
			curi = wfs->XYZToI(Point3Int((cur - min).DotDivide(stepsize).Round()));
		} else if (col == 3) {
			if (curi >= 0) {
				wfs->mWind[curi].x = val;
			}
		} else if (col == 4) {
			if (curi >= 0) {
				wfs->mWind[curi].y = val;
			}
		} else if (col == 5) {
			if (curi >= 0) {
				wfs->mWind[curi].z = val;
			}
		}
	}
}
