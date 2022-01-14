// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "WindFieldSnapshot.h"
#include "TextFileReaderDouble.h"
#include "DataFileReader.h"
#include "DataFileWriter.h"
#define	THISCLASS WindFieldSnapshot

THISCLASS::WindFieldSnapshot():
		mTime(0), mWind(NULL), mArraySize(), mOrigin(), mGridSize() {

}

THISCLASS::~WindFieldSnapshot() {
	AllocateArray(Point3Int(0, 0, 0));
}

void THISCLASS::AllocateArray(const Point3Int &arraysize) {
	if (arraysize == mArraySize) {
		return;
	}

	// Delete old array
	if (mWind) {
		delete [] mWind;
		mWind = NULL;
	}

	// Create new array
	int cc = mArraySize.Volume();
	if (cc > 0) {
		mArraySize = arraysize;
		mWind = new Point3[cc];
	}
}

int THISCLASS::XYZToI(const Point3Int &p) const {
	if (p.x < 0) {
		return -1;
	}
	if (p.y < 0) {
		return -1;
	}
	if (p.z < 0) {
		return -1;
	}
	if (p.x >= mArraySize.x) {
		return -1;
	}
	if (p.y >= mArraySize.y) {
		return -1;
	}
	if (p.z >= mArraySize.z) {
		return -1;
	}
	return (p.z*mArraySize.y + p.y)*mArraySize.x + p.x;
}

void THISCLASS::ReadTextFile(const std::string filename) {
	TextFileReaderDouble f(filename);
}

void THISCLASS::ReadBinaryFile(const std::string filename) {
	DataFileReader f(filename);

	Point3Int arraysize;
	arraysize.Read(f);
	AllocateArray(arraysize);
	mOrigin.Read(f);
	mGridSize.Read(f);
	f.mFile.read((char*)mWind, sizeof(Point3)*mArraySize.Volume());
	f.Close();
}

void THISCLASS::WriteBinaryFile(const std::string filename) {
	// Note that we don't store the time in the file. Timing information comes from the filename or from the folder structure.
	DataFileWriter f(filename);
	mArraySize.Write(f);
	mOrigin.Write(f);
	mGridSize.Write(f);
	f.mFile.write((char*)mWind, sizeof(Point3)*mArraySize.Volume());
	f.Close();
}

Point3 THISCLASS::GetWindSpeed(const Point3Int &p) const {
	int i = XYZToI(p);
	if (i < 0) {
		return Point3(0, 0, 0);
	}
	return mWind[i];
}

Point3 THISCLASS::GetWindSpeed(const Point3 &preal) const {
	Point3 p = (preal - mOrigin).DotDivide(mGridSize);
	Point3Int p1 = p.Floor();
	Point3Int p2 = p1 + Point3Int(1, 1, 1);
	Point3 pdiff1 = (Point3)p2 - p;
	Point3 pdiff2 = p - (Point3)p1;

	Point3 wind;
	wind += GetWindSpeed(Point3Int(p1.x, p1.y, p1.z)) * pdiff1.x * pdiff1.y * pdiff1.z;
	wind += GetWindSpeed(Point3Int(p1.x, p1.y, p2.z)) * pdiff1.x * pdiff1.y * pdiff2.z;
	wind += GetWindSpeed(Point3Int(p1.x, p2.y, p1.z)) * pdiff1.x * pdiff2.y * pdiff1.z;
	wind += GetWindSpeed(Point3Int(p1.x, p2.y, p2.z)) * pdiff1.x * pdiff2.y * pdiff2.z;
	wind += GetWindSpeed(Point3Int(p2.x, p1.y, p1.z)) * pdiff2.x * pdiff1.y * pdiff1.z;
	wind += GetWindSpeed(Point3Int(p2.x, p1.y, p2.z)) * pdiff2.x * pdiff1.y * pdiff2.z;
	wind += GetWindSpeed(Point3Int(p2.x, p2.y, p1.z)) * pdiff2.x * pdiff2.y * pdiff1.z;
	wind += GetWindSpeed(Point3Int(p2.x, p2.y, p2.z)) * pdiff2.x * pdiff2.y * pdiff2.z;

	return wind;
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<WindFieldSnapshot>" << std::endl;
	out << "\t<Time>" << mTime << "</Time>" << std::endl;
	out << "\t<ArraySize>" << mArraySize << "</ArraySize>" << std::endl;
	out << "\t<Origin>" << mOrigin << "</Origin>" << std::endl;
	out << "\t<GridSize>" << mGridSize << "</GridSize>" << std::endl;
	out << "</WindFieldSnapshot>" << std::endl;
}
