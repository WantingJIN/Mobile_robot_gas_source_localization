// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <plugins/physics.h>
#include "FilamentList.h"
#include "Constants.h"
#include <GL/gl.h>
#define	THISCLASS FilamentList

THISCLASS::FilamentList(Simulation *sim, int count):
		SimulationInterface(sim), mFilament(NULL), mCountAllocated(0) {

	SetCount(count);
	mSimulation->mFilamentList = this;
}

THISCLASS::~FilamentList() {
	if (mSimulation->mFilamentList == this) {
		mSimulation->mFilamentList = 0;
	}
	AllocateFilaments(0);
}

void THISCLASS::SetCount(int count) {
	// If more filaments than allocated are requested, create a new set of filaments.
	if (count > mCountAllocated) {
		AllocateFilaments(count);
		return;
	}

	// If less than half of the allocated filaments are requested, create a new set of filaments
	if (count*2 < mCountAllocated) {
		AllocateFilaments(count);
		return;
	}

	// Initialize the filaments
	InitializeFilaments();
}

void THISCLASS::InitializeFilaments() {
	// Write default values
	for (int i = 0; i < mCountAllocated; i++) {
		mFilament[i] = Filament();
		mFilament[i].mID = i;
		mFilament[i].mFilamentList = this;
	}
}

void THISCLASS::AllocateFilaments(int count) {
	// Delete old filaments
	if (mFilament) {
		delete [] mFilament;
		mFilament = NULL;
	}

	// Create new filaments and initialize them
	mCountAllocated = count;
	mLastAddedFilamentID = 0;
	if (mCountAllocated < 1) {
		return;
	}
	mFilament = new Filament[count];
	for (int i = 0; i < mCountAllocated; i++) {
		mFilament[i].mID = i;
		mFilament[i].mFilamentList = this;
	}
}

Filament *THISCLASS::AddFilament() {
	mLastAddedFilamentID++;
	if (mLastAddedFilamentID >= mCountAllocated) {
		mLastAddedFilamentID = 0;
	}

	// Make sure we remove a potentially existing filament
	RemoveFilament(mLastAddedFilamentID);

	// Return the new filament
	mFilament[mLastAddedFilamentID].mExists = true;
	return &(mFilament[mLastAddedFilamentID]);
}

bool THISCLASS::RemoveFilament(int id) {
	if (! mFilament[id].mExists) {
		return false;
	}
	mFilament[id].mExists = false;
	return true;
}

void THISCLASS::OnSimulationStart() {
	InitializeFilaments();
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
}

void THISCLASS::OnWebotsPhysicsDraw() {
	glDisable(GL_LIGHTING);
	glLineWidth(1.0);
	//glPointSize(4.0);	// TODO

	// Draw filaments
	glColor3f(0.3, 0.3, 0.9);
	for (int i = 0; i < mCountAllocated; i++) {
		if (mFilament[i].mExists) {
			glBegin(GL_LINE_LOOP);
			for (double angle = 0; angle < 2*PI; angle += PI / 3) {
				glVertex3f(mFilament[i].mPosition.x + cos(angle)*mFilament[i].mWidth, mFilament[i].mPosition.y - sin(angle)*mFilament[i].mWidth, mFilament[i].mPosition.z);
				//glVertex3f(mFilament[i].mPosition.x + cos(angle)*mFilament[i].mWidth, mFilament[i].mPosition.y, mFilament[i].mPosition.z - sin(angle)*mFilament[i].mWidth);
			}
			glEnd();
		}
	}

	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<FilamentList>" << std::endl;
	out << "\t<Size>" << mCountAllocated << "</Size>" << std::endl;
	out << "</FilamentList>" << std::endl;
}

void THISCLASS::WritePosition(std::ostream &out) {
	for (int i = 0; i < mCountAllocated; i++) {
		if (mFilament[i].mExists) {
			out << i << "\t" << mFilament[i].mPosition.x << "\t" << mFilament[i].mPosition.y << "\t" << mFilament[i].mPosition.z << std::endl;
		}
	}
}
