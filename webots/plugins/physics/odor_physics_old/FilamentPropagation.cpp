// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include "FilamentPropagation.h"
#include "Random.h"
#define	THISCLASS FilamentPropagation

THISCLASS::FilamentPropagation(Simulation *sim):
		SimulationInterface(sim) {

	mSimulation->mFilamentPropagation = this;
	mConfiguration.mStdDev = 0;
	mConfiguration.mFilamentGrowthGamma = 0;
}

THISCLASS::~FilamentPropagation() {
	if (mSimulation->mFilamentPropagation == this) {
		mSimulation->mFilamentPropagation = 0;
	}
}

void THISCLASS::OnSimulationStart() {
}

void THISCLASS::OnSimulationEnd() {
}

void THISCLASS::OnSimulationStep() {
	WindField *wf = mSimulation->mWindField;
	FilamentList *fl = mSimulation->mFilamentList;
	ObstacleList *of = mSimulation->mObstacleList;
	double simstep = mSimulation->mSimulationTimeStep;
	double stddev = mConfiguration.mStdDev * simstep;
	int count = fl->GetCount();

	Random r;
	for (int i = 0; i < count; i++) {
		Filament *f = fl->Get(i);
		if (f->mExists) {
			// Advection
			Point3 newpos = f->mPosition + wf->GetWindSpeed(f->mPosition) * simstep;
                          
			// Stochastic process (vmi)
			newpos.x += r.Normal(0, stddev);
			newpos.y += r.Normal(0, stddev);
			newpos.z += r.Normal(0, stddev);

			//if(i==100) printf("old:(%f %f %f) new:(%f %f %f) ws:(%f %f %f)\n",f->mPosition.x,f->mPosition.y,f->mPosition.z,newpos.x, newpos.y, newpos.z, wf->GetWindSpeed(f->mPosition).x, wf->GetWindSpeed(f->mPosition).y, wf->GetWindSpeed(f->mPosition).z);

			// Simple way of dealing with obstacles: if the new filament position is inside an obstacle, simply don't move
			if (of->GetObstacle(newpos) == 0) {
				f->mPosition = newpos;
			}

			// Filament growth
			f->mWidth += 0.5 * mConfiguration.mFilamentGrowthGamma / f->mWidth;
			
			//printf("ID %d : %d cout %d i; f->mWidth %f stddev %f\n",f->mID, count, i ,f->mWidth, stddev);
			/*// fa
			if (f->mPosition.x < 0) {
				fl->RemoveFilament(f->mID);
			}*/
		}
	}
}

void THISCLASS::WriteConfiguration(std::ostream &out) {
	out << "<FilamentPropagation>" << std::endl;
	out << "\t<StdDev>" << mConfiguration.mStdDev << "</StdDev>" << std::endl;
	out << "\t<FilamentGrowthGamma>" << mConfiguration.mFilamentGrowthGamma << "</FilamentGrowthGamma>" << std::endl;
	out << "</FilamentPropagation>" << std::endl;
}
