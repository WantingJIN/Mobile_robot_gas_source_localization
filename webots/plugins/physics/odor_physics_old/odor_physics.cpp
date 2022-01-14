// Copyright (c) 2005-2008, Thomas Lochmatter, thomas.lochmatter@epfl.ch
// Documentation: http://en.wikibooks.org/wiki/Webots_Odor_Simulation

#include <ode/ode.h>
#include <plugins/physics.h>
#include <assert.h>

#include "Simulation.h"
#include "FilamentSourceConstant.h"
#include "WindFieldConstant.h"
#include "WindFieldStatic.h"
#include "ObstacleList.h"
#include "FilamentList.h"
#include "FilamentSourceList.h"
#include "FilamentSourceConstant.h"
#include "SensorOdor.h"
#include "SensorWind.h"
#include "OdorModel.h"
#include <fstream>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <GL/gl.h>

static const int source_odor_count_max = 16;
static const int sensor_odor_count_max = 16;
static const int sensor_wind_count_max = 16;
static bool save_odor_profile = false; // if this is true, then the odor profile will be saved to disk after 20 seconds

Simulation *simulation;

void webots_physics_init() {
	// Create a simulation object
	simulation = new Simulation();
	simulation->mSimulationTimeStep = 0.032;
	simulation->mSimulationTime = dWebotsGetTime() / 1000.;

	// Store world and space (in case we need that somewhere)
	//simulation->mWebotsInterface.mWorldID = w;
	//simulation->mWebotsInterface.mSpaceID = s;

	// Read parameters from the environment
	char *windsensor_noise_stddev=getenv("WINDSENSOR_NOISE_STDDEV");
	char *filament_stddev=getenv("FILAMENT_STDDEV");
	char *filament_growth_gamma=getenv("FILAMENT_GROWTH_GAMMA");
	char *filament_n=getenv("FILAMENT_N");
	char *source_radius=getenv("SOURCE_RADIUS");
        char *save_profile=getenv("SAVE_ODOR_PROFILE");
        
        save_odor_profile = (NULL != save_profile);

	// Prepare the results folder
	char buffer[1024];
	if (getcwd(buffer, 1024) == 0) {
		std::cout << "Failed to get CWD - we won't be able write any results." << std::endl;
	}
	simulation->mResultsFolder = std::string(buffer);
	simulation->mResultsFolder += "/results";
	mkdir(simulation->mResultsFolder.c_str(), 0755);

	// Add a filament list
	// The number of filaments should be adjusted such that the filaments disappear roughly at the border of the simulation area.
	new FilamentList(simulation, (filament_n ? strtol(filament_n, 0, 0) : 1600));//1600

	// Add a constant wind field
	new ObstacleList(simulation);
	WindFieldConstant *wf = new WindFieldConstant(simulation);
	wf->SetWindSpeed(Point3(-0.9, 0.0, 0.0)); // XXX: constant?
	//WindFieldStatic *wf = new WindFieldStatic(simulation);
	//wf->SetFile("/data/lochmatt/DebugMeshes/Test1/plumeDebug1/samples/9/velField_U.xy");
	//wf->WriteConfiguration(std::cout);

	// Add a filament propagation model
	FilamentPropagation *fp = new FilamentPropagation(simulation);
	fp->mConfiguration.mStdDev = (filament_stddev ? strtof(filament_stddev, 0) : 0.02); //0.02
	fp->mConfiguration.mFilamentGrowthGamma = (filament_growth_gamma ? strtof(filament_growth_gamma, 0) : 4e-7);//4e-7
	
	//printf("Running with stddev=%f gamma=%f\n", fp->mConfiguration.mStdDev, fp->mConfiguration.mFilamentGrowthGamma);

	// Add an odor concentration model
	OdorModel *om = new OdorModel(simulation);
	om->mCutRadius = 1;

	// Add a list of filament source and the sensors
	FilamentSourceList *filamentsourcelist = new FilamentSourceList(simulation);
	SensorList *sensorlist = new SensorList(simulation);

	// Add odor sources
	for (int i = 0; i < source_odor_count_max; i++) {
		char buffer[32];
		sprintf(buffer, "SOURCE_ODOR_%d", i);
		dGeomID geometry_id = dWebotsGetGeomFromDEF(buffer);
		if (geometry_id > 0) {
			FilamentSourceConstant *fs = new FilamentSourceConstant(simulation);
			fs->mWebotsInterface.mGeometryID = geometry_id;
			fs->mConfiguration.mRadius = (source_radius ? strtof(source_radius, 0) : 0.01);//0.12
			fs->mConfiguration.mFilamentAmount = 8.3e2; //8.3e2
			fs->mConfiguration.mFilamentWidth = 0.08;//0.1
			fs->mConfiguration.mFilamentOdorType = 0;//0
			fs->mConfiguration.mReleaseAmount = 20;//20//100;
			filamentsourcelist->AddFilamentSource(fs);
		}
	}

	// Add odor sensors
	for (int i = 0; i < sensor_odor_count_max; i++) {
		char buffer[32];
		sprintf(buffer, "SENSOR_ODOR_%d", i);
		dGeomID geometry_id = dWebotsGetGeomFromDEF(buffer);
		if (geometry_id > 0) {
			SensorOdor *s = new SensorOdor(simulation);
			s->mWebotsInterface.mGeometryID = geometry_id;
			s->mWebotsInterface.mChannel = i;
			s->mWebotsInterface.mLogFile.open((simulation->mResultsFolder + "/" + buffer).c_str());
			s->mConfiguration.mOdorType = 0;
			s->mConfiguration.mNoiseStdDev = 0;
			s->mConfiguration.mRunningAverageFactor = 0.0;
			sensorlist->AddSensor(s);
		printf("sensor %d detected \n", i);
		}
	}

	// Add wind sensors
	for (int i = 0; i < sensor_wind_count_max; i++) {
		char buffer[32];
		sprintf(buffer, "SENSOR_WIND_%d", i);
		dGeomID geometry_id = dWebotsGetGeomFromDEF(buffer);
		if (geometry_id > 0) {
			SensorWind *s = new SensorWind(simulation);
			s->mWebotsInterface.mGeometryID = geometry_id;
			s->mWebotsInterface.mChannel = i + 100;
			s->mWebotsInterface.mLogFile.open((simulation->mResultsFolder + "/" + buffer).c_str());
			s->mConfiguration.mNoiseStdDev = (windsensor_noise_stddev ? strtod(windsensor_noise_stddev, 0) : 0.1);
			s->mConfiguration.mRunningAverageFactor = 0.0;
			sensorlist->AddSensor(s);
		}
	}

	// Initialize everything
	simulation->OnSimulationStart();

	// Write configuration
	std::ofstream file((simulation->mResultsFolder + "/configuration.xml").c_str());
	file << "<?xml version=\"1.0\" ?>" << std::endl;
	file << std::endl;
	file << "<!-- This XML file is written by the odor_physics plugin at the beginning of each simulation with the configuration. It cannot be read by the the odor_physics plugin, but two such files can be compared to check for configuration differences. -->" << std::endl;
	file << std::endl;
	file << "<OdorPhysics>" << std::endl;
	simulation->WriteConfiguration(file);
	file << "</OdorPhysics>" << std::endl;
}

void webots_physics_step() {
	// Run one timestep of the simulation
	double simulation_time = dWebotsGetTime() / 1000.;
	simulation->mSimulationTimeStep = simulation_time - simulation->mSimulationTime;
	simulation->mSimulationTime = simulation_time;
	std::cout << simulation->mSimulationTime << std::endl;
	simulation->OnSimulationStep();

	// Store an odor profile after 20 seconds
  if (save_odor_profile) {
    if ((simulation->mSimulationTime - 20. > 0) && (simulation->mSimulationTime - 20. < 0.064)) {
      std::cout << "Writing odor profile" << std::endl;
      std::ofstream file((simulation->mResultsFolder+"/concentration20").c_str());
      // Startpoint, endpoint, resolution
      // //simulation->mOdorModel->WriteConcentration(file, 0, Point3(0, 0, -2), Point3(16, 1, 2), Point3(0.01, 2, 0.01));
      //simulation->mOdorModel->WriteConcentration(file, 0, Point3(0, 0, -2), Point3(20, 1, 2), Point3(0.01, 2, 0.01));
      simulation->mOdorModel->WriteConcentration(file, 0, Point3(-14, 0, -2), Point3(6, 1, 2), Point3(0.01, 2, 0.01));
      exit(0);
    }
  }
}

int webots_physics_collide(dGeomID g1, dGeomID g2) {
	return 0;	// collision not handled, otherwise should return 1
}

void webots_physics_cleanup() {
	// Cleanup everything
	simulation->OnSimulationEnd();
}

void webots_physics_draw(int pass, const char *view) {
	// Draw the world
	simulation->OnWebotsPhysicsDraw();

	// Draw coordinates
	
	glDisable(GL_LIGHTING);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glColor3f(1, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glColor3f(0, 0, 1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();
	glLineWidth(1.0);
	glEnable(GL_LIGHTING);
	
}
