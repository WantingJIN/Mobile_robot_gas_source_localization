/*!
 * (c) 2007 - 2007 EPFL, Lausanne, Switzerland
 * Jim Pugh, Sven Gowal, Thomas Lochmatter
 * Inaki Navarro (webots k3t)
 */

#include "relativepositionboard.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define PI 3.14159265358979


#include <gsl/gsl_rng.h> // GSL random number generators
#include <gsl/gsl_randist.h> // GSL random distributions
#include <time.h>

#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/robot.h>


#define RELPOS_CHANNEL 200
static WbDeviceTag rpreceiver, rpemitter;

struct sRelativePositionBoard relativepositionboard;
gsl_rng *rng_ptr; // pointer to random number generator (rng)


/* Initialization */
void relativepositionboard_init() {
  // Set the ID and calibration values
  int channel;

  relativepositionboard.configwebots.max_range = 3.3;
  relativepositionboard.configwebots.noise = 1;
  relativepositionboard.configwebots.proportional_distance_noise = 0.1;
  relativepositionboard.configwebots.absolute_angular_noise = 0.1;
  relativepositionboard.configwebots.packet_time_step = 32;

  rpemitter = wb_robot_get_device("relposemitter");
  channel = wb_emitter_get_channel(rpemitter);
    if (channel != RELPOS_CHANNEL) {
      wb_emitter_set_channel(rpemitter, RELPOS_CHANNEL);
    }
  rpreceiver = wb_robot_get_device("relposreceiver");
  wb_receiver_enable(rpreceiver, relativepositionboard.configwebots.packet_time_step);
  wb_receiver_set_channel(rpreceiver, RELPOS_CHANNEL);

  wb_emitter_set_range (rpemitter, relativepositionboard.configwebots.max_range);

  rng_ptr = gsl_rng_alloc (gsl_rng_taus); // allocate the rng
}


/* Start / Stop Transmitter and Receiver */
void relativepositionboard_start(unsigned int robot_id) {
  // Set the ID. In real robots done in relativepositionboard_init()
  relativepositionboard.robot_id = robot_id;

  // Enable emitter and receiver
  relativepositionboard.configuration.rx_enabled = 1;
  relativepositionboard.configuration.tx_enabled = 1;
  relativepositionboard.configuration.tx_data = robot_id;

  // If an invalid robot id was provided, disable the emitter
  if (robot_id >= 32) {
    printf("Relative position board: invalid robot_id. The emitter has been switched off!\n");
    relativepositionboard.configuration.tx_enabled = 0;
    relativepositionboard.configuration.tx_data = 0;
  }

  gsl_rng_set (rng_ptr, (int) time(NULL) + robot_id); // seed the rng with id and time

}

void relativepositionboard_stop() {
  relativepositionboard.configuration.tx_enabled = 0;
  relativepositionboard.configuration.rx_enabled = 0;
}

/* Stream Reading */

void relativepositionboard_stream_start() {
  // Start the stream
}

void relativepositionboard_stream_step() {
  // Sending message. Placed here in webots version.
  const int *msg = &relativepositionboard.configuration.tx_data;
  const int *const_received_msg;
  double distance;
  double bearing;
  const double *rp_direction;
  int r;
  int robot_id;

  wb_emitter_send(rpemitter, msg, sizeof(int));

  // Increment the age counter
  for (r = 0; r < 32; r++) {
    relativepositionboard.robot[r].age++;
  }

  //Read messages got from the relative positioning system
  while (wb_receiver_get_queue_length(rpreceiver) > 0) {
    //Read current packet's data
    const_received_msg = wb_receiver_get_data(rpreceiver);
    rp_direction = wb_receiver_get_emitter_direction(rpreceiver);
    bearing = atan2(rp_direction[0],rp_direction[2]);
    distance = sqrt(1/wb_receiver_get_signal_strength(rpreceiver));
    if(relativepositionboard.configwebots.noise){
      distance = distance*(1.0 + gsl_ran_gaussian (rng_ptr, relativepositionboard.configwebots.proportional_distance_noise));
      bearing = bearing +  gsl_ran_gaussian (rng_ptr,relativepositionboard.configwebots.absolute_angular_noise);
    }

    robot_id = *const_received_msg;
    relativepositionboard.robot[robot_id].range = distance;
    while (bearing > PI) bearing -= 2.0*PI;
    while (bearing < -PI) bearing += 2.0*PI;
    relativepositionboard.robot[robot_id].bearing = bearing;
    relativepositionboard.robot[robot_id].age = 0;

    wb_receiver_next_packet(rpreceiver);
  }

}

//! Sets the corresponding configuration value.
void relativepositionboard_configuration_rx_enabled_set(){}
//! Sets the corresponding configuration value.
void relativepositionboard_configuration_tx_enabled_set(){}
//! Sets the corresponding configuration value.
void relativepositionboard_configuration_tx_data_set(){}
//! Sets the corresponding configuration value.
void relativepositionboard_configuration_tx_power_set(){}
//! Sets the corresponding configuration value.
void relativepositionboard_configuration_tx_sectors_set(){}
//! Sets the corresponding configuration value.
void relativepositionboard_configuration_tx_interval_set(){}

//! Reads the corresponding configuration value.
void relativepositionboard_configuration_rx_enabled_get(){}
//! Reads the corresponding configuration value.
void relativepositionboard_configuration_tx_enabled_get(){}
//! Reads the corresponding configuration value.
void relativepositionboard_configuration_tx_data_get(){}
//! Reads the corresponding configuration value.
void relativepositionboard_configuration_tx_power_get(){}
//! Reads the corresponding configuration value.
void relativepositionboard_configuration_tx_sectors_get(){}
//! Reads the corresponding configuration value.
void relativepositionboard_configuration_tx_interval_get(){}

//! Starts the calibration of all sectors. This will take about 6 ms per sector.
void relativepositionboard_calibrate(float threshold_factor){}
//! Sets the calibration values.
void relativepositionboard_calibration_set(){}
//! Reads the calibration values.
int relativepositionboard_calibration_get(){return 0;}
//! Saves the calibration values to EEPROM. These values will *not* be automatically loaded upon reset of the board.
void relativepositionboard_calibration_save(){}
//! Loads the calibration values from EEPROM.
void relativepositionboard_calibration_load(){}
