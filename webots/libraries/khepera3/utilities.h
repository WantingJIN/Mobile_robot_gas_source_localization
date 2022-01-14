/* UTILITIES HEADER */

#ifndef UTILITIES_H
#define UTILITIES_H

#include <webots/robot.h>

#ifdef __cplusplus
extern "C" {
#endif

//! Performs webots step synchronization
void utilities_sync_step(int step_ms);

#ifdef __cplusplus
}
#endif

#endif

