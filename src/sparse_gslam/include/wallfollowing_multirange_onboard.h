/*
 * wallfollowing_multirange_onboard.h
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */
#pragma once
#include <cmath>
#include <chrono>
int wall_follower(float *vel_x, float *vel_y, float *vel_w, float front_range, float side_range, float current_heading,
                  int direction_turn);

void adjustDistanceWall(float distance_wall_new);

void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref, int init_state);