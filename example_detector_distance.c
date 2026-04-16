// Copyright (c) Acconeer AB, 2022-2025
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "acc_definitions_a121.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"
#include "main.h"

/** \example example_detector_distance.c
 * @brief This is an example on how the Detector Distance API can be used
 * @n
 * This example executes as follows:
 *   - Retrieve HAL integration
 *   - Initialize distance detector resources:
 *     + Create distance detector configuration
 *     + Update configuration settings
 *     + Create Distance detector handle
 *     + Create buffer for detector calibration data
 *     + Create buffer for sensor data
 *   - Create and calibrate the sensor
 *   - Calibrate the detector
 *   - Measure distances with the detector (loop):
 *     + Prepare sensor with the detector
 *     + Measure and wait until a read can be done
 *     + Process measurement and print the result
 *     + Handle "calibration_needed" indication
 *   - Cleanup:
 *     + Destroy detector configuration
 *     + Destroy detector handle
 *     + Destroy sensor data buffer
 *     + Destroy detector calibration data buffer
 */

extern UART_HandleTypeDef huart2;

#define VALID_MIN_DIST (0.35f)
#define VALID_MAX_DIST (1.2f)
#define MAX_SCORE_VEL (3.0f)
#define WEIGHT_CLOSENESS (0.45f)
#define WEIGHT_APPROACH (0.45f)
#define WEIGHT_CONTINUITY (0.1f)
#define TRACK_THRESH (0.25f)
#define DIST_ALPHA (0.25f)
#define HIST_LEN (5)
#define MIN_WINDOW_DT (0.05f)
#define APPROACH_ON_VEL (0.3f)
#define APPROACH_OFF_VEL (0.0f)
#define MAX_MISSES_BEFORE_DROP (5)

typedef struct
{
	bool has_selection;
	float selected_raw_dist;
	float selected_filt_dist;

	uint8_t miss_count;
	uint8_t detection_latched;
	uint8_t alert_latched;

	float dist_hist[HIST_LEN];
	uint32_t time_hist[HIST_LEN];
	uint8_t hist_count;
	uint8_t hist_head;

	uint32_t prev_update_time;

	uint8_t output_state;

} blindspot_tracker_t;

typedef struct
{
	float dist;
	float score;
	float closeness_score;
	float approach_score;
	float continuity_score;
	float candidate_vel;
} target_candidate_t;

static blindspot_tracker_t tracker = {0};

typedef enum
{
	DISTANCE_PRESET_CONFIG_NEXUSCYCLE,
	DISTANCE_PRESET_CONFIG_BALANCED,
	DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

#define SENSOR_ID (1U)
// 2 seconds should be enough even for long ranges and high signal quality
#define SENSOR_TIMEOUT_MS (2000U)

typedef struct
{
	acc_sensor_t                     *sensor;
	acc_detector_distance_config_t   *config;
	acc_detector_distance_handle_t   *handle;
	void                             *buffer;
	uint32_t                          buffer_size;
	uint8_t                          *detector_cal_result_static;
	uint32_t                          detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
} distance_detector_resources_t;

static void cleanup(distance_detector_resources_t *resources);

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);

static bool initialize_detector_resources(distance_detector_resources_t *resources);

static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *sensor_cal_result, void *buffer, uint32_t buffer_size);

static bool do_full_detector_calibration(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result);

static bool do_detector_calibration_update(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result);

static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result);

static void print_distance_result(const acc_detector_distance_result_t *result);

int acconeer_main(int argc, char *argv[]);

static float clamp(float score);

static bool score_target(float candidate_dist, const blindspot_tracker_t *tracker, uint32_t now, target_candidate_t *out);

static bool select_most_dangerous_target(const acc_detector_distance_result_t *result, const blindspot_tracker_t *tracker, uint32_t now, target_candidate_t *best_out);

static void update_windows(blindspot_tracker_t *tracker, float dist, float time);

static bool get_window_vel(const blindspot_tracker_t *tracker, float *vel);

int acconeer_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;
	distance_detector_resources_t resources = {0};

	printf("Acconeer software version %s\n", acc_version_get());

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal))
	{
		return EXIT_FAILURE;
	}

	resources.config = acc_detector_distance_config_create();
	if (resources.config == NULL)
	{
		printf("acc_detector_distance_config_create() failed\n");
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	set_config(resources.config, DISTANCE_PRESET_CONFIG_NEXUSCYCLE);

	if (!initialize_detector_resources(&resources))
	{
		printf("Initializing detector resources failed\n");
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	// Print the configuration
	acc_detector_distance_config_log(resources.handle, resources.config);

	/* Turn the sensor on */
	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	resources.sensor = acc_sensor_create(SENSOR_ID);
	if (resources.sensor == NULL)
	{
		printf("acc_sensor_create() failed\n");
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	acc_cal_result_t sensor_cal_result;

	if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
	{
		printf("Sensor calibration failed\n");
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	if (!do_full_detector_calibration(&resources, &sensor_cal_result))
	{
		printf("Detector calibration failed\n");
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	while (true)
	{
		acc_detector_distance_result_t result = {0};

		if (!do_detector_get_next(&resources, &sensor_cal_result, &result))
		{
			printf("Could not get next result\n");
			cleanup(&resources);
			return EXIT_FAILURE;
		}

		/* If "calibration needed" is indicated, the sensor needs to be recalibrated and the detector calibration updated */
		if (result.calibration_needed)
		{
			printf("Sensor recalibration and detector calibration update needed ... \n");

			if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
			{
				printf("Sensor calibration failed\n");
				cleanup(&resources);
				return EXIT_FAILURE;
			}

			/* Once the sensor is recalibrated, the detector calibration should be updated and measuring can continue. */
			if (!do_detector_calibration_update(&resources, &sensor_cal_result))
			{
				printf("Detector calibration update failed\n");
				cleanup(&resources);
				return EXIT_FAILURE;
			}

			printf("Sensor recalibration and detector calibration update done!\n");
		}
		else
		{
			print_distance_result(&result);
		}
	}

	cleanup(&resources);

	printf("Done!\n");

	return EXIT_SUCCESS;
}

static void cleanup(distance_detector_resources_t *resources)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	acc_detector_distance_config_destroy(resources->config);
	acc_detector_distance_destroy(resources->handle);

	acc_integration_mem_free(resources->buffer);
	acc_integration_mem_free(resources->detector_cal_result_static);

	if (resources->sensor != NULL)
	{
		acc_sensor_destroy(resources->sensor);
	}
}

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset)
{
	// Add configuration of the detector here
	switch (preset)
	{
		case DISTANCE_PRESET_CONFIG_NEXUSCYCLE:
			acc_detector_distance_config_start_set(detector_config, VALID_MIN_DIST);
			acc_detector_distance_config_end_set(detector_config, VALID_MAX_DIST);
			acc_detector_distance_config_max_step_length_set(detector_config, 2U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.65f);
			acc_detector_distance_config_signal_quality_set(detector_config, 25.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;

		case DISTANCE_PRESET_CONFIG_BALANCED:
			acc_detector_distance_config_start_set(detector_config, 0.25f);
			acc_detector_distance_config_end_set(detector_config, 3.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 0U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 15.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;

		case DISTANCE_PRESET_CONFIG_HIGH_ACCURACY:
			acc_detector_distance_config_start_set(detector_config, 0.25f);
			acc_detector_distance_config_end_set(detector_config, 3.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 2U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_3);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 20.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;
	}
}

static bool initialize_detector_resources(distance_detector_resources_t *resources)
{
	resources->handle = acc_detector_distance_create(resources->config);
	if (resources->handle == NULL)
	{
		printf("acc_detector_distance_create() failed\n");
		return false;
	}

	if (!acc_detector_distance_get_sizes(resources->handle, &(resources->buffer_size), &(resources->detector_cal_result_static_size)))
	{
		printf("acc_detector_distance_get_sizes() failed\n");
		return false;
	}

	resources->buffer = acc_integration_mem_alloc(resources->buffer_size);
	if (resources->buffer == NULL)
	{
		printf("sensor buffer allocation failed\n");
		return false;
	}

	resources->detector_cal_result_static = acc_integration_mem_alloc(resources->detector_cal_result_static_size);
	if (resources->detector_cal_result_static == NULL)
	{
		printf("calibration buffer allocation failed\n");
		return false;
	}

	return true;
}

static bool do_sensor_calibration(acc_sensor_t *sensor, acc_cal_result_t *sensor_cal_result, void *buffer, uint32_t buffer_size)
{
	bool           status              = false;
	bool           cal_complete        = false;
	const uint16_t calibration_retries = 1U;

	// Random disturbances may cause the calibration to fail. At failure, retry at least once.
	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
		// Reset sensor before calibration by disabling/enabling it
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);

		do
		{
			status = acc_sensor_calibrate(sensor, &cal_complete, sensor_cal_result, buffer, buffer_size);

			if (status && !cal_complete)
			{
				status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
			}
		} while (status && !cal_complete);
	}

	if (status)
	{
		/* Reset sensor after calibration by disabling/enabling it */
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}

	return status;
}

static bool do_full_detector_calibration(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_calibrate(resources->sensor,
		                                         resources->handle,
		                                         sensor_cal_result,
		                                         resources->buffer,
		                                         resources->buffer_size,
		                                         resources->detector_cal_result_static,
		                                         resources->detector_cal_result_static_size,
		                                         &resources->detector_cal_result_dynamic,
		                                         &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}

static bool do_detector_calibration_update(distance_detector_resources_t *resources, const acc_cal_result_t *sensor_cal_result)
{
	bool done = false;
	bool status;

	do
	{
		status = acc_detector_distance_update_calibration(resources->sensor,
		                                                  resources->handle,
		                                                  sensor_cal_result,
		                                                  resources->buffer,
		                                                  resources->buffer_size,
		                                                  &resources->detector_cal_result_dynamic,
		                                                  &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}

static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result)
{
	bool result_available = false;

	do
	{
		if (!acc_detector_distance_prepare(resources->handle,
		                                   resources->config,
		                                   resources->sensor,
		                                   sensor_cal_result,
		                                   resources->buffer,
		                                   resources->buffer_size))
		{
			printf("acc_detector_distance_prepare() failed\n");
			return false;
		}

		if (!acc_sensor_measure(resources->sensor))
		{
			printf("acc_sensor_measure() failed\n");
			return false;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS))
		{
			printf("Sensor interrupt timeout\n");
			return false;
		}

		if (!acc_sensor_read(resources->sensor, resources->buffer, resources->buffer_size))
		{
			printf("acc_sensor_read() failed\n");
			return false;
		}

		if (!acc_detector_distance_process(resources->handle,
		                                   resources->buffer,
		                                   resources->detector_cal_result_static,
		                                   &resources->detector_cal_result_dynamic,
		                                   &result_available,
		                                   result))
		{
			printf("acc_detector_distance_process() failed\n");
			return false;
		}
	} while (!result_available);

	return true;
}

//static void print_distance_result(const acc_detector_distance_result_t *result)
//{
//    if (result == NULL)
//    {
//        return;
//    }
//
//    printf("num_distances=%u near_edge=%u temp=%d\n",
//           result->num_distances,
//           result->near_start_edge_status,
//           result->temperature);
//
//    for (uint8_t i = 0; i < result->num_distances; i++)
//    {
//        printf("peak[%u]: dist=%.3f m strength=%.2f dB\n",
//               i,
//               result->distances[i],
//               result->strengths[i]);
//    }
//}

// *FILTERING ALGORITHM FUNCTIONS BELOW *

static float clamp(float score)
{
	if (score < 0.0f)
	{
		return 0.0f;
	}

	if (score > 1.0f)
	{
		return 1.0f;
	}

	return score;
}

static void tracker_reset(blindspot_tracker_t *t)
{
	t->has_selection = false;
	t->selected_raw_dist_m = 0.0f;
	t->filtered_dist_m = 0.0f;

	memset(t->dist_hist, 0, sizeof(t->dist_hist));
	memset(t->time_hist_ms, 0, sizeof(t->time_hist_ms));
	t->hist_count = 0U;
	t->hist_head = 0U;

	t->on_count = 0U;
	t->off_count = 0U;
	t->lost_count = 0U;
	t->output_state = 0U;
	t->prev_update_ms = 0U;
}

static bool score_target(float candidate_dist, const blindspot_tracker_t *tracker, uint32_t now, target_candidate_t *out)
{
	float closeness_score;
	float approach_score = 0.0f;
	float candidate_vel = 0.0f;
	float continuity_score = 0.0f;
	float score;

	if ((out == NULL) || (candidate_dist < VALID_MIN_DIST) || (candidate_dist > VALID_MAX_DIST))
		{
			return false;
		}

	// closer objects are more dangerous
	closeness_score = (VALID_MAX_DIST - candidate_dist) / (VALID_MAX_DIST - VALID_MIN_DIST);
	closeness_score = clamp(closeness_score);

	if (tracker->has_selection && tracker->prev_update_time > 0 && now > tracker->prev_update_time)
	{
		float dt = (float)(now - tracker->prev_update_time) / 1000.0f; // in seconds

		if (dt > 0.0f)
		{
			candidate_vel = -(candidate_dist - tracker->selected_raw_dist) / dt;

			if (candidate_vel > 0.0f)
			{
				approach_score = clamp(candidate_vel / MAX_SCORE_VEL);
			}
			continuity_score = 1.0f - fabsf(candidate_dist - tracker->selected_raw_dist) / TRACK_THRESH;
			continuity_score = clamp(continuity_score);
		}
	}

	score = (WEIGHT_CLOSENESS * closeness_score) + (WEIGHT_APPROACH * approach_score) + (WEIGHT_CONTINUITY * continuity_score);

	out->dist = candidate_dist;
	out->score = score;
	out->closeness_score = closeness_score;
	out->approach_score = approach_score;
	out->continuity_score = continuity_score;
	out->candidate_vel = candidate_vel;

	return true;
}

static bool select_most_dangerous_target(const acc_detector_distance_result_t *result, const blindspot_tracker_t *tracker, uint32_t now, target_candidate_t *best_out)
{
	bool found = false;
	target_candidate_t candidate;
	target_candidate_t best = {0};

	if (best_out == NULL || result->num_distances == 0)
	{
		return false;
	}

	for (int i = 0; i < result->num_distances; i++)
	{
		float dist = result->distances[i];

		if (!score_target(dist, tracker, now, &candidate))
		{
			continue;
		}

		if (!found || (candidate.score > best.score))
		{
			best = candidate;
			found = true;
		}
	}

	if (found)
	{
		*best_out = best;
		return true;
	}

	return false;
}

static void update_windows(blindspot_tracker_t *tracker, float dist, float time)
{
	tracker->dist_hist[(int)tracker->hist_head] = dist;
	tracker->time_hist[(int)tracker->hist_head] = time;

	tracker->hist_head = (uint8_t)((tracker->hist_head + 1) % HIST_LEN);

	if (tracker->hist_count < HIST_LEN)
	{
		tracker->hist_count++;
	}
}

static bool get_window_vel(const blindspot_tracker_t *tracker, float *vel)
{
	if ((tracker->hist_count < 2) || (vel == NULL))
	{
		return false;
	}

	uint8_t newest_index;
	uint8_t oldest_index;

	if (tracker->hist_count < HIST_LEN)
	{
		oldest_index = 0;
		newest_index = (uint8_t)(tracker->hist_count - 1);
	}
	else
	{
		oldest_index = tracker->hist_head;
		newest_index = (uint8_t)((tracker->hist_head + HIST_LEN - 1) % HIST_LEN);
	}

	float d_old = tracker->dist_hist[oldest_index];
	float d_new = tracker->dist_hist[newest_index];
	uint32_t t_old = tracker->time_hist[oldest_index];
	uint32_t t_new = tracker->time_hist[newest_index];

	if (t_new <= t_old)
	{
		return false;
	}

	float dt = (float)((t_new - t_old) / 1000.0f);
	if (dt < MIN_WINDOW_DT)
	{
		return false;
	}

	*vel = -(d_new - d_old) / dt;
	return true;
}

static void print_distance_result(const acc_detector_distance_result_t *result)
{
	blindspot_tracker_t *t = &tracker;
	uint32_t curr_time = HAL_GetTick();
	target_candidate_t best = {0};
	bool have_target = false;
	bool have_vel = false;
	float window_vel = 0.0f;
	uint8_t byte = t->output_state;

	if (result == NULL)
	{
		return;
	}

//	printf("num_distances=%d\n", result->num_distances);
//	for (uint8_t i = 0; i < result->num_distances; i++)
//	{
//	    printf("dist[%d]=%.3f\n", i, result->distances[i]);
//	}

	have_target = select_most_dangerous_target(result, t, curr_time, &best);

	if (!have_target)
	{
		if (t->has_selection)
		{
			t->miss_count++;

			if (t->miss_count < MAX_MISSES_BEFORE_DROP)
			{
//				printf("TRACK LOST TEMP: miss=%u state=%u\n", t->miss_count, t->output_state);
				return;
			}
		}

		t->has_selection = false;
		t->hist_count = 0;
		t->hist_head = 0;
		t->miss_count = 0;
		if (t->output_state != 0)
		{
			t->output_state = 0;
			byte = 0;
			HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY);
		}

		printf("OFF: no valid target\n");
		return;
	}

	t->miss_count = 0;

	if (!t->has_selection)
	{
		t->has_selection = true;
		t->selected_raw_dist = best.dist;
		t->selected_filt_dist = best.dist;
		t->prev_update_time = curr_time;
		update_windows(t, t->selected_filt_dist, curr_time);

		return;
	}

	if (t->has_selection)
	{
	    if (fabsf(best.dist - t->selected_filt_dist) > TRACK_THRESH)
	    {
	        printf("REJECT: jump too large old=%.3f new=%.3f\n", t->selected_filt_dist, best.dist);
	        return;
	    }
	}

	t->selected_raw_dist = best.dist;
	t->selected_filt_dist = (DIST_ALPHA * best.dist) + ((1.0f - DIST_ALPHA) * t->selected_filt_dist);
	t->prev_update_time = curr_time;
	update_windows(t, t->selected_filt_dist, curr_time);

	have_vel = get_window_vel(t, &window_vel);

	if (!have_vel)
	{
		return;
	}

	if ((window_vel >= APPROACH_ON_VEL) && !t->alert_latched)
	{
	    byte = 1;
	    HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY);
	    t->alert_latched = 1;
	    t->output_state = 1;
	    printf("ALERT ON: d=%.3f vel=%.3f\n", best.dist, window_vel);
	}
	else if ((window_vel <= APPROACH_OFF_VEL) && t->alert_latched)
	{
	    byte = 0;
	    HAL_UART_Transmit(&huart2, &byte, 1, HAL_MAX_DELAY);
	    t->alert_latched = 0;
	    t->output_state = 0;
	    printf("ALERT OFF: d=%.3f vel=%.3f\n", best.dist, window_vel);
	}

}
