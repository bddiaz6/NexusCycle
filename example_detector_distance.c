// Copyright (c) Acconeer AB, 2022-2023
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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

/**
 * Rear-facing bike blind-spot detector built on the Acconeer distance detector.
 *
 * Design intent:
 * - Radar is mounted facing straight back from the bike.
 * - We want to alert BEFORE or AS a vehicle enters the blind-spot region.
 * - We do NOT want to alert on targets that are stationary relative to the road
 *   and are moving away from the radar after the bike passes them.
 * - A laterally passing car loses radial velocity near broadside, so once the
 *   alert is triggered we HOLD it for a short time through the pass.
 *
 * Core strategy:
 * 1) Use the distance detector as the primary signal.
 * 2) Track the best target over time.
 * 3) Estimate filtered radial velocity from range change.
 * 4) Alert on persistent APPROACHING targets in a risk range or TTC window.
 * 5) Hold the alert through the blind-spot pass.
 * 6) Clear on persistent receding / target loss.
 *
 * This file is meant to be a strong starting point, not the final tuned product.
 * You will still need to tune thresholds from real bike logs.
 */

typedef enum
{
	DISTANCE_PRESET_CONFIG_NONE = 0,
	DISTANCE_PRESET_CONFIG_BALANCED,
	DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

typedef enum
{
	BIKE_STATE_IDLE = 0,
	BIKE_STATE_CANDIDATE,
	BIKE_STATE_APPROACHING,
	BIKE_STATE_ALERT,
	BIKE_STATE_HOLD,
} bike_state_t;

#define SENSOR_ID                  (1U)
#define SENSOR_TIMEOUT_MS          (2000U)

/*
 * IMPORTANT: replace FRAME_DT_S with your measured frame period once known.
 * The alert logic depends on velocity/TTC, so dt should eventually be measured,
 * not guessed.
 */
#define DEFAULT_FRAME_DT_S         (0.10f)

/* --------------------------- Range tuning --------------------------- */
/* Targets outside this band are ignored. Tune based on lane width and mounting. */
#define SEARCH_RANGE_MIN_M         (0.75f)
#define SEARCH_RANGE_MAX_M         (6.50f)

/* Enter risk evaluation once target is this close or TTC is short enough. */
#define ALERT_RANGE_M              (3.50f)
#define EXIT_RANGE_M               (7.00f)

/* -------------------------- Tracking tuning ------------------------- */
#define DIST_ALPHA                 (0.35f)
#define VEL_ALPHA                  (0.25f)
#define MAX_TRACK_JUMP_M           (1.20f)
#define REACQUIRE_CLOSER_BY_M       (1.10f)
#define MAX_MISSED_FRAMES           (4U)
#define SPARSE_ENTRY_QUIET_FRAMES   (6U)
#define SPARSE_ENTRY_CAND_RANGE_M   (2.80f)
#define SPARSE_ENTRY_ALERT_RANGE_M  (2.80f)
#define SPARSE_SINGLE_HIT_ALERT_RANGE_M (2.40f)
#define SPARSE_TWO_HIT_ALERT_RANGE_M    (2.70f)
#define LATERAL_OCCUPANCY_ALERT_RANGE_M (2.80f)
#define LATERAL_OCCUPANCY_MAX_AGE_FRAMES (5U)
#define LATERAL_OCCUPANCY_MAX_FILT_VEL_MPS (0.12f)
#define LATERAL_OCCUPANCY_MAX_INST_VEL_MPS (0.28f)
#define SPARSE_TWO_HIT_MAX_RECEDE_COUNT  (1U)
#define LATERAL_OCCUPANCY_MAX_RECEDE_COUNT (1U)
#define WEAK_TRACK_REPLACE_BY_M     (1.50f)
#define MAX_NEW_TRACK_RANGE_M       (3.50f)
#define TOTAL_HITS_FOR_ALERT        (2U)

/* ------------------------ Motion classification --------------------- */
/* Negative = approaching a rear-facing radar. Lateral passes only show a
 * small radial component, so these thresholds must be much softer than a
 * head-on detector.
 */
#define APPROACH_VEL_MPS           (-0.03f)  /* Roadside log showed real passes only reaching about -0.03 to -0.06 m/s filtered radial speed. */
#define STRONG_APPROACH_VEL_MPS    (-0.08f)
#define STATIONARY_ABS_VEL_MPS     (0.06f)
#define RECEDE_VEL_MPS             (0.10f)

/* ------------------------ State machine tuning ---------------------- */
#define CANDIDATE_MIN_PRESENT         (2U)
#define APPROACH_MIN_FRAMES           (1U)
#define ALERT_MIN_FRAMES              (1U)
#define RECEDE_CLEAR_FRAMES           (2U)
#define ENTRY_INST_APPROACH_VEL_MPS   (-0.10f) /* One-frame negative radial kick used for lateral pass entry. */
#define ENTRY_SHOCK_INST_VEL_MPS      (-0.25f)
#define ENTRY_SHOCK_DROP_M            (0.18f)
#define ENTRY_SHOCK_RAW_RANGE_M       (2.40f)
#define EXIT_INST_RECEDE_VEL_MPS      (0.16f)
#define CLOSE_RANGE_ALERT_M           (2.10f)
#define CLOSE_RANGE_PRESENT_FRAMES    (2U)
#define CLOSE_RANGE_DROP_M            (0.008f)
#define STATIONARY_CLEAR_FRAMES       (10U)
#define HOLD_FRAMES                   (12U)
#define MAX_TTC_ALERT_S               (2.5f)
#define MIN_RANGE_DROP_TO_APPROACH_M  (0.03f)
#define MIN_RANGE_DROP_TO_ALERT_M     (0.04f)
#define MIN_DROP_EVAL_AGE_FRAMES      (3U)

typedef struct
{
	acc_sensor_t                         *sensor;
	acc_detector_distance_config_t       *config;
	acc_detector_distance_handle_t       *handle;
	void                                 *buffer;
	uint32_t                              buffer_size;

	uint8_t                              *detector_cal_result_static;
	uint32_t                              detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t     detector_cal_result_dynamic;
} distance_detector_resources_t;

typedef struct
{
	bike_state_t state;
	bool         alert_active;

	bool         has_track;
	float        raw_range_m;
	float        filt_range_m;
	float        prev_filt_range_m;
	float        inst_vel_mps;
	float        filt_vel_mps;
	float        min_range_seen_m;
	float        max_range_seen_m;
	float        range_drop_m;
	float        ttc_s;
	float        last_dt_s;

	uint32_t     track_age_frames;
	uint32_t     present_count;
	uint32_t     total_hit_count;
	uint32_t     entry_quiet_frames;
	uint32_t     idle_quiet_frames;
	uint32_t     miss_count;
	uint32_t     approach_count;
	uint32_t     strong_approach_count;
	uint32_t     stationary_count;
	uint32_t     recede_count;
	uint32_t     hold_count;
} bike_tracker_t;

static void cleanup(distance_detector_resources_t *resources);
static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);
static bool initialize_detector_resources(distance_detector_resources_t *resources);
static bool do_sensor_calibration(acc_sensor_t     *sensor,
                                  acc_cal_result_t *sensor_cal_result,
                                  void             *buffer,
                                  uint32_t         buffer_size);
static bool do_full_detector_calibration(distance_detector_resources_t *resources,
                                         const acc_cal_result_t        *sensor_cal_result);
static bool do_detector_update_calibration(distance_detector_resources_t *resources,
                                           const acc_cal_result_t        *sensor_cal_result);
static bool do_detector_get_next(distance_detector_resources_t  *resources,
                                 const acc_cal_result_t         *sensor_cal_result,
                                 acc_detector_distance_result_t *result);

static void bike_tracker_init(bike_tracker_t *tracker);
static void bike_tracker_update(bike_tracker_t *tracker, const acc_detector_distance_result_t *result, float dt_s);
static void print_bike_status(const acc_detector_distance_result_t *result, const bike_tracker_t *tracker);

static bool select_best_target(const acc_detector_distance_result_t *result,
                               const bike_tracker_t                *tracker,
                               float                               *selected_range_m);
static bool range_in_search_band(float d);
static bool compute_target_ttc(float range_m, float vel_mps, float *ttc_s);
static bool has_closing_evidence(const bike_tracker_t *tracker, float min_range_drop_m);
static bool has_immediate_closing_evidence(const bike_tracker_t *tracker, float min_range_drop_m);
static bool has_entry_shock_alert_evidence(const bike_tracker_t *tracker);
static bool has_sparse_two_hit_alert_evidence(const bike_tracker_t *tracker);
static void bike_tracker_clear_track(bike_tracker_t *tracker);
static bool has_close_proximity_alert_evidence(const bike_tracker_t *tracker);
static const char *state_to_string(bike_state_t state);

int acconeer_main(int argc, char *argv[]);

/* Avoid a hard dependency on HAL headers in this file. */
extern uint32_t HAL_GetTick(void);

int acconeer_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	distance_detector_resources_t resources = { 0 };
	bike_tracker_t               tracker   = { 0 };

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

	set_config(resources.config, DISTANCE_PRESET_CONFIG_BALANCED);

	if (!initialize_detector_resources(&resources))
	{
		printf("Initializing detector resources failed\n");
		cleanup(&resources);
		return EXIT_FAILURE;
	}

	acc_detector_distance_config_log(resources.handle, resources.config);

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

	bike_tracker_init(&tracker);

	uint32_t last_frame_ms = 0U;

	while (true)
	{
		acc_detector_distance_result_t result = { 0 };
		float dt_s = DEFAULT_FRAME_DT_S;

		if (!do_detector_get_next(&resources, &sensor_cal_result, &result))
		{
			printf("Could not get next result\n");
			cleanup(&resources);
			return EXIT_FAILURE;
		}

		if (!result.calibration_needed)
		{
			uint32_t now_ms = HAL_GetTick();

			if (last_frame_ms != 0U)
			{
				uint32_t delta_ms = now_ms - last_frame_ms;
				if (delta_ms > 0U)
				{
					dt_s = ((float)delta_ms) / 1000.0f;
				}
			}

			last_frame_ms = now_ms;
			bike_tracker_update(&tracker, &result, dt_s);
			print_bike_status(&result, &tracker);

			/* Replace this with UART / GPIO / CAN message as needed. */
			if (tracker.alert_active)
			{
				printf("ALERT=1\n");
			}
			else
			{
				printf("ALERT=0\n");
			}
		}

		if (result.calibration_needed)
		{
			printf("Sensor- and detector recalibration needed ...\n");

			if (!do_sensor_calibration(resources.sensor, &sensor_cal_result, resources.buffer, resources.buffer_size))
			{
				printf("Sensor calibration failed\n");
				cleanup(&resources);
				return EXIT_FAILURE;
			}

			if (!do_detector_update_calibration(&resources, &sensor_cal_result))
			{
				printf("Detector recalibration failed\n");
				cleanup(&resources);
				return EXIT_FAILURE;
			}

			printf("Sensor- and detector recalibration done!\n");
		}
	}

	cleanup(&resources);
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
	switch (preset)
	{
		case DISTANCE_PRESET_CONFIG_NONE:
			break;

		case DISTANCE_PRESET_CONFIG_BALANCED:
			acc_detector_distance_config_start_set(detector_config, 0.50f);
			acc_detector_distance_config_end_set(detector_config, 7.50f);
			acc_detector_distance_config_max_step_length_set(detector_config, 0U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_num_frames_recorded_threshold_set(detector_config, 100U);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.55f);
			acc_detector_distance_config_signal_quality_set(detector_config, 15.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, true);
			break;

		case DISTANCE_PRESET_CONFIG_HIGH_ACCURACY:
			acc_detector_distance_config_start_set(detector_config, 0.50f);
			acc_detector_distance_config_end_set(detector_config, 7.50f);
			acc_detector_distance_config_max_step_length_set(detector_config, 2U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_3);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_num_frames_recorded_threshold_set(detector_config, 100U);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.55f);
			acc_detector_distance_config_signal_quality_set(detector_config, 20.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, true);
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

	if (!acc_detector_distance_get_sizes(resources->handle,
	                                     &(resources->buffer_size),
	                                     &(resources->detector_cal_result_static_size)))
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

	resources->detector_cal_result_static =
		(uint8_t *)acc_integration_mem_alloc(resources->detector_cal_result_static_size);
	if (resources->detector_cal_result_static == NULL)
	{
		printf("detector static calibration allocation failed\n");
		return false;
	}

	memset(&(resources->detector_cal_result_dynamic), 0, sizeof(resources->detector_cal_result_dynamic));
	return true;
}

static bool do_sensor_calibration(acc_sensor_t     *sensor,
                                  acc_cal_result_t *sensor_cal_result,
                                  void             *buffer,
                                  uint32_t         buffer_size)
{
	bool           status              = false;
	bool           cal_complete        = false;
	const uint16_t calibration_retries = 1U;

	for (uint16_t i = 0; !status && (i <= calibration_retries); i++)
	{
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
		acc_hal_integration_sensor_disable(SENSOR_ID);
		acc_hal_integration_sensor_enable(SENSOR_ID);
	}

	return status;
}

static bool do_full_detector_calibration(distance_detector_resources_t *resources,
                                         const acc_cal_result_t        *sensor_cal_result)
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
		                                         &(resources->detector_cal_result_dynamic),
		                                         &done);

		if (status && !done)
		{
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status && !done);

	return status;
}

static bool do_detector_update_calibration(distance_detector_resources_t *resources,
                                           const acc_cal_result_t        *sensor_cal_result)
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
		                                                  &(resources->detector_cal_result_dynamic),
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
		                                   &(resources->detector_cal_result_dynamic),
		                                   &result_available,
		                                   result))
		{
			printf("acc_detector_distance_process() failed\n");
			return false;
		}
	} while (!result_available);

	return true;
}

static void bike_tracker_init(bike_tracker_t *tracker)
{
	memset(tracker, 0, sizeof(*tracker));
	tracker->state = BIKE_STATE_IDLE;
	tracker->min_range_seen_m = 999.0f;
	tracker->max_range_seen_m = 0.0f;
	tracker->range_drop_m = 0.0f;
	tracker->ttc_s = 999.0f;
	tracker->last_dt_s = DEFAULT_FRAME_DT_S;
}

static bool range_in_search_band(float d)
{
	return (d >= SEARCH_RANGE_MIN_M) && (d <= SEARCH_RANGE_MAX_M);
}

/*
 * Select the target that best matches the current track continuity.
 * If no track exists, choose the closest valid target in the search band.
 */
static bool select_best_target(const acc_detector_distance_result_t *result,
                               const bike_tracker_t                *tracker,
                               float                               *selected_range_m)
{
	bool  found = false;
	float best_r = 0.0f;
	float best_score = 1.0e9f;

	if ((result == NULL) || (selected_range_m == NULL))
	{
		return false;
	}

	for (uint8_t i = 0; i < result->num_distances; i++)
	{
		float r = result->distances[i];
		if (!range_in_search_band(r))
		{
			continue;
		}

		if (!tracker->has_track && (r > MAX_NEW_TRACK_RANGE_M))
		{
			continue;
		}

		float score;
		if (tracker->has_track)
		{
			float jump = fabsf(r - tracker->filt_range_m);
			if (jump > MAX_TRACK_JUMP_M)
			{
				bool closer_reacquire = (tracker->miss_count >= 1U) &&
				                        (r < (tracker->filt_range_m - REACQUIRE_CLOSER_BY_M));
				bool weak_track_replace = (tracker->present_count <= 1U) &&
				                          (tracker->track_age_frames <= 1U) &&
				                          (r < (tracker->filt_range_m - WEAK_TRACK_REPLACE_BY_M));

				if (!(closer_reacquire || weak_track_replace))
				{
					continue;
				}

				/* A much closer in-band target is more important than preserving a stale
				 * or weak farther track. Favor immediate reacquisition/replacement. */
				score = 0.05f * r;
			}
			else
			{
				/* Slightly prefer closer targets once continuity is acceptable. */
				score = jump + (0.10f * r);
			}
		}
		else
		{
			score = r;
		}

		if (!found || (score < best_score))
		{
			found = true;
			best_score = score;
			best_r = r;
		}
	}

	if (found)
	{
		*selected_range_m = best_r;
	}

	return found;
}

static bool compute_target_ttc(float range_m, float vel_mps, float *ttc_s)
{
	if ((ttc_s == NULL) || (vel_mps >= 0.0f) || (range_m <= 0.0f))
	{
		return false;
	}

	float closing_speed = -vel_mps;
	if (closing_speed < 0.01f)
	{
		return false;
	}

	*ttc_s = range_m / closing_speed;
	return true;
}

static bool has_closing_evidence(const bike_tracker_t *tracker, float min_range_drop_m)
{
	if (tracker == NULL)
	{
		return false;
	}

	return tracker->has_track &&
	       (tracker->track_age_frames >= MIN_DROP_EVAL_AGE_FRAMES) &&
	       (tracker->range_drop_m >= min_range_drop_m);
}

static bool has_immediate_closing_evidence(const bike_tracker_t *tracker, float min_range_drop_m)
{
	if (!has_closing_evidence(tracker, min_range_drop_m))
	{
		return false;
	}

	return (tracker->filt_vel_mps <= APPROACH_VEL_MPS) ||
	       (tracker->inst_vel_mps <= ENTRY_INST_APPROACH_VEL_MPS);
}

static bool has_entry_shock_alert_evidence(const bike_tracker_t *tracker)
{
	if ((tracker == NULL) || !tracker->has_track)
	{
		return false;
	}

	/* Fast lateral entries often appear as one strong negative radial kick after a brief miss gap.
	 * In that case the filtered range can lag the raw target by a few frames, so gate on raw range.
	 */
	if (tracker->track_age_frames < 2U)
	{
		return false;
	}

	if (tracker->raw_range_m > ENTRY_SHOCK_RAW_RANGE_M)
	{
		return false;
	}

	if (tracker->inst_vel_mps > ENTRY_SHOCK_INST_VEL_MPS)
	{
		return false;
	}

	if (tracker->range_drop_m < ENTRY_SHOCK_DROP_M)
	{
		return false;
	}

	if (tracker->recede_count > 0U)
	{
		return false;
	}

	return true;
}

static bool has_sparse_entry_candidate_evidence(const bike_tracker_t *tracker)
{
	if ((tracker == NULL) || !tracker->has_track)
	{
		return false;
	}

	if (tracker->present_count < 1U)
	{
		return false;
	}

	if (tracker->entry_quiet_frames < SPARSE_ENTRY_QUIET_FRAMES)
	{
		return false;
	}

	if (tracker->raw_range_m > SPARSE_ENTRY_CAND_RANGE_M)
	{
		return false;
	}

	return true;
}

static bool has_sparse_single_frame_alert_evidence(const bike_tracker_t *tracker)
{
	if (!has_sparse_entry_candidate_evidence(tracker))
	{
		return false;
	}

	/* A lone hit is too weak unless it is extremely close.
	 * This keeps random clutter spikes from directly causing an alert.
	 */
	if (tracker->total_hit_count != 1U)
	{
		return false;
	}

	return tracker->raw_range_m <= SPARSE_SINGLE_HIT_ALERT_RANGE_M;
}

static bool has_sparse_two_hit_alert_evidence(const bike_tracker_t *tracker)
{
	if (!has_sparse_entry_candidate_evidence(tracker))
	{
		return false;
	}

	/* For roadside lateral passes, sparse detections are common.
	 * Two hits inside a short-lived track is much more credible than one hit,
	 * so allow a slightly wider range gate here than the one-hit emergency path.
	 */
	if (tracker->total_hit_count < 2U)
	{
		return false;
	}

	/* A lateral pass can produce one slightly positive radial-velocity sample
	 * immediately after the closest point. Do not throw away the alert on that
	 * alone; just reject clearly receding tracks.
	 */
	if (tracker->recede_count > SPARSE_TWO_HIT_MAX_RECEDE_COUNT)
	{
		return false;
	}

	return tracker->raw_range_m <= SPARSE_TWO_HIT_ALERT_RANGE_M;
}

static bool has_lateral_occupancy_alert_evidence(const bike_tracker_t *tracker)
{
	if ((tracker == NULL) || !tracker->has_track)
	{
		return false;
	}

	/* A laterally passing vehicle beside a rear-facing radar often produces
	 * weak or near-zero radial velocity at the exact moment we most want the
	 * alert. When that happens, the more reliable cue is a clean scene becoming
	 * occupied by a short-range target that persists for at least two hits and
	 * is not showing receding motion.
	 */
	if (tracker->entry_quiet_frames < SPARSE_ENTRY_QUIET_FRAMES)
	{
		return false;
	}

	if (tracker->total_hit_count < 2U)
	{
		return false;
	}

	if (tracker->raw_range_m > LATERAL_OCCUPANCY_ALERT_RANGE_M)
	{
		return false;
	}

	if (tracker->track_age_frames > LATERAL_OCCUPANCY_MAX_AGE_FRAMES)
	{
		return false;
	}

	/* Lateral traffic beside a rear-facing radar often flips to a small positive
	 * radial velocity right after the closest point, even though it is still a
	 * legitimate blind-spot occupant. Permit one receding sample and a slightly
	 * larger instantaneous positive velocity, but still reject persistent
	 * outward-moving tracks.
	 */
	if (tracker->recede_count > LATERAL_OCCUPANCY_MAX_RECEDE_COUNT)
	{
		return false;
	}

	if ((tracker->filt_vel_mps > LATERAL_OCCUPANCY_MAX_FILT_VEL_MPS) ||
	    (tracker->inst_vel_mps > LATERAL_OCCUPANCY_MAX_INST_VEL_MPS))
	{
		return false;
	}

	return true;
}

static void bike_tracker_clear_track(bike_tracker_t *tracker)
{
	if (tracker == NULL)
	{
		return;
	}

	uint32_t idle_quiet_frames = tracker->idle_quiet_frames;

	tracker->has_track = false;
	tracker->track_age_frames = 0U;
	tracker->present_count = 0U;
	tracker->total_hit_count = 0U;
	tracker->entry_quiet_frames = 0U;
	tracker->idle_quiet_frames = idle_quiet_frames;
	tracker->miss_count = 0U;
	tracker->approach_count = 0U;
	tracker->strong_approach_count = 0U;
	tracker->stationary_count = 0U;
	tracker->recede_count = 0U;
	tracker->raw_range_m = 0.0f;
	tracker->filt_range_m = 0.0f;
	tracker->prev_filt_range_m = 0.0f;
	tracker->inst_vel_mps = 0.0f;
	tracker->filt_vel_mps = 0.0f;
	tracker->min_range_seen_m = 999.0f;
	tracker->max_range_seen_m = 0.0f;
	tracker->range_drop_m = 0.0f;
	tracker->ttc_s = 999.0f;
}

static bool has_close_proximity_alert_evidence(const bike_tracker_t *tracker)
{
	if ((tracker == NULL) || !tracker->has_track)
	{
		return false;
	}

	/* Use both raw and filtered range here.
	 * Filter lag was preventing real lateral passes from alerting until too late.
	 * Requiring both to be outside the entry range keeps this from becoming trigger-happy.
	 */
	if ((tracker->raw_range_m > CLOSE_RANGE_ALERT_M) &&
	    (tracker->filt_range_m > CLOSE_RANGE_ALERT_M))
	{
		return false;
	}

	if (tracker->present_count < CLOSE_RANGE_PRESENT_FRAMES)
	{
		return false;
	}

	if (tracker->recede_count > 0U)
	{
		return false;
	}

	return (tracker->range_drop_m >= CLOSE_RANGE_DROP_M) ||
	       (tracker->approach_count >= 1U) ||
	       (tracker->strong_approach_count >= 1U) ||
	       (tracker->inst_vel_mps <= ENTRY_INST_APPROACH_VEL_MPS);
}

static void bike_tracker_update(bike_tracker_t *tracker, const acc_detector_distance_result_t *result, float dt_s)
{
	if (dt_s < 0.001f)
	{
		dt_s = DEFAULT_FRAME_DT_S;
	}

	tracker->last_dt_s = dt_s;

	float selected_range_m = 0.0f;
	bool  valid_target = select_best_target(result, tracker, &selected_range_m);

	if (valid_target)
	{
		tracker->present_count++;
		tracker->total_hit_count++;
		tracker->miss_count = 0U;
		tracker->raw_range_m = selected_range_m;

		if (!tracker->has_track)
		{
			tracker->has_track = true;
			tracker->track_age_frames = 1U;
			tracker->entry_quiet_frames = tracker->idle_quiet_frames;
			tracker->idle_quiet_frames = 0U;
			tracker->filt_range_m = selected_range_m;
			tracker->prev_filt_range_m = selected_range_m;
			tracker->inst_vel_mps = 0.0f;
			tracker->filt_vel_mps = 0.0f;
			tracker->min_range_seen_m = selected_range_m;
			tracker->max_range_seen_m = selected_range_m;
			tracker->range_drop_m = 0.0f;
			tracker->ttc_s = 999.0f;
		}
		else
		{
			tracker->idle_quiet_frames = 0U;
			tracker->track_age_frames++;
			tracker->filt_range_m = (DIST_ALPHA * selected_range_m) +
			                        ((1.0f - DIST_ALPHA) * tracker->filt_range_m);
			tracker->inst_vel_mps = (tracker->filt_range_m - tracker->prev_filt_range_m) / dt_s;
			tracker->filt_vel_mps = (VEL_ALPHA * tracker->inst_vel_mps) +
			                      ((1.0f - VEL_ALPHA) * tracker->filt_vel_mps);
			tracker->prev_filt_range_m = tracker->filt_range_m;

			if (tracker->filt_range_m < tracker->min_range_seen_m)
			{
				tracker->min_range_seen_m = tracker->filt_range_m;
			}

			if (tracker->filt_range_m > tracker->max_range_seen_m)
			{
				tracker->max_range_seen_m = tracker->filt_range_m;
			}

			tracker->range_drop_m = tracker->max_range_seen_m - tracker->min_range_seen_m;
		}
	}
	else
	{
		tracker->miss_count++;
		tracker->present_count = 0U;
		tracker->approach_count = 0U;
		tracker->strong_approach_count = 0U;
		tracker->stationary_count = 0U;
		tracker->recede_count = 0U;

		if (tracker->has_track)
		{
			/* Soft decay instead of hard reset immediately. */
			tracker->filt_vel_mps *= 0.5f;
		}

		if (tracker->miss_count >= MAX_MISSED_FRAMES)
		{
			bike_tracker_clear_track(tracker);
		}

		if (!tracker->has_track &&
		    (tracker->idle_quiet_frames < 1000U) &&
		    !tracker->alert_active &&
		    (tracker->state != BIKE_STATE_HOLD) &&
		    (tracker->hold_count == 0U))
		{
			/* Do not immediately re-arm sparse entry while we are still inside an
			 * active alert/hold window. That was fragmenting one pass into multiple
			 * separate alerts.
			 */
			tracker->idle_quiet_frames++;
		}
	}

	if (tracker->has_track)
	{
		float ttc_s = 999.0f;
		bool  approach_motion = (tracker->filt_vel_mps <= APPROACH_VEL_MPS) ||
		                        (tracker->inst_vel_mps <= ENTRY_INST_APPROACH_VEL_MPS);
		bool  strong_approach = (tracker->filt_vel_mps <= STRONG_APPROACH_VEL_MPS) ||
		                        (tracker->inst_vel_mps <= (1.5f * ENTRY_INST_APPROACH_VEL_MPS));
		bool  receding_motion = (tracker->filt_vel_mps >= RECEDE_VEL_MPS) ||
		                        (tracker->inst_vel_mps >= EXIT_INST_RECEDE_VEL_MPS);
		bool  stationary_motion = fabsf(tracker->filt_vel_mps) <= STATIONARY_ABS_VEL_MPS;

		if (compute_target_ttc(tracker->filt_range_m, tracker->filt_vel_mps, &ttc_s))
		{
			tracker->ttc_s = ttc_s;
		}
		else
		{
			tracker->ttc_s = 999.0f;
		}

		if (approach_motion)
		{
			tracker->approach_count++;
			tracker->recede_count = 0U;
			if (!stationary_motion)
			{
				tracker->stationary_count = 0U;
			}
		}
		else if (receding_motion)
		{
			tracker->recede_count++;
			tracker->approach_count = 0U;
			tracker->strong_approach_count = 0U;
			tracker->stationary_count = 0U;
		}
		else if (stationary_motion)
		{
			tracker->stationary_count++;
			if (tracker->approach_count > 0U)
			{
				tracker->approach_count--;
			}
			if (tracker->recede_count > 0U)
			{
				tracker->recede_count--;
			}
		}
		else
		{
			/* Uncertain middle band: decay evidence slowly instead of hard reset. */
			if (tracker->approach_count > 0U)
			{
				tracker->approach_count--;
			}
			if (tracker->recede_count > 0U)
			{
				tracker->recede_count--;
			}
		}

		if (strong_approach)
		{
			tracker->strong_approach_count++;
		}
		else if (tracker->strong_approach_count > 0U)
		{
			tracker->strong_approach_count--;
		}
	}

	/* ---------------------- Alert state machine ---------------------- */
	switch (tracker->state)
	{
		case BIKE_STATE_IDLE:
			tracker->alert_active = false;
			tracker->hold_count = 0U;
			if (has_entry_shock_alert_evidence(tracker) ||
			    has_close_proximity_alert_evidence(tracker))
			{
				/* Keep true emergency cases immediate, but do not let the new
				 * widened sparse single-hit path jump straight to ALERT from IDLE.
				 * That was the source of the fragile first-hit alerts in v11.
				 */
				tracker->state = BIKE_STATE_ALERT;
				tracker->alert_active = true;
				tracker->hold_count = HOLD_FRAMES;
			}
			else if (has_sparse_single_frame_alert_evidence(tracker) ||
			         has_sparse_two_hit_alert_evidence(tracker) ||
			         has_lateral_occupancy_alert_evidence(tracker) ||
			         ((tracker->has_track && (tracker->track_age_frames >= CANDIDATE_MIN_PRESENT)) ||
			          has_sparse_entry_candidate_evidence(tracker)))
			{
				tracker->state = BIKE_STATE_CANDIDATE;
			}
			break;

		case BIKE_STATE_CANDIDATE:
			tracker->alert_active = false;
			tracker->hold_count = 0U;

			if (!tracker->has_track)
			{
				tracker->state = BIKE_STATE_IDLE;
			}
			else if (tracker->recede_count >= RECEDE_CLEAR_FRAMES)
			{
				bike_tracker_clear_track(tracker);
				tracker->state = BIKE_STATE_IDLE;
			}
			else if ((tracker->stationary_count >= STATIONARY_CLEAR_FRAMES) &&
			         !has_closing_evidence(tracker, MIN_RANGE_DROP_TO_APPROACH_M))
			{
				bike_tracker_clear_track(tracker);
				tracker->state = BIKE_STATE_IDLE;
			}
			else if (has_entry_shock_alert_evidence(tracker) ||
			         has_sparse_two_hit_alert_evidence(tracker) ||
			         has_lateral_occupancy_alert_evidence(tracker) ||
			         has_close_proximity_alert_evidence(tracker))
			{
				/* Very close lateral traffic may have weak radial velocity near closest approach.
				 * Also allow a fast entry shock to alert before the filtered range fully catches up.
				 */
				tracker->state = BIKE_STATE_ALERT;
				tracker->alert_active = true;
				tracker->hold_count = HOLD_FRAMES;
			}
			else if ((tracker->total_hit_count >= TOTAL_HITS_FOR_ALERT) &&
			         has_immediate_closing_evidence(tracker, MIN_RANGE_DROP_TO_ALERT_M) &&
			         ((tracker->filt_range_m <= ALERT_RANGE_M) || (tracker->ttc_s <= MAX_TTC_ALERT_S)))
			{
				/* Roadside tuning: allow a direct CAND -> ALERT jump on one credible closing frame. */
				tracker->state = BIKE_STATE_ALERT;
				tracker->alert_active = true;
				tracker->hold_count = HOLD_FRAMES;
			}
			else if ((tracker->total_hit_count >= TOTAL_HITS_FOR_ALERT) &&
			         (((tracker->approach_count >= APPROACH_MIN_FRAMES) &&
			           has_closing_evidence(tracker, MIN_RANGE_DROP_TO_APPROACH_M)) ||
			          ((tracker->strong_approach_count >= 1U) &&
			           has_closing_evidence(tracker, MIN_RANGE_DROP_TO_APPROACH_M))))
			{
				tracker->state = BIKE_STATE_APPROACHING;
			}
			break;

		case BIKE_STATE_APPROACHING:
			tracker->alert_active = false;
			tracker->hold_count = 0U;

			if (!tracker->has_track)
			{
				tracker->state = BIKE_STATE_IDLE;
			}
			else if (tracker->recede_count >= RECEDE_CLEAR_FRAMES)
			{
				bike_tracker_clear_track(tracker);
				tracker->state = BIKE_STATE_IDLE;
			}
			else if ((tracker->stationary_count >= STATIONARY_CLEAR_FRAMES) &&
			         !has_closing_evidence(tracker, MIN_RANGE_DROP_TO_ALERT_M))
			{
				bike_tracker_clear_track(tracker);
				tracker->state = BIKE_STATE_IDLE;
			}
			else if (has_entry_shock_alert_evidence(tracker) ||
			         has_sparse_two_hit_alert_evidence(tracker) ||
			         has_lateral_occupancy_alert_evidence(tracker) ||
			         has_close_proximity_alert_evidence(tracker) ||
			         ((has_immediate_closing_evidence(tracker, MIN_RANGE_DROP_TO_ALERT_M) ||
			           ((tracker->approach_count >= ALERT_MIN_FRAMES) &&
			            has_closing_evidence(tracker, MIN_RANGE_DROP_TO_ALERT_M)) ||
			           ((tracker->strong_approach_count >= 1U) &&
			            has_closing_evidence(tracker, MIN_RANGE_DROP_TO_APPROACH_M))) &&
			          ((tracker->filt_range_m <= ALERT_RANGE_M) || (tracker->ttc_s <= MAX_TTC_ALERT_S))))
			{
				tracker->state = BIKE_STATE_ALERT;
				tracker->alert_active = true;
				tracker->hold_count = HOLD_FRAMES;
			}
			break;

		case BIKE_STATE_ALERT:
			tracker->alert_active = true;

			if (tracker->has_track)
			{
				tracker->hold_count = HOLD_FRAMES;
			}
			else if (tracker->hold_count > 0U)
			{
				tracker->hold_count--;
			}

			if ((tracker->recede_count >= RECEDE_CLEAR_FRAMES) ||
			    (!tracker->has_track && (tracker->hold_count == 0U)) ||
			    (tracker->has_track && (tracker->filt_range_m > EXIT_RANGE_M)))
			{
				if (tracker->recede_count >= RECEDE_CLEAR_FRAMES)
				{
					bike_tracker_clear_track(tracker);
				}
				tracker->state = BIKE_STATE_IDLE;
				tracker->alert_active = false;
				tracker->hold_count = 0U;
			}
			else if (tracker->has_track && (fabsf(tracker->filt_vel_mps) <= STATIONARY_ABS_VEL_MPS))
			{
				tracker->state = BIKE_STATE_HOLD;
				tracker->alert_active = true;
			}
			break;

		case BIKE_STATE_HOLD:
			tracker->alert_active = true;

			if (tracker->has_track)
			{
				if ((tracker->recede_count >= RECEDE_CLEAR_FRAMES) ||
				    (tracker->filt_range_m > EXIT_RANGE_M))
				{
					if (tracker->recede_count >= RECEDE_CLEAR_FRAMES)
					{
						bike_tracker_clear_track(tracker);
					}
					tracker->state = BIKE_STATE_IDLE;
					tracker->alert_active = false;
					tracker->hold_count = 0U;
				}
				else if (has_entry_shock_alert_evidence(tracker) ||
				         (has_closing_evidence(tracker, MIN_RANGE_DROP_TO_APPROACH_M) &&
				          (tracker->filt_vel_mps <= APPROACH_VEL_MPS)))
				{
					tracker->state = BIKE_STATE_ALERT;
					tracker->alert_active = true;
					tracker->hold_count = HOLD_FRAMES;
				}
				else
				{
					if (tracker->hold_count > 0U)
					{
						tracker->hold_count--;
					}
					if (tracker->hold_count == 0U)
					{
						tracker->state = BIKE_STATE_IDLE;
						tracker->alert_active = false;
					}
				}
			}
			else
			{
				if (tracker->hold_count > 0U)
				{
					tracker->hold_count--;
				}
				if (tracker->hold_count == 0U)
				{
					tracker->state = BIKE_STATE_IDLE;
					tracker->alert_active = false;
				}
			}
			break;

		default:
			tracker->state = BIKE_STATE_IDLE;
			tracker->alert_active = false;
			tracker->hold_count = 0U;
			break;
	}

}

static const char *state_to_string(bike_state_t state)
{
	switch (state)
	{
		case BIKE_STATE_IDLE:        return "IDLE";
		case BIKE_STATE_CANDIDATE:   return "CAND";
		case BIKE_STATE_APPROACHING: return "APP";
		case BIKE_STATE_ALERT:       return "ALERT";
		case BIKE_STATE_HOLD:        return "HOLD";
		default:                     return "UNK";
	}
}

static void print_bike_status(const acc_detector_distance_result_t *result, const bike_tracker_t *tracker)
{
	printf("FRAME: num=%u ", result->num_distances);

	if (result->num_distances > 0)
	{
		printf("raws=[");
		for (uint8_t i = 0; i < result->num_distances; i++)
		{
			printf("%" PRIfloat, ACC_LOG_FLOAT_TO_INTEGER(result->distances[i]));
			if (i + 1U < result->num_distances)
			{
				printf(",");
			}
		}
		printf("] ");
	}

	printf("have=%d age=%lu hits=%lu q=%lu eq=%lu dt=%" PRIfloat " raw=%" PRIfloat " filt=%" PRIfloat
	       " instV=%" PRIfloat " filtV=%" PRIfloat
	       " ttc=%" PRIfloat " minR=%" PRIfloat " maxR=%" PRIfloat " drop=%" PRIfloat
	       " p=%lu a=%lu sa=%lu s=%lu r=%lu miss=%lu hold=%lu st=%s alert=%d\n",
	       tracker->has_track ? 1 : 0,
	       (unsigned long)tracker->track_age_frames,
	       (unsigned long)tracker->total_hit_count,
	       (unsigned long)tracker->idle_quiet_frames,
	       (unsigned long)tracker->entry_quiet_frames,
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->last_dt_s),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->raw_range_m),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->filt_range_m),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->inst_vel_mps),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->filt_vel_mps),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->ttc_s),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->min_range_seen_m),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->max_range_seen_m),
	       ACC_LOG_FLOAT_TO_INTEGER(tracker->range_drop_m),
	       (unsigned long)tracker->present_count,
	       (unsigned long)tracker->approach_count,
	       (unsigned long)tracker->strong_approach_count,
	       (unsigned long)tracker->stationary_count,
	       (unsigned long)tracker->recede_count,
	       (unsigned long)tracker->miss_count,
	       (unsigned long)tracker->hold_count,
	       state_to_string(tracker->state),
	       tracker->alert_active ? 1 : 0);
}
