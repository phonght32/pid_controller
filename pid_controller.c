#include "stdlib.h"
#include "pid_controller.h"

typedef struct pid_controller {
	float kp;					/*!< Proportional gain */
	float ki;					/*!< Integral gain */
	float kd;					/*!< Derivative gain */
	float tau;					/*!< Derivative low pass filter time constant */
	float lim_min;				/*!< Output limit min */
	float lim_max;				/*!< Output limit max */
	float int_lim_min;			/*!< Integrator limit min */
	float int_lim_max;			/*!< Integrator limit max */
	float sample_time;			/*!< Sample time */
	float prev_error;			/*!< Previous error */
	float prev_measurement;		/*!< Previous measurement */
	float integrator;			/*!< Integrator */
	float differentiator;		/*!< Differentiator */
} pid_controller_t;

pid_controller_handle_t pid_controller_init(void)
{
	pid_controller_handle_t handle = calloc(1, sizeof(pid_controller_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

pid_controller_status_t pid_controller_set_config(pid_controller_handle_t handle, pid_controller_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return PID_CONTROLLER_STATUS_INVALID_ARG;
	}

	handle->kp = config.kp;
	handle->ki = config.ki;
	handle->kd = config.kd;
	handle->tau = config.tau;
	handle->lim_min = config.lim_min;
	handle->lim_max = config.lim_max;
	handle->int_lim_min = config.int_lim_min;
	handle->int_lim_max = config.int_lim_max;
	handle->sample_time = config.sample_time;
	handle->prev_error = 0;
	handle->prev_measurement = 0;
	handle->integrator = 0;
	handle->differentiator = 0;

	return PID_CONTROLLER_STATUS_SUCCESS;
}

pid_controller_status_t pid_controller_config(pid_controller_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return PID_CONTROLLER_STATUS_INVALID_ARG;
	}

	return PID_CONTROLLER_STATUS_SUCCESS;
}

pid_controller_status_t pid_controller_update(pid_controller_handle_t handle, float set_point, float measurement, float *pid_out)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return PID_CONTROLLER_STATUS_INVALID_ARG;
	}

	float error, proportional, integrator, output;

	/* Calculate error signal */
	error = set_point - measurement;

	/* Calculate proportional */
	proportional = handle->kp * error;

	/* Calculate integral */
	integrator = handle->integrator + 0.5f * handle->ki * handle->sample_time * (error + handle->prev_error);

	/* Anti-wind-up via integrator clamping */
	if (integrator > handle->int_lim_max)
	{
		handle->integrator = handle->int_lim_max;
	}
	else if (integrator < handle->int_lim_min)
	{
		handle->integrator = handle->int_lim_min;
	}
	else
	{
		handle->integrator = integrator;
	}

	/* Derivative (band-limited differentiator) */
	/* Note: derivative on measurement, therefore minus sign in front of equation! */
	handle->differentiator = - (2.0f * handle->kd * (measurement - handle->prev_measurement)
	                            + (2.0f * handle->tau - handle->sample_time) * handle->differentiator)
	                         / (2.0f * handle->tau + handle->sample_time);

	/* Compute output and apply limits */
	output = proportional + handle->integrator + handle->differentiator;

	if (output > handle->lim_max)
	{
		*pid_out = handle->lim_max;
	}
	else if (output < handle->lim_min)
	{
		*pid_out = handle->lim_min;
	}
	else
	{
		*pid_out = output;
	}

	/* Store error and measurement for later use */
	handle->prev_error = error;
	handle->prev_measurement = measurement;

	return PID_CONTROLLER_STATUS_SUCCESS;
}
