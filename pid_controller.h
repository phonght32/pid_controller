// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __PID_CONTROLLER_H__
#define __PID_CONTROLLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

/**
 * @brief   Handle structure.
 */
typedef struct pid_controller *pid_controller_handle_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	float kp;					/*!< Proportional gain */
	float ki;					/*!< Integral gain */
	float kd;					/*!< Derivative gain */
	float tau;					/*!< Derivative low pass filter time constant */
	float lim_min;				/*!< Output limit min */
	float lim_max;				/*!< Output limit max */
	float int_lim_min;			/*!< Integrator limit min */
	float int_lim_max;			/*!< Integrator limit max */
	float sample_time;			/*!< Sample time */
} pid_controller_cfg_t;

/*
 * @brief   Initialize PID controller with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
pid_controller_handle_t pid_controller_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t pid_controller_set_config(pid_controller_handle_t handle, pid_controller_cfg_t config);

/*
 * @brief   Configure PID controller to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t pid_controller_config(pid_controller_handle_t handle);

/*
 * @brief   Update PID controller.
 *
 * @param 	handle Handle structure.
 * @param 	set_point Set point.
 * @param 	measurement Measurement value.
 * @param 	pid_out Output value.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t pid_controller_update(pid_controller_handle_t handle, float set_point, float measurement, float *pid_out);

#ifdef __cplusplus
}
#endif

#endif /* __PID_CONTROLLER_H__ */