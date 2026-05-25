#ifndef CURRENT_LOOP_CONTROLLER_H
#define CURRENT_LOOP_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CURRENT_LOOP_OK = 0,
    CURRENT_LOOP_ERROR_NULL = -1,
    CURRENT_LOOP_ERROR_BAD_TS = -2,
    CURRENT_LOOP_ERROR_BAD_LIMIT = -3
} CurrentLoop_Status;

typedef struct {
    float kp;
    float ki;
    float integrator_min;
    float integrator_max;
} PIController_Config;

typedef struct {
    float integrator;
    float p_term;
    float i_term;
    float output_raw;
    float output;
    float error;
    bool saturated;
} PIController_State;

typedef struct {
    PIController_Config d_axis;
    PIController_Config q_axis;
    float voltage_limit;
    float ts_min;
    float ts_max;
} CurrentLoop_Config;

typedef struct {
    PIController_State d_axis;
    PIController_State q_axis;
    bool voltage_vector_saturated;
    uint32_t update_counter;
} CurrentLoop_State;

typedef struct {
    float id_ref;
    float iq_ref;
    float id_meas;
    float iq_meas;
    float ts;
} CurrentLoop_Input;

typedef struct {
    float ud;
    float uq;
    float vd_raw;
    float vq_raw;
    float voltage_magnitude;
    CurrentLoop_Status status;
} CurrentLoop_Output;

void CurrentLoop_Controller_Init(CurrentLoop_State *state);

CurrentLoop_Status CurrentLoop_Controller_Update(
    CurrentLoop_State *state,
    const CurrentLoop_Config *config,
    const CurrentLoop_Input *input,
    CurrentLoop_Output *output);

#ifdef __cplusplus
}
#endif

#endif

