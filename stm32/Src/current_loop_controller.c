#include "current_loop_controller.h"

#include <math.h>
#include <stddef.h>

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static bool is_same_sign(float a, float b)
{
    return ((a > 0.0f) && (b > 0.0f)) || ((a < 0.0f) && (b < 0.0f));
}

static CurrentLoop_Status validate_args(
    const CurrentLoop_State *state,
    const CurrentLoop_Config *config,
    const CurrentLoop_Input *input,
    const CurrentLoop_Output *output)
{
    if ((state == NULL) || (config == NULL) || (input == NULL) || (output == NULL)) {
        return CURRENT_LOOP_ERROR_NULL;
    }

    if ((config->voltage_limit <= 0.0f) ||
        (config->d_axis.integrator_min > config->d_axis.integrator_max) ||
        (config->q_axis.integrator_min > config->q_axis.integrator_max)) {
        return CURRENT_LOOP_ERROR_BAD_LIMIT;
    }

    if ((input->ts <= 0.0f) ||
        (config->ts_min <= 0.0f) ||
        (config->ts_max < config->ts_min) ||
        (input->ts < config->ts_min) ||
        (input->ts > config->ts_max)) {
        return CURRENT_LOOP_ERROR_BAD_TS;
    }

    return CURRENT_LOOP_OK;
}

static float pi_preview(
    PIController_State *state,
    const PIController_Config *config,
    float error)
{
    state->error = error;
    state->p_term = config->kp * error;
    state->i_term = state->integrator;
    state->output_raw = state->p_term + state->integrator;
    return state->output_raw;
}

static void pi_commit_integrator(
    PIController_State *state,
    const PIController_Config *config,
    float ts,
    float saturated_output)
{
    const bool saturated = fabsf(saturated_output - state->output_raw) > 1.0e-6f;

    state->saturated = saturated;

    /*
     * Conditional integration:
     * - integrate normally while not saturated;
     * - while saturated, integrate only if the error moves output back
     *   toward the linear region.
     */
    if (!saturated || !is_same_sign(state->error, state->output_raw)) {
        const float next_integrator = state->integrator + (config->ki * state->error * ts);
        state->integrator = clampf(next_integrator, config->integrator_min, config->integrator_max);
    }

    state->i_term = state->integrator;
    state->output = saturated_output;
}

void CurrentLoop_Controller_Init(CurrentLoop_State *state)
{
    if (state == NULL) {
        return;
    }

    state->d_axis.integrator = 0.0f;
    state->d_axis.p_term = 0.0f;
    state->d_axis.i_term = 0.0f;
    state->d_axis.output_raw = 0.0f;
    state->d_axis.output = 0.0f;
    state->d_axis.error = 0.0f;
    state->d_axis.saturated = false;

    state->q_axis.integrator = 0.0f;
    state->q_axis.p_term = 0.0f;
    state->q_axis.i_term = 0.0f;
    state->q_axis.output_raw = 0.0f;
    state->q_axis.output = 0.0f;
    state->q_axis.error = 0.0f;
    state->q_axis.saturated = false;

    state->voltage_vector_saturated = false;
    state->update_counter = 0U;
}

CurrentLoop_Status CurrentLoop_Controller_Update(
    CurrentLoop_State *state,
    const CurrentLoop_Config *config,
    const CurrentLoop_Input *input,
    CurrentLoop_Output *output)
{
    const CurrentLoop_Status status = validate_args(state, config, input, output);
    if (status != CURRENT_LOOP_OK) {
        if (output != NULL) {
            output->ud = 0.0f;
            output->uq = 0.0f;
            output->vd_raw = 0.0f;
            output->vq_raw = 0.0f;
            output->voltage_magnitude = 0.0f;
            output->status = status;
        }
        return status;
    }

    const float err_d = input->id_ref - input->id_meas;
    const float err_q = input->iq_ref - input->iq_meas;

    float ud = pi_preview(&state->d_axis, &config->d_axis, err_d);
    float uq = pi_preview(&state->q_axis, &config->q_axis, err_q);

    output->vd_raw = ud;
    output->vq_raw = uq;

    const float voltage_sq = (ud * ud) + (uq * uq);
    const float voltage_limit_sq = config->voltage_limit * config->voltage_limit;

    state->voltage_vector_saturated = false;
    if (voltage_sq > voltage_limit_sq) {
        const float magnitude = sqrtf(voltage_sq);
        const float scale = config->voltage_limit / magnitude;
        ud *= scale;
        uq *= scale;
        state->voltage_vector_saturated = true;
    }

    pi_commit_integrator(&state->d_axis, &config->d_axis, input->ts, ud);
    pi_commit_integrator(&state->q_axis, &config->q_axis, input->ts, uq);

    output->ud = ud;
    output->uq = uq;
    output->voltage_magnitude = sqrtf((ud * ud) + (uq * uq));
    output->status = CURRENT_LOOP_OK;

    state->update_counter++;

    return CURRENT_LOOP_OK;
}
