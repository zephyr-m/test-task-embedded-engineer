#include "current_loop_controller.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct {
    unsigned passed;
    unsigned failed;
} TestStats;

static void check(TestStats *stats, const char *group, const char *name, bool condition)
{
    printf("  %s %s\n", condition ? "PASS" : "FAIL", name);
    if (!condition) {
        printf("       group: %s\n", group);
    }
    stats->passed += condition ? 1U : 0U;
    stats->failed += condition ? 0U : 1U;
}

static bool closef(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

static CurrentLoop_Config default_config(void)
{
    CurrentLoop_Config config;

    config.d_axis.kp = 2.0f;
    config.d_axis.ki = 900.0f;
    config.d_axis.integrator_min = -12.0f;
    config.d_axis.integrator_max = 12.0f;

    config.q_axis.kp = 2.0f;
    config.q_axis.ki = 900.0f;
    config.q_axis.integrator_min = -12.0f;
    config.q_axis.integrator_max = 12.0f;

    config.voltage_limit = 12.0f;
    config.ts_min = 20.0e-6f;
    config.ts_max = 50.0e-6f;

    return config;
}

static void test_validation(TestStats *stats)
{
    printf("\n[validation]\n");

    CurrentLoop_State state;
    CurrentLoop_Output output;
    CurrentLoop_Config config = default_config();
    CurrentLoop_Input input = {
        .id_ref = 0.0f,
        .iq_ref = 0.0f,
        .id_meas = 0.0f,
        .iq_meas = 0.0f,
        .ts = 50.0e-6f,
    };

    CurrentLoop_Controller_Init(&state);
    check(
        stats,
        "validation",
        "null state is rejected",
        CurrentLoop_Controller_Update(NULL, &config, &input, &output) == CURRENT_LOOP_ERROR_NULL);

    input.ts = 1.0e-3f;
    check(
        stats,
        "validation",
        "out-of-range Ts is rejected",
        CurrentLoop_Controller_Update(&state, &config, &input, &output) == CURRENT_LOOP_ERROR_BAD_TS);

    input.ts = 50.0e-6f;
    config.voltage_limit = 0.0f;
    check(
        stats,
        "validation",
        "bad voltage limit is rejected",
        CurrentLoop_Controller_Update(&state, &config, &input, &output) == CURRENT_LOOP_ERROR_BAD_LIMIT);
}

static void test_zero_command(TestStats *stats)
{
    printf("\n[zero_command]\n");

    CurrentLoop_State state;
    CurrentLoop_Output output;
    CurrentLoop_Config config = default_config();
    CurrentLoop_Input input = {
        .id_ref = 0.0f,
        .iq_ref = 0.0f,
        .id_meas = 0.0f,
        .iq_meas = 0.0f,
        .ts = 50.0e-6f,
    };

    CurrentLoop_Controller_Init(&state);
    CurrentLoop_Status status = CurrentLoop_Controller_Update(&state, &config, &input, &output);

    check(stats, "zero_command", "zero command returns OK", status == CURRENT_LOOP_OK);
    check(stats, "zero_command", "Ud is zero", closef(output.ud, 0.0f, 1.0e-6f));
    check(stats, "zero_command", "Uq is zero", closef(output.uq, 0.0f, 1.0e-6f));
    check(stats, "zero_command", "update counter increments", state.update_counter == 1U);
}

static void test_voltage_limit(TestStats *stats)
{
    printf("\n[voltage_limit]\n");

    CurrentLoop_State state;
    CurrentLoop_Output output;
    CurrentLoop_Config config = default_config();
    CurrentLoop_Input input = {
        .id_ref = 0.0f,
        .iq_ref = 100.0f,
        .id_meas = 0.0f,
        .iq_meas = 0.0f,
        .ts = 50.0e-6f,
    };

    CurrentLoop_Controller_Init(&state);
    CurrentLoop_Status status = CurrentLoop_Controller_Update(&state, &config, &input, &output);

    check(stats, "voltage_limit", "large command returns OK", status == CURRENT_LOOP_OK);
    check(stats, "voltage_limit", "vector saturation is flagged", state.voltage_vector_saturated);
    check(stats, "voltage_limit", "voltage magnitude is limited", output.voltage_magnitude <= config.voltage_limit + 1.0e-5f);
    check(stats, "voltage_limit", "q-axis direction is preserved", closef(output.ud, 0.0f, 1.0e-6f) && output.uq > 0.0f);
    check(stats, "voltage_limit", "q integrator freezes during same-direction saturation", closef(state.q_axis.integrator, 0.0f, 1.0e-6f));
}

static void test_integrator(TestStats *stats)
{
    printf("\n[anti_windup]\n");

    CurrentLoop_State state;
    CurrentLoop_Output output;
    CurrentLoop_Config config = default_config();
    CurrentLoop_Input input = {
        .id_ref = 0.0f,
        .iq_ref = 2.5f,
        .id_meas = 0.0f,
        .iq_meas = 2.0f,
        .ts = 50.0e-6f,
    };

    CurrentLoop_Controller_Init(&state);
    CurrentLoop_Status status = CurrentLoop_Controller_Update(&state, &config, &input, &output);

    check(stats, "anti_windup", "small command returns OK", status == CURRENT_LOOP_OK);
    check(stats, "anti_windup", "unsaturated command is not vector-limited", !state.voltage_vector_saturated);
    check(stats, "anti_windup", "q integrator increases for positive error", state.q_axis.integrator > 0.0f);

    state.q_axis.error = -1.0f;
    state.q_axis.output_raw = 20.0f;
    state.q_axis.integrator = 0.0f;

    input.iq_ref = -1.0f;
    input.iq_meas = 0.0f;
    status = CurrentLoop_Controller_Update(&state, &config, &input, &output);

    check(stats, "anti_windup", "opposite error update returns OK", status == CURRENT_LOOP_OK);
    check(stats, "anti_windup", "negative error can reduce q integrator", state.q_axis.integrator < 0.1f);
}

int main(void)
{
    TestStats stats = {0U, 0U};

    printf("C behavioral tests\n");
    printf("==================\n");

    test_validation(&stats);
    test_zero_command(&stats);
    test_voltage_limit(&stats);
    test_integrator(&stats);

    printf("\nSummary\n");
    printf("-------\n");
    printf("Total:  %u\n", stats.passed + stats.failed);
    printf("Passed: %u\n", stats.passed);
    printf("Failed: %u\n", stats.failed);

    return stats.failed == 0U ? 0 : 1;
}

