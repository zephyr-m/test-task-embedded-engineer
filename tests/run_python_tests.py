#!/usr/bin/env python3
import math
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_DIR = PROJECT_ROOT / "simulation"
sys.path.insert(0, str(SIMULATION_DIR))

from current_loop_sim import CurrentLoopController, simulate_axis  # noqa: E402
from pi_controller import PIConfig, PIController, clamp  # noqa: E402


@dataclass
class TestResult:
    group: str
    name: str
    passed: bool
    message: str = ""


class TestRunner:
    def __init__(self) -> None:
        self.results: list[TestResult] = []

    def check(self, group: str, name: str, condition: bool, message: str = "") -> None:
        self.results.append(TestResult(group, name, condition, message))

    def close(self, actual: float, expected: float, tolerance: float) -> bool:
        return abs(actual - expected) <= tolerance

    def report(self) -> int:
        current_group = None
        passed = 0
        failed = 0

        print("Python behavioral tests")
        print("=======================")

        for result in self.results:
            if result.group != current_group:
                current_group = result.group
                print(f"\n[{current_group}]")

            status = "PASS" if result.passed else "FAIL"
            print(f"  {status} {result.name}")
            if not result.passed and result.message:
                print(f"       {result.message}")

            passed += int(result.passed)
            failed += int(not result.passed)

        total = passed + failed
        print("\nSummary")
        print("-------")
        print(f"Total:  {total}")
        print(f"Passed: {passed}")
        print(f"Failed: {failed}")

        return 0 if failed == 0 else 1


def test_clamp(runner: TestRunner) -> None:
    group = "utility"
    runner.check(group, "clamp keeps value inside range", clamp(0.5, -1.0, 1.0) == 0.5)
    runner.check(group, "clamp limits lower bound", clamp(-2.0, -1.0, 1.0) == -1.0)
    runner.check(group, "clamp limits upper bound", clamp(2.0, -1.0, 1.0) == 1.0)


def test_pi_controller(runner: TestRunner) -> None:
    group = "pi_controller"
    config = PIConfig(kp=2.0, ki=10.0, integrator_min=-5.0, integrator_max=5.0)

    controller = PIController(config)
    raw = controller.preview(reference=1.0, measurement=0.75)
    controller.commit(dt=0.1, saturated_output=raw)

    runner.check(group, "preview computes proportional plus integrator", runner.close(raw, 0.5, 1e-12))
    runner.check(
        group,
        "integrator updates when output is not saturated",
        runner.close(controller.state.integrator, 0.25, 1e-12),
        f"integrator={controller.state.integrator}",
    )
    runner.check(group, "unsaturated output is not flagged", not controller.state.saturated)

    controller = PIController(config)
    raw = controller.preview(reference=10.0, measurement=0.0)
    controller.commit(dt=0.1, saturated_output=1.0)
    runner.check(group, "positive saturation is detected", controller.state.saturated)
    runner.check(
        group,
        "integrator freezes when saturated in same direction",
        runner.close(controller.state.integrator, 0.0, 1e-12),
        f"integrator={controller.state.integrator}",
    )

    controller.state.error = -1.0
    controller.state.output_raw = 10.0
    controller.commit(dt=0.1, saturated_output=1.0)
    runner.check(
        group,
        "integrator unwinds when error opposes saturation",
        controller.state.integrator < 0.0,
        f"integrator={controller.state.integrator}",
    )

    controller = PIController(PIConfig(kp=0.0, ki=100.0, integrator_min=-0.5, integrator_max=0.5))
    for _ in range(20):
        raw = controller.preview(reference=1.0, measurement=0.0)
        controller.commit(dt=0.1, saturated_output=raw)
    runner.check(
        group,
        "integrator is clamped to configured maximum",
        runner.close(controller.state.integrator, 0.5, 1e-12),
        f"integrator={controller.state.integrator}",
    )


def test_current_loop(runner: TestRunner) -> None:
    group = "current_loop"
    voltage_limit = 12.0
    controller = CurrentLoopController(
        d_axis=PIConfig(kp=2.0, ki=900.0, integrator_min=-voltage_limit, integrator_max=voltage_limit),
        q_axis=PIConfig(kp=2.0, ki=900.0, integrator_min=-voltage_limit, integrator_max=voltage_limit),
        voltage_limit=voltage_limit,
    )

    ud, uq = controller.update(id_ref=0.0, iq_ref=100.0, id_meas=0.0, iq_meas=0.0, dt=50e-6)
    magnitude = math.hypot(ud, uq)
    runner.check(group, "voltage vector saturation is flagged", controller.vector_saturated)
    runner.check(
        group,
        "voltage vector magnitude is limited",
        magnitude <= voltage_limit + 1e-9,
        f"magnitude={magnitude}",
    )
    runner.check(group, "vector direction is preserved for q-axis command", abs(ud) < 1e-9 and uq > 0.0)

    controller = CurrentLoopController(
        d_axis=PIConfig(kp=2.0, ki=900.0, integrator_min=-voltage_limit, integrator_max=voltage_limit),
        q_axis=PIConfig(kp=2.0, ki=900.0, integrator_min=-voltage_limit, integrator_max=voltage_limit),
        voltage_limit=voltage_limit,
    )
    id_current = 0.0
    iq_current = 0.0
    dt = 50e-6
    resistance = 0.45
    inductance = 0.00035

    for step in range(int(0.05 / dt)):
        time_s = step * dt
        iq_ref = 0.0 if time_s < 0.005 else 2.5
        ud, uq = controller.update(0.0, iq_ref, id_current, iq_current, dt)
        id_current = simulate_axis(id_current, ud, resistance, inductance, dt)
        iq_current = simulate_axis(iq_current, uq, resistance, inductance, dt)

    runner.check(
        group,
        "simulation tracks Iq reference",
        abs(iq_current - 2.5) < 0.02,
        f"iq_current={iq_current}",
    )
    runner.check(
        group,
        "simulation keeps Id near zero",
        abs(id_current) < 0.001,
        f"id_current={id_current}",
    )


def main() -> int:
    runner = TestRunner()
    test_clamp(runner)
    test_pi_controller(runner)
    test_current_loop(runner)
    return runner.report()


if __name__ == "__main__":
    raise SystemExit(main())

