import csv
import math
from pathlib import Path

from pi_controller import PIConfig, PIController


class CurrentLoopController:
    def __init__(self, d_axis: PIConfig, q_axis: PIConfig, voltage_limit: float):
        self.d_axis = PIController(d_axis)
        self.q_axis = PIController(q_axis)
        self.voltage_limit = voltage_limit
        self.vector_saturated = False

    def update(
        self,
        id_ref: float,
        iq_ref: float,
        id_meas: float,
        iq_meas: float,
        dt: float,
    ) -> tuple[float, float]:
        ud = self.d_axis.preview(id_ref, id_meas)
        uq = self.q_axis.preview(iq_ref, iq_meas)

        magnitude = math.hypot(ud, uq)
        self.vector_saturated = magnitude > self.voltage_limit
        if self.vector_saturated:
            scale = self.voltage_limit / magnitude
            ud *= scale
            uq *= scale

        self.d_axis.commit(dt, ud)
        self.q_axis.commit(dt, uq)
        return ud, uq


def simulate_axis(current: float, voltage: float, resistance: float, inductance: float, dt: float) -> float:
    di_dt = (voltage - resistance * current) / inductance
    return current + di_dt * dt


def main() -> None:
    dt = 50e-6
    sim_time = 0.05
    steps = int(sim_time / dt)

    resistance = 0.45
    inductance = 0.00035
    voltage_limit = 12.0

    controller = CurrentLoopController(
        d_axis=PIConfig(kp=2.0, ki=900.0, integrator_min=-voltage_limit, integrator_max=voltage_limit),
        q_axis=PIConfig(kp=2.0, ki=900.0, integrator_min=-voltage_limit, integrator_max=voltage_limit),
        voltage_limit=voltage_limit,
    )

    id_current = 0.0
    iq_current = 0.0

    rows = []
    for step in range(steps):
        time_s = step * dt
        id_ref = 0.0
        iq_ref = 0.0 if time_s < 0.005 else 2.5

        ud, uq = controller.update(id_ref, iq_ref, id_current, iq_current, dt)

        id_current = simulate_axis(id_current, ud, resistance, inductance, dt)
        iq_current = simulate_axis(iq_current, uq, resistance, inductance, dt)

        rows.append(
            {
                "time_s": time_s,
                "id_ref_a": id_ref,
                "iq_ref_a": iq_ref,
                "id_a": id_current,
                "iq_a": iq_current,
                "ud_v": ud,
                "uq_v": uq,
                "sat": int(controller.vector_saturated),
                "d_integrator": controller.d_axis.state.integrator,
                "q_integrator": controller.q_axis.state.integrator,
            }
        )

    output_path = Path(__file__).with_name("current_loop_result.csv")
    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    final = rows[-1]
    print(f"Saved: {output_path}")
    print(f"Final Id: {final['id_a']:.3f} A, Iq: {final['iq_a']:.3f} A")
    print(f"Final Ud: {final['ud_v']:.3f} V, Uq: {final['uq_v']:.3f} V")


if __name__ == "__main__":
    main()

