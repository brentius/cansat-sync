"""GPL-3.0-only. Detection latency of the flight state machine on synthetic flights.

Writes results/flight_state_mc.json. Simulated stimulus only: these numbers
describe the logic under the modelled launch profile, not a real launch.
"""
import json
import statistics
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1]/'code'))
sys.path.insert(0, str(HERE.parents[1]/'code'/'tests'))
from cansat.flight_state import FlightStateMachine  # noqa: E402
from sim_flight import random_flight  # noqa: E402

CASES = {
    'both sensors': {},
    'barometer only': {'accel_ok': False},
    'accelerometer only': {'baro_ok': False},
    'no ejection shock': {'eject_spike_g': 0.0},
    'both sensors at 20 Hz': {'dt': 0.05},
}
FLIGHTS = 500


def summary(values):
    ordered = sorted(values)
    return {'min': ordered[0], 'median': statistics.median(ordered),
            'p95': ordered[int(0.95*(len(ordered) - 1))], 'max': ordered[-1]}


def main():
    results = {'flights_per_case': FLIGHTS}
    for name, overrides in CASES.items():
        launch, descent, landed, apogee, reasons, wrong = [], [], [], [], {}, 0
        for seed in range(FLIGHTS):
            samples, truth = random_flight(seed, **overrides)
            machine = FlightStateMachine()
            for t, accel, pressure in samples:
                machine.update(t, accel, pressure)
            if [e[1] for e in machine.events] != ['ASCENT', 'DESCENT', 'LANDED']:
                wrong += 1
                continue
            launch.append(machine.t_launch - truth['t_liftoff'])
            descent.append(machine.t_descent - truth['t_eject'])
            landed.append(machine.t_landed - truth['t_touchdown'])
            if machine.max_altitude == machine.max_altitude:
                apogee.append(machine.max_altitude - truth['apogee_m'])
            for _, state, reason in machine.events:
                key = state + ' by ' + reason
                reasons[key] = reasons.get(key, 0) + 1
        results[name] = {
            'wrong_sequence': wrong,
            'launch_delay_s': summary(launch),
            'descent_delay_after_ejection_s': summary(descent),
            'landed_delay_s': summary(landed),
            'apogee_error_m': summary(apogee) if apogee else None,
            'transition_causes': reasons,
        }
    out = HERE/'results'
    out.mkdir(exist_ok=True)
    (out/'flight_state_mc.json').write_text(json.dumps(results, indent=1))
    for name in CASES:
        r = results[name]
        print('%-24s wrong %d  launch %.2f/%.2f  descent %.2f/%.2f  landed %.2f/%.2f (median/max s)' % (
            name, r['wrong_sequence'], r['launch_delay_s']['median'], r['launch_delay_s']['max'],
            r['descent_delay_after_ejection_s']['median'], r['descent_delay_after_ejection_s']['max'],
            r['landed_delay_s']['median'], r['landed_delay_s']['max']))


if __name__ == '__main__':
    main()
