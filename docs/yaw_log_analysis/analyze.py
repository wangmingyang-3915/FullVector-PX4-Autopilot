"""Reproduce yaw analysis: python3 docs/yaw_log_analysis/analyze.py."""
from pathlib import Path
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

OUT = Path(__file__).resolve().parent
ROOT = OUT.parent.parent


def analyze(number):
    u = ULog(str(ROOT / f'docs/log_{number}_UnknownDate.ulg'))
    d = u.get_dataset('fullvector_control_diagnostics').data
    ts = d['timestamp']
    t = (ts.astype(float) - u.start_timestamp) / 1e6

    def hold(topic, key):
        q = u.get_dataset(topic).data
        ix = np.searchsorted(q['timestamp'], ts, side='right') - 1
        return q[key][np.clip(ix, 0, len(q['timestamp']) - 1)]

    armed = hold('vehicle_status', 'arming_state') == 2
    airborne = ~hold('vehicle_land_detected', 'landed').astype(bool)
    # ULog bool fields are int8: cast explicitly before Boolean indexing.
    valid = d['control_setpoint_valid'].astype(bool)
    active = hold('fullvector_control_status', 'fullvector_active').astype(bool)
    flight = armed & airborne & valid & active
    sp = d['angular_velocity_sp[2]']
    err = d['angular_velocity_error[2]']
    rate = sp - err  # Controller's exact rate measurement, same cycle as sp.
    gyro = hold('vehicle_angular_velocity', 'xyz[2]')
    stick = hold('manual_control_setpoint', 'yaw')
    acc = hold('vehicle_angular_acceleration_setpoint', 'xyz[2]')
    nav = hold('vehicle_status', 'nav_state')
    relative = d['attitude_error_source'] != 0
    motors = np.array([hold('actuator_motors', f'control[{i}]') for i in range(4)])
    servos = np.array([hold('actuator_servos', f'control[{i}]') for i in range(4)])

    def metrics(mask):
        if not mask.any():
            return None
        return {
            'samples': int(mask.sum()),
            'duration_approx_s': float(mask.sum() * np.median(np.diff(t))),
            'rate_sp_mean_rad_s': float(np.mean(sp[mask])),
            'rate_mean_rad_s': float(np.mean(rate[mask])),
            'rate_error_rms_rad_s': float(np.sqrt(np.mean(err[mask]**2))),
            'heading_error_rms_deg': float(np.rad2deg(np.sqrt(np.mean(d['attitude_error[2]'][mask]**2)))),
            'heading_error_abs_p95_deg': float(np.rad2deg(np.percentile(abs(d['attitude_error[2]'][mask]), 95))),
            'rate_sp_abs_p95_rad_s': float(np.percentile(abs(sp[mask]), 95)),
            'rate_abs_p95_rad_s': float(np.percentile(abs(rate[mask]), 95)),
            'rate_sp_at_2rad_s_pct': float(100*np.mean(abs(sp[mask]) >= 1.99)),
            'acc_cmd_abs_p95_rad_s2': float(np.percentile(abs(acc[mask]), 95)),
            'acc_cmd_abs_max_rad_s2': float(np.max(abs(acc[mask]))),
            'acc_cmd_mean_rad_s2': float(np.mean(acc[mask])),
            'actuator_saturated_pct': float(100*np.mean(d['actuator_saturated'][mask])),
            'motor_upper_saturated_pct': float(100*np.mean(d['motor_upper_saturation_mask'][mask] != 0)),
            'motor_lower_saturated_pct': float(100*np.mean(d['motor_lower_saturation_mask'][mask] != 0)),
            'servo_saturated_pct': float(100*np.mean(d['servo_saturation_mask'][mask] != 0)),
            'motor_control_min_max': [float(np.min(motors[:, mask])), float(np.max(motors[:, mask]))],
            'servo_control_abs_max': float(np.max(abs(servos[:, mask]))),
            'motor_pair_difference_mean': float(np.mean((motors[0]+motors[1]-motors[2]-motors[3])[mask])),
            'servo_common_mean_deg': float(np.rad2deg(np.mean(servos[:, mask])*u.initial_parameters['FV_TILT_MAX'])),
            'modes': sorted(map(int, np.unique(nav[mask]))),
            'attitude_error_sources': sorted(map(int, np.unique(d['attitude_error_source'][mask]))),
        }

    windows = [(6.5, 8.5), (32, 44), (47, 53), (61, 65), (92, 100)] if number == 170 else [(13.5, 17.8), (18.5, 20.4), (23, 25), (29, 32)]
    result = {
        'log': number,
        'time_origin': 'seconds since ULog start_timestamp',
        'firmware': u.msg_info_dict['ver_sw'],
        'dropouts': len(u.dropouts),
        'parameters': {k: float(v) for k, v in u.initial_parameters.items() if k.startswith('FV_')},
        'changed_parameters': u.changed_parameters,
        'flight': metrics(flight),
        'absolute_stabilized': metrics(flight & (nav == 15) & ~relative),
        'absolute_posctl': metrics(flight & (nav == 2) & ~relative),
        'relative': metrics(flight & relative),
        'windows': {f'{a}-{b}': metrics(flight & (t >= a) & (t < b)) for a, b in windows},
        'diag_rate_vs_held_gyro_rms_rad_s': float(np.sqrt(np.mean((rate[flight]-gyro[flight])**2))),
    }

    def plot(bounds=None):
        fig, ax = plt.subplots(6, 1, figsize=(14, 15), sharex=True)
        ax[0].plot(t, np.rad2deg(sp), label='Controller rate target')
        ax[0].plot(t, np.rad2deg(rate), label='Controller measured body rate')
        ax[0].set_ylabel('Yaw rate (deg/s)')
        ax[1].plot(t, np.rad2deg(d['attitude_error[2]']), label='Heading error')
        ax[1].set_ylabel('Heading error (deg)')
        ax[2].plot(t, acc, label='Commanded angular acceleration')
        ax[2].set_ylabel('Yaw accel (rad/s2)')
        for i in range(4):
            ax[3].plot(t, motors[i], label=f'Motor {i+1}')
        ax[3].set_ylabel('Motor command')
        ax[4].plot(t, np.rad2deg(np.mean(servos, axis=0)*u.initial_parameters['FV_TILT_MAX']), label='Mean commanded tilt (not servo feedback)')
        ax[4].set_ylabel('Common tilt (deg)')
        ax[5].plot(t, stick, label='Yaw stick')
        ax[5].step(t, nav/15, label='Nav state / 15', alpha=.6)
        ax[5].step(t, relative, label='Relative attitude source', alpha=.6)
        ax[5].step(t, flight.astype(float), label='Analysis flight mask', alpha=.6)
        ax[5].set_ylabel('Stick / state')
        ax[5].set_xlabel('Seconds since ULog start')
        for a in ax:
            a.grid(alpha=.25)
            a.legend(loc='upper right', fontsize=8)
            if bounds:
                a.set_xlim(*bounds)
        fig.suptitle(f'Log {number}: yaw outer P={u.initial_parameters["FV_ATT_P_Z"]:.1f}, inner P/I/D=0.25/0.10/0\nNav: 2=Position, 14=Offboard, 15=Stabilized; curves include ground samples; statistics exclude them')
        fig.tight_layout()
        suffix = '_detail' if bounds else ''
        fig.savefig(OUT/f'log_{number}_yaw{suffix}.png', dpi=150)
        plt.close(fig)

    plot()
    plot((90, 104) if number == 170 else (10, 23))
    return result


if __name__ == '__main__':
    results = [analyze(n) for n in (170, 174)]
    (OUT/'metrics.json').write_text(json.dumps(results, indent=2) + '\n')
    fig, axes = plt.subplots(2, 1, figsize=(12, 6.5))
    for ax, number, bounds, shade in zip(axes, [174, 170], [(10, 23), (90, 101)], [(13.5, 17.8), (92, 100)]):
        u = ULog(str(ROOT / f'docs/log_{number}_UnknownDate.ulg'))
        d = u.get_dataset('fullvector_control_diagnostics').data
        t = (d['timestamp'].astype(float) - u.start_timestamp) / 1e6
        sp = d['angular_velocity_sp[2]']
        rate = sp - d['angular_velocity_error[2]']
        ax.plot(t, np.rad2deg(sp), label='Target yaw body rate', lw=1.8)
        ax.plot(t, np.rad2deg(rate), label='Measured yaw body rate', lw=1.8)
        ax.set_xlim(*bounds)
        ax.axvspan(*shade, color='gray', alpha=.15)
        ax.set_ylabel('Yaw rate (deg/s)')
        ax.set_xlabel('Seconds since ULog start')
        ax.set_title(f'Log {number}: outer P={u.initial_parameters["FV_ATT_P_Z"]:.1f}, inner P=0.25 / I=0.10 / D=0')
        ax.grid(alpha=.3)
        ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(OUT/'yaw_comparison.png', dpi=160)
    plt.close(fig)
    for r in results:
        print(r['log'], json.dumps({k: r[k] for k in ['flight', 'absolute_stabilized', 'absolute_posctl', 'relative', 'windows']}, indent=2))
