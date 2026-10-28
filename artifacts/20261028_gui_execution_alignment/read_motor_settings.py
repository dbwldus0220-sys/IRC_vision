#!/usr/bin/env python3
"""Read-only bus audit: no torque, goal, profile, EEPROM, or reset writes."""
import argparse
import json
from pathlib import Path
import subprocess
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

REGISTERS = [
    ('model', 0, 2), ('firmware', 6, 1), ('baud_register', 8, 1),
    ('return_delay', 9, 1), ('drive_mode', 10, 1), ('operating_mode', 11, 1),
    ('protocol', 13, 1), ('homing_offset', 20, 4), ('pwm_limit', 36, 2),
    ('current_limit', 38, 2), ('shutdown', 63, 1), ('torque_enable', 64, 1),
    ('hardware_error_status', 70, 1), ('position_d_gain', 80, 2),
    ('position_i_gain', 82, 2), ('position_p_gain', 84, 2),
    ('profile_acceleration', 108, 4), ('profile_velocity', 112, 4),
    ('goal_position', 116, 4), ('present_position', 132, 4),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    path = str(Path(args.port).resolve(strict=True))
    users = subprocess.run(['fuser', path], capture_output=True, text=True)
    if users.returncode != 1:
        raise RuntimeError(f'Port occupied or ownership check failed: {users.stdout} {users.stderr}')
    port = PortHandler(path)
    packet = PacketHandler(2.0)
    report = dict(port=path, baud=4000000, protocol=2.0, motor_writes=0,
                  begin_ns=time.monotonic_ns(), motors={})
    try:
        if not port.openPort() or not port.setBaudRate(4000000):
            raise RuntimeError('cannot open/configure serial port')
        for motor_id in range(23):
            fields = report['motors'][motor_id] = {}
            for name, address, size in REGISTERS:
                begin = time.monotonic_ns()
                value, result, error = getattr(packet, f'read{size}ByteTxRx')(port, motor_id, address)
                end = time.monotonic_ns()
                fields[name] = dict(address=address, bytes=size, raw=value,
                                    begin_ns=begin, end_ns=end, sdk_result=result, device_error=error)
                if name == 'model' and result != COMM_SUCCESS:
                    break
            print(motor_id, {name: value['raw'] for name, value in fields.items()
                             if name in ('model', 'firmware', 'drive_mode', 'torque_enable', 'hardware_error_status')},
                  'model_result', fields['model']['sdk_result'], flush=True)
    finally:
        port.closePort()
        report['end_ns'] = time.monotonic_ns()
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
