#!/usr/bin/env python3
"""Read-only X-series control-table capture. Does not instantiate the robot backend."""
import argparse
import json
from pathlib import Path
import time

# Addresses match the GUI/SDK X-series control table. Keep model_number in every record.
REGISTERS = {
    'model_number': (0,2), 'firmware': (6,1), 'id': (7,1), 'baud_code': (8,1),
    'return_delay': (9,1), 'drive_mode': (10,1), 'operating_mode': (11,1),
    'homing_offset': (20,4), 'pwm_limit': (36,2), 'current_limit': (38,2),
    'velocity_limit': (44,4), 'max_position_limit': (48,4), 'min_position_limit': (52,4),
    'shutdown': (63,1), 'torque_enable': (64,1), 'hardware_error': (70,1),
    'position_d': (80,2), 'position_i': (82,2), 'position_p': (84,2),
    'profile_acceleration': (108,4), 'profile_velocity': (112,4),
    'goal_position': (116,4), 'present_position': (132,4),
}
def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',required=True)
    ap.add_argument('--baud',type=int,default=4000000)
    ap.add_argument('--label',required=True,help='e.g. GUI-working-before / algorithm-after-init')
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
    port=PortHandler(args.port); packet=PacketHandler(2.0)
    latency=Path('/sys/bus/usb-serial/devices')/Path(args.port).resolve().name/'latency_timer'
    result=dict(label=args.label,port=args.port,baud=args.baud,protocol=2.0,
                latency_timer=latency.read_text().strip() if latency.exists() else None,
                motors={},errors=[])
    try:
        if not port.openPort() or not port.setBaudRate(args.baud):
            raise RuntimeError('cannot open requested serial port/baud')
        for motor in range(23):
            values={}
            for name,(address,width) in REGISTERS.items():
                begin=time.perf_counter()
                value,comm,error=getattr(packet,f'read{width}ByteTxRx')(port,motor,address)
                values[name]=dict(raw=value if comm==COMM_SUCCESS and not error else None,
                                  begin_sec=begin,duration_ms=(time.perf_counter()-begin)*1000,
                                  communication_result=comm,device_error=error)
                if comm!=COMM_SUCCESS or error: result['errors'].append([motor,name,comm,error])
            result['motors'][motor]=values
    finally:
        port.closePort()
        args.output.write_text(json.dumps(result,indent=2)+'\n')
    return int(bool(result['errors']))
if __name__=='__main__':raise SystemExit(main())
