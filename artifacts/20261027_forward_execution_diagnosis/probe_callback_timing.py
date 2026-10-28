"""Exercise executor callbacks with simulated hardware and isolated ROS topics."""

import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / "callback_probe"
TOPICS = ["/motion/executor/request", "/motion/executor/cancel", "/motion/executor/status",
          "/motion/executor/heartbeat", "/vision/ball_info", "/vision/hurdle_info",
          "/navigation/motion_command"]


def isolated(topic):
    return "/step_timing_probe" + topic


def main():
    OUT.mkdir(exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID="219", ROS_LOCALHOST_ONLY="1", ROS_LOG_DIR=str(OUT / "ros_logs"))
    import rclpy
    from std_msgs.msg import String

    trace = OUT / "simulated.csv"
    callback_path = Path(str(trace) + ".callbacks.csv")
    callback_path.unlink(missing_ok=True)
    command = [str(ROOT / "build/irc_step_motion_executor/sdk_motion_executor"), "--ros-args",
               "-p", "backend_type:=simulated", "-p", "enable_robot_hardware:=false",
               "-p", "explicit_torque_approval:=false", "-p", "startup_pose_enabled:=false",
               "-p", "running_polls:=100000", "-p", "motion_trace_enabled:=true",
               "-p", f"motion_trace_path:={trace}",
               "-p", f"motion_aliases_file:={ROOT}/src/irc_step_motion_executor/config/motion_aliases.yaml"]
    for topic in TOPICS:
        command += ["-r", f"{topic}:={isolated(topic)}"]
    phases, statuses = [], []
    rclpy.init()
    node = rclpy.create_node("step_callback_probe")
    publishers = {topic: node.create_publisher(String, isolated(topic), 10) for topic in
                  [TOPICS[0], "/vision/ball_info", "/vision/hurdle_info", "/navigation/motion_command"]}
    subscription = node.create_subscription(
        String, isolated("/motion/executor/status"), lambda msg: statuses.append(json.loads(msg.data)), 10)

    def publish(topic, data):
        publishers[topic].publish(String(data=json.dumps(data)))

    with (OUT / "executor.log").open("w") as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while publishers[TOPICS[0]].get_subscription_count() == 0:
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("simulated executor did not become available; see executor.log")
                rclpy.spin_once(node, timeout_sec=0.05)
            publish(TOPICS[0], {"request_id": 9001, "command_id": 9001, "event_id": None,
                               "action": "MANUAL_TEST", "motion_id": "line_forward_6", "timeout_ms": 60000})
            deadline = time.monotonic() + 5
            while not any(s.get("status") == "RUNNING" for s in statuses):
                if time.monotonic() > deadline:
                    raise RuntimeError(f"simulated request not running: {statuses}")
                rclpy.spin_once(node, timeout_sec=0.01)
            for label, rate, head in [("idle_messages", 0, False),
                                      ("three_topics_30hz", 30, False),
                                      ("three_topics_30hz_head_active", 30, True)]:
                start = time.monotonic()
                next_message = start
                count = 0
                while time.monotonic() - start < 8:
                    now = time.monotonic()
                    if rate and now >= next_message:
                        publish("/navigation/motion_command", {
                            "valid": True, "source": "ball" if head else "line", "phase": "AUTO",
                            "action": "STRAIGHT", "command_id": 9001})
                        publish("/vision/ball_info", {"detected": head, "head_down_requested": head})
                        publish("/vision/hurdle_info", {"detected": False, "confirmation_confirmed": False,
                                                       "head_down_requested": False})
                        count += 3
                        next_message = now + 1 / rate
                    rclpy.spin_once(node, timeout_sec=0.002)
                phases.append({"name": label, "start_ms": start * 1000,
                               "end_ms": time.monotonic() * 1000, "published_messages": count})
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            process.wait(timeout=15)
            node.destroy_node()
            rclpy.shutdown()
            (OUT / "phases.json").write_text(json.dumps({
                "scope": "Simulated backend only; no vision inference, serial I/O or physical movement.",
                "phases": phases, "statuses": statuses, "exit_code": process.returncode,
            }, indent=2) + "\n")
    if process.returncode != 0:
        raise RuntimeError(f"executor failed: {process.returncode}")
    print(callback_path)


if __name__ == "__main__":
    main()
