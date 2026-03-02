#!/usr/bin/env python3
import argparse
import time
import threading

import rospy
from std_msgs.msg import Bool, String


def parse_args():
    p = argparse.ArgumentParser(description="Dialogue flow monitor")
    p.add_argument("--interval", type=float, default=1.0)
    p.add_argument("--once", action="store_true")
    p.add_argument("--runner", default="unknown")
    return p.parse_args()


def main():
    args = parse_args()
    state = {
        "status": None,
        "status_t": 0.0,
        "result": None,
        "result_t": 0.0,
        "action": None,
        "action_t": 0.0,
        "trigger_count": 0,
        "action_count": 0,
    }
    lock = threading.Lock()

    def now():
        return time.time()

    def cb_status(msg):
        with lock:
            state["status"] = msg.data.strip()
            state["status_t"] = now()

    def cb_result(msg):
        with lock:
            val = bool(msg.data)
            state["result"] = val
            state["result_t"] = now()
            if val:
                state["trigger_count"] += 1

    def cb_action(msg):
        with lock:
            state["action"] = bool(msg.data)
            state["action_t"] = now()
            state["action_count"] += 1

    def age_str(ts):
        if ts <= 0:
            return "n/a"
        return f"{now() - ts:.1f}s"

    def node_up(name):
        try:
            rospy.get_master().lookupNode(name)
            return "up"
        except Exception:
            return "down"

    rospy.init_node("dialogue_flow_watchdog", anonymous=True)
    rospy.Subscriber("/target_follower/status", String, cb_status, queue_size=50)
    rospy.Subscriber("/target_follower/result", Bool, cb_result, queue_size=50)
    rospy.Subscriber("/trash_action", Bool, cb_action, queue_size=50)

    print("[dialogue-monitor] watching /target_follower/status /target_follower/result /trash_action")
    print(f"[dialogue-monitor] interval={args.interval}s runner={args.runner}")

    hz = max(0.2, 1.0 / max(args.interval, 0.05))
    rate = rospy.Rate(hz)

    while not rospy.is_shutdown():
        with lock:
            st = state["status"]
            rs = state["result"]
            ac = state["action"]
            st_age = age_str(state["status_t"])
            rs_age = age_str(state["result_t"])
            ac_age = age_str(state["action_t"])
            trig_cnt = state["trigger_count"]
            act_cnt = state["action_count"]

        triggered = "yes" if (rs is True or st == "WAITING_ACTION") else "no"
        waiting = "yes" if st == "WAITING_ACTION" else "no"

        print(
            f"[{time.strftime('%H:%M:%S')}] "
            f"triggered={triggered} waiting={waiting} "
            f"status={st}({st_age}) result={rs}({rs_age}) "
            f"trash_action={ac}({ac_age}) "
            f"count(trigger/action)={trig_cnt}/{act_cnt} "
            f"nodes(tf/nav2udp/udp2ros)="
            f"{node_up('/target_follower')}/{node_up('/navigation_success_udp_bridge')}/{node_up('/udp_trash_action_bridge')} "
            f"runner={args.runner}"
        )

        if args.once:
            break
        rate.sleep()


if __name__ == "__main__":
    main()
