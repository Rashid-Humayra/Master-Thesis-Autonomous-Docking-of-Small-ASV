#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


def quat_to_yaw(q):
    """Quaternion to yaw in rad."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def read_bag(bag_path):
    bag_path = Path(bag_path)
    typestore = get_typestore(Stores.ROS2_JAZZY)

    data = {
        "fused": [],
        "filtered": [],
        "reference": [],
        "cmd": [],
        "valid": [],
        "state": [],
        "gt_pose": [],
        "dock0pose": [],
        "dock1pose": [],
        "dock3pose": [],
        
    }

    topics = {
        "/dock/fused_pose": "fused",
        "/dock/filtered_pose": "filtered",
        "/dock/los_reference": "reference",
        "/cmd_vel": "cmd",
        "/dock/fused_pose_valid": "valid",
        "/dock/filtered_pose_valid": "valid",
        "/dock/los_state": "state",
        "/model/blueboat/pose": "gt_pose",
        "/dock/tag_0_pose": "dock0pose",
        "/dock/tag_1_pose": "dock1pose",
        "/dock/tag_3_pose": "dock3pose",
    }

    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [
            c for c in reader.connections if c.topic in topics
        ]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            t = timestamp * 1e-9
            key = topics[connection.topic]

            if connection.topic in ["/dock/fused_pose", "/dock/filtered_pose", "/dock/los_reference", "/dock/tag_0_pose", "/dock/tag_1_pose", "/dock/tag_3_pose"]:
                yaw = quat_to_yaw(msg.pose.orientation)
                data[key].append({
                    "t": t,
                    "x": msg.pose.position.x,
                    "z": msg.pose.position.z,
                    "yaw": yaw,
                })
                
            elif connection.topic == "/model/blueboat/pose":
                if len(msg.poses) == 0:
                    continue

                pose = msg.poses[0]

                yaw = quat_to_yaw(pose.orientation)
                yaw = wrap_angle(yaw-math.pi)

                data["gt_pose"].append({
                    "t": t,
                    "x": -pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "yaw": yaw,
                })

            elif connection.topic == "/cmd_vel":
                data["cmd"].append({
                    "t": t,
                    "u": msg.linear.x,
                    "r": msg.angular.z,
                })

            elif connection.topic in ["/dock/fused_pose_valid", "/dock/filtered_pose_valid"]:
                data["valid"].append({
                    "t": t,
                    "topic": connection.topic,
                    "valid": int(msg.data),
                })

            elif connection.topic == "/dock/los_state":
                data["state"].append({
                    "t": t,
                    "state": msg.data,
                })

    return {k: pd.DataFrame(v) for k, v in data.items()}


def normalize_time(dfs):
    t0_candidates = []
    for df in dfs.values():
        if not df.empty and "t" in df.columns:
            t0_candidates.append(df["t"].min())

    if not t0_candidates:
        return dfs

    t0 = min(t0_candidates)

    for df in dfs.values():
        if not df.empty and "t" in df.columns:
            df["time_s"] = df["t"] - t0

    return dfs


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def save_plot(fig, outdir, name):
    outdir.mkdir(parents=True, exist_ok=True)
    path = outdir / name
    fig.savefig(path, dpi=300, bbox_inches="tight")
    print(f"Saved: {path}")





def plot_results(dfs, outdir):
    fused = dfs["fused"]
    filtered = dfs["filtered"]
    ref = dfs["reference"]
    gt_pose = dfs["gt_pose"]
    cmd = dfs["cmd"]
    dock0pose = dfs["dock0pose"]
    dock1pose = dfs["dock1pose"]
    dock3pose = dfs["dock3pose"]

    # Plots of the lateral offsets
    fig = plt.figure()
    if not fused.empty:
        plt.plot(fused["time_s"], fused["x"], linewidth=3, label="Fused x")
    if not filtered.empty:
        plt.plot(filtered["time_s"], filtered["x"], label="Filtered x")
    if not dock0pose.empty:
        plt.plot(dock0pose["time_s"], dock0pose["x"], label="Tag 0 pose x")
    if not dock1pose.empty:
        plt.plot(dock1pose["time_s"], dock1pose["x"], label="Tag 1 pose x")
    if not dock3pose.empty:
        plt.plot(dock3pose["time_s"], dock3pose["x"], label="Tag 3 pose x")
    if not ref.empty:
        plt.plot(ref["time_s"], ref["x"], "--", label="Reference x")
    if not gt_pose.empty:
        plt.plot(gt_pose["time_s"], gt_pose["y"], label="BlueBoat world y")
        
    plt.xlabel("Time [s]")
    plt.ylabel("Lateral offset x [m]")
    plt.ylim(-5, 5)
    plt.title("Lateral Offset During Docking")
    plt.grid(True)
    plt.legend()
    save_plot(fig, outdir, "01_lateral_offset_x.png")

    # Plots for forward distances
    fig = plt.figure()
    if not fused.empty:
        plt.plot(fused["time_s"], fused["z"], label="Fused z")
    if not filtered.empty:
        plt.plot(filtered["time_s"], filtered["z"], label="Filtered z")
    if not dock0pose.empty:
        plt.plot(dock0pose["time_s"], dock0pose["z"], label="Tag 0 pose z")
    if not dock1pose.empty:
        plt.plot(dock1pose["time_s"], dock1pose["z"], label="Tag 1 pose z")
    if not dock3pose.empty:
        plt.plot(dock3pose["time_s"], dock3pose["z"], label="Tag 3 pose z")
    if not ref.empty:
        plt.plot(ref["time_s"], ref["z"], "--", label="Reference stop z")
    if not gt_pose.empty:
        plt.plot(gt_pose["time_s"], gt_pose["x"], label="BlueBoat world x")
    plt.xlabel("Time [s]")
    plt.ylabel("Forward distance z [m]")
    plt.ylim(-5, 10)
    plt.title("Forward Distance to Dock")
    plt.grid(True)
    plt.legend()
    save_plot(fig, outdir, "02_forward_distance_z.png")

    # Plots of yaw and desired yaw
    fig = plt.figure()
    if not fused.empty:
        plt.plot(fused["time_s"], fused["yaw"], linewidth=3, label="Fused yaw")
    if not filtered.empty:
        plt.plot(filtered["time_s"], filtered["yaw"], label="Filtered yaw")
    if not dock0pose.empty:
        plt.plot(dock0pose["time_s"], dock0pose["yaw"], label="Tag 0 pose yaw")
    if not dock1pose.empty:
        plt.plot(dock1pose["time_s"], dock1pose["yaw"], label="Tag 1 pose yaw")
    if not dock3pose.empty:
        plt.plot(dock3pose["time_s"], dock3pose["yaw"], label="Tag 3 pose yaw")
    if not ref.empty:
        plt.plot(ref["time_s"], ref["yaw"], "--", label="Desired yaw")
    if not gt_pose.empty:
        plt.plot(gt_pose["time_s"], gt_pose["yaw"], label="BlueBoat yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw [rad]")
    plt.ylim(-2.5, 1.5)
    plt.title("Yaw Tracking")
    plt.grid(True)
    plt.legend()
    save_plot(fig, outdir, "03_yaw_tracking.png")

    # Plots of yaw error
    if not filtered.empty and not ref.empty:
        merged = pd.merge_asof(
            filtered.sort_values("time_s"),
            ref.sort_values("time_s"),
            on="time_s",
            suffixes=("_filtered", "_ref"),
            direction="nearest",
            tolerance=0.2,
        ).dropna()

        if not merged.empty:
            yaw_error = [
                math.atan2(
                    math.sin(yr - yf),
                    math.cos(yr - yf)
                )
                for yf, yr in zip(merged["yaw_filtered"], merged["yaw_ref"])
            ]

            fig = plt.figure()
            plt.plot(merged["time_s"], yaw_error)
            plt.xlabel("Time [s]")
            plt.ylabel("Yaw error [rad]")
            plt.ylim(-0.5, 0.5)
            plt.title("Yaw Error")
            plt.grid(True)
            save_plot(fig, outdir, "04_yaw_error.png")

    # Plots for Command velocities
    fig = plt.figure()
    if not cmd.empty:
        plt.plot(cmd["time_s"], cmd["u"], label="cmd.linear.x")
        plt.plot(cmd["time_s"], cmd["r"], label="cmd.angular.z")
    plt.xlabel("Time [s]")
    plt.ylabel("Command")
    plt.title("Controller Commands")
    plt.grid(True)
    plt.legend()
    save_plot(fig, outdir, "05_cmd_vel.png")

    
    
    fig = plt.figure()
    # if not fused.empty:
    #     plt.plot(fused["z"], fused["x"], label="Fused trajectory")
    if not filtered.empty:
        plt.plot(filtered["x"], filtered["z"], label="Filtered trajectory")
        # plt.plot(los_point_x, los_point_z, '--', label= 'LOS Target Path')
        # plt.plot(curve_x, curve_z, '--', label= 'LOS Direction curve')
        plt.scatter(filtered["x"].iloc[0],
                    filtered["z"].iloc[0],
                    marker="o",
                    s=100,
                    label="start")
        plt.scatter(filtered["x"].iloc[-1],
                    filtered["z"].iloc[-1],
                    marker="x",
                    s=100,
                    label="stop")


    if not gt_pose.empty:
        plt.plot(gt_pose["y"], gt_pose["x"], label="Actual trajectory")
        
        
    
    
    ###########################
    traj = pd.merge_asof(
        filtered.sort_values('time_s'),
        ref[['time_s', 'yaw']].sort_values('time_s'),
        on = 'time_s',
        direction= 'nearest',
        tolerance= 0.2,
        suffixes = ('_filtered', '_des')
    ).dropna()
    
  
    step = 10
    for i in range(0, len(traj), step):
        x = traj["x"].iloc[i]
        z = traj["z"].iloc[i]
        yaw_des = traj['yaw_des'].iloc[i]
        
        
        plt.arrow(x,z,
                  0.3*np.sin(yaw_des),
                  -0.3*np.cos(yaw_des),
                  head_width = 0.05,
                  length_includes_head= True)
    #########################################
    
    plt.ylabel("Forward distance z [m]")
    plt.xlabel("Lateral offset x [m]")
    plt.title("Dock-Relative Approach Trajectory")
    plt.grid(True)
    plt.legend()
    #plt.gca().invert_xaxis()
    save_plot(fig, outdir, "06_xz_trajectory.png")

    plt.show()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Path to rosbag folder containing .mcap")
    parser.add_argument("--outdir", default="docking_plots", help="Output plot folder")
    args = parser.parse_args()

    dfs = read_bag(args.bag)
    dfs = normalize_time(dfs)

    outdir = Path(args.outdir)
    plot_results(dfs, outdir)

    # saving the CSVs to help plot something else
    for name, df in dfs.items():
        if not df.empty:
            df.to_csv(outdir / f"{name}.csv", index=False)


if __name__ == "__main__":
    main()