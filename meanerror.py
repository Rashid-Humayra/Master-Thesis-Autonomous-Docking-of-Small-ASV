############################################

import pandas as pd
import numpy as np


est = pd.read_csv("/home/humiii/venvs/estimation_check_plots/45degleft/filtered.csv")
gt  = pd.read_csv("/home/humiii/venvs/estimation_check_plots/45degleft/gt_pose.csv")


est = est.sort_values("time_s")
gt  = gt.sort_values("time_s")


merged = pd.merge_asof(
    est,
    gt,
    on="time_s",
    direction="nearest",
    tolerance=0.05,
    suffixes=("_est", "_gt")
)

merged = merged.dropna()


merged["x_error"] = np.abs(merged["x_est"] - merged["y"])
merged["z_error"] = np.abs(merged["z_est"] - merged["x_gt"])

yaw_err = merged["yaw_est"] - merged["yaw_gt"]
yaw_err = np.arctan2(np.sin(yaw_err), np.cos(yaw_err))
merged["yaw_error_deg"] = np.abs(np.rad2deg(yaw_err))

print("Mean X Error:", merged["x_error"].mean())
print("Mean Z Error:", merged["z_error"].mean())
print("Mean Yaw Error:", merged["yaw_error_deg"].mean())