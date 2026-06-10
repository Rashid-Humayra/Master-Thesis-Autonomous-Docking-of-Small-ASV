#####################################################


import pandas as pd
import matplotlib.pyplot as plt

# the file record of the errors
df = pd.read_csv("/home/humiii/venvs/ErrorVsViewAngle.csv")
fig, ax1 = plt.subplots(figsize=(8,5))


ax1.plot(df["Angle(degree)"], df["Mean x error"],
         marker='o', linewidth=1,
         label='Lateral Error')

ax1.plot(df["Angle(degree)"], df["Mean z error"],
         marker='s', linewidth=1,
         label='Distance Error')

ax1.set_xlabel("Viewing Angle (deg)")
ax1.set_ylabel("Position Error (m)")
ax1.set_ylim(-0.5, 1)
ax1.grid(True)


ax2 = ax1.twinx()

ax2.plot(df["Angle(degree)"], df["Mean yaw error(degree)"],
         marker='^',
         linestyle='--',
         linewidth=1,
         label='Yaw Error')
ax2.scatter(df["Angle(degree)"], df["Visible Tag"],
         marker='x',
        #  linestyle='--',
          linewidth=2,
         label='Visible tags')

for xi, yi in zip(df["Angle(degree)"], df["Visible Tag"]):
    plt.annotate(f'({xi}\N{DEGREE SIGN}, {yi})', 
                 xy=(xi, yi), 
                 xytext=(5, 5), 
                 textcoords='offset points',
                 fontsize=9)
ax2.set_ylabel("Yaw Error (deg) & Visible Tags")
ax2.set_ylim(-5,15)


lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()

ax1.legend(lines1 + lines2,
           labels1 + labels2,
           loc='upper center')

plt.title("Estimation Error vs Viewing Angle")

plt.tight_layout()
plt.savefig("_error_vs_viewing_angle.png", dpi=300)
plt.show()
