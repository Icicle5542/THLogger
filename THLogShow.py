import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from pathlib import Path

LOG_FILE = Path(__file__).parent / "thlog.txt"

df = pd.read_csv(
    LOG_FILE,
    sep=r"\s{2,}",
    engine="python",
    skiprows=1,
    header=None,
    names=["timestamp", "temp_c", "humidity_pct", "vib_rms_mg", "vib_peak_mg"],
)

df["datetime"] = pd.to_datetime(df["timestamp"], format="%Y-%m-%d %H:%M:%S", errors="coerce")
df = df.dropna(subset=["datetime"])

fig, ax1 = plt.subplots(figsize=(12, 5))

color_temp = "tab:red"
ax1.set_xlabel("Time (UTC)")
ax1.set_ylabel("Temperature (°C)", color=color_temp)
ax1.plot(df["datetime"], df["temp_c"], color=color_temp, linewidth=1.2, label="Temp (°C)")
ax1.tick_params(axis="y", labelcolor=color_temp)

ax2 = ax1.twinx()
color_vib = "tab:blue"
ax2.set_ylabel("VibPeak (mg)", color=color_vib)
ax2.plot(df["datetime"], df["vib_peak_mg"], color=color_vib, linewidth=1.2, label="VibPeak (mg)")
ax2.tick_params(axis="y", labelcolor=color_vib)

ax1.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H:%M"))
fig.autofmt_xdate(rotation=30, ha="right")

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")

plt.title("Temperature & Vibration Peak Log")
plt.tight_layout()
plt.show()
