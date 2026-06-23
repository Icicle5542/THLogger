import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

LOG_FILE = Path(__file__).parent / "thlog.txt"

df = pd.read_csv(
    LOG_FILE,
    sep=r"\s{2,}",
    engine="python",
    skiprows=2,
    header=None,
    names=["timestamp", "temp_c", "humidity_pct"],
)

fig, ax1 = plt.subplots(figsize=(12, 5))

color_temp = "tab:red"
ax1.set_xlabel("Sample Index")
ax1.set_ylabel("Temperature (°C)", color=color_temp)
ax1.plot(df["temp_c"].values, color=color_temp, linewidth=1.2, label="Temp (°C)")
ax1.tick_params(axis="y", labelcolor=color_temp)

ax2 = ax1.twinx()
color_hum = "tab:blue"
ax2.set_ylabel("Humidity (%)", color=color_hum)
ax2.plot(df["humidity_pct"].values, color=color_hum, linewidth=1.2, label="Humidity (%)")
ax2.tick_params(axis="y", labelcolor=color_hum)

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")

plt.title("Temperature & Humidity Log")
plt.tight_layout()
plt.show()
