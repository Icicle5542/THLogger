#!/usr/bin/env python3
"""
THLogBLE.py — Download THLogger data over BLE and plot it.

Connects to an nRF54L15 running THLogger firmware via the Nordic UART
Service (NUS), sends the "show" command, receives a binary log dump,
decodes it, saves it to a timestamped text file, and displays a
dual-axis plot of temperature and vibration.

Wire protocol (binary "show" response):
  Offset  Size  Content
       0     4  SOF  0xAA 0x55 0xDD 0x22
       4     4  entry count  (uint32, little-endian)
       8  N×32  th_log_entry structs (little-endian):
                  int64  timestamp_s
                  int32  temp_val1, temp_val2  (sensor_value; val2 in µ-units)
                  int32  hum_val1,  hum_val2
                  int32  vibration_rms_mg
                  int32  vibration_peak_mg
   8+N×32   4  EOF  0x22 0xDD 0x55 0xAA

Requirements:
    pip install bleak matplotlib

Usage:
    python THLogBLE.py                          # scan for "IcicleTHLogger"
    python THLogBLE.py --name MyDevice          # scan for a specific name
    python THLogBLE.py --timeout 30             # longer scan timeout
"""

import argparse
import asyncio
import struct
import sys
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from bleak import BleakScanner, BleakClient

# ── NUS characteristic UUIDs ───────────────────────────────────────────────
NUS_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC → device
NUS_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # device → PC

# ── Binary show-protocol framing ──────────────────────────────────────────
SOF_BYTES  = bytes([0xAA, 0x55, 0xDD, 0x22])
EOF_BYTES  = bytes([0x22, 0xDD, 0x55, 0xAA])
ENTRY_FMT  = "<qiiiiii"                      # int64 + 6×int32 (little-endian)
ENTRY_SIZE = struct.calcsize(ENTRY_FMT)       # 32 bytes


def _parse_binary(data: bytes) -> list:
    """Parse the binary payload into a list of entry dicts."""
    sof_idx = data.find(SOF_BYTES)
    if sof_idx < 0:
        print("ERROR: SOF marker not found in received data.")
        return []

    pos = sof_idx + 4
    if len(data) < pos + 4:
        print("ERROR: data truncated — missing entry count.")
        return []

    (entry_count,) = struct.unpack_from("<I", data, pos)
    pos += 4

    entries = []
    for i in range(entry_count):
        if len(data) < pos + ENTRY_SIZE:
            print(f"WARNING: data truncated at entry {i}/{entry_count}.")
            break
        ts, tv1, tv2, hv1, hv2, vrms, vpeak = struct.unpack_from(ENTRY_FMT, data, pos)
        entries.append(
            {
                "timestamp_s": ts,
                "temp_c":      tv1 + tv2 / 1_000_000.0,
                "humidity":    hv1 + hv2 / 1_000_000.0,
                "vib_rms_mg":  vrms,
                "vib_peak_mg": vpeak,
            }
        )
        pos += ENTRY_SIZE

    return entries


async def download_log(device_name: str, scan_timeout: float) -> list:
    """Connect to the THLogger device, download and decode all log entries."""

    print(f'Scanning for "{device_name}" …')
    device = await BleakScanner.find_device_by_name(
        device_name, timeout=scan_timeout
    )
    if device is None:
        print(
            f'ERROR: "{device_name}" not found.\n'
            "Make sure the device is powered on and advertising."
        )
        sys.exit(1)

    print(f"Found {device.name} ({device.address}).  Connecting …")

    received_data  = bytearray()
    expected_total = None   # set once we parse SOF + count from the header

    def on_notification(_sender, data: bytearray) -> None:
        nonlocal received_data, expected_total
        received_data.extend(data)

        # Once we have SOF + count we know the exact expected frame size
        if expected_total is None and len(received_data) >= 8:
            sof_idx = bytes(received_data).find(SOF_BYTES)
            if sof_idx >= 0 and len(received_data) >= sof_idx + 8:
                (n,) = struct.unpack_from("<I", received_data, sof_idx + 4)
                expected_total = 4 + 4 + n * ENTRY_SIZE + 4  # SOF+count+data+EOF

        cur = len(received_data)
        if expected_total:
            pct = min(100, cur * 100 // expected_total)
            print(
                f"\r  {cur:,} / {expected_total:,} bytes  ({pct}%) …",
                end="", flush=True,
            )
        else:
            print(f"\r  {cur:,} bytes received …", end="", flush=True)

    async with BleakClient(device) as client:
        print("Connected.  Subscribing to notifications …")
        await client.start_notify(NUS_TX_CHAR_UUID, on_notification)

        print('Sending "show" command …')
        await client.write_gatt_char(NUS_RX_CHAR_UUID, b"show", response=False)
        print("Downloading log data …")

        # Poll for the binary EOF marker.  Polling avoids threading issues
        # with asyncio.Event.set() being called from the WinRT callback thread.
        loop     = asyncio.get_running_loop()
        deadline = loop.time() + 300.0
        stall_at = loop.time() + 15.0
        prev_len = 0

        while True:
            await asyncio.sleep(0.25)
            now     = loop.time()
            cur_len = len(received_data)

            if cur_len != prev_len:          # new bytes arrived — reset stall timer
                stall_at = now + 15.0
                prev_len = cur_len

            if len(received_data) >= 12 and received_data[-4:] == bytearray(EOF_BYTES):
                break
            if now >= stall_at:
                print("\nWARNING: No new data for 15 s — assuming transfer complete.")
                break
            if now >= deadline:
                print("\nWARNING: Timed out after 300 s.")
                break

        await client.stop_notify(NUS_TX_CHAR_UUID)

    print(f"\nDone — {len(received_data):,} bytes received.")
    entries = _parse_binary(bytes(received_data))
    print(f"Decoded {len(entries)} log entries.")
    return entries


def save_log(entries: list) -> Path:
    """Save decoded entries as a human-readable text file."""
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = Path(__file__).parent / f"thlog_ble_{ts}.txt"

    with filename.open("w", encoding="utf-8") as f:
        f.write(
            f"{'Timestamp (UTC)':<21}  {'Temp (C)':>8}   "
            f"{'Humidity (%)':>12}   {'VibRMS (mg)':>11}  {'VibPeak (mg)':>12}\n"
        )
        for e in entries:
            dt_str = datetime.fromtimestamp(
                e["timestamp_s"], tz=timezone.utc
            ).strftime("%Y-%m-%d %H:%M:%S")
            f.write(
                f"{dt_str:<21}  {e['temp_c']:>8.2f}   "
                f"{e['humidity']:>12.2f}   {e['vib_rms_mg']:>11}  {e['vib_peak_mg']:>12}\n"
            )

    print(f"Log saved to {filename}")
    return filename


def plot_entries(entries: list) -> None:
    """Display a dual-axis temperature / vibration plot."""
    if not entries:
        print("No data entries — nothing to plot.")
        return

    print(f"Plotting {len(entries)} entries …")

    datetimes = [
        datetime.fromtimestamp(e["timestamp_s"], tz=timezone.utc) for e in entries
    ]
    temps    = [e["temp_c"]      for e in entries]
    vib_rms  = [e["vib_rms_mg"]  for e in entries]
    vib_peak = [e["vib_peak_mg"] for e in entries]

    fig, ax1 = plt.subplots(figsize=(12, 5))

    color_temp = "tab:red"
    ax1.set_xlabel("Time (UTC)")
    ax1.set_ylabel("Temperature (°C)", color=color_temp)
    ax1.plot(datetimes, temps, color=color_temp, linewidth=1.2, label="Temp (°C)")
    ax1.tick_params(axis="y", labelcolor=color_temp)

    ax2 = ax1.twinx()
    ax2.set_ylabel("Vibration (mg)")
    ax2.plot(datetimes, vib_rms,  color="tab:blue",   linewidth=1.2, label="VibRMS (mg)")
    ax2.plot(datetimes, vib_peak, color="tab:orange", linewidth=1.2, label="VibPeak (mg)")

    ax1.xaxis.set_major_formatter(mdates.DateFormatter("%m-%d %H:%M"))
    fig.autofmt_xdate(rotation=30, ha="right")

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper left")

    plt.title("THLogger — Temperature & Vibration (BLE Download)")
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Download THLogger data over BLE and plot it."
    )
    parser.add_argument(
        "--name", default="IcicleTHLogger",
        help="BLE device name to scan for (default: IcicleTHLogger)"
    )
    parser.add_argument(
        "--timeout", type=float, default=15.0,
        help="BLE scan timeout in seconds (default: 15)"
    )
    args = parser.parse_args()

    entries = asyncio.run(download_log(args.name, args.timeout))

    if not entries:
        print("No data received.")
        sys.exit(0)

    save_log(entries)
    plot_entries(entries)


if __name__ == "__main__":
    main()
