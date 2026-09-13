"""
Visualizes gps_denial_log.csv: plots the drone's estimated ground track
and altitude over time, with the GPS-dropout window highlighted in red.

Requires: pip install matplotlib
"""

import csv
import matplotlib.pyplot as plt


LOG_FILE = "gps_denial_log.csv"


def load_log(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "t_s": float(row["t_s"]),
                "lat": float(row["lat"]),
                "lon": float(row["lon"]),
                "alt_rel_m": float(row["alt_rel_m"]),
                "gps_ok": row["gps_ok"] == "True",
            })
    return rows


def find_dropout_window(rows):
    start, end = None, None
    for row in rows:
        if not row["gps_ok"] and start is None:
            start = row["t_s"]
        if not row["gps_ok"]:
            end = row["t_s"]
    return start, end


def main():
    rows = load_log(LOG_FILE)
    if not rows:
        print(f"No data found in {LOG_FILE} — did gps_denial_test.py run successfully?")
        return

    times = [r["t_s"] for r in rows]
    lats = [r["lat"] for r in rows]
    lons = [r["lon"] for r in rows]
    alts = [r["alt_rel_m"] for r in rows]

    dropout_start, dropout_end = find_dropout_window(rows)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    ok_lats = [r["lat"] for r in rows if r["gps_ok"]]
    ok_lons = [r["lon"] for r in rows if r["gps_ok"]]
    bad_lats = [r["lat"] for r in rows if not r["gps_ok"]]
    bad_lons = [r["lon"] for r in rows if not r["gps_ok"]]

    ax1.plot(lons, lats, '-', color='gray', alpha=0.4, linewidth=1, zorder=1)
    ax1.scatter(ok_lons, ok_lats, c='tab:blue', s=12, label='GPS healthy', zorder=2)
    ax1.scatter(bad_lons, bad_lats, c='tab:red', s=12, label='GPS OFF (dropout)', zorder=3)
    ax1.set_xlabel("Longitude")
    ax1.set_ylabel("Latitude")
    ax1.set_title("Estimated Ground Track")
    ax1.legend()
    ax1.ticklabel_format(useOffset=False, style='plain')

    ax2.plot(times, alts, '-', color='tab:blue', linewidth=1.5)
    if dropout_start is not None:
        ax2.axvspan(dropout_start, dropout_end, color='red', alpha=0.15,
                    label=f'GPS dropout ({dropout_start:.0f}s - {dropout_end:.0f}s)')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Relative Altitude (m)")
    ax2.set_title("Altitude Estimate Over Time")
    ax2.legend()

    plt.tight_layout()
    output_path = "gps_denial_plot.png"
    plt.savefig(output_path, dpi=150)
    print(f"-- Plot saved to {output_path}")
    plt.show()


if __name__ == "__main__":
    main()
