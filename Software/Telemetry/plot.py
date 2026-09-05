#!/usr/bin/env python3
"""
plot.py — prima telemetriju sa drona preko UDP-a (port 2391), crta je u realnom
vremenu i upisuje u CSV.

Upotreba:
  python plot.py                  # prikazuje roll
  python plot.py --axis pitch     # prikazuje pitch
  python plot.py --log let1.csv   # ime log fajla (default: telemetry_<vreme>.csv)

Pre pokretanja pošalji "hello" dronu da zna kome da šalje:
  python tune.py --hello

Zahteva: pip install matplotlib
"""
import argparse
import csv
import socket
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

PORT = 2391
WINDOW = 600  # uzoraka na ekranu (~30 s na 20 Hz)

COLS = [
    "t_ms", "dt_ms",
    "sp_roll", "roll", "roll_rate", "roll_P", "roll_I", "roll_D", "roll_out",
    "sp_pitch", "pitch", "pitch_rate", "pitch_P", "pitch_I", "pitch_D", "pitch_out",
    "thrust", "m1", "m2", "m3", "m4",
]

data = {c: deque(maxlen=WINDOW) for c in COLS}
lock = threading.Lock()
stats = {"rx": 0, "bad": 0, "last": 0.0}


def receiver(log_path: str) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT))
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLS)
        while True:
            raw, _ = sock.recvfrom(1024)
            for line in raw.decode(errors="ignore").splitlines():
                if not line.startswith("T,"):
                    continue
                parts = line[2:].split(",")
                if len(parts) != len(COLS):
                    stats["bad"] += 1
                    continue
                try:
                    vals = [float(p) for p in parts]
                except ValueError:
                    stats["bad"] += 1
                    continue
                writer.writerow(vals)
                with lock:
                    for c, v in zip(COLS, vals):
                        data[c].append(v)
                stats["rx"] += 1
                stats["last"] = time.time()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--axis", choices=["roll", "pitch"], default="roll")
    ap.add_argument("--log", default=None)
    args = ap.parse_args()

    log_path = args.log or time.strftime("telemetry_%Y%m%d_%H%M%S.csv")
    threading.Thread(target=receiver, args=(log_path,), daemon=True).start()
    print(f"slušam UDP {PORT}, log -> {log_path}")

    ax_name = args.axis
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    fig.suptitle(f"PID tuning — {ax_name}")

    # 1: ugao, setpoint, brzina
    l_sp,   = a1.plot([], [], "k--", lw=1, label=f"sp_{ax_name} [°]")
    l_ang,  = a1.plot([], [], lw=1.5, label=f"{ax_name} [°]")
    l_rate, = a1.plot([], [], lw=0.8, alpha=0.7, label=f"{ax_name}_rate/10 [°/s]")
    a1.set_ylabel("ugao"); a1.grid(alpha=0.3); a1.legend(loc="upper left", fontsize=8)

    # 2: P, I, D, out
    l_p,   = a2.plot([], [], lw=1, label="P")
    l_i,   = a2.plot([], [], lw=1, label="I")
    l_d,   = a2.plot([], [], lw=1, label="D")
    l_out, = a2.plot([], [], "k", lw=1.5, label="out")
    a2.set_ylabel("thrust korekcija"); a2.grid(alpha=0.3); a2.legend(loc="upper left", fontsize=8)

    # 3: motori + thrust
    l_m = [a3.plot([], [], lw=1, label=f"m{i}")[0] for i in range(1, 5)]
    l_thr, = a3.plot([], [], "k--", lw=1, label="thrust")
    a3.set_ylabel("motori"); a3.set_xlabel("t [s]"); a3.grid(alpha=0.3)
    a3.legend(loc="upper left", fontsize=8, ncol=5)

    status = fig.text(0.99, 0.01, "", ha="right", va="bottom", fontsize=8)

    def update(_):
        with lock:
            if len(data["t_ms"]) < 2:
                return
            t0 = data["t_ms"][0]
            t = [(x - t0) / 1000.0 for x in data["t_ms"]]
            g = {c: list(data[c]) for c in COLS}

        l_sp.set_data(t, g[f"sp_{ax_name}"])
        l_ang.set_data(t, g[ax_name])
        l_rate.set_data(t, [v / 10.0 for v in g[f"{ax_name}_rate"]])

        l_p.set_data(t, g[f"{ax_name}_P"])
        l_i.set_data(t, g[f"{ax_name}_I"])
        l_d.set_data(t, g[f"{ax_name}_D"])
        l_out.set_data(t, g[f"{ax_name}_out"])

        for i, ln in enumerate(l_m, start=1):
            ln.set_data(t, g[f"m{i}"])
        l_thr.set_data(t, g["thrust"])

        for a in (a1, a2, a3):
            a.relim(); a.autoscale_view()
        a1.set_xlim(t[0], max(t[-1], t[0] + 1))

        age = time.time() - stats["last"] if stats["last"] else float("inf")
        dt_ms = g["dt_ms"][-1]
        status.set_text(
            f"rx={stats['rx']}  bad={stats['bad']}  dt={dt_ms:.2f} ms  "
            f"{'LIVE' if age < 1 else f'nema podataka {age:.0f}s'}"
        )

    _anim = FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.tight_layout(rect=(0, 0.02, 1, 0.96))
    plt.show()


if __name__ == "__main__":
    main()
