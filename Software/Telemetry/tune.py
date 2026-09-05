#!/usr/bin/env python3
"""
tune.py — šalje PID gain-ove dronu u letu (CRTP port 0x0F).

Upotreba:
  python tune.py 800 0 100          # Kp=800 Ki=0 Kd=100 za roll i pitch (osa 3)
  python tune.py 800 0 100 0        # samo roll
  python tune.py 800 0 100 1        # samo pitch
  python tune.py 300 0 0 2          # yaw (rate petlja)
  python tune.py --hello            # ne menja ništa, samo registruje laptop za telemetriju

Interaktivno:
  python tune.py -i                 # unosiš "kp ki kd [osa]" liniju po liniju

Paket = packed struct iz comm.h:
  uint8  header  (port<<4 | channel)
  float  roll    -> Kp
  float  pitch   -> Ki
  float  yaw     -> Kd
  uint16 thrust  -> osa (0 roll, 1 pitch, 2 yaw, 3 roll+pitch, 255 hello)
  uint8  reserved
"""
import socket
import struct
import sys

DRONE_IP = "192.168.43.42"
DRONE_PORT = 2390
PORT_TUNING = 0x0F
AXIS_HELLO = 255

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(kp: float, ki: float, kd: float, axis: int) -> None:
    header = (PORT_TUNING << 4) | 0
    pkt = struct.pack("<BfffHB", header, kp, ki, kd, axis, 0)
    sock.sendto(pkt, (DRONE_IP, DRONE_PORT))
    if axis == AXIS_HELLO:
        print("hello poslat -> telemetrija ide na ovaj laptop (UDP 2391)")
    else:
        names = {0: "roll", 1: "pitch", 2: "yaw", 3: "roll+pitch"}
        print(f"poslato: osa={names.get(axis, axis)}  Kp={kp}  Ki={ki}  Kd={kd}")


def interactive() -> None:
    print("Unos: kp ki kd [osa]   (osa: 0 roll, 1 pitch, 2 yaw, 3 oba; q za izlaz)")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ("q", "quit", "exit"):
            break
        parts = line.split()
        if len(parts) < 3:
            print("treba bar 3 broja")
            continue
        try:
            kp, ki, kd = map(float, parts[:3])
            axis = int(parts[3]) if len(parts) > 3 else 3
        except ValueError:
            print("nevalidan unos")
            continue
        send(kp, ki, kd, axis)


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
    elif args[0] == "--hello":
        send(0, 0, 0, AXIS_HELLO)
    elif args[0] == "-i":
        send(0, 0, 0, AXIS_HELLO)
        interactive()
    else:
        kp, ki, kd = map(float, args[:3])
        axis = int(args[3]) if len(args) > 3 else 3
        send(kp, ki, kd, axis)
