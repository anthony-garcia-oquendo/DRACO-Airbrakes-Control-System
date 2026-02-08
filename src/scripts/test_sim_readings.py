#!/usr/bin/python3
import time
import csv
import math
from dataclasses import dataclass
from typing import List, Tuple

import board

from devices.sensors import BMP390, ICM20948

G_FTPS2 = 32.174  # same value used on ESP32

def feet_to_meters(ft: float) -> float:
    return ft / 3.28084

@dataclass
class OrSample:
    t_s: float
    alt_m: float
    a_vert_ftps2: float

def load_openrocket_csv(path: str) -> List[OrSample]:
    """
    Parses OpenRocket export where data lines are comma-separated and
    comment/header/event lines start with '#'.

    Expected columns:
      time_s, altitude_m, altitude_asl_m, vertical_velocity_ftps, vertical_accel_ftps2
    """
    samples: List[OrSample] = []
    with open(path, "r", newline="") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 5:
                continue

            t_s = float(parts[0])
            alt_m = float(parts[1])
            a_vert = float(parts[4])

            samples.append(OrSample(t_s=t_s, alt_m=alt_m, a_vert_ftps2=a_vert))

    if not samples:
        raise RuntimeError(f"No samples parsed from {path}")

    return samples

def nearest_sample(samples: List[OrSample], t_s: float) -> OrSample:
    # since OR data is already uniform-ish, nearest neighbor is fine for a test script
    # (you can swap to interpolation later)
    best = samples[0]
    best_dt = abs(samples[0].t_s - t_s)
    for s in samples[1:]:
        dt = abs(s.t_s - t_s)
        if dt < best_dt:
            best, best_dt = s, dt
        else:
            # early exit if times are increasing and we passed the closest point
            if s.t_s > t_s and dt > best_dt:
                break
    return best

def main():
    # --- CONFIG ---
    FLIGHT_CSV_PATH = "flight.csv"   # put alongside this script or give absolute path
    WARMUP_SEC = 0.5
    TEST_DURATION_SEC = 8.0          # how long to compare
    SAMPLE_RATE_HZ = 50              # match your sim (20 ms)
    ALT_TOL_M = 0.25                 # tolerance in meters (adjust)
    AZ_TOL_FTPS2 = 2.0               # tolerance in ft/s^2 (adjust)
    # --------------

    or_samples = load_openrocket_csv(FLIGHT_CSV_PATH)
    print(f"Loaded {len(or_samples)} OpenRocket samples from {FLIGHT_CSV_PATH}")

    i2c = board.I2C()

    # IMPORTANT: your modified classes must support simulation=True and raw register reads
    altimeter = BMP390(i2c, simulation=True, sim_addr=0x28, sim_alt_reg=0x04, sim_alt_scale=1000.0, sim_alt_is_hpa=False)
    imu = ICM20948(i2c, simulation=True, sim_addr=0x28, sim_accel_reg=0x08, sim_accel_scale=1000.0)

    print("Warming up...")
    time.sleep(WARMUP_SEC)

    start = time.monotonic()
    end = start + TEST_DURATION_SEC
    period = 1.0 / SAMPLE_RATE_HZ

    ok = 0
    bad = 0
    worst_alt = 0.0
    worst_az = 0.0

    print("t(s)  alt_sim(m)  alt_csv(m)  d_alt(m)   az_sim(ft/s2)  az_csv_imu(ft/s2)  d_az")
    while time.monotonic() < end:
        t = time.monotonic() - start

        # Read from sim devices
        alt_ft = altimeter.altitude()              # your wrapper returns feet
        alt_m_sim = feet_to_meters(alt_ft)

        ax, ay, az_sim = imu.acceleration()        # ft/s^2

        # Get expected from CSV
        s = nearest_sample(or_samples, t)
        az_expected = s.a_vert_ftps2 + G_FTPS2      # IMU-like az

        d_alt = alt_m_sim - s.alt_m
        d_az = az_sim - az_expected

        worst_alt = max(worst_alt, abs(d_alt))
        worst_az = max(worst_az, abs(d_az))

        pass_alt = abs(d_alt) <= ALT_TOL_M
        pass_az = abs(d_az) <= AZ_TOL_FTPS2

        if pass_alt and pass_az:
            ok += 1
        else:
            bad += 1

        print(f"{t:5.2f}  {alt_m_sim:9.3f}  {s.alt_m:9.3f}  {d_alt:8.3f}   {az_sim:12.3f}  {az_expected:14.3f}  {d_az:6.2f}")

        time.sleep(period)

    total = ok + bad
    print("\n--- Summary ---")
    print(f"Samples checked: {total}")
    print(f"Pass: {ok}  Fail: {bad}")
    print(f"Worst |d_alt|: {worst_alt:.3f} m (tol {ALT_TOL_M} m)")
    print(f"Worst |d_az| : {worst_az:.3f} ft/s^2 (tol {AZ_TOL_FTPS2} ft/s^2)")

    if bad == 0:
        print("✅ Simulator readings match CSV (within tolerance).")
    else:
        print("⚠️ Some samples were out of tolerance. Common causes:")
        print("- Time alignment offset (sim playback start vs script start)")
        print("- Different g constant or sign flip")
        print("- CSV acceleration is inertial while your IMU-like uses +g (verify expected mapping)")

if __name__ == "__main__":
    main()
