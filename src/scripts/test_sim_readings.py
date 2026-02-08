#!/usr/bin/python3
import time
from dataclasses import dataclass
from typing import List

import board
from smbus2 import SMBus, i2c_msg

from devices.sensors import BMP390, ICM20948

G_FTPS2 = 32.174

SIM_ADDR = 0x28
CTRL_REG = 0x1F

CMD_STOP  = 0x00
CMD_START = 0x01
CMD_RESET = 0x02

def feet_to_meters(ft: float) -> float:
    return ft / 3.28084

@dataclass
class OrSample:
    t_s: float
    alt_m: float
    a_vert_ftps2: float

def load_openrocket_csv(path: str) -> List[OrSample]:
    samples: List[OrSample] = []
    with open(path, "r", newline="") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 5:
                continue
            samples.append(
                OrSample(
                    t_s=float(parts[0]),
                    alt_m=float(parts[1]),
                    a_vert_ftps2=float(parts[4]),
                )
            )
    if not samples:
        raise RuntimeError(f"No samples parsed from {path}")
    return samples

def nearest_sample(samples: List[OrSample], t_s: float) -> OrSample:
    best = samples[0]
    best_dt = abs(samples[0].t_s - t_s)
    for s in samples[1:]:
        dt = abs(s.t_s - t_s)
        if dt < best_dt:
            best, best_dt = s, dt
        else:
            if s.t_s > t_s and dt > best_dt:
                break
    return best

# -------- ESP32 control helpers --------

def sim_stop(bus: SMBus):
    bus.write_i2c_block_data(SIM_ADDR, CTRL_REG, [CMD_STOP])

def sim_reset(bus: SMBus):
    bus.write_i2c_block_data(SIM_ADDR, CTRL_REG, [CMD_RESET])

def sim_start(bus: SMBus):
    bus.write_i2c_block_data(SIM_ADDR, CTRL_REG, [CMD_START])

# -------- Raw register read (for t_ms) --------

def read_regs(bus: SMBus, addr: int, start_reg: int, n: int) -> bytes:
    w = i2c_msg.write(addr, [start_reg])
    r = i2c_msg.read(addr, n)
    bus.i2c_rdwr(w, r)
    return bytes(r)

def read_u32_le(b: bytes) -> int:
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)

def main():
    # --- CONFIG ---
    FLIGHT_CSV_PATH = "flight.csv"
    WARMUP_SEC = 0.15
    TEST_DURATION_SEC = 8.0
    SAMPLE_RATE_HZ = 50

    ALT_TOL_M = 0.25
    AZ_TOL_FTPS2 = 2.0
    # --------------

    or_samples = load_openrocket_csv(FLIGHT_CSV_PATH)
    print(f"Loaded {len(or_samples)} OpenRocket samples from {FLIGHT_CSV_PATH}")

    bus = SMBus(1)

    i2c = board.I2C()
    altimeter = BMP390(
        i2c,
        simulation=True,
        sim_addr=SIM_ADDR,
        sim_alt_reg=0x04,
        sim_alt_scale=1000.0,
        sim_alt_is_hpa=False,
    )
    imu = ICM20948(
        i2c,
        simulation=True,
        sim_addr=SIM_ADDR,
        sim_accel_reg=0x08,
        sim_accel_scale=1000.0,
    )

    # ---- deterministic sync ----
    print("Syncing simulator (RESET -> START)...")
    sim_stop(bus)
    time.sleep(0.05)
    sim_reset(bus)   # idx=0, paused, publishes first sample
    time.sleep(WARMUP_SEC)
    sim_start(bus)   # idx=0, running, playback_start_ms=now
    time.sleep(0.02) # let t_ms become nonzero

    period = 1.0 / SAMPLE_RATE_HZ

    ok = bad = 0
    worst_alt = 0.0
    worst_az = 0.0

    # Determine when to stop based on sim time
    t0_ms = read_u32_le(read_regs(bus, SIM_ADDR, 0x00, 4))
    stop_at_ms = t0_ms + int(TEST_DURATION_SEC * 1000)

    print("t(s)  alt_sim(m)  alt_csv(m)  d_alt(m)   az_sim(ft/s2)  az_csv_imu(ft/s2)  d_az")

    try:
        while True:
            t_ms = read_u32_le(read_regs(bus, SIM_ADDR, 0x00, 4))
            if t_ms >= stop_at_ms:
                break

            t_s = t_ms / 1000.0

            # Read sim outputs via your device wrappers
            alt_ft = altimeter.altitude()
            alt_m_sim = feet_to_meters(alt_ft)

            ax, ay, az_sim = imu.acceleration()

            # Expected from CSV at the sim's timebase
            s = nearest_sample(or_samples, t_s)
            az_expected = s.a_vert_ftps2 + G_FTPS2

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

            print(
                f"{t_s:5.2f}  {alt_m_sim:9.3f}  {s.alt_m:9.3f}  {d_alt:8.3f}   "
                f"{az_sim:12.3f}  {az_expected:14.3f}  {d_az:6.2f}"
            )

            time.sleep(period)
    finally:
        sim_stop(bus)
        bus.close()

    total = ok + bad
    print("\n--- Summary ---")
    print(f"Samples checked: {total}")
    print(f"Pass: {ok}  Fail: {bad}")
    print(f"Worst |d_alt|: {worst_alt:.3f} m (tol {ALT_TOL_M} m)")
    print(f"Worst |d_az| : {worst_az:.3f} ft/s^2 (tol {AZ_TOL_FTPS2} ft/s^2)")

    if bad == 0:
        print("✅ Simulator readings match CSV (within tolerance).")
    else:
        print("⚠️ Some samples were out of tolerance.")
        print("Common causes:")
        print("- Z axis flip (FLIP_Z) or g constant mismatch")
        print("- Wrong CSV column (vertical accel vs total accel)")
        print("- Different fixed-point scaling on ESP32")

if __name__ == "__main__":
    main()
