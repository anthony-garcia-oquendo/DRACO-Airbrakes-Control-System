#!/usr/bin/python3
import time
from dataclasses import dataclass
from typing import List
from smbus2 import SMBus, i2c_msg

SIM_ADDR = 0x28
CTRL_REG = 0x1F

CMD_STOP  = 0x00
CMD_START = 0x01
CMD_RESET = 0x02

G_FTPS2 = 32.174

@dataclass
class OrSample:
    t_s: float
    alt_m: float
    a_vert_ftps2: float

def load_openrocket_csv(path: str) -> List[OrSample]:
    out: List[OrSample] = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 5:
                continue
            out.append(OrSample(
                t_s=float(parts[0]),
                alt_m=float(parts[1]),
                a_vert_ftps2=float(parts[4]),
            ))
    if not out:
        raise RuntimeError(f"No samples parsed from {path}")
    return out

def nearest_sample(samples: List[OrSample], t_s: float) -> OrSample:
    best = samples[0]
    best_dt = abs(best.t_s - t_s)
    for s in samples[1:]:
        dt = abs(s.t_s - t_s)
        if dt < best_dt:
            best, best_dt = s, dt
        else:
            if s.t_s > t_s and dt > best_dt:
                break
    return best

def write_ctrl(bus: SMBus, cmd: int):
    # write [cmd] to register 0x1F
    bus.write_i2c_block_data(SIM_ADDR, CTRL_REG, [cmd])

def read_regs(bus: SMBus, start_reg: int, n: int, retries: int = 5) -> bytes:
    # Robust read with retries (handles occasional NACKs)
    last_err = None
    for _ in range(retries):
        try:
            w = i2c_msg.write(SIM_ADDR, [start_reg])
            r = i2c_msg.read(SIM_ADDR, n)
            bus.i2c_rdwr(w, r)
            return bytes(r)
        except OSError as e:
            last_err = e
            time.sleep(0.01)  # 10ms backoff
    raise last_err

def i32_le(b: bytes) -> int:
    v = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)
    # signed
    if v & 0x80000000:
        v -= 0x100000000
    return v

def u32_le(b: bytes) -> int:
    return b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)

def main():
    FLIGHT_CSV_PATH = "flight.csv"
    TEST_DURATION_SEC = 8.0
    SAMPLE_RATE_HZ = 50
    ALT_TOL_M = 0.25
    AZ_TOL_FTPS2 = 2.0

    samples = load_openrocket_csv(FLIGHT_CSV_PATH)
    print(f"Loaded {len(samples)} OpenRocket samples from {FLIGHT_CSV_PATH}")

    period = 1.0 / SAMPLE_RATE_HZ

    with SMBus(1) as bus:
        # Sync sequence
        write_ctrl(bus, CMD_STOP)
        time.sleep(0.05)
        write_ctrl(bus, CMD_RESET)
        time.sleep(0.10)
        write_ctrl(bus, CMD_START)
        time.sleep(0.05)

        t0_ms = u32_le(read_regs(bus, 0x00, 4))
        stop_at_ms = t0_ms + int(TEST_DURATION_SEC * 1000)

        ok = bad = 0
        worst_alt = 0.0
        worst_az = 0.0

        print("t(s)  alt_sim(m)  alt_csv(m)  d_alt(m)   az_sim(ft/s2)  az_csv_imu(ft/s2)  d_az")

        while True:
            t_ms = u32_le(read_regs(bus, 0x00, 4))
            if t_ms >= stop_at_ms:
                break
            t_s = t_ms / 1000.0

            # Read sim registers
            alt_m = i32_le(read_regs(bus, 0x04, 4)) / 1000.0
            az_sim = i32_le(read_regs(bus, 0x10, 4)) / 1000.0

            # Expected from CSV (IMU-like)
            s = nearest_sample(samples, t_s)
            az_expected = s.a_vert_ftps2 + G_FTPS2

            d_alt = alt_m - s.alt_m
            d_az = az_sim - az_expected

            worst_alt = max(worst_alt, abs(d_alt))
            worst_az = max(worst_az, abs(d_az))

            pass_alt = abs(d_alt) <= ALT_TOL_M
            pass_az = abs(d_az) <= AZ_TOL_FTPS2

            if pass_alt and pass_az:
                ok += 1
            else:
                bad += 1

            print(f"{t_s:5.2f}  {alt_m:9.3f}  {s.alt_m:9.3f}  {d_alt:8.3f}   {az_sim:12.3f}  {az_expected:14.3f}  {d_az:6.2f}")

            time.sleep(period)

        # Stop sim at end (optional)
        write_ctrl(bus, CMD_STOP)

    total = ok + bad
    print("\n--- Summary ---")
    print(f"Samples checked: {total}")
    print(f"Pass: {ok}  Fail: {bad}")
    print(f"Worst |d_alt|: {worst_alt:.3f} m (tol {ALT_TOL_M} m)")
    print(f"Worst |d_az| : {worst_az:.3f} ft/s^2 (tol {AZ_TOL_FTPS2} ft/s^2)")

if __name__ == "__main__":
    main()
