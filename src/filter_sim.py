import numpy as np
from filter import DataFilter   # or from <your_file> import DataFilter
import matplotlib.pyplot as plt

def simulate_motion(n_steps=1000, dt=0.5,
                    a_true=-9.81,
                    alt_noise_std=10.0,
                    acc_noise_std=0.2,
                    seed=0):
    """
    Simulates 1D motion with constant acceleration.
    Returns:
        t: time array
        true_alt, true_vel, true_acc: ground-truth arrays
        meas_alt, meas_acc: noisy sensor readings
    """
    rng = np.random.default_rng(seed)

    t = np.arange(n_steps) * dt

    true_alt = np.zeros(n_steps)
    true_vel = np.zeros(n_steps)
    true_acc = np.ones(n_steps) * a_true

    x = 0.0
    v = 0.0

    for k in range(n_steps):
        # basic kinematics: x_{k+1} = x + v*dt + 0.5*a*dt^2, v_{k+1} = v + a*dt
        x = x + v * dt + 0.5 * a_true * dt**2
        v = v + a_true * dt

        true_alt[k] = x
        true_vel[k] = v

    # Add noise to "sensors"
    meas_alt = true_alt + rng.normal(0.0, alt_noise_std, size=n_steps)
    meas_acc = true_acc + rng.normal(0.0, acc_noise_std, size=n_steps)

    return t, true_alt, true_vel, true_acc, meas_alt, meas_acc


def test_data_filter():
    dt = 0.01
    n_steps = 500

    # 1) Simulate motion and sensor readings
    (t,
     true_alt, true_vel, true_acc,
     meas_alt, meas_acc) = simulate_motion(n_steps=n_steps, dt=dt,
                                           a_true=-9.81,
                                           alt_noise_std=5.0,
                                           acc_noise_std=1)

    # 2) Create your Kalman filter wrapper
    df = DataFilter()

    est_alt = []
    est_vel = []
    est_acc = []

    # 3) Run the filter on every measurement
    for z_alt, z_acc in zip(meas_alt, meas_acc):
        df.filter_data(z_alt, z_acc, dt=dt)  # <<< USING the dt we simulated with
        est_alt.append(df.kalman_altitude)
        est_vel.append(df.kalman_velocity)
        est_acc.append(df.kalman_acceleration)

    est_alt = np.array(est_alt)
    est_vel = np.array(est_vel)
    est_acc = np.array(est_acc)

    # 4) Compute error metrics
    alt_rmse = np.sqrt(np.mean((est_alt - true_alt)**2))
    vel_rmse = np.sqrt(np.mean((est_vel - true_vel)**2))
    acc_rmse = np.sqrt(np.mean((est_acc - true_acc)**2))

    print(f"Altitude RMSE:     {alt_rmse:.3f}")
    print(f"Velocity RMSE:     {vel_rmse:.3f}")
    print(f"Acceleration RMSE: {acc_rmse:.3f}")

    # 5) Quick pass/fail style check (you can tune thresholds)
    if alt_rmse < 10.0 and vel_rmse < 5.0 and acc_rmse < 2.0:
        print("✅ DataFilter passes this basic test.")
    else:
        print("⚠️ Errors are larger than expected, might need tuning.")

    # 6) Optional: visualize
    plt.figure()
    plt.title("Altitude")
    plt.plot(t, true_alt, label="True altitude")
    plt.plot(t, meas_alt, label="Measured altitude", alpha=0.4)
    plt.plot(t, est_alt, label="Kalman altitude")
    plt.legend()
    plt.xlabel("Time [s]")
    plt.ylabel("Altitude")

    plt.figure()
    plt.title("Velocity")
    plt.plot(t, true_vel, label="True velocity")
    plt.plot(t, est_vel, label="Kalman velocity")
    plt.legend()
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity")

    plt.figure()
    plt.title("Acceleration")
    plt.plot(t, true_acc, label="True acceleration")
    plt.plot(t, est_acc, label="Kalman acceleration")
    plt.legend()
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration")

    plt.show()


if __name__ == "__main__":
    test_data_filter()
