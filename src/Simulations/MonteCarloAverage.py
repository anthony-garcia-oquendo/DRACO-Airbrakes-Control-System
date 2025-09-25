import numpy as np
import matplotlib.pyplot as plt

# Monte Carlo parameters
sims = 10000000

scenarios = np.array([
    (0,  0,  0.05),
    (5,  5,  0.10),
    (5, 10,  0.70),
    (10, 15, 0.10),
    (10, 20, 0.05),
], dtype=float)

angles  = scenarios[:, 0]
winds   = scenarios[:, 1]
weights = scenarios[:, 2]

APOGEE_MIN = 4480.0
APOGEE_MAX = 4750.0

# --- Use observed OpenRocket averages (one per scenario, in same order) ---
obs_means = np.array([4642.0, 4628.8, 4634.35, 4555.0, 4482.9])

# If any obs_means are outside [min,max], warn and clip
if np.any((obs_means < APOGEE_MIN) | (obs_means > APOGEE_MAX)):
    raise ValueError("One or more observed means fall outside APOGEE_MIN/APOGEE_MAX")

# Solve for triangular mode: b = 3*mu - a - c
mode_by_obs = 3.0 * obs_means - APOGEE_MIN - APOGEE_MAX
# Clip to valid range
mode_by_obs = np.clip(mode_by_obs, APOGEE_MIN, APOGEE_MAX)

# If the computed mode ends up equal to endpoints frequently, consider a different distribution
print("calibrated triangular modes by scenario:", mode_by_obs)

# --- Sampling: stratified approach (vectorized per scenario) ---
# Determine number of samples per scenario (exact proportion)
n_per_scenario = np.random.multinomial(sims, weights)

apogees = np.empty(sims, dtype=float)
pos = 0
for i, n in enumerate(n_per_scenario):
    if n == 0:
        continue
    a = APOGEE_MIN
    b = mode_by_obs[i]
    c = APOGEE_MAX
    # Draw n triangular samples
    apogees[pos:pos+n] = np.random.triangular(a, b, c, size=n)
    pos += n

# Shuffle so distribution isn't block-structured
np.random.shuffle(apogees)

# Quick diagnostics
emp_mean_by_scenario = []
pos = 0
for n in n_per_scenario:
    if n == 0:
        emp_mean_by_scenario.append(np.nan)
        continue
    seg = apogees[pos:pos+n]
    emp_mean_by_scenario.append(seg.mean())
    pos += n

print("empirical means per scenario (from MC):", emp_mean_by_scenario)
print("overall MC mean:", apogees.mean())

# Estimate mode from histogram (as before)
counts, edges = np.histogram(apogees, bins=200, range=(APOGEE_MIN, APOGEE_MAX))
mode_est = 0.5 * (edges[counts.argmax()] + edges[counts.argmax() + 1])
print(f"Estimated most probable apogee: {mode_est:.1f} ft")

plt.figure(figsize=(8, 4))
plt.hist(apogees, bins=200, range=(APOGEE_MIN, APOGEE_MAX), density=True)
plt.axvline(mode_est, linestyle="--")
plt.xlabel("Apogee (ft)")
plt.ylabel("Probability Density")
plt.title("Calibrated Monte Carlo Apogee Distribution")
plt.show()
