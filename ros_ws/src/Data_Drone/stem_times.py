import numpy as np
import matplotlib.pyplot as plt
from scipy.io import savemat

simid_o = 505202503
simid_p = 505202502

ORIGINAL_FILE = f"src/Data_Drone/data_files/computation_times_{simid_o}.npy"
PARALLEL_FILE = f"src/Data_Drone/data_files/computation_times_{simid_p}_parallel.npy"

START_IDX = 100

original_times = np.load(ORIGINAL_FILE, allow_pickle=True)
parallel_times = np.load(PARALLEL_FILE, allow_pickle=True)

original_times = original_times.item()
original_times = np.array(original_times['comp_times'])

parallel_times = parallel_times.item()
parallel_times = np.array(parallel_times['comp_times'])

timesteps = np.arange(1, len(parallel_times) + 1)

avg_original = np.mean(original_times[START_IDX:])
avg_parallel = np.mean(parallel_times[START_IDX:])

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(timesteps[START_IDX:], original_times[START_IDX:], marker='o', color='tab:blue', label="Sequential", markersize=4, linewidth=1, alpha=1)
plt.plot(timesteps[START_IDX:], parallel_times[START_IDX:], marker='o', color='tab:orange', label="Parallel", markersize=4, linewidth=1, alpha=1)

plt.axhline(avg_original, color='magenta', linestyle='--', linewidth=2, label=f"Seq Avg = {avg_original:.3f} s")
plt.axhline(avg_parallel, color='green', linestyle='--', linewidth=2, label=f"Par Avg = {avg_parallel:.3f} s")

plt.xlabel("Time Index n")
plt.ylabel("Computation Time (s)")
plt.title("MPC Solve Time per Step for 3 Drones")
plt.grid(True)
plt.legend(loc='upper right')
plt.tight_layout()

plt.savefig("computation_time_3_drone", dpi=300, bbox_inches='tight')

plt.show()
# savemat(f"computation_times_{simid_o}_{simid_p}.mat", {
#     "seq": original_times,
#     "par": parallel_times,
# })
