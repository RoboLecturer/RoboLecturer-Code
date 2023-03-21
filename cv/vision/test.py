import numpy as np

arr = np.array([0, 200])
key = arr.tostring()

# Reconstruct the key to the original array form
reconstructed_arr = np.frombuffer(key, dtype=int)


print(np.divide([2, 100], [2, 100]))



