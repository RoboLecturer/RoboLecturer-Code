import numpy as np


def reject_points(array, threshold=15):
    reject_idxs = []
    array_len = array.shape[0]
    for i in range(array_len):
        coords_1 = array[i]
        for j in range(i + 1, array_len):
            coords_2 = array[j]
            dist = np.linalg.norm(coords_2 - coords_1)

            if dist < threshold and dist != 0:
                reject_idxs.append(j)

    array = np.delete(array, reject_idxs, axis=0)

    return array


if __name__ == "__main__":
    a = np.array([[100, 100], [2, 100], [101, 100], [99, 101]])
    print(a)
    arr = reject_points(a)
    print(arr)