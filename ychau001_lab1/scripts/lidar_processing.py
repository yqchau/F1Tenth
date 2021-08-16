import numpy as np 

def process_lidar(arr):
    # filters out inf / NaN in a numpy array
    arr = np.array(arr)
    arr = arr[np.isfinite(arr)]
    return arr


### EXAMPLE ###
if __name__ == '__main__':
    x = np.array([np.NaN, np.inf, 1, 2, 3])
    print (x)
    print (process_lidar(x))
