#!/usr/bin/env python3

if __name__ == '__main__':
    ## Installed
    import numpy as np
    from scipy.spatial.transform import Rotation as R

    a = R.from_matrix([
        [0.0225226,0.999745,0.0017194],
        [0.0648765,-0.00317777,0.997888],
        [0.997639,-0.0223635,-0.0649315]
        ])
    
    b = R.from_matrix([
        [0,0,1],
        [-1,0,0],
        [0,-1,0]
        ])
    
    ba = np.matmul(b.as_matrix(), a.as_matrix())
    
    q = R.from_matrix(ba)
    
    print("a:", a.as_quat())
    print("b:", b.as_quat())
    print("q:", q.as_quat())
    
