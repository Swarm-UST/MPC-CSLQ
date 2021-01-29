#!/usr/bin/env python
from trajectory_opt.srv import OptimizeTrajectory
import sys
import rospy
from std_msgs.msg import Float32MultiArray,MultiArrayDimension
import time
import numpy as np






def listToMutliArray(input):
    msg = Float32MultiArray()
    msg.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
    msg.layout.dim[0].size = 1
    msg.layout.dim[1].size = 3
    msg.data = input
    return msg

def npToMultiArray(input):
    msg = Float32MultiArray()
    msg.layout.dim = [MultiArrayDimension(),MultiArrayDimension()]
    msg.layout.dim[0].size = input.shape[1]
    msg.layout.dim[1].size = input.shape[0]
    msg.data = (input.T).reshape(input.size).tolist()[0]
    return msg

def MultiArrayTonp(input):
    dims = map(lambda x: x.size, input.layout.dim)
    return np.array(input.data, dtype=float).reshape(dims).astype(float)
   





def main():
    x_init = listToMutliArray([0.0,0.0,0.0])
    x_tar = listToMutliArray([1.0,1.5,0.0])
    c = np.matrix("1,1.2,0.1,0.2,0.03,1.0;0.5,0,0.2,0.2,0.03,1.0;0,0.5,0.2,0.2,0.03,0.0").T
    cons = npToMultiArray(c)
    rospy.init_node('Opt_client')
    start = time.time()
    rospy.wait_for_service("optimize_trajectory")
    service = rospy.ServiceProxy("optimize_trajectory", OptimizeTrajectory)
    result = service(x_init,x_tar,cons,0.5)
    end = time.time()
    print(end-start)
    print(MultiArrayTonp(result.opt_state).shape)
    print("")
    print(MultiArrayTonp(result.opt_input).shape)
    print("")
    print(MultiArrayTonp(result.opt_feedback).shape)

    return 

if __name__ == "__main__":  
    main()
