# lidar-camera-fisheye-calibration

this calibration code is based on pnp algorithm,you should manually selet feature pointcloud from pcd and correspondens feature pixel on image.you better select over 
three pairs to get better result.

if you want calibrate fisheye between lidar: just change distortion vector D in calib.py (k1,k2,k3,k4)

if you want calibrate pinhole betwwen lidar: just change distortion vector D in calib.py to D =[[k1],[k2],[k3],[p1],[p2]]

you can calibrate rotation and translation between fisheye/pinhole and lidar by calib.py.

I provide a test sample to let you try.

below is reprojection result:


![image](https://user-images.githubusercontent.com/42079541/215654599-92a5e60d-8b7d-437f-8c54-6d151bbc3eaa.png)

