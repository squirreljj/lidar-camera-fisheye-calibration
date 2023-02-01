# lidar-camera-fisheye-calibration

this calibration code is based on pnp algorithm,you should manually selet feature pointcloud from pcd and correspondens feature pixel on image.you better select over 
three pairs to get better result.

if you want calibrate fisheye between lidar: just change distortion vector D in calib.py (k1,k2,k3,k4)

if you want calibrate pinhole betwwen lidar: just change distortion vector D in calib.py to D =[[k1],[k2],[k3],[p1],[p2]]

you can calibrate rotation and translation between fisheye/pinhole and lidar by calib.py.

I provide a test sample to let you try.

below is reprojection result:

![fisheye1](https://user-images.githubusercontent.com/42079541/215985248-4bf2c664-d873-4305-9ffa-f648a70e0dae.jpg)
![fisheye2](https://user-images.githubusercontent.com/42079541/215985271-ee6b5226-3661-4491-8e12-14ca817a19e7.jpg)
![img_fisheye7 png](https://user-images.githubusercontent.com/42079541/215974353-62d38da0-fd00-4ffc-a2e1-32b5e34a3290.png)

