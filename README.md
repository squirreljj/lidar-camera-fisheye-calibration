# lidar-camera-fisheye-calibration


This calibration code is based on pnp algorithm.

Our algorithm support calibration between (fisheye/pinhole)camera and lidar.

## Solid_state lidar is highly recommened,and do not move the devices until all data is recored. (enough sample has been collected.The num of sample better >=5)
We give two way to calibrate lidar and camera:

## 1)calib.py

You can manually selet pointcloud feature and image corner feature


<div align=center><img width="600" height="350" src="https://user-images.githubusercontent.com/42079541/215989229-b936952a-116c-4696-9771-133aabd85228.png"/></div>
<div align=center><img width="600" height="350" src="https://user-images.githubusercontent.com/42079541/215989345-e45a4015-1454-4cf0-8410-fe126b7b9093.png"/></div>

Change the number of feature point at code :for i in range(4):

## 2)calib_multi.py

Image should be save as: 
> your_path/lidar_camera_calib/calib_data/img/
<div align=center><img width="800" height="200" src="https://user-images.githubusercontent.com/42079541/215990850-e83d1986-8e44-44ac-8912-83ab1fb511eb.png"/></div>


Pcd should be save as: 
> your_path/lidar_camera_calib/calib_data/pcd/
<div align=center><img width="700" height="250" src="https://user-images.githubusercontent.com/42079541/215990902-d05b2782-b907-4fb3-ad82-fcf2e87d6ae1.png"/></div>

Set `cropped_flag = True`if your pointcloud haven't been processed,and then you can crop chessboard in pointcloud.



Set `cropped_flag = False`to skip the pointcloud isolation and directly calibrate.(**PS:make sure chessboard pointcloud is ready**)

## Isolated the chessboard pointcloud as perfect as you can,a perfect pointcloud means a perfect calibration.

<div align=center><img width="300" height="350" src="https://user-images.githubusercontent.com/42079541/215991733-24545882-e0b1-450d-afca-48dea7570139.png"/></div>

After crop chessboard,save pcd as cropped_1.pcd,cropped_2.pcd,cropped_3.pcd...
(**PS:Start from 1**)


The rotation vetor and translation vetor and rotation matrix between cam and lidar would be print in terminal:
<div align=center><img width="350" height="300" src="https://user-images.githubusercontent.com/42079541/215993428-53335830-cad8-4d36-aa55-08a4321e8461.png"/></div>

# Change camera intrinsic

`cv2.solvePnP `support 4、5、8 distort coefficient : **{[(k1,k2,p1,p2),k3],k4,k5,k6}** ,fisheye use k1、k2、k3、k4，pinhole use k1,k2,p1,p2,k3
For different camera model, just change distortion vector https://github.com/squirreljj/lidar-camera-fisheye-calibration/blob/86dcca3fa5d46aa14d2f6676c0452a1dc3e32ebc/calib.py#L68
`np.array([[k1],[k2],[0],[0],[k3],[k4],[0],[0]])` for KB4

`np.array([[k1],[k2],[p1],[p2][k3],[0],[0].[0]])`for PINHOLE

# Quick start
You can verify that this repository runs sucessfully by running this package on our provided quick-start data. 

Folder
<calib >for calib.py

Folder
>calib_multi 

for calib_multi.py

# Reprojection
The image below shows the reprojection result：

<div align=center><img width="800" height="800" src="https://user-images.githubusercontent.com/42079541/215985248-4bf2c664-d873-4305-9ffa-f648a70e0dae.jpg"/></div>
<div align=center><img width="800" height="800" src="https://user-images.githubusercontent.com/42079541/215985271-ee6b5226-3661-4491-8e12-14ca817a19e7.jpg"/></div>
<div align=center><img width="800" height="800" src="https://user-images.githubusercontent.com/42079541/215974353-62d38da0-fd00-4ffc-a2e1-32b5e34a3290.png"/></div>
