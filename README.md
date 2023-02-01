# lidar-camera-fisheye-calibration


this calibration code is based on pnp algorithm.

our algorithm support calibration between (fisheye/pinhole)camera and lidar 

We give two way to calibrate lidar and camera:

1)calib.py

you can manually selet pointcloud feature and image corner feature

![image](https://user-images.githubusercontent.com/42079541/215989229-b936952a-116c-4696-9771-133aabd85228.png)
![image](https://user-images.githubusercontent.com/42079541/215989345-e45a4015-1454-4cf0-8410-fe126b7b9093.png)

you can change the number of feature point at code :for i in range(4):

2)calib_multi.py

you should store your image in folder: your_path/lidar_camera_calib/calib_data/img
![image](https://user-images.githubusercontent.com/42079541/215990850-e83d1986-8e44-44ac-8912-83ab1fb511eb.png)


you should store your pcd in folder: your_path/lidar_camera_calib/calib_data/pcd
![image](https://user-images.githubusercontent.com/42079541/215990902-d05b2782-b907-4fb3-ad82-fcf2e87d6ae1.png)

set cropped_flag = True,and then you can crop chessboard in pointcloud

when pointcloud winodw is open,press k to make window stable and then press ctrl+left mouse to crop chessboard.

![image](https://user-images.githubusercontent.com/42079541/215991733-24545882-e0b1-450d-afca-48dea7570139.png)
after crop chessboard ,press s in window and save pcd like crop1.pcd,crop2.pcd,crop3.pcd...(attention:you should start from 1)

after done this,you will get rotation vetor and translation vetor and rotation matrix between cam and lidar 
![image](https://user-images.githubusercontent.com/42079541/215993428-53335830-cad8-4d36-aa55-08a4321e8461.png)

# how to change camera intrinsic
if you want calibrate fisheye(kb4 model) between lidar: just change distortion vector D dist_coeffs = np.array([[k1],[k2],[0],[0],[k3],[k4],[0],[0]])

if you want calibrate pinhole betwwen lidar: just change distortion vector D in calib.py to D =[[k1],[k2],[k3],[p1],[p2]]

you can calibrate rotation and translation between fisheye/pinhole and lidar by calib.py.

I provide a test sample to let you try.

# result
below is reprojection result:

![fisheye1](https://user-images.githubusercontent.com/42079541/215985248-4bf2c664-d873-4305-9ffa-f648a70e0dae.jpg)
![fisheye2](https://user-images.githubusercontent.com/42079541/215985271-ee6b5226-3661-4491-8e12-14ca817a19e7.jpg)
![img_fisheye7 png](https://user-images.githubusercontent.com/42079541/215974353-62d38da0-fd00-4ffc-a2e1-32b5e34a3290.png)

