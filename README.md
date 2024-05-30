# robot_arm_control
## Repo Structure

move_and_record.py control the robot arm and record video

myConfig.py record the six dimensions of points and save at joint_configs

joint_configs.json recorded points

set_pt.py control ur5 move to the specific point

set_pts.py control ur5 to move along the points in "joint_configs.json"

camera.py control the camera to record video

circle.py control ur5 move in a circle

## Example Usages

### To record a video：

ur5_driver

ur5_moveit1

ur5_moveit2

roslaunch realsense2_camera rs_camera.launch

cd work/src

python move_and_record.py


### To create dataset and transforms

open 'developer command prompt'

Python D:\BR\Instant-NGP-for-RTX-3000-and-4000\Instant-NGP-for-RTX-3000-and-4000\scripts\colmap2nerf.py --video_in D:\BR\COLMAP\COLMAP-3.9.1-windows-cuda\video3\video.mp4 --video_fps 8 --run_colmap --aabb_scale 16

### just create dataset

ffmpeg -i video.mp4 -vf fps=8 %04d.jpg

mkdir images

mv *.jpg images/


## To train model

drag the folder containing the dataset and transformers files to instant-ngp

