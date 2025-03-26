# How to use
進入到'isaac_ros2_nav2_create_images_file'floder，執行以下
```
docker build -t isaac_sim_lab_ros .
```
# 啟動容器方法

# for test
```
docker run --name isaac_sim_lab_container \
    --gpus '"device=0"' \
    -it --rm \
    --network=host \
    --env-file .env \
    isaac_sim_lab_ros:latest
```
# example1
```
docker run --name isaac_sim_lab_container \
    --gpus '"device=0"' \
    -it --rm \
    --network=host \
    --env-file .env \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v /media/Pluto/binghua/TSMC_docker/isaac-sim/documents:/root/Documents:rw \
    -v /media/Pluto/binghua/Isaac_sim_other_resource:/isaac-sim/Isaac_sim_other_resource:rw \
    -v /media/Pluto/binghua/TSMC_docker/IsaacLab:/opt/IsaacLab/:rw \
    isaac_sim_lab_ros:latest
```
# example2
```
docker run --name isaac_sim_lab_container \
    --gpus '"device=0"' \
    -it --rm \
    --network=host \
    --env-file .env \
    -v /Path/to/User/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v /Path/to/User/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v /Path/to/User/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v /Path/to/User/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v /Path/to/User/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v /Path/to/User/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v /Path/to/User/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v /Path/to/User/docker/isaac-sim/documents:/root/Documents:rw \
    -v /Path/to/User/docker/Isaac_sim_other_resource:/isaac-sim/Isaac_sim_other_resource:rw \
    -v /Path/to/User/docker/IsaacLab:/opt/IsaacLab/:rw \
    isaac_sim_lab_ros:latest
```
# 如果X11vnc沒有自動啟動
執行:
```
/opt/startup/start_xvfb_vnc.sh
```
# 測試ROS2與X11正確啟用
```
echo $DISPLAY
```
如果輸出不是':1'，請手動設定
```
export DISPLAY=:1
```

啟動ROS2測試
```
ros2 run rviz2 rviz2
```

# 測試Isaac sim與ROS2連接
啟動Isaac sim
```
./runheadless.sh
```
查看ROS2 topic
```
ros2 topic list
```

