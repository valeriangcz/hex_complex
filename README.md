从当前容器构建镜像
```
docker commit \
  ros1_isaacgym_dev（容器） \
  my_ros1_isaacgym:v1（镜像）
  ```

镜像保存为.tar格式
```
docker save \
  -o my_ros1_isaacgym_v1.tar \
  my_ros1_isaacgym:v1
```

从.tar格式加载镜像
```
docker load -i my_ros1_isaacgym_v1.tar
```


从镜像运行容器
```
docker run -it \
  --name from_iamge_ros1 \
  --network host \
  --ipc host \
  --gpus all \
  --device /dev/input/by-id/usb-Microsoft_Controller_3039373030323836363032343531-joystick:/dev/input/js0 \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/val/valerian_ws/docker_ros1:/root/workspace \
  ros1_isaacgym:v1  \
  bash
  ```


  catkin_make不编译tests中的内容
  git 中没有追踪tests
  ```
  catkin_make -DCATKIN_ENABLE_TESTING=OFF
  ```
  