### Gazebo 资源设置

需要设置 <u>GAZEBO_RESOURCE_PATH</u> 环境变量

比如：

> export GAZEBO_RESOURCE_PATH="~/.ignition/models"

或者 直接写在 bashrc文件中

然后在该路径下创建包名文件夹，把网格文件放在对应的文件夹里

仿真步长不能太短，GPU和CPU如果跟不上，数据会不对，比如反馈的关节位置和速度

### ROS Jazzy版本

仿真注意安装对应控制器和状态发布器

> ros-jazzy-velocity-controllers
>
> ros-jazzy-position-controllers
>
> ros-jazzy-joint-state-broadcaster

注意不是ros-jazzy-joint-state-publisher,这个可以用来在rviz 中查看关节是否安装对了。gazebo 中是用的发布器

### 手动启动控制器

现在已经集成到driver的代码里了，不用手动开启了

```bash
ros2 control load_controller --set-state active   joint_state_broadcaster
ros2 control load_controller --set-state active   position_controller
ros2 control load_controller --set-state active   velocity_controller
```

### 特别注意

ros2 humble 和jazzy 中有些东西改了，比如某个东西用向量表示的，之前x,y,z 对应a,b,c，现在有可能对应，b,a,c了，尤其和gazebo 相关的东西，humble和gazebo集成度不好，不要用了，用jazzy,对应ubuntu24.04