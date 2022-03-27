# formation

A ROS package for multi-agent formation/rendezvous 

### Acknowledgement

   [Xu Fang](https://scholar.google.com/citations?user=w8vxb4gAAAAJ&hl=en), [Chen Wang](https://wang-chen.github.io), [Lihua Xie](https://www.ntu.edu.sg/home/elhxie/), [Jie Chen](https://baike.baidu.com/item/%E9%99%88%E6%9D%B0/3890094), "Cooperative Pursuit with Multi-Pursuer and One Faster Free-moving Evader", IEEE Transactions on Cybernetics, 2020.


## Install Dependencies

   Only **Ubuntu 16 (ROS Kinetic)** is supported.

      sudo apt install python-tk
      sudo apt install ros-kinetic-hector-*
      sudo apt install ros-kinetic-hardware-interface
      sudo apt install ros-kinetic-controller-interface
      sudo apt install ros-kinetic-gazebo-ros-control
   
   Put following repo in your ROS workspace

      https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor

## Compile (catkin_make) then Run
      roslaunch formation multiquadcopters.launch

# The simulation for formation control (click to watch video):
     
   [<img src="https://img.youtube.com/vi/TrHX4kf0_jU/maxresdefault.jpg" width="100%">](https://youtu.be/TrHX4kf0_jU)

# The simulation for rendezvous control (pursuit):

   [<img src="https://img.youtube.com/vi/-UGwG7mGVNw/maxresdefault.jpg" width="100%">](https://youtu.be/-UGwG7mGVNw)

# Please consider cite:

      @article{fang2020cooperative,
        title={Cooperative pursuit with multi-pursuer and one faster free-moving evader},
        author={Fang, Xu and Wang, Chen and Xie, Lihua and Chen, Jie},
        journal={IEEE transactions on cybernetics},
        year={2020},
        publisher={IEEE}
      }
