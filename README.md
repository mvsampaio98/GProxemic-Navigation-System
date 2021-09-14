# GProxemic-Navigation-System

With the robotics development, social robots interact with people, demanding they model the human being behavior to increase social navigation, considering proxemic spaces. However, human proxemic preferences can change in function of different social restrictions (e.g., culture, gender,local, the environment). Thus, robots should consider all these aspects to tailor their navigation. Towards an adaptable social navigation, we develop the GProxemic Navigation system that allows identifying the robot localization in a geo-referenced map, with semantic annotations related to social restrictions, in function of which, they chose the correct proxemic spaces they most respect in their autonomous navigation process.

## Installation

Install all dependencies: [Pepper Tutorial V7](https://docs.google.com/document/d/1K9oeyg6SG2pGCebCjUjPsZqBrHpuPNQ1/edit#).

Make a copy of all archives.`

```bash
git clone https://github.com/mvsampaio98/GProxemic-Navigation-System
```

Move the folder `~/GProxemicNavigation/pepper_nav` to `~/pepper_sim_ws/src/pepper_virtual`

In the folder `~/pepper_sim_ws`, make a compilation:

```bash
catkin_make
```

- **Make the Proxemic Navigation**

First, initialize the Pepper Robot and the Environment in Gazebo.

```bash
  // To open the Office
roslaunch pepper_nav CPU_office.launch
  // To open the Restaurant
roslaunch pepper_nav restaurant.launch
  // To open the Shopping
roslaunch pepper_nav square.launch
```

After that, initialize the basic navigation packages of ROS.


```bash
  // To the Office
roslaunch pepper_nav amcl_office.launch
  // To the Restaurant
roslaunch pepper_nav amcl_restaurant.launch
  // To the Shopping
roslaunch pepper_nav amcl_square.launch
```

**EDITAR Open the index.html. EDITAR**

Now, open the MATLAB and open the `~/GProxemicNavigation/ProxemicNavigation/GNS.m` and run.

Lastly, indicate the goal, the people position and the possible service robots directions.

For more information, watch the video below

**VIDEO LINK**
