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

Now, open the MATLAB and open the `~/GProxemicNavigation/ProxemicNavigation/GNS.m` 



<p align="center"><img src="https://raw.githubusercontent.com/maverickjoy/pepper-codes/master/docs/navigation-1.png" width="650"></p>

---

<p align="center"><img src="https://raw.githubusercontent.com/maverickjoy/pepper-codes/master/docs/navigation-2.png" width="650"></p>

---

- **Video Link**

[![ASTHAMA SERACH VIDEO][video-image-1]][video-url-1]

- **Other Video Links**

[![FALL DETECTION VIDEO][video-image-2]][video-url-2]

[![TICTACTOE VIDEO][video-image-3]][video-url-3]


## License

MIT License 2018 © Vikram Singh and [contributors](https://github.com/maverickjoy/pepper-codes/graphs/contributors)

[sdk-url]: https://community.ald.softbankrobotics.com/en/resources/software/language/en-gb/robot/pepper-3
[sdk-image]: https://img.shields.io/badge/Python%202.7%20SDK-2.5.5-008C96.svg?style=flat

[naoqi-url]: https://developer.softbankrobotics.com/us-en/downloads/pepper
[naoqi-image]: https://img.shields.io/badge/NAOqi-2.5.5-008C96.svg

[mit-image]: https://img.shields.io/badge/license-MIT-blue.svg
[mit-url]: https://opensource.org/licenses/MIT

[video-image-1]: https://img.youtube.com/vi/lcxtWwkrp4c/0.jpg
[video-url-1]: https://youtu.be/lcxtWwkrp4c

[video-image-2]: https://img.youtube.com/vi/n_cCs7YTf70/0.jpg
[video-url-2]: https://youtu.be/n_cCs7YTf70

[video-image-3]: https://img.youtube.com/vi/a2yzU2n8eSA/0.jpg
[video-url-3]: https://youtu.be/a2yzU2n8eSA
