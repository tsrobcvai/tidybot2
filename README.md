# tidybot2

This code release accompanies the following project:

### TidyBot++: An Open-Source Holonomic Mobile Manipulator for Robot Learning

Jimmy Wu, William Chong, Robert Holmberg, Aaditya Prasad, Yihuai Gao, Oussama Khatib, Shuran Song, Szymon Rusinkiewicz, Jeannette Bohg

*Conference on Robot Learning (CoRL)*, 2024

[Project Page](http://tidybot2.github.io) | [PDF](http://tidybot2.github.io/paper.pdf) | [arXiv](https://arxiv.org) | [Video](https://youtu.be/6MH7dhyIUdQ)

**Abstract:** Exploiting the promise of recent advances in imitation learning for mobile manipulation will require the collection of large numbers of human-guided demonstrations. This paper proposes an open-source design for an inexpensive, robust, and flexible mobile manipulator that can support arbitrary arms, enabling a wide range of real-world household mobile manipulation tasks. Crucially, our design uses powered casters to enable the mobile base to be fully holonomic, able to control all planar degrees of freedom independently and simultaneously. This feature makes the base more maneuverable and simplifies many mobile manipulation tasks, eliminating the kinematic constraints that create complex and time-consuming motions in nonholonomic bases. We equip our robot with an intuitive mobile phone teleoperation interface to enable easy data acquisition for imitation learning. In our experiments, we use this interface to collect data and show that the resulting learned policies can successfully perform a variety of common household mobile manipulation tasks.

## Overview

The following components will be open-sourced in the coming days. Please stay tuned!

* Holonomic base hardware design
* Holonomic base low-level controller
* Kinova Gen3 arm low-level controller
* Mobile phone teleoperation interface
* Policy learning setup

## Citation

If you find this work useful for your research, please consider citing:

```
@inproceedings{wu2024tidybot,
  title = {TidyBot++: An Open-Source Holonomic Mobile Manipulator for Robot Learning},
  author = {Wu, Jimmy and Chong, William and Holmberg, Robert and Prasad, Aaditya and Gao, Yihuai and Khatib, Oussama and Song, Shuran and Rusinkiewicz, Szymon and Bohg, Jeannette},
  booktitle = {Conference on Robot Learning},
  year = {2024}
}
```
