
<h1 align="center">
   Active Vision and Zero-Shot Learning for Enhancing Agricultural Environment Perception
</h1>

<p align="center">
  Implementation of the research project developed during the Master's thesis of Michele Carlo La Greca at Politecnico di Milano, corresponding to the work related to the <a href="https://arxiv.org/abs/2409.12602">preprint</a> available on <i>arXiv</i>.
  <p align="center">
    <img src="images/main.png" alt="Main Image" width="80%">
  </p>
</p>

<hr>


[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

<!-- ![GitHubWorkflowStatus](https://img.shields.io/github/actions/workflow/status/AIRLab-POLIMI/active-vision/main.yml?logo=github&style=flat-square)
[![GitHubcontributors](https://img.shields.io/github/contributors/AIRLab-POLIMI/active-vision?style=flat-square)](CONTRIBUTING.md)
[![License](https://img.shields.io/github/license/AIRLab-POLIMI/active-vision?style=flat-square)](LICENSE) -->

<hr>

## Table of contents
  * [Introduction](#introduction)
  * [Installation](#installation)
     * [Dependencies](#dependencies)
     * [Igus ReBeL](#igus-rebel) 
     * [Gazebo Ignition](#gazebo-ignition)
  * [Visualization](#visualization)
  * [OctoMap Creation](#octomap-creation)
  * [Active Vision](#active-vision) 
  * [Contributing](CONTRIBUTING.md)
  * [License](LICENSE)

<hr>

# Introduction

Agriculture is essential to society. Robotics can boost productivity in agriculture, and robots must be able to accurately perceive the unstructured, dynamic, and possibly covered environment of plants and crops. This complexity presents significant challenges for traditional management methods. The proposed research introduces an approach to overcome the challenges and complexities of fruit perception through Active Vision (AV). Instead of relying on passive observation, AV allows robots to actively perceive, explore, and reconstruct at run-time their surroundings by planning the optimal position of the camera viewpoint using the Next-Best View (NBV) planning, which maximizes the information gained regarding plants and crops. This ensures that even hidden or occluded parts of the environment are effectively captured.

This work applies Zero-Shot Learning (ZSL) to provide useful segmentation, enabling the robot to generalize and adapt to various crops or environmental features without requiring specific training data for each scenario. By leveraging both 3D and semantic data, the robot can reconstruct a detailed, semantic, and context-aware map of the environment, allowing it to strategically adjust its movements and positioning, leading to more effective interactions with the environment.

This work focuses on the following contributions:  
1. Developed a modular architecture in ROS 2, C++, and Python for Active Vision in agricultural robotics, addressing the challenge of detecting occluded fruits.  
2. To the best of the author’s knowledge, this is the first work to integrate Zero-Shot Learning with Active Vision exploration, enabling environment-independent operation in agriculture.  
3. Conducted extensive evaluations both in simulation and real-world scenarios, in contrast to state-of-the-art methods that primarily focus on simulated environments with supervised learning.  
4. Set a benchmark standard for the lack of reproducibility and availability of open-source code in the context of Active Vision in agricultural robotics.

    

# Installation
  
<details>
  <summary>
    Step 1: Install the ROS 2 Humble distribution. 
  </summary>
  
- #### Ubuntu 22.04:
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
</details>
  
<details>
  <summary>
    Step 3: Clone the main branch of this repository.
  </summary>
  
#### TODO
  
  
#### TODO
  
  
</details>


# Visualization


# OctoMap Creation










[rolling-badge]: https://img.shields.io/badge/-ROLLING-orange?style=flat-square&logo=ros
[rolling]: https://docs.ros.org/en/rolling/index.html
[foxy-badge]: https://img.shields.io/badge/-foxy-orange?style=flat-square&logo=ros
[foxy]: https://docs.ros.org/en/foxy/index.html
[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[iron-badge]: https://img.shields.io/badge/-IRON-orange?style=flat-square&logo=ros
[iron]: https://docs.ros.org/en/iron/index.html
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/
[ubuntu20-badge]: https://img.shields.io/badge/-UBUNTU%2020%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu20]: https://releases.ubuntu.com/focal/
