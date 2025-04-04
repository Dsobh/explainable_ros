# explainable_ros [WIP]

## Table of Contents

1. [How To](#how-to)
    - [Installation](#installation)
    - [Usage](#usage)
2. [Related Projects](#related-projects)
    - [Other Software Proyects](#other-software-proyects)
    - [Related Datasets](#related-datasets)
    - [Papers](#papers)
3. [Cite](#cite)
4. [Acknowledgments](#acknowledgments)

## How To

### Installation

### Usage
```shell
$ ros2 launch explicability_bringup explicability_ros.launch.py
```

```shell
$ ros2 service call /question explicability_msgs/srv/Question "{'question': 'What is happening?'}"
```

## Related Works

### Other Software Proyects

- [llama_ros](https://github.com/mgonzs13/llama_ros) → A repository that provides a set of ROS 2 packages to integrate llama.cpp into ROS 2.

### Related Datasets

A series of rosbags (ROS 2 Humble) published in Zenodo are listed below. This data can be used to test the explainability capabilities of the project.

- Sobrín Hidalgo, D. (2024). Navigation Test in Simulated Environment Rosbag. Human obstacle detection. (1.0.0) [Data set]. Robotics Group. https://doi.org/10.5281/zenodo.10896141
- Sobrín Hidalgo, D. (2024). Navigation Benchmark Rosbags Inspired by ERL Competition Test (1.0.0) [Data set]. Robotics Group. https://doi.org/10.5281/zenodo.10518775

### Papers 

- Sobrín-Hidalgo, D., González-Santamarta, M. A., Guerrero-Higueras, Á. M., Rodríguez-Lera, F. J., & Matellán-Olivera, V. (2024). [Explaining Autonomy: Enhancing Human-Robot Interaction through Explanation Generation with Large Language Models](https://arxiv.org/abs/2402.04206). arXiv preprint arXiv:2402.04206.
- Sobrín-Hidalgo, D., González-Santamarta, M. Á., Guerrero-Higueras, Á. M., Rodríguez-Lera, F. J., & Matellán-Olivera, V. (2024). [Enhancing Robot Explanation Capabilities through Vision-Language Models: a Preliminary Study by Interpreting Visual Inputs for Improved Human-Robot Interaction](https://arxiv.org/abs/2404.09705). arXiv preprint arXiv:2404.09705.

## Cite

If your work uses this repository, please, cite the repository or the following paper:

```
@article{sobrin2024explaining,
  title={Explaining Autonomy: Enhancing Human-Robot Interaction through Explanation Generation with Large Language Models},
  author={Sobr{\'\i}n-Hidalgo, David and Gonz{\'a}lez-Santamarta, Miguel A and Guerrero-Higueras, {\'A}ngel M and Rodr{\'\i}guez-Lera, Francisco J and Matell{\'a}n-Olivera, Vicente},
  journal={arXiv preprint arXiv:2402.04206},
  year={2024}
}
```

## Acknowledgments

This project has been partially funded by the Recovery, Transformation, and Resilience Plan, financed by the European Union (Next Generation) thanks to the TESCAC project (Traceability and Explainability in Autonomous Cystems for improved Cybersecurity) granted by INCIBE to the University of León, and by grant PID2021-126592OB-C21 funded by
MCIN/AEI/10.13039/501100011033 EDMAR (Explainable Decision Making in Autonomous Robots) project, PID2021-126592OB-C21 funded by MCIN/AEI/10.13039/501100011033 and by ERDF ”A way of making Europe”.

<img src="https://github.com/Dsobh/explainable_ROS/blob/main/images/logos/logo_demarce.png" width="150"/>           <img src="https://github.com/Dsobh/explainable_ROS/blob/main/images/logos/logo_edmarce.png" width="140"/>

<img src="https://github.com/Dsobh/explainable_ROS/blob/main/images/logos/BandaLogos_INCIBE_page-0001.jpg" width="2000"/>


