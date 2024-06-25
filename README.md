# Research on Multi-agent Collaborative Exploration Technology in Complex Marine Environments
复杂海域下多智能体协同探查技术研究，第十八届“挑战杯”全国大学生课外学术科技作品竞赛“揭榜挂帅”专项赛，全国一等奖

The 18th "Challenge Cup" National College Students Extracurricular Academic Science and Technology Works Competition "Jiebing Guashuai" Special Competition, National First Prize

[中文版](./README_zh.md)
# Intro

The marine environment is complex and constantly changing, which makes traditional platform detection less accurate. To improve accuracy and reduce safety risks, we can use close-range observation and intelligent agents to explore designated complex sea areas.

In this task, participants must use multi-agent dynamic programming to cover an irregular enclosed sea area. Each intelligent agent must explore the entire area and all static targets while maintaining safe navigation.

To explore static targets, the number of explorations and exploration angle of the intelligent agents are dynamically adjusted in real-time. The software algorithm model must perform dynamic allocation and planning to ensure full coverage of the entire area.

Through effective exploration, the detection payload can obtain high-precision position, speed, image, or video information of static targets while disrupting full coverage route planning.

![IMG_6758](./assets/IMG_6758.JPEG)

# Install and Run

### 1. clone this repo

```shell
git clone --depth 1 https://github.com/Gaochengzhi/MACM.git
```

### 2. if you want to run matlab code

First enter

Matlab version code utilize [GCAA optimizer](https://github.com/MartinBraquet/task-allocation-auctions), loacled in ./orignial_matlab_code.

to run , simply execute the main function:

```matlab
coverage_competition_main_json.m % main function point
```

### 3. if you want to run Python code

Python code utilize the `linear_sum_assignment`  in `scipy.optimize`, using the Hungarian algorithm to assign targets to agents

to install packages and run it, simple execute the follwoing command in Unix compatible environment.

```shell
pip3 install -r requirements.txt
python3 test_main.py 
```

