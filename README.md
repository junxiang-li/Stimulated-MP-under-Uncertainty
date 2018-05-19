
![](https://img.shields.io/badge/version-v4.0-green.svg)

# Introduction
-------

This demo is built for the motion prediction and motion planning of Intelligent Vehicles (IVs) in the platform of Prescan ([The introduction of Prescan](https://tass.plm.automation.siemens.com/prescan)).

The version of Prescan we used is 8.0, and matlab is 2016b. Note that the pex file can only be opened at Prescan V8.0+.

# Description
-------
* Prescan建模文件

    -pex文件
    
    - Trajectoies文件夹：用以保存所有在prescan中**预先**设定的轨迹
    
* Simulink建模文件

    - _cs.mdl文件（其他为自动生成的自动保存文件）
    
* Matlab程序

    -SubFunction文件夹
    
    -GlobalReferencePathInitiation文件夹：用以处理prescan的参考路坐标从0开始，不是实际的坐标大小
* 实验结果

    -Results
    
        ResultAnalysis：存放数据处理的Matlab程序

# CHANGELOG
-------
-4.1

优化：调整组织结构

-4.0

除bug：在原程序的基础上，对simulink文件中速度规划模块进行了修改，利用Pathfollower对路径进行跟踪；

~~[CHANGELOG]~~

# FeedBack
-------
If you find any bugs or have any suggestions, please contact me or modify directly. I am thankful for your feedback and will definitely improve the program. At the same time, I would like to discuss/learn state-of-the-art methods to make planning more smooth and human-like.

# License
-------
Currently Released Under GPLv3


# Contact
-------
enginelee@yeah.net
