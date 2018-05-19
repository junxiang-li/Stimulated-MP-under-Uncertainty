
![](https://img.shields.io/badge/version-v4.0-green.svg)

# Introduction
-------

本程序是在Prescan平台上开发的自主驾驶运动预测与规划方法Demo

Prescan版本是8.0(只能在8.0以上版本才能打开pex文件)，matlab为2016b

# Description
-------
* Prescan建模文件

    -pex文件
    
    -Trajectoies文件夹：用以保存所有在prescan中**预先**设定的轨迹
    
* Simulink建模文件

    -_cs.mdl文件（其他为自动生成的自动保存文件）
    
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
If you found any bug or have any suggestion, please do file issues. I am graceful for any feedback and will do my best to improve this package.

# License
-------
Currently Released Under GPLv3


# Contact
-------
enginelee@yeah.net
