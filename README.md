# ThermalOdo

关于红外odometery的研究

从ORBSLAM中抽取以下部分组成的单独模块：

1. Frame.cc中的入口构造，以及特征点分配部分
2. ORBextractor.cc 中的大部分，提取特征点位置以及其描述子
3. ORBmatcher.cc中的特征点匹配工作
4. Initializer.cc中单目初始化中的计算初始化位姿Rt部分

整体的接口入口在RtGenerator中。

## 架构

1. Utils中包含一些常用的通用的库,目前正在完善中
2. localUtils包含当前工程中用到的一些简单的函数
3. orb_extract本质上应当在Utils中
