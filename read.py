#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time         : 2024/4/25 17:19
# @Author       : Wang Song
# @File         : read.py
# @Software     : PyCharm
# @Description  :
import numpy as np
import open3d as o3d
import itertools

# 使用numpy的load函数读取npy文件
data1 = np.load('pointcloud_semantic_0042.npy')
data2 = np.load('pointcloud_0042.npy')

# 确保data1和data2的长度相同
assert len(data1) == len(data2)

# 使用zip函数将data1和data2的数据配对起来
paired_data = list(zip(data1, data2))

# 使用sorted函数根据语义ID对配对的数据进行排序
sorted_data = sorted(paired_data, key=lambda x: x[0])

# 使用itertools.groupby函数根据语义ID将点云数据进行分组
for semantic_id, group in itertools.groupby(sorted_data, key=lambda x: x[0]):
    # 提取点云数据
    points = np.array([item[1] for item in group])

    # 创建一个PointCloud对象
    pcd = o3d.geometry.PointCloud()

    # 从numpy数组中设置点云数据
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])  # 取前三列作为点的坐标

    # 可视化点云
    print(f"Visualizing points with semantic ID: {semantic_id}")
    o3d.visualization.draw_geometries([pcd])