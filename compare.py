
# import numpy as np
# import matplotlib.pyplot as plt
# # from mpl_toolkits.mplot3d import Axes3D  # 导入3D绘图工具
# file_path='D:/Documents/SoftwareDoc/isaac_sim/repData'
# for i in range(1,50):
#     for j in range(i,i+1):
#         cloud_path=f"{file_path}/pointcloud_{i*30:04d}.npy"
#         label_path=f"{file_path}/label_{j*30:04d}.npy"
#         # 加载第一个点云数据
#         point_cloud1 = np.load(cloud_path)  # 请替换为第一个点云文件的路径
#         # 加载第二个点云数据
#         point_cloud2 = np.load(label_path)  # 请替换为第二个点云文件的路径

#         # 检查点云数据的形状和大小
#         print("Point cloud 1 shape:", point_cloud1.shape)
#         print("Point cloud 2 shape:", point_cloud2.shape)

#         # 创建一个新的图形窗口
#         fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')

#         # 绘制第一个点云
#         ax.scatter(point_cloud1[:, 0], point_cloud1[:, 1], point_cloud1[:, 2], c='b', label=f'Point Cloud{i*30}')

#         # 绘制第二个点云
#         ax.scatter(point_cloud2[:, 0], point_cloud2[:, 1], point_cloud2[:, 2], c='r', label=f'label Cloud{j*30}')

#         # 设置图形标题和标签
#         ax.set_title('Comparison of Point Clouds')
#         ax.set_xlabel('X')
#         ax.set_ylabel('Y')
#         ax.set_zlabel('Z')
#         ax.legend()

#         # 显示图形
#         plt.show()
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 导入3D绘图工具

file_path = 'D:/Documents/SoftwareDoc/isaac_sim/repData'
for i in range(1, 100):
    for j in range(i, i + 1):
        cloud_path = f"{file_path}/pointcloud_{i*30:04d}.npy"
        label_path = f"{file_path}/label_{j*30:04d}.npy"
        
        # 加载第一个点云数据
        point_cloud1 = np.load(cloud_path)  # 请替换为第一个点云文件的路径
        # 加载第二个点云数据
        point_cloud2 = np.load(label_path)  # 请替换为第二个点云文件的路径

        # 检查点云数据的形状和大小
        print("Point cloud 1 shape:", point_cloud1.shape)
        print("Point cloud 2 shape:", point_cloud2.shape)

        # 创建一个新的图形窗口
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # 绘制第一个点云
        ax.scatter(point_cloud1[:, 0], point_cloud1[:, 1], point_cloud1[:, 2], c='b', label=f'Point Cloud{i*30}')

        # 绘制第二个点云
        ax.scatter(point_cloud2[:, 0], point_cloud2[:, 1], point_cloud2[:, 2], c='r', label=f'label Cloud{j*30}')

        # 设置图形标题和标签
        ax.set_title('Comparison of Point Clouds')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.legend()

        # 设置相同的比例
        def set_axes_equal(ax):
            '''确保x, y, z轴具有相同的比例.'''
            extents = np.array([ax.get_xlim(), ax.get_ylim(), ax.get_zlim()])
            centers = np.mean(extents, axis=1)
            max_size = max(abs(extents[:, 1] - extents[:, 0]))
            r = max_size / 2
            ax.set_xlim([centers[0] - r, centers[0] + r])
            ax.set_ylim([centers[1] - r, centers[1] + r])
            ax.set_zlim([centers[2] - r, centers[2] + r])

        set_axes_equal(ax)

        # 显示图形
        plt.show()
