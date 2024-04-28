# import numpy as np

# # 创建两个示例数组
# a = np.array([1, 2, 3])
# b = np.array([4, 5, 6])

# # 创建一个新的空数组，用于存储相乘结果
# result = np.empty_like(a)

# # 对应点位相乘并将结果存储到 result 数组中
# result=np.multiply(a, b)

# # 输出结果
# print("Result array:", result)
# import asyncio
# import os

# import carb.settings
# import omni.kit.app
# import omni.replicator.core as rep
# import omni.timeline
# import omni.usd

# NUM_APP_FRAMES = 50
# TIMELINE_FPS = 60.0
# FPS_LOW = 7.0
# FPS_HI = 12.0
# DT_LOW = 1.0 / FPS_LOW
# DT_HI = 1.0 / FPS_HI
# # If True, render products are only enabled when needed (i.e. when data is accessed)
# TOGGLE_RP_UPDATES = True


# async def run_fps_example_async():
#     omni.usd.get_context().new_stage()
#     carb.settings.get_settings().set("/omni/replicator/captureOnPlay", False)
#     rep.create.cube(semantics=[("class", "cube")])# 创建了一个物块儿

#     rp = rep.create.render_product("/OmniverseKit_Persp", (512, 512), name="rp") # 创建渲染器
#     if TOGGLE_RP_UPDATES:
#         rp.hydra_texture.set_updates_enabled(False)

#     # Create writers
#     out_dir_rgb = os.getcwd() + "/_out_writer_fps_rgb"
#     writer_rgb = rep.WriterRegistry.get("BasicWriter")
#     writer_rgb.initialize(output_dir=out_dir_rgb, rgb=True)
#     writer_rgb.attach(rp, trigger=None)  # NOTE: trigger=None is needed for writer schedule triggering

#     out_dir_depth = os.getcwd() + "/_out_writer_fps_depth"
#     writer_depth = rep.WriterRegistry.get("BasicWriter")
#     writer_depth.initialize(output_dir=out_dir_depth, distance_to_camera=True)
#     writer_depth.attach(rp, trigger=None)  # NOTE: trigger=None is needed for writer schedule triggering
#     print(f"Writer data will be written to: {out_dir_rgb} and {out_dir_depth}")

#     # Set the timeline parameters to fit the deisred scenario
#     timeline = omni.timeline.get_timeline_interface()
#     timeline.set_play_every_frame(True)# Turns frame skipping off (true) or on (false)
#     timeline.set_auto_update(True)
#     timeline.set_looping(False)
#     timeline.set_current_time(0.0)
#     timeline.set_end_time(1000)
#     timeline.set_target_framerate(TIMELINE_FPS)# 设置目标帧频，它将影响游戏模式下运行循环的推导 FPS。精确的运行速度通常与此值不同，因为它总是 get_time_codes_per_seconds 的倍数
#     timeline.play()

#     # Test the timeline framerate by advancing it by the frame rate number of frames (i.e. 1 second of time)
#     print(f"Initial timeline time: {timeline.get_current_time():.4f}")
#     previous_time = timeline.get_current_time()
#     for i in range(int(TIMELINE_FPS)):
#         await omni.kit.app.get_app().next_update_async()
#         print(f"[{i}][{timeline.get_current_time():.4f}] frame dt={(timeline.get_current_time() - previous_time):.4f}")
#         previous_time = timeline.get_current_time()
#     print(f"After {int(TIMELINE_FPS)} frames, the timeline time is: {timeline.get_current_time():.4f}")

#     # Access annotator data at different framerates
#     previous_time = timeline.get_current_time()
#     elapsed_time_fps_low = 0.0
#     elapsed_time_fps_hi = 0.0
#     for i in range(NUM_APP_FRAMES):
#         current_time = timeline.get_current_time()
#         delta_time = current_time - previous_time
#         elapsed_time_fps_low += delta_time
#         elapsed_time_fps_hi += delta_time
#         print(
#             f"[{i}][{current_time:.4f}] elapsed_time_fps_low={elapsed_time_fps_low:.4f}/{DT_LOW:.4f},\t elapsed_time_fps_hi={elapsed_time_fps_hi:.4f}/{DT_HI:.4f},\t dt={delta_time:.4f};"
#         )

#         should_trigger_fps_low = elapsed_time_fps_low >= DT_LOW
#         should_trigger_fps_hi = elapsed_time_fps_hi >= DT_HI
#         if should_trigger_fps_low or should_trigger_fps_hi:
#             # Enable render products for data access
#             if TOGGLE_RP_UPDATES:
#                 rp.hydra_texture.set_updates_enabled(True)

#             # Access data directly from annotators
#             if should_trigger_fps_low:
#                 # Difference to the optimal trigger time (if the timeline framerate is not divisible by the sensor framerate)
#                 diff = elapsed_time_fps_low - DT_LOW
#                 print(f"\t[fps low] data shape: writer_rgb.schedule_write(), diff={diff:.4f}")
#                 writer_rgb.schedule_write()
#                 # Carry over the difference to the next trigger time
#                 elapsed_time_fps_low = diff
#                 # OR: elapsed_time_fps_low = 0.0 for a more simple reset

#             if should_trigger_fps_hi:
#                 # Difference to the optimal trigger time (if the timeline framerate is not divisible by the sensor framerate)
#                 diff = elapsed_time_fps_hi - DT_HI
#                 print(f"\t[fps hi] data shape: writer_depth.schedule_write(); diff={diff:.4f}")
#                 writer_depth.schedule_write()
#                 # Carry over the difference to the next trigger time
#                 elapsed_time_fps_hi = diff
#                 # OR: elapsed_time_fps_hi = 0.0 for a more simple reset

#             # Step needs to be called after scheduling the write
#             await rep.orchestrator.step_async()

#             # Disable render products for performance reasons until the next trigger time
#             if TOGGLE_RP_UPDATES:
#                 rp.hydra_texture.set_updates_enabled(False)

#             # Restart the timeline if it has been paused by the replicator step function
#             if not timeline.is_playing():
#                 timeline.play()

#         previous_time = current_time
#         # Advance the app (timeline) by one frame
#         await omni.kit.app.get_app().next_update_async()


# asyncio.ensure_future(run_fps_example_async())


# # NOTE:
# # - To avoid FPS delta misses make sure the sensor framerate is divisible by the timeline framerate
# import numpy as np
# file_path="D:/Documents/SoftwareDoc/isaac_sim/repData/pointcloud_0000.npy"
# # 加载 .npy 文件
# loaded_data_array:np.ndarray = np.load(file_path)
# print(loaded_data_array)
# # 将 NumPy 数组转换回列表
# loaded_data_list = loaded_data_array.tolist()
# print(loaded_data_list)
# print("Loaded data from .npy file:")
# print(loaded_data_list)

import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D  # 导入3D绘图工具
file_path='D:/Documents/SoftwareDoc/isaac_sim/repData'
for i in range(17,50):
    for j in range(i,i+1):
        cloud_path=f"{file_path}/pointcloud_{i*30:04d}.npy"
        label_path=f"{file_path}/label_{j*30:04d}.npy"
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

        # 显示图形
        plt.show()
