import numpy as np

# 创建两个示例数组
a = np.array([1, 2, 3])
b = np.array([4, 5, 6])

# 创建一个新的空数组，用于存储相乘结果
result = np.empty_like(a)

# 对应点位相乘并将结果存储到 result 数组中
result=np.multiply(a, b)

# 输出结果
print("Result array:", result)
