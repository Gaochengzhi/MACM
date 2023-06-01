import matplotlib.pyplot as plt
import numpy as np

for i in range(10):
    plt.clf()   # 清除当前图形
    plt.plot(np.random.rand(10))  # 用一些随机数据进行绘图
    plt.draw()   # 更新图形
    # plt.pause(0.1)  # 暂停以便看到更新的图形

# 使用 plt.show() 来阻塞执行，直到关闭了图形窗口
plt.show()

