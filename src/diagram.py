import matplotlib.pyplot as plt

def plot_viewpoint_scores(viewpoint_details):
    """
    Plots the scores of viewpoints against their indices.

    Parameters:
    - viewpoint_details: List of dictionaries containing viewpoint index, position, direction, view, and score.
    """
    # 提取索引和得分
    indices = [vp['index'] + 1 for vp in viewpoint_details]  # 索引从1开始
    scores = [vp['score'] for vp in viewpoint_details]

    # 创建折线图
    plt.figure(figsize=(10, 6))  # 设置图表大小
    plt.plot(indices, scores, marker='o', linestyle='-', color='b')  # 折线图，蓝色线条和圆圈标记
    plt.title('Viewpoint Scores')  # 图表标题
    plt.xlabel('Viewpoint Index')  # x轴标签
    plt.ylabel('Score')  # y轴标签
    plt.grid(True)  # 显示网格
    plt.xticks(indices)  # 设置x轴刻度标签
    plt.show()  # 显示图表



