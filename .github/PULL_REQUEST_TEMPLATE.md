<!--
  请将 PR 标题改为：姓名 - 学号
  例如：张三 - 2026000000
-->

## 代码功能展示

<!-- 可以上传一个清华云盘的公共链接，也可以直接拖拽视频文件到 description 区域 -->
<!-- 最终录屏需要为 `timed` 模式的测试过程 -->

- Level 1 录屏（`python navigation/main.py --level 1 --record level1.mp4`）：
https://cloud.tsinghua.edu.cn/d/c49c6929039a4c1297c3/
- Level 2 录屏（`python navigation/main.py --level 2 --record level2.mp4`）：
https://cloud.tsinghua.edu.cn/d/ec1d004994554d93adb0/
## 关键参数说明

<!-- 简单说明你在 `nav/` 里选定的几个关键算法和参数 -->

### 代价地图
使用了ndimage库计算距离。inflation_radius设成3，在inflation_radius内按照离墙的距离把cost从255线性衰减为0
### 全局规划
dijistra算法
### 局部控制
使用了提示的算法：（简单的追踪：从最近的路径点向前扫描，直到累计距离超过前瞻半径（一个可调常数，例如 1.5-2.5 个网格单位）。速度方向为 `look_ahead - current_pose`，速度大小为 `max_speed`（如果剩余路径长度较短，则使用较小的速度值，以便更轻松地到达目标））
## 遇到的问题与反馈

<!-- 可选，遇到的问题、对作业 / 文档的建议 -->
