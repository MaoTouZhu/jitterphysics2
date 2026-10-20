# 后续说明

### `PointCloudShape` 与 `ConvexHullShape`

在本例中，我们使用 `PointCloudShape` 来模拟一个具有凸包的刚体。顾名思义，这种形状只需要一组点——这些点不必精确地落在凸面上。按照设计，Jitter 会把这些点的凸包当作碰撞检测时实际使用的形状。

尽管这里使用的算法是暴力式的，但它可以非常快：数据以线性内存布局存储，并使用 SIMD 指令加速支撑函数。因此，`PointCloudShape` 通常是快速、高效地为模拟添加简单凸几何体的最佳选择。

另一方面，`ConvexHullShape` 面向更复杂、更精细的凸模型。与 `PointCloudShape` 不同，该形状需要提供预先计算好的凸包，形式为三角形列表（`List<JTriangle>`）。Jitter 不会为你生成它——你需要使用 Blender 之类的第三方工具或专门的凸包库。输入网格必须是严格凸的，碰撞检测才能正确工作。

在内部，`ConvexHullShape` 使用爬山算法来计算支撑函数。虽然这种方法在算法上比暴力法更高效，但性能优势只有在形状较大时才会显现。一般经验法则是：当顶点数达到约 300 个或更多时，`ConvexHullShape` 开始比 `PointCloudShape` 表现更好。
