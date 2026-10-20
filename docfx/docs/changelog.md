# 更新日志

### Jitter 2.12.0 (2026-10-19)

- 新增 `AngularServo`、`TwistServo`、`LinearServo` 和 `LinearAxisServo`，用于在力矩或力限值下驱动目标朝向、角度和位置。
- 新增 `JointDrive`，用于在构造铰链、棱柱和万向节时选择马达或伺服。现有的 `hasMotor` 构造函数仍然可用，但已过时。
- 新增 `DebugDrawSettings.PointRadius`，用于独立于约束辅助线控制点标记的大小。
- 通过在少于四个点的辅助流形中保留原始接触，改进了滚动稳定性。

### Jitter 2.11.0 (2026-10-18)

- **破坏性变更：** `IDebugDrawer.DrawSegment` 与 `DrawTriangle` 现在要求颜色为按 `0xRRGGBBAA` 打包的 `uint`。自定义绘制器必须实现这两个带颜色的方法。
- 改进了接触与约束求解器性能，尤其是在多线程场景中，以及对于附加到 `World.NullBody` 的约束。
- 通过修复可能人为加速滚动对象的接触冲量，改进了带子步的滚动稳定性。
- 修复了角速度错误影响推测接触的问题。碰撞预测与推测响应现在一致地使用线性运动。
- 新增 `EllipticalConeLimit`，可在两个互相垂直的方向上设置不同的摆动限位，并支持可选的内部限位和不受限制的扭转。
- 扩展了形状、刚体、约束和关节的调试绘制，锚点、限位、轴和马达方向显示得更清晰。

### Jitter 2.10.1 (2026-10-15)

- 新增 `Constraint.Tag`，用于向约束附加自定义用户数据，与 `RigidBody.Tag` 一致。

### Jitter 2.10.0 (2026-10-11)

- **破坏性变更：** 新增 `TimeStep`，包含整步与子步计时信息。世界步与子步回调以及 `SoftBody.WorldOnPostStep` 现在接收 `TimeStep`；自定义约束求解器回调与接触准备现在接收 `in TimeStep`，而不再是时间步的倒数。
- **破坏性变更：** 将 `BoxShape` 构造函数的三个参数重命名为 `sizeX`、`sizeY` 和 `sizeZ`。请更新使用命名参数的调用。
- **破坏性变更：** `ConvexPolytope` 现在包含可复用的托管存储，其 `InitTetrahedron` 方法返回成功标志。新增 `AddVertexDetailed` 用于报告扩展结果。
- 修复了马达力和力矩限值以及约束柔软度（包括软体弹簧）在子步下的缩放问题。这会改变 `SubstepCount > 1` 时的模拟行为。
- 新增 `AngularMotor.MaximumTorque`，并将其 `MaximumForce` 别名标记为已弃用。
- 通过过滤后的精确谓词和双精度接触计算改进了 EPA 的稳健性，并改进了小分离情况下 GJK 最近点的精度。
- 修复了 `TriangleShape` 射线投射报告射线原点之后交点的问题。
- **破坏性变更：** 刚体运动类型变更不再移除约束或引发 `World.ConstraintRemoved`。约束会保留其启用状态，当两个相连刚体都不是动态刚体时被挂起，并在某个动态刚体唤醒时恢复。如需永久移除，请使用 `World.Remove(constraint)`。
- 修复了运动的运动学刚体错误进入休眠的问题，包括速度极低以及停用时间为零的情况。
- 新增 `DynamicTree.Update(proxy, dt)`，并修复了速度、冲量和推测接触设置变更后推测宽相包围盒的问题，包括首次模拟步之前的情况。
- 当动态树的过滤器变更时立即重建碰撞对，并且如果新过滤器抛出异常，则保留先前的过滤器和碰撞对。
- 修复了 `BeginCollide` 处理器移除刚体、形状或仲裁器时的接触与岛状态，包括取消挂起的接触。
- 使 `RigidBody.AddShape` 在注册或质量计算失败时回滚，并使树代理注册在过滤抛出异常时回滚。
- 修复了 `World.Clear` 将形状残留在 `World.NullBody` 上的问题。
- 使 `SoftBody.Destroy` 在重复调用、清空世界或释放之后仍然安全。为 `SoftBodyTriangle.Thickness` 新增了校验和包围盒更新。
- 修复了大型凸包中的邻接索引溢出，以及分离的凸包和点云形状的包围盒初始化问题。
- 修复了嵌套 `TransformedShape` 实例的惯性计算。
- 改进了对已移除刚体和约束、传给 `World.CreateConstraint` 的无效刚体以及未注册动态树代理的校验。
- 为无效的刚体和约束数据访问以及无效的仲裁器创建参数新增了调试断言。
- `PartitionedSet` 的枚举现在会在集合被修改时抛出异常，而不是静默跳过元素。

### Jitter 2.9.0 (2026-10-03)

- 通过 `RigidBody.AllowedMotion` 和 `MotionAxes` 新增世界空间下的平移与旋转锁定，包括平面运动预设。
- **破坏性变更：** 惯性 API 现在使用 `JSymmetricMatrix`；`RigidBodyData.InverseMass` 被 `InverseMassVector` 取代。请相应地更新自定义形状和约束。
- 修复了约束质量求解、角度奇异点以及固定限位和边界限位的问题。
- 修复了推测接触偏置和 CCD 冲量保持问题。
- 修复了旋转四元数计算中可能出现的溢出。
- 新增 `TriangleEdgeCollisionFilter.FilterDynamicBodies`，用于控制动态三角形刚体的边过滤（默认禁用）。

### Jitter 2.8.13 (2026-09-19)

- 改进了用户代码抛出异常时从无效内部状态恢复的能力。

### Jitter 2.8.12 (2026-09-15)

- 新增 `World.Trim`，用于在场景大幅变化后释放多余的内部存储。
- 修复了多线程接触创建中的竞态，该竞态可能创建重复的碰撞对。
- 修复了跨工作线程的仲裁器池化导致的托管内存增长。
- 改进了对已释放世界、移除调用、动态树优化、工作线程池和非托管内存辅助方法的校验与异常行为。
- 改进了有关接触注册线程规则、仲裁器生命周期和公共异常行为的 XML 文档。

### Jitter 2.8.11 (2026-09-09)

- 新增 `World.ConstraintRemoved`，用于响应显式和自动的约束移除。
- 通过自动增量优化和不增加成本的旋转，改进了 `DynamicTree` 在常规更新期间的质量。

### Jitter 2.8.10 (2026-08-31)

- 新增 `World.IsDisposed` 以及刚体和约束的 `IsValid` 属性，从而可以安全地检测不可用的原生状态，包括被自动移除的约束。
- 确保仲裁器在 `EndCollide` 回调运行期间保持有效。
- 通过消除缓冲区分区之间不必要的数据移动，改进了接触分配和移除的性能。

### Jitter 2.8.9 (2026-07-24)

- 通过减少读写锁中的争用和缓存行乒乓，改进了多线程性能。

### Jitter 2.8.8 (2026-05-24)

- 通过在 EPA 之后重新评估接触，修复了三角形边接触过滤。
- 在构造期间拒绝退化的 `PointCloudShape` 体积和零质量的 `ConvexHullShape` 输入。
- 改进了校验失败时的异常报告。同时为公共异常行为新增了 XML 文档。

### Jitter 2.8.7 (2026-05-17)

- 新增 `World.ForceSleepIsland`，用于立即停用一个岛。
- 为无效的形状尺寸、质量、约束轴和限位新增了参数校验。
- 新增了针对运行时状态变更的仅调试检查。
- 改进了模拟失败或被中断时的清理。
- 修复了空 `DynamicTree` 的成本计算。

### Jitter 2.8.6 (2026-05-15)

- 新增对从不同宿主线程使用多个 `World` 实例的支持。
- 新增 `World.IslandActivated` 和 `World.IslandDeactivated` 事件，用于观察模拟岛的唤醒/休眠转换。
- 修复了 `SolveMode.Deterministic` 中确定性求解器迭代的处理。
- 修复了嵌套动态树查询，使回调可以安全地执行额外的 `RayCast`、`SweepCast` 和 `FindNearest` 查询。
- 修复了更改刚体 `MotionType` 时残留的排队力和力矩。
- **破坏性变更：** 先前已过时的兼容 API 现在会产生编译期错误。请使用过时消息中给出的替代 API。

### Jitter 2.8.5 (2026-04-25)

- 改进了 `CollisionManifold`。

### Jitter 2.8.4 (2026-04-19)

- 通过执行完整裁剪改进了 `CollisionManifold` 的生成，产生更好的接触集。
- 修复了具有重复接缝顶点的网格的索引 `TriangleMesh` 邻接生成。
- 在 `ShapeHelper` 中新增了泛型方法。
- 改进了整个代码库中的 XML 文档。

### Jitter 2.8.3 (2026-04-13)

- 新增 `SolveMode.Deterministic`——一种可选的跨平台确定性求解器模式。
- 修复了休眠逻辑中的缺陷。
- 新增 `PersistentContactManifold` 属性。

### Jitter 2.8.2 (2026-03-30)

- 新增 `DynamicTree.SweepCast` 和 `DynamicTree.FindNearest`——结合宽相剪枝与精确窄相测试的新树查询。`SweepCast` 使形状在树中扫掠；`FindNearest` 按距离查找最近的代理。两者都包含有界变体以及针对常见形状的便捷重载。
- 新增 `World.Stabilize`——在速度层面求解现有接触和约束，而不推进刚体变换。适用于在恢复正常模拟之前对已恢复的场景进行热启动。

### Jitter 2.8.1 (2026-03-21)

- 修复了 `RigidBody` 中的速度设置器缺陷——该缺陷引入于 2.8.0。
- 较小的 API 改进。

### Jitter 2.8.0 (2026-03-19)

- 新增 `RigidBody.AddImpulse` 方法，用于施加瞬时冲量（线性冲量以及在世界空间某位置施加的冲量）。
- 修复了唤醒逻辑中的边界情况：将速度设为零不再无必要地激活休眠中的刚体。
- 引入 `ISink<T>` 接口和 `CollectionSink<T>` 适配器，作为热路径上 `ICollection<T>` 的、对结构体友好的替代方案。
- 为 `ShapeHelper.Tessellate` 和 `DynamicTree.Query` 新增 `ref TSink` 重载。
- 将默认约束调优值提取为 `Constraint` 基类上的命名常量（`DefaultLinearBias`、`DefaultLinearSoftness`、`DefaultAngularBias` 等）。
- **破坏性变更：** `DynamicTree.Nodes` 从 `Node[]` 改为 `ReadOnlySpan<Node>`，只暴露内部数组已占用的部分。
- **破坏性变更：** `World.DebugTimings` 现在是 `ReadOnlySpan<double>`，而不是 `double[]`。
- 将 `NarrowPhase` 和 `CollisionManifold` 中的泛型类型参数从 `TA`/`TB` 重命名为 `Ta`/`Tb`。

### Jitter 2.7.9 (2026-02-14)

- 修复了 `SpringConstraint` 中的距离计算。
- 修复了 `UniversalJoint` 中的注册缺陷。
- 修复了 `ConeLimit.Limit` 属性并新增了上界校验。
- 修复了标量与 SIMD 接触求解器路径之间摩擦钳制不一致的问题。两条路径现在都使用更新前的法向冲量来计算摩擦锥。
- 修复了与静态刚体接触的休眠刚体被无必要唤醒的问题。
- 为约束新增了基本的调试绘制方法。
- 在 `World.Remove` 和 `World.CreateConstraint` 中新增了世界归属校验。传入来自不同世界的刚体、约束或仲裁器现在会抛出 `ArgumentException`。
- 改进了整个代码库中的 XML 文档。

### Jitter 2.7.8 (2026-01-18)

- 修复了 `DynamicTree` 中大型代理导致树质量下降的问题。
- 新增 `JQuaternion.Lerp`、`JQuaternion.Slerp`、`JQuaternion.Dot` 和 `JQuaternion.Inverse` 方法，用于四元数插值和数学运算。
- 新增 `JTriangle.GetCenter`、`JTriangle.GetNormal`、`JTriangle.GetArea`、`JTriangle.GetBoundingBox` 和 `JTriangle.ClosestPoint` 辅助方法。
- 新增 `JBoundingBox.Disjoint` 和 `JBoundingBox.Contains` 方法（`TreeBox` 中也有）。
- 新增 `World.GetArbiter(id0, id1)`，用于获取现有仲裁器而不创建新的仲裁器。
- 改进了整个代码库中的 XML 文档。
- 修复了 `TransformedShape` 中 `CalculateMassInertia` 返回错误值的问题。
- 修复了 `TriangleEdgeCollisionFilter` 中的归一化缺陷。
- 修复了 `SphereShape` 半径设置器中的校验。
- 修复了推测接触实现中的一个缺陷。
- 改进了 `TriangleMesh`。新增直接从顶点和索引构建的选项。
- 在整个引擎中新增了 `Span` 重载（位于 `TriangleMesh`、`ShapeHelper`、`PointCloudShape`、`ConvexHullShape`）。

### Jitter 2.7.7 (2026-01-08)

- 新增 `vector.UnsafeAs<T>` 和 `JVector.UnsafeFrom<T>`。`JQuaternion` 同理。
- 新增 `world.PreSubStep` 和 `world.PostSubStep` 事件。

### Jitter 2.7.6 (2025-12-18)

- 移除了 `World.Capacity`。`World` 中的实体数量不再需要预先指定。
- 修复了 `TwistAngle` 中的一个缺陷。
- 新增 `WeldJoint`。
- 在 `ConeLimit` 中暴露了更多属性。

### Jitter 2.7.5 (2025-11-16)

- 改进了 `DynamicTree` 的成本启发式。

### Jitter 2.7.4 (2025-10-29)

- 为 `RigidBody` 实例新增 `MotionType` 属性。刚体可以是 `Dynamic`、`Static` 或 `Kinematic`。
具有非零速度的静态刚体不再受支持，必须改用运动学刚体。
- 接触图信息被用于优化接触的内存布局（`ReorderContacts` 步）。这使求解器对缓存更友好。
- 降低了引擎中 `CheckDeactivation` 步的开销。

### Jitter 2.7.3 (2025-10-19)

- 改进了多线程性能。

### Jitter 2.7.2 (2025-09-21)

- 减少了 `DynamicTree.Optimize` 中的 GC。
- 如果未为 `RigidBodyShape` 设置刚体，则默认使用 LocalRayCast。
- 为 `AddForce` 重载新增唤醒参数，以便更精细地控制激活状态。
- 新增泛型 `ICloneableShape<T>` 接口，用于类型安全的形状克隆。

### Jitter 2.7.1 (2025-06-28)

- 新增 `RigidBody.EnableGyroscopicForces`，用于启用陀螺力。

### Jitter 2.7.0 (2025-06-14)

- **放弃 .NET7 支持**
- 新增 `JQuaternion.ToAxisAngle` 方法
- 将 `JBBox` 重命名为 `JBoundingBox`，将 `TreeBBox` 重命名为 `TreeBox`。
- 各种较小的 API 变更。

### Jitter 2.6.7 (2025-06-09)

- 为 `DynamicTree` 引入 SIMD 加速的 `TreeBBox`。

### Jitter 2.6.6 (2025-05-31)

- `JVector` 和 `JQuaternion` 支持从元组隐式转换，例如 `cube.Position = (1, 2, 3);`。
- 在 `World.Timings` 中新增 `PreStep` 和 `PostStep`。
- 新增 `NarrowPhase.Sweep` 重载，可计算旋转形状的撞击时间（TOI）。
- `RegisterContact` 不再需要 `penetration` 和 `speculative` 参数。
- 缺陷修复：`MathHelper.RotationQuaternion`，修复了较大 `dt` 时生成错误/未归一化四元数的问题。
- 为 `NarrowPhase.Distance` 新增了一个额外的 `normal` 输出参数。
- 将 `JVector.TransposedTransform(in JVector vector, in JQuaternion quat)` 重命名为 `ConjugatedTransform`。
- 为 `BallSocket` 新增 `Anchor1` 和 `Anchor2` 属性。
- 缺陷修复：`TriangleMesh` 中跳过退化三角形现在可正常工作。

### Jitter 2.6.5 (2025-05-21)

- 刚体现在会在速度或力变化时激活。
- 移除了 FatTriangleShape。
- 将 'Active' 重命名为 'ActiveCount'，并在 ReadOnlyPartitionedSet 中新增基于 span 的访问器。
- 修复了 TriangleEdgeCollisionFilter 中的缺陷。

### Jitter 2.6.4 (2025-05-19)

- **破坏性变更：** `TriangleMesh` 中的三角形绕序现在为逆时针（CCW），以表示正面三角形。
*如果你正在使用 `TriangleMesh`，请交换顶点顺序以保持正确的法向朝向。*
- 新增 `JTriangle.RayIntersect` 方法。
- 将 `ConvexHullIntersection` 重命名为 `CollisionManifold`。
- 修改了 `BoxShape` 的支撑函数。

### Jitter 2.6.3 (2025-05-17)

- 将刚体（`RigidBodyData`）按 64 字节边界对齐（减少伪共享）。
- 修复了推测接触中的缺陷。

### Jitter 2.6.2 (2025-05-06)

- 在 `NarrowPhase.cs` 中使用**泛型**（避免对实现 `ISupportMappable` 接口的结构体进行装箱）。
- 在 `Contact.cs` 中为静态刚体新增特殊代码路径（避免不必要的缓存行失效）。
- 为 `RigidBody` 新增 `PredictPosition`、`PredictOrientation` 和 `PredictPose`。
- 在 `JQuaternion` 中新增 `CreateFromAxisAngle` 和 `Normalize` 方法。

### Jitter 2.6.1 (2025-04-24)

- 修复了 `TriangleEdgeCollisionFilter` 中针对推测接触的缺陷。

### Jitter 2.6.0 (2025-04-24)

- 为 `ShapeHelper` 新增 `SampleHull` 和 `MakeHull`。
- 修复了 `ConvexHullShape` 爬山算法卡住的问题。
- 为 `PointCloudShape` 新增 SIMD 支持。
- 新增在 `TriangleMesh` 中忽略退化三角形的选项。
- 使厚度参数在 `FatTriangleShape` 中成为必需项。
- 为 `DynamicTree.Optimize` 新增 Fisher-Yates 洗牌。
- 优化了 `TriangleEdgeCollisionFilter`。

### Jitter 2.5.9 (2025-04-17)

- 在 MPREPA 中使用 `CollideEpsilon` 1e-5。
- 修复了 `ShardedDictionary` 中的一个缺陷。

### Jitter 2.5.8 (2025-04-16)

- 修复了 `DynamicTree.Optimize` 打乱碰撞对的问题。
- 重构了 `SoftBody.cs`
- 改进了 `TriangleEdgeCollisionFilter`。
- 进一步减少了 GC。

### Jitter 2.5.7 (2025-04-06)

- 修复了动态地将刚体设为静态时可能导致的崩溃。
- 改进了内存占用并减少了 GC。
- 新增 `Logger` 作为 `Trace` 的替代。

### Jitter 2.5.6 (2025-03-08)

- 修复了 `world.GetArbiter` 中的并发缺陷。

### Jitter 2.5.5 (2025-03-02)

- 新增针对 System.Numerics Vector3 和 Quaternion 的隐式转换运算符。
- 用 Trace.Information、Warning、Error 替换了 Trace.WriteLine。

### Jitter 2.5.4 (2025-02-08)

- 将 `JAngle.Radiant` 重命名为 `JAngle.Radian`。
- 将命名空间 `Jitter2.UnmanagedMemory` 重命名为 `Jitter2.Unmanaged`。
- 修复了针对一次性迭代器的 `body.AddShape(IEnumerable<RigidBodyShape> shapes)`。
- XML 文档的较小改进。

### Jitter 2.5.3 (2025-01-12)

- DynamicTree，`Optimize` 现在接受委托。
- 修复了 `TriangleShape` 射线投射不返回归一化法向的问题。
- 移除了 `CollisionHelper` 类。
- 将 `ActiveList` 和 `UnmanagedActiveList` 分别重命名为 `PartitionedSet` 和 `PartitionedBuffer`。
- 各种较小的改进（ToString() 重载、IEquality\<T\> 实现、XML 文档）

### Jitter 2.5.2 (2025-01-08)

- 为 `DynamicTree` 新增枚举方法，并将 `PairHashSet` 设为内部。
- **破坏性变更：** 移除了 `UseFullEPASolver` 选项。
- 进一步改进了高锁争用场景下的模拟性能。

### Jitter 2.5.1 (2024-12-31)

- 修复了 `PairHashSet` 中的缺陷。

### Jitter 2.5.0 (2024-12-23)

- 更好地利用多核系统。
- 修复了碰撞检测中的缺陷（可能产生 NaN 值）。

### Jitter 2.4.9 (2024-12-18)

- `DynamicTree` 实现的巨大改进。

### Jitter 2.4.8 (2024-11-27)

- 新增以双精度模式构建的选项。
- 将 `Constraint` 构造函数设为公共，以允许自定义约束。

### Jitter 2.4.7 (2024-11-18)

- **破坏性变更：** 放弃 .NET6 支持，新增 .NET9。
- 接触的 SIMD。
- 修复接触流形溢出。
- 更改默认阻尼。
- 改进了辅助接触点。
- 较小的 API 变更。

### Jitter 2.4.6 (2024-10-28)

- **破坏性变更：** Jitter 世界现在使用 World.Capacity 构造。
- **破坏性变更：** World.RayCast 移至 World.DynamicTree.RayCast。
- **破坏性变更：** 将 NumberSubsteps 重命名为 SubstepCount。
- 新增分裂冲量。**破坏性变更：** SolverIterations 属性现在是元组。
- API 中的若干较小改进。


### Jitter 2.4.5 (2024-10-07)

- 为 NarrowPhase 新增方法：Distance 和 Overlap。
- **破坏性变更：** 将 NarrowPhase.SweepTest 重命名为 NarrowPhase.Sweep。
- **破坏性变更：** 将 NarrowPhase.GJKEPA 重命名为 NarrowPhase.Collision。
- 使 PointTest、Raycast 和 SweepTest 使用新的 SimplexSolver 和 SimplexSolverAB 实现。
- 修复了 GJKEPA 中分离情况下的法向。

### Jitter 2.4.4 (2024-09-14)

- 实现了在附加调试器时使用 Jitter 的修复和变通方案。

### Jitter 2.4.3 (2024-08-31)

- 修复了 MPR 碰撞检测中因拼写错误导致的边界情况错误（缺陷修复）。
- FatTriangleShape 层级未正确考虑变换（缺陷修复）。

### Jitter 2.4.2 (2024-08-26)

- 新增 FatTriangleShape，为三角形赋予厚度，这对静态三角形网格很有用。
- DynamicTree 中从潜在碰撞对中移除现在会忽略过滤器（缺陷修复）。
- 对推测接触使用扫掠测试，大幅改进了该场景下的模拟质量。
- **破坏性变更：** 重新定义 NarrowPhase.SweepTest 的结果。
- 改进了 TriangleEdgeCollisionFilter。

### Jitter 2.4.1 (2024-08-21)

- 改进了 TriangleEdgeCollisionFilter。
- 实现了盒体和球体的解析射线投射。
- 在 Contact 中将 Restitution 和 Friction 设为公共。
- 改进了 DynamicTree 接口。
- 在 ActiveList 中新增 Debug.Asserts。
- 将 ArbiterKey 标记为 readonly。
- 为 PairHashSet 新增枚举器。
- 更改 ShapeHelper.MakeHull 以接受 ICollection 类型的泛型。

### Jitter 2.4.0 (2024-08-10)

- 改进了 TrimPotentialPairs 逻辑。
- 优化了四元数向量变换。
- 扩展了 ContactData.UsageMask 的功能。
- **破坏性变更：** 彻底重构了形状系统。常规形状（盒体、球体、胶囊体等）现在派生自 RigidBodyShape。某些方法签名略有变化，例如射线投射。
- 改进了异常。
- 新增 ReferenceFrameAttribute。

### Jitter 2.3.1 (2024-06-02)

- 只读包装器（ReadOnlyList、ReadOnlyHashset）现在是结构体。
- 尺寸非常小的形状可能具有接近零或为零的质量/惯性。用它们创建刚体现在会抛出异常（使用 body.AddShape(shape, setMassInertia: false) 以不使用形状的质量属性）。
- 为每个刚体新增 BeginCollide 和 EndCollide 事件。

### Jitter 2.3.0 (2024-05-20)

- 新增 RigidBody.RemoveShape 重载，可一次移除多个形状。
- 将 Rigid.ClearShapes 标记为已弃用。
- **破坏性变更：** 使用 JQuaternion 表示朝向。对 API 破坏深表歉意。

### Jitter 2.2.1 (2024-04-29)

- 为 world.AddShape 新增可选的 activate 参数。
- 新增 NarrowPhase.SweepTest。
- EPA 碰撞检测：各种改进。
- 改进 RayCast 和 PointTest 的退出条件。
- 移除针对零质量形状的冗余 ArgumentException。
- 处理零时间步。对负时间步抛出 ArgumentException。
- 为关节类新增关节基类。

### Jitter 2.2.0 (2024-01-02)

- **破坏性变更：** 将 `Raycast` 重命名为 `RayCast`。
- `world.Remove(world.NullBody)` 现在会移除与 NullBody 关联的所有形状、约束和接触。
- `world.AddShape(shape)` 会遵循关联刚体的激活状态。最值得注意的是：将 `TriangleShape` 直接添加到 world.NullBody 以表示静态几何体时的性能改进。
- ConvexHullShape 的性能改进。
- 改进了 GJKEPA 碰撞检测中的终止条件。

### Jitter 2.1.1 (2023-12-17)

- 修复了 `TriangleMesh` 中因哈希冲突导致的 O(n^2) 问题。
- 即使未附加 `RigidBody`，`Shape` 的 `WorldBoundingBox` 现在也会更新。

### Jitter 2.1.0 (2023-12-10)

- 新增刚体的调试绘制（`RigidBody.DebugDraw`）。
- 修复了 `TransformedShape.cs` 中 `CalculateMassInertia` 的一个缺陷。
- 改进了射线投射性能并引入 `NarrowPhase.PointTest`。
- **破坏性变更：** 反转了 `BroadPhaseCollisionFilter` 的行为。
- **破坏性变更：** 反转了 `RigidBody.Damping` 中阻尼系数的定义（0 = 无阻尼，1 = 立即停止）。
- 新增 `RigidBody.SetMassInertia` 重载，可将逆惯性设为零。
- 当刚体质量被设为零时现在会抛出异常。
- 修复了 `Contact.cs` 中摩擦处理的一个缺陷。

### Jitter 2.0.1 (2023-10-28)

- 修复了接触初始化中影响软体物理的一个缺陷。

### Jitter 2.0.0 (2023-10-22)

初始稳定版本。
