# 形状

形状定义刚体如何与其他物体碰撞。
形状实现 `ISupportMappable` 接口，并且始终是凸的。
它们可以传给 `NarrowPhase` 类中定义的静态方法用于碰撞检测。
形状还实现 `IDynamicTreeProxy` 接口，可以添加到 `DynamicTree` 类中。
当形状添加到刚体时，这一操作会自动完成（`world.DynamicTree`）。

> [!NOTE]
> **凹形状**
> 凹形状可以通过在单个刚体上组合多个凸形状来表示。
> 可以使用第三方的“凸分解”库从任意网格生成凸包。

## 修改已附加的形状

在形状添加到刚体之后修改其尺寸或局部变换，不会自动刷新动态树、刚体的质量属性或已有的接触点。请在两次 `world.Step` 调用之间进行这些更改，然后显式更新受影响的状态：

```cs
box.Size = new JVector(2, 1, 1);
world.DynamicTree.Update(box); // 刷新此形状的包围盒和宽相配对。
body.ClearContactCache();      // 丢弃基于旧几何形状的接触点。
body.SetMassInertia();          // 仅当刚体使用由其形状推导出的质量属性时。
```

`world.DynamicTree.Update(shape)` 会立即更新一个已注册的形状。常规的世界步进会更新活动代理，但不会跟踪非活动代理的更改。如果更改影响到休眠的动态刚体，请使用 `body.SetActivationState(true)` 显式激活该刚体。更新静态形状的代理不会唤醒新近与其重叠的休眠刚体；请自行激活受影响的动态刚体。

形状更改也可能使刚体相互穿透。引擎无法决定应如何将它们分开，因此应用程序必须自行选择是移动它们、唤醒它们，还是让求解器处理新的接触。如果质量或惯性是显式设置的，请根据应用程序的质量策略保留或重新计算它，而不要调用无参数的 `SetMassInertia()`。

## 默认类型

默认形状的继承层次结构：

```text
Shape
├── RigidBodyShape
│   ├── BoxShape
│   ├── CapsuleShape
│   ├── ConeShape
│   ├── ConvexHullShape
│   ├── CylinderShape
│   ├── PointCloudShape
│   ├── TransformedShape
│   └── TriangleShape
└── SoftBodyShape
    ├── SoftBodyTetrahedron
    └── SoftBodyTriangle
```

大多数形状的含义不言自明（更多细节见 API 文档），下面概述一些特殊之处。

### ConvexHullShape

`ConvexHullShape` 的构造函数接收一个三角形列表。

```cs
public ConvexHullShape(List<JTriangle> triangles)
```

提供的三角形*必须*构成凸包。
除了拒绝零质量凸包外，不会检查凸形状的有效性。
无效的形状可能导致碰撞出现异常，
或使碰撞检测中的算法无法终止。
这些三角形用于构建内部加速结构，通过爬山法加速该形状的碰撞检测。

`convexHullShape.Clone()` 方法可用于克隆形状：
此后两个形状会共用同一内部数据结构。

### PointCloudShape

`PointCloudShape` 与 `ConvexHullShape` 非常相似。
构造函数接收一个顶点列表。

```cs
public PointCloudShape(List<JVector> vertices)
```

顶点不需要构成凸包；不过碰撞检测会对这些顶点进行“收缩包裹”，因此最终的碰撞形状是凸的。
例如，将立方体的 8 个顶点传给构造函数会生成一个立方体形状；在立方体中心添加第 9 个顶点没有任何影响。

> [!WARNING]
> **顶点数量**
> `PointCloudShape` 只应用于少量到中等数量的顶点（$\approx{}300$）。更大的顶点数会对性能产生负面影响。
> 对于更复杂的凸包，`ConvexHullShape` 是更好的选择。

### TransformedShape

`TransformedShape` 接收另一个形状作为输入并对其进行变换。

```cs
public TransformedShape(RigidBodyShape shape, in JVector translation, in JMatrix transform)
```

任何仿射变换都是可行的。
被包裹的形状可以被平移、旋转、缩放和剪切。
例如，球体形状可以变换为椭球体。

### TriangleShape

`TriangleShape` 没有体积。
它主要用于静态几何，但也可以添加到非静态刚体。
`TriangleShape` 由 `TriangleMesh` 和索引构造。

```cs
public TriangleShape(TriangleMesh mesh, int index)
```

`TriangleMesh.Triangle` 存储有关相邻三角形的信息。
该信息用于 `TriangleEdgeCollisionFilter`（默认启用）中，以解决形状在相连三角形之间的边上滑动时出现的碰撞伪影。
这些边通常被称为“内部边”，在向游戏添加关卡几何时可能造成严重问题。

### SoftBodyShape

`SoftBodyShape` 的顶点由刚体表示。
形状（三角形和四面体）由顶点的位置动态定义。
`SoftBodyShape` 不会添加到刚体。

## 调试绘制

形状可以单独绘制：

```cs
var shape = new SphereShape(radius: 1);
shape.DebugDraw(drawer);
```

已附加的形状会跟随其刚体的位置和旋转。上面的球体在被附加到刚体之前会出现在世界
原点处。有关设置绘制器以及调整颜色和平滑度的内容，参见[在刚体上调试形状](bodies.md#debugging-shapes)。

绘制器中的 `DrawTriangle` 实现决定使用填充三角形还是线框。
绘制方法参见 <xref:Jitter2.IDebugDrawer>。

> [!WARNING]
> 调试绘制可能很慢。点云和自定义形状可能出现间隙或面重叠。
> 更高的细节设置会增加开销，且并不能保证绘制更准确。

## 自定义形状

自定义形状可以很容易地实现。
形状由其支撑函数定义——该函数可以查阅或推导。

以下示例演示如何实现一个半径为 1 的半球（对称轴与 y 轴对齐）：

```cs
public class HalfSphereShape : RigidBodyShape
{
    public override void SupportMap(in JVector direction, out JVector result)
    {
        const float centerOfMassOffset = 3.0f / 8.0f;

        if (direction.Y >= 0.0f)
        {
            result = JVector.Normalize(direction);
        }
        else
        {
            JVector pDir = new JVector(direction.X, 0.0f, direction.Z);
            float pDirSq = pDir.LengthSquared();

            if (pDirSq < 1e-12f) result = JVector.Zero;
            else result = (1.0f / MathF.Sqrt(pDirSq)) * pDir;
        }

        // 平移，使 (0, 0, 0) 成为质心
        result.Y -= centerOfMassOffset;
    }

    public override void GetCenter(out JVector point)
    {
        point = JVector.Zero;
    }
}
```

质量属性和包围盒会使用 `ShapeHelper` 类中的方法从支撑映射自动计算。
可以通过在形状类中直接提供重写来优化性能：

```cs
public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)

public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out float mass)

public override bool LocalRayCast(in JVector origin, in JVector direction, out JVector normal, out float lambda)
```

`ShapeHelper` 类还可以用于生成形状（或任何实现 `ISupportMappable` 的类）的三角网格表示：

```cs
public static List<JTriangle> Tessellate(ISupportMappable support, int subdivisions = 3)
```
