# 刚体

刚体是动力学系统的主要实体。

## 创建刚体

刚体与 `World` 类的一个实例相关联：

```cs
var world = new World();
var body = world.CreateRigidBody();
```

## 添加形状

可以向刚体添加多个形状，例如：

```cs
body.AddShapes([new SphereShape(radius: 2), new BoxShape(size: 1)]);
```

形状决定刚体之间如何碰撞。

> [!WARNING]
> **一次创建多个刚体**
> 调用 `body.AddShape(shape)` 时，形状会被注册到引擎的碰撞系统中，并立即加入空间树结构（`DynamicTree`）以进行高效的宽相碰撞检测。
> 在添加形状之前先设置刚体位置，避免在 $(0, 0, 0)$ 处注册大量对象。

> [!WARNING]
> **将同一实例传递给多个刚体**
> Jitter 不允许将同一个形状实例传递给多个刚体，否则会抛出异常。

球体形状的定义使其几何中心与（局部）坐标系的中心 $(0, 0, 0)$ 对齐。
所有基本图元（球体、盒体、胶囊体、圆锥、圆柱）也是如此。

添加形状后，会相应地计算关联刚体的质量属性（质量和惯性）。
计算时假定单位密度。

只添加一个球体

```cs
body.AddShape(new SphereShape(radius: 1));
```

将得到一个具有教科书式惯性、半径为 1 的单位密度球体的质量和惯性。

刚体的质量属性也可以使用 `body.SetMassInertia` 直接设置。
惯性张量使用 `JSymmetricMatrix`，它存储对称矩阵的六个独立分量。
`body.InverseInertia` 和形状质量属性计算都使用该类型。

```cs
body.SetMassInertia(JSymmetricMatrix.CreateScale(2, 3, 4), mass: 5);
```

该张量在刚体空间中、相对于刚体质心表示。

向 `body.AddShape(...)` 或 `body.AddShapes(...)` 传入 `MassInertiaUpdateMode.Preserve` 可阻止添加形状时自动重新计算质量属性。

> [!IMPORTANT]
> 刚体的位置必须与其质心对齐。
> 在刚体的局部参考系中，质心为 $(0, 0, 0)$。
> 形状或形状组合必须相应地平移。

### 调试形状

调试绘制会按形状当前的位置和旋转显示附加到刚体上的碰撞形状。
要显示它们，请实现 `IDebugDrawer` 将 Jitter 连接到你的渲染器：

```cs
using Jitter2;
using Jitter2.LinearMath;

public sealed class PhysicsDebugDrawer : IDebugDrawer
{
    public void DrawSegment(in JVector pA, in JVector pB, uint color)
    {
        // 使用你的渲染器从 pA 到 pB 绘制一条线。
        // 位置位于世界空间；颜色按 0xRRGGBBAA 打包。
    }

    public void DrawTriangle(in JVector pA, in JVector pB, in JVector pC, uint color)
    {
        DrawSegment(pA, pB, color);
        DrawSegment(pB, pC, color);
        DrawSegment(pC, pA, color);
    }
}
```

将 `DrawSegment` 中的注释替换为对渲染器线条绘制函数的调用。
创建一个绘制器并复用它：

```cs
IDebugDrawer drawer = new PhysicsDebugDrawer();
```

然后在物理步完成之后，在应用程序的绘制循环中调用：

```cs
body.DebugDraw(drawer);
```

> [!WARNING]
> **调试绘制可能很慢**
> 绘制大量刚体或使用高细节设置会显著降低帧率。
> 点云和自定义形状的绘制尤其昂贵。
> 请将调试绘制限制在需要检查的对象上。

这会绘制附加到 `body` 的所有形状，包括它们的偏移和旋转。上面的绘制器
将每个三角形绘制为三条边，生成线框。如需填充表面，请在 `DrawTriangle` 中调用渲染器的
三角形绘制函数；参见[形状绘制](shapes.md#debug-drawing)。

默认绘制为白色。要绘制绿色形状并使曲面更平滑：

```cs
body.DebugDraw(drawer, new DebugDrawSettings
{
    CurrentColor = DebugDrawColors.Green,
    Segments = 64
});
```

球体、胶囊体、圆柱和圆锥使用三角形绘制。`Segments` 设置每个圆形横截面周围的分段数；默认值为 48。例如，`Segments = 8` 会使
圆柱呈现八边形轮廓，而 `64` 则让它看起来圆润得多。值越高需要越多
三角形。盒体、三角形形状和凸包已经具有平面，因此 `Segments` 对它们没有
影响。这些设置仅影响可视化。

`PointCloudShape` 和自定义形状在调试绘制时可能使用近似表面。有关其局限性，参见
[形状绘制](shapes.md#debug-drawing)。

## 力和冲量

可以使用 `body.AddForce` 施加力和力矩。它们会累积到下一次 `world.Step`，并在该调用结束时清除。由此产生的速度变化会在**下一次**步中应用。对于初始静止、不受重力或其他影响的刚体：

```cs
body.AddForce(force);
world.Step(dt); // 存储速度变化；速度和位置保持不变。
world.Step(dt); // 将变化应用到速度并移动刚体。
```

这种一步延迟同样适用于重力。请保持时间步和子步数固定：速度变化使用一步的子步时长计算，并在下一步中每个子步应用一次。

如需瞬时速度变化，请使用 `body.ApplyImpulse`：

```cs
// 线性冲量——立即改变速度。
body.ApplyImpulse(new JVector(0, 10, 0));

// 在世界空间某位置施加冲量——同时改变线性速度和角速度。
body.ApplyImpulse(new JVector(0, 10, 0), hitPosition);
```

两个重载都接受可选的 `wakeup` 参数（默认 `true`）。
当设为 `false` 时，如果刚体处于休眠状态，冲量会被静默忽略。

## 重力

可以使用 `world.Gravity` 设置世界的重力。
属性 `body.AffectedByGravity` 可用于为单个刚体禁用重力。
重力遵循上文所述的[力积分时序](#forces-and-impulses)。

## 限制运动与平面模拟

`body.AllowedMotion` 指定模拟允许的平移和旋转。
默认值为 `MotionAxes.All`。每次赋值都会替换先前的选择。
所有轴都位于**世界空间**，包括角轴。

对于 XY 平面内的运动：

```cs
body.AllowedMotion = MotionAxes.PlaneXY;
// 等价于 LinearX | LinearY | AngularZ。
```

`PlaneXZ` 和 `PlaneYZ` 提供其他坐标平面对应的预设。
你也可以组合各个轴：

```cs
body.AllowedMotion = MotionAxes.LinearX | MotionAxes.AngularZ;
```

角标志表示旋转轴：`AngularZ` 允许绕世界 Z 轴旋转。
`MotionAxes.Linear` 允许所有平移，`MotionAxes.Angular` 允许所有旋转。

限制会改变**求解器使用的有效逆质量和逆惯性**。
被禁止的方向表现为具有无限质量或惯性。`body.Mass` 仍然是标量，
`body.InverseInertia` 继续描述刚体空间中的原始惯性。
对于耦合惯性张量，限制旋转也会改变其余轴周围的有效响应。
引擎从原始张量推导该响应。

原始质量属性会被保留。调用 `SetMassInertia` 和形状变化会更新
这些属性，同时保留当前限制。要从最新质量属性恢复不受限制的响应，
请赋值：

```cs
body.AllowedMotion = MotionAxes.All;
```

更改 `AllowedMotion` 会立即清除被禁止的速度分量，使缓存的
接触和关节冲量失效，并安排受影响的刚体激活。当再次允许运动时，
被清除的速度不会恢复。限制适用于力、重力、冲量、
接触、关节和运动学速度。位置和朝向仍可直接赋值。
当任何线性轴或角轴受到限制时，会跳过陀螺积分。
不要与 `World.Step` 并发更改该属性。

`MotionAxes.None` 禁止模拟运动，同时保留刚体的 `MotionType` 以及
碰撞/岛参与。对于普通的静态几何体，请使用 `MotionType.Static`。

平面运动使用现有的 3D 碰撞形状和接触生成。刚体会保留沿被禁止平移轴的
初始坐标；构建平面场景时请一致地设置该坐标。这不会新增单独的 2D 形状类型
或碰撞检测。

### 自定义约束

底层的 `RigidBodyData.InverseMassVector` 是 `JVector`。使用
`JVector.Multiply(data.InverseMassVector, impulse)` 获得逐分量的速度响应，
使用 `JVector.Multiply(direction, data.InverseMassVector) * direction` 获得求解器某行的平移逆有效质量。
用 `*` 对两个 `JVector` 值相乘会计算点积，因此它不能替代
逐分量乘法。

`RigidBodyData.InverseInertiaWorld` 是存储在六个标量中的 `JSymmetricMatrix`。它支持
`JVector.Transform`，并可使用 `ToMatrix()` 展开为 `JMatrix`。自定义求解器必须处理被禁止方向上的零
响应，包括奇异的有效质量矩阵。刚体数据在单精度下占 128 字节，在双精度下占 256 字节。
请按名称访问其字段，而不要依赖字段偏移。

对于对称的半正定有效质量矩阵，使用
`MathHelper.PseudoInverseSymmetric` 在保持被阻止方向零响应的同时，对可响应子空间求逆：

```cs
JMatrix effectiveMass = MathHelper.PseudoInverseSymmetric(inverseEffectiveMass);
```

在约束准备期间计算该值，并在求解器迭代中复用结果。
该方法假定输入有限、对称且半正定。在对奇异或近似奇异的
矩阵进行对角化时，它使用数值截断；有关依赖于精度的容差，
参见其 API 文档。

## 阻尼

Jitter2 使用简单的阻尼系统来减慢刚体。
这提高了模拟稳定性，也类似于现实世界中机械系统损失能量。
每个刚体都有一个线性阻尼因子和一个角阻尼因子，可使用 `body.Damping` 设置。
每次 `world.Step`，每个动态刚体的角速度和线速度都会乘以 $1-\gamma$，其中 $\gamma$ 是阻尼因子。
出于性能考虑，阻尼系统没有时间依赖性。
因此，时间步更小的模拟中的刚体会经历更大的阻尼。

## 推测接触

Jitter 内置的推测接触**仅作用于线性运动**。它们有助于防止快速移动的小物体
在帧间平移会跳过碰撞时穿透薄障碍物。
可使用 `body.EnableSpeculativeContacts` 按刚体启用它们。

推测包围盒扩展仅使用线速度。扫掠在保持形状当前朝向的情况下
预测平移。推测约束还会忽略角
表面速度（`ω × r`），并且仅对线速度施加冲量。它们无法阻止由旋转引起的
穿透，包括在时间步内由其他接触或约束引入的旋转。刚体仍会正常旋转，
普通重叠接触仍保留其角响应。

`world.SpeculativeVelocityThreshold` 设置生成推测
接触的最小相对线速度。`world.SpeculativeRelaxationFactor` 控制刚体可闭合
预测间隙的程度。这种近似以碰撞响应精度换取更少的穿透；
对于快速的平移或旋转，可能仍需要更小的时间步。

## 摩擦与恢复系数

摩擦和恢复系数可通过 `body.Friction` 和 `body.Restitution` 设置。
对于具有不同系数的两个刚体的碰撞，会取每个刚体值的最大值。

## 碰撞事件

`RigidBody` 提供两个事件：`BeginCollide` 和 `EndCollide`。
只要创建或移除涉及该刚体的仲裁器，就会触发这些事件。
有关详细信息和示例，参见[仲裁器](arbiters.md#collision-events)。

## 激活/停用

刚体总是被分配到一个岛。
岛由通过接触或约束两两相互作用的刚体组成。
不同的岛之间不以任何方式相互作用。

一旦活动刚体的角速度和线速度在 `body.DeactivationTime` 定义的时间段内持续低于
`body.DeactivationThreshold` 中定义的阈值，世界就可能将其标记为停用。
如果岛内所有非静态刚体都准备好休眠，整个岛就会被停用。
非活动刚体的模拟成本实际上为零。
一旦注册到与活动刚体的碰撞，岛（及其关联刚体）就可能被唤醒。

使用 `body.SetActivationState`，用户可以安排刚体激活或停用。
它不会立即改变刚体的激活状态（`body.IsActive`）。
下一次 `world.Step` 会考虑该刚体及其连接的岛进行激活或停用。
例如，对一个速度大于 `body.DeactivationThreshold` 的下落刚体调用 `body.SetActivationState(false)` 不会有持久效果，因为该刚体在更新期间会再次将其岛标记为活动。

如需显式立即停用，请使用 `world.ForceSleepIsland(island)`。
这会一次性停用整个岛，忽略正常的停用阈值，并清除岛内所有刚体的线速度、角速度、排队力和排队力矩。

激活状态对动态刚体和运动学刚体最有意义：

- 动态刚体在唤醒时被主动模拟，在休眠时被跳过。
- 运动学刚体可以属于岛。求解器将它们视为无限质量的刚体，但它们可以通过接触或约束唤醒相连的动态刚体。
- 静态刚体通常是处于非活动状态的簿记对象。它们不形成常规的岛连接，且 `body.SetActivationState(true)` 不会使静态刚体本身变为活动。

## 静态刚体

静态刚体（`body.MotionType == MotionType.Static`）具有无限质量，因此不受碰撞或约束影响。
它们也不会加入岛。
静态刚体不会与其他静态或非活动刚体产生碰撞。
即使调用了 `body.SetActivationState(true)`，静态刚体通常也是非活动的。

更改静态刚体的位置或朝向会更新其宽相代理，并在下一步唤醒受影响的非静态刚体。
静态刚体本身保持非活动；必须对变化的接触状态作出反应的刚体会被激活。
这对于偶尔编辑静态关卡几何体很有用，但从求解器的角度看，这是一次不连续的变换。
对于持续移动的平台或障碍物，请优先使用运动学刚体。

## 运动学刚体

运动学刚体（`body.MotionType == MotionType.Kinematic`）可以具有速度，因此会改变其位置。
它们在碰撞期间的作用类似于静态刚体——与普通刚体碰撞时其速度不会改变。
它们确实参与碰撞岛。
因此，运动学刚体像其他非静态刚体一样参与岛的激活和停用。
