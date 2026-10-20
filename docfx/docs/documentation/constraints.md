# 约束

约束限制刚体的自由度。
在 Jitter2 中，约束总是在两个刚体之间起作用。
关节是多个约束的集合，因此它可以作用于多个刚体。

## 自由度

刚体有 6 个自由度（DOF）：3 个平移自由度和 3 个旋转自由度。
约束会移除其中一个或多个自由度。

| 约束 | 移除的自由度 | 说明 |
|------------|-------------|-------------|
| `BallSocket` | 3 个平移 | 将每个刚体上的一个点锚定在一起 |
| `FixedAngle` | 3 个旋转 | 锁定刚体之间的相对朝向 |
| `HingeAngle` | 2 个旋转（带限位时为 3 个） | 仅允许绕单个轴旋转，可选地强制角度限位 |
| `PointOnLine` | 2 个平移（带限位时为 3 个） | 将点约束到一条直线上，可选地限制沿轴的距离 |
| `PointOnPlane` | 1 个平移（限位生效时） | 将点约束到一个平面上，仅在违反限位时强制 |
| `DistanceLimit` | 1 个平移（限位生效时） | 约束锚点之间的距离，仅在超出限位范围时强制 |
| `TwistAngle` | 1 个旋转（限位生效时） | 限制绕轴的相对扭转，仅在超出限位范围时强制 |
| `ConeLimit` | 1 个旋转（限位生效时） | 限制两个轴之间的角度倾斜 |
| `EllipticalConeLimit` | 1 个旋转（限位生效时） | 限制内部和外部椭圆锥之间的摆动；扭转保持自由 |
| `AngularMotor` | — | 驱动角速度（不移除自由度） |
| `LinearMotor` | — | 驱动线速度（不移除自由度） |
| `SpringConstraint` | 1 个平移 | 沿锚点连接方向的类弹簧力 |

## 关节

关节是针对常见用例的约束组合：

| 关节 | 使用的约束 | 剩余自由度 |
|-------|------------------|---------------|
| `HingeJoint` | `BallSocket` + `HingeAngle`（可选 `AngularMotor` 或 `TwistServo`） | 1 个旋转 |
| `PrismaticJoint` | `PointOnLine` + `FixedAngle` 或 `HingeAngle`（可选 `LinearMotor` 或 `LinearAxisServo`） | 1 个平移 |
| `UniversalJoint` | `BallSocket` + `TwistAngle`（可选 `AngularMotor` 或 `AngularServo`） | 2 个旋转 |
| `WeldJoint` | `BallSocket` + `FixedAngle` | 0（完全锁定） |

### 关节驱动

`HingeJoint`、`PrismaticJoint` 和 `UniversalJoint` 可以创建可选的马达或伺服。
选择 `JointDrive.Motor` 进行速度控制，或选择 `JointDrive.Servo` 进行位置控制。
省略驱动时只创建机械关节（`JointDrive.None`）。

```cs
var elbow = new HingeJoint(
    world, upperArm, lowerArm, center, axis,
    angle: AngularLimit.FromDegree(0, 135),
    drive: JointDrive.Servo);

elbow.Servo!.TargetAngle = JAngle.FromDegree(90);
elbow.Servo.MaximumTorque = 20;
```

铰链会暴露一个 `TwistServo`；其目标角度使用与 `HingeAngle.Angle` 相同的零点和方向。
移动关节会暴露一个 `LinearAxisServo`，使用与 `Slider.Distance` 相同的坐标：

```cs
var slider = new PrismaticJoint(
    world, body1, body2, center, axis,
    limit: new LinearLimit(0, 2),
    drive: JointDrive.Servo);

slider.Servo!.TargetPosition = 1.5f;
slider.Servo.MaximumForce = 100;
```

万向节会暴露一个 `AngularServo`，其目标朝向是刚体 2 相对于刚体 1 的朝向。
它的初始目标就是当前的相对朝向：

```cs
var universal = new UniversalJoint(
    world, body1, body2, center, axis1, axis2,
    drive: JointDrive.Servo);

universal.Servo!.TargetRelativeOrientation = targetRelativeOrientation;
universal.Servo.MaximumTorque = 20;
```

扭转约束会保持生效，因此只能到达关节所允许的朝向。

驱动开始时最大力或力矩为零。机械限位与驱动相互独立。
关节在启用、禁用、移除或为调试而绘制时都会包含其驱动。
选择 `JointDrive.Motor` 时 `Motor` 可用；选择 `JointDrive.Servo` 时 `Servo` 可用。否则相应属性为 null。

较旧的 `hasMotor` 构造函数重载已过时，但仍受支持。将 `hasMotor: true` 替换为
`drive: JointDrive.Motor`，或在不需要马达时省略驱动。

## 创建约束

约束通过世界类中的创建方法实例化：

```cs
var constraint = world.CreateConstraint<ConstraintType>(body1, body2);
```

Jitter2 在内部为约束分配并指派非托管内存，使其可以像普通类一样使用。
求解器会与非托管内存结构交互，具体类型为 `ConstraintData`（256 字节）或 `SmallConstraintData`（128 字节），取决于是否设置了 `constraint.IsSmallConstraint`。
小型约束专为更简单、内存占用更小的约束而设计，例如软体的弹簧约束。

> [!CAUTION]
> **约束初始化**
> 对于所有默认约束，必须在 `world.CreateConstraint` 之后调用一次 `constraint.Initialize`。

### 世界空间初始化

约束使用世界空间坐标（锚点、轴）进行初始化。
在调用 `Initialize` 之前，应将刚体置于有效的配置中。
随后，约束会根据刚体当前的姿态计算并存储每个刚体的局部参考系。
这意味着约束会“记住”初始设置，并在模拟期间尝试维持它。

### 示例：铰链关节

一扇摆动的门可以通过组合两个约束来实现：一个 `HingeAngle` 约束移除两个角度自由度（刚体只能绕单个轴旋转），以及一个 `BallSocket` 约束移除全部三个平移自由度：

```cs
var hingeAngle = world.CreateConstraint<HingeAngle>(body1, body2);
hingeAngle.Initialize(hingeAxis, AngularLimit.Full);

var ballSocket = world.CreateConstraint<BallSocket>(body1, body2);
ballSocket.Initialize(hingeCenter);
```

或者，为方便起见可以使用 `HingeJoint`：

```cs
var hinge = new HingeJoint(world, body1, body2, hingeCenter, hingeAxis);
```

### 锥形限位

`ConeLimit` 将两个轴之间的角度限制在 0° 到 180° 的范围内。
诸如 `[0°, 45°]`、`[135°, 180°]` 和 `[0°, 180°]` 的范围是有效的。
相等的限位会在角度严格介于 0° 和 180° 之间时固定该角度，例如 `[30°, 30°]`。

固定范围 `[0°, 0°]` 和 `[180°, 180°]` 会被 `Initialize` 和 `Limit` 属性拒绝。
要在允许自由扭转的同时保持轴平行或反平行，请使用带 `AngularLimit.Full` 的 `HingeAngle`：

```cs
var hingeAngle = world.CreateConstraint<HingeAngle>(body1, body2);
hingeAngle.Initialize(hingeAxis, AngularLimit.Full);
```

在初始化之前，将刚体放置到预期轴对齐或反平行的位置。
`HingeAngle` 会将其当前相对朝向存储为参考；它不会在初始化期间
将两个任意错位的轴拉向对齐。

### 椭圆锥限位

`EllipticalConeLimit` 允许在两个互相垂直的方向上设置不同的摆动限位。刚体 1 拥有
锥体框架；刚体 2 拥有被约束的轴。所有初始化方向都以
世界空间提供，并相对于各自刚体存储。

```cs
var cone = world.CreateConstraint<EllipticalConeLimit>(body1, body2);
cone.Initialize(
    axis: JVector.UnitY,
    referenceDirection: JVector.UnitX,
    limitAlongReference: AngularLimit.FromDegree(15, 60),
    limitAlongPerpendicular: AngularLimit.FromDegree(5, 25));
```

下界定义内部椭圆（15° 和 5°）；上界定义外部椭圆
（60° 和 25°）。允许在这些边界之间摆动。将两个下界都设为零
会移除内部边界。将每个下界设为其上界，会将摆动约束到
一个椭圆锥面。两个方向上的范围相同则得到圆形锥体。

椭圆在**摆动角坐标**中定义：沿参考方向和 `axis cross referenceDirection`
分解的最短倾斜角。每个边界都满足
`(swing1 / angle1)^2 + (swing2 / angle2)^2 = 1`。这些是耦合边界，而不是对两个角度的
独立钳制。参考方向会投影为垂直于锥体轴；
平行方向会被拒绝。刚体 1 的旋转会旋转整个椭圆框架。

每个范围必须满足 `0 <= From <= To < 180°`，且上界为正。下界
必须同时为零或同时为正。不支持固定的零度和 180° 倾斜。请使用
`SetLimits` 一起更改范围；使用 `GetSwing` 检查当前坐标。
双轴 `Initialize` 重载支持初始倾斜的刚体 2。单轴重载
从零倾斜开始，这违反正的下界，并将由求解器纠正。

刚体 2 沿其被约束轴的平移和扭转保持自由。将此约束
与 `BallSocket` 组合成关节，并可选择使用 `TwistAngle` 来限制扭转。

### 伺服

伺服驱动刚体朝向目标构型。它们可以从该构型之外开始，并且目标可以在模拟步之间改变。
更改目标会在下一步唤醒处于休眠状态的相连动态刚体。马达控制速度；伺服控制位置或朝向。

| 伺服 | 控制 | 目标 |
|-------|----------|--------|
| `AngularServo` | 3 个角度方向 | `TargetRelativeOrientation`：刚体 2 在刚体 1 坐标系中的朝向 |
| `TwistServo` | 1 个角度方向 | `TargetAngle`：相对于初始零扭转姿态的扭转 |
| `LinearServo` | 3 个线方向 | `TargetPosition`：从锚点 1 到锚点 2 的偏移，位于刚体 1 坐标系中 |
| `LinearAxisServo` | 1 个线方向 | `TargetPosition`：沿附着于刚体 1 的轴的有符号锚点间距 |

最大力或力矩默认为 **零**，与马达相同。设置 `MaximumForce` 或 `MaximumTorque`
以启用驱动。正无穷允许无限制的力或力矩。
`Bias` 控制伺服接近目标的积极程度；`Softness` 允许一定的柔软度。

例如，驱动两个机械臂段朝向某个相对朝向：

```cs
var servo = world.CreateConstraint<AngularServo>(upperArm, lowerArm);
servo.Initialize(JQuaternion.CreateRotationX((Real)0.5));
servo.MaximumTorque = 20;

// 稍后选择不同的构型，而无需直接移动刚体。
servo.TargetRelativeOrientation = JQuaternion.CreateRotationX((Real)1.0);
```

无参数的 `AngularServo.Initialize()` 则捕获当前的相对朝向。
`LinearServo.Initialize(anchor1, anchor2)` 接受独立的世界空间附着点；
其默认目标为零，会将它们拉向重合。`LinearAxisServo.Initialize` 还
接受期望的有符号间距。`TwistServo.Initialize` 接受相对于其初始参考的目标角度，
因此非零目标会以角度误差开始。

`TwistServo` 使用 -180° 到 +180° 的有符号关节角度。从 -170° 移动到 +170°
会经过零，行程为 340°。此区间之外的目标会被拒绝；不跟踪圈数。使用机械限位使关节的运动保持在该角度区间内。
`AngularServo` 以朝向为目标，并使用最短旋转到达，因此
等价的朝向表示相同的目标。

伺服与机械约束协同工作。对于肘部，使用铰链保持臂段相连
并强制执行关节的角度限位，然后在同一轴上添加 `TwistServo`。
其力矩限制控制执行器的力量，而铰链保留其机械限位。

### 固定约束

要相对世界空间约束刚体，请将 `world.NullBody` 用作其中一个刚体。
例如，使胶囊体保持直立：

```cs
var upright = world.CreateConstraint<HingeAngle>(capsule, world.NullBody);
upright.Initialize(JVector.UnitY, AngularLimit.Full);
```

### 柔软度与偏置

对于大多数约束，可以设置柔软度和偏置值。
这些值定义约束限位被执行的严格程度。
更柔软的约束可能提高模拟稳定性，但不会完全执行约束限位。
可以调整柔软度和偏置参数以获得最佳效果。
通过子步也可以获得更好的约束行为，参见[世界](world.md)。

### 启用/禁用约束

可以使用 `constraint.IsEnabled` 临时启用或禁用约束。

## 调试绘制

约束和关节可以通过 `IDebugDrawer` 绘制其锚点、轴和限位：

```cs
constraint.DebugDraw(drawer);
joint.DebugDraw(drawer);
```

参见[设置调试绘制器](bodies.md#debugging-shapes)，将其连接到你的渲染器。
在物理步之后、且世界未被其他线程修改时调用这些方法。

绿色表示参考辅助线和上限或固定限位，红色表示下限，白色表示
当前锚点和方向。坐标框架使用红色表示 X、绿色表示 Y、蓝色表示 Z。

| 约束 | 绘制内容解读 |
|------------|---------------------|
| `BallSocket` | 两个锚点标记应重合。它们之间的连线表示分离距离。 |
| `DistanceLimit` | 第一个锚点周围的球体显示到第二个锚点允许的距离。 |
| `PointOnLine` | 叉号标记直线上的止挡；箭头标记无界端点。白线显示偏离直线的位移。 |
| `PointOnPlane` | 网格显示限制平面。当锚点位于允许区域之外时会出现白色投影线。平面会延伸到网格之外。 |
| `ConeLimit`, `EllipticalConeLimit` | 白色轴必须保持在外锥之内，并在任何红色内锥之外。相等的限位允许沿一个锥面运动。 |
| `HingeAngle` | 圆弧显示允许的角度，或由绿色射线标记固定角度。白色射线显示当前角度。短轴箭头揭示铰链的错位。 |
| `TwistAngle` | 表盘显示当前扭转及其限位。白色轴显示当前刚体方向；摆动保持自由。 |
| `FixedAngle` | 每个刚体上的 RGB 框架体现初始相对朝向。满足目标时对应的轴保持平行。`World.NullBody` 不绘制框架。 |
| `LinearMotor` | 绿色箭头显示刚体 1 对请求的相对运动的贡献，白色箭头显示刚体 2 的贡献。这些显示的是目标方向，而不是测量速度；静态刚体保持不动。 |
| `AngularMotor` | 绿色圆圈和白色箭头显示刚体 2 的目标旋转，当刚体 2 为静态或运动学刚体时则显示刚体 1 的。短线显示两个马达轴。箭头显示目标方向，而不是测量速度；目标为零时没有箭头。 |
| `AngularServo` | 绿色目标框架和 RGB 当前框架共享原点。达到目标时对应的轴对齐。 |
| `TwistServo` | 表盘标记目标扭转和当前扭转角。 |
| `LinearServo` | 绿色和白色标记显示目标和当前锚点位置；连线显示位置误差。 |
| `LinearAxisServo` | 标记显示投影到参考轴上的目标位置和当前位置。垂直方向的运动不受控制。 |
| `SpringConstraint` | 线圈连接锚点；绿色叉号标记沿弹簧方向的静止长度。 |

传入 `DebugDrawSettings` 可更改外观：

```cs
constraint.DebugDraw(drawer, new DebugDrawSettings
{
    GuideSize = 2,
    FrameSize = 1.5f
});
```

`GuideSize` 更改辅助线的大小。`FrameSize` 以世界单位设置坐标框架
轴的长度，`PointRadius` 设置锚点球体的半径。箭头大小和
杆粗细分别通过 `ArrowHeadSize` 和 `ArrowShaftRadius` 调整。
物理距离和角度限位保持其实际值。
有关颜色、箭头和细节设置，参见 <xref:Jitter2.DebugDrawSettings>。

箭头默认是实心的。将 `SolidArrows = false` 设为线框箭头。`ArrowSegments` 控制
箭头的平滑度。线框箭头将底面周围的每个顶点连接到尖端。
绘制器的 `DrawTriangle` 实现决定实心箭头和锚点标记
显示为填充还是三角形边。绘制自己的辅助线时，`DrawArrow` 和 `DrawFrame`
使用相同的默认值；`DrawWireArrow` 和 `DrawWireFrame` 显式绘制线框几何体。

## 约束生命周期

约束附加到它们的两个刚体上。在下列情况下，它们可能在没有显式
调用 `world.Remove(constraint)` 时变得无效：

- 任一相连刚体被移除；
- 更改刚体的运动类型导致两个相连刚体都变为非动态；
- `world.Clear()` 移除相连的刚体；或
- 世界被释放。

当另一个操作可能已移除保留的约束引用时，在访问它之前
使用 `constraint.IsValid`：

```cs
world.Remove(body1); // 同时移除附加到 body1 的约束

if (constraint.IsValid)
{
    constraint.IsEnabled = false;
}
```

一旦 `IsValid` 为 `false`，约束的非托管数据就不再可用，不得访问其
基于数据的属性和方法。刚体提供了等效的
`body.IsValid` 检查，用于刚体移除或世界释放之后。

如需响应式清理，当约束被显式或自动移除时会引发 `World.ConstraintRemoved`：

```cs
world.ConstraintRemoved += removedConstraint =>
{
    // 丢弃与 removedConstraint 关联的任何应用侧状态。
};
```

回调运行时约束已无效。禁用约束和释放
世界不会引发此事件。
