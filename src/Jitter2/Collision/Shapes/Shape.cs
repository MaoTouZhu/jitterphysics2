/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.DataStructures;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 碰撞系统的主要实体。为窄相实现 <see cref="ISupportMappable"/>，为宽相碰撞检测
/// 实现 <see cref="IDynamicTreeProxy"/>。形状本身没有位置或朝向。形状可与
/// <see cref="RigidBody"/> 实例关联。
/// </summary>
public abstract class Shape : IDynamicTreeProxy, IUpdatableBoundingBox, ISupportMappable, IRayCastable, ISweepTestable, IDistanceTestable, IDebugDrawable
{
    int IPartitionedSetIndex.SetIndex { get; set; } = -1;

    /// <summary>
    /// 表示形状 ID 的 64 位整数。需要将形状按明确定义的顺序排列的算法会使用它。
    /// </summary>
    public readonly ulong ShapeId = World.RequestId();

    /// <summary>
    /// 形状在世界空间中的包围盒。当对应的 <see cref="RigidBody"/> 实例的位置或
    /// 朝向发生变化时，它会自动更新。
    /// </summary>
    public JBoundingBox WorldBoundingBox { get; protected set; }

    int IDynamicTreeProxy.NodePtr { get; set; }

    protected void SweptExpandBoundingBox(Real dt)
    {
        Real swept = dt * Velocity.Length();
        JBoundingBox box = WorldBoundingBox;
        box.Min -= new JVector(swept);
        box.Max += new JVector(swept);
        WorldBoundingBox = box;
    }

    internal bool IsRegistered => (this as IPartitionedSetIndex).SetIndex != -1;

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract JVector Velocity { get; }

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract void UpdateWorldBoundingBox(Real dt = (Real)0.0);

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract bool RayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda);

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract bool Sweep<T>(in T support, in JQuaternion orientation, in JVector position, in JVector sweep,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where T : ISupportMappable;

    [ReferenceFrame(ReferenceFrame.World)]
    public abstract bool Distance<T>(in T support, in JQuaternion orientation, in JVector position,
        out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
        where T : ISupportMappable;

    /// <inheritdoc/>
    [ReferenceFrame(ReferenceFrame.Local)]
    public abstract void SupportMap(in JVector direction, out JVector result);

    /// <inheritdoc/>
    [ReferenceFrame(ReferenceFrame.Local)]
    public abstract void GetCenter(out JVector point);

    /// <summary>使用默认可视化设置绘制形状。</summary>
    public void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <summary>使用提供的设置将形状绘制为世界空间三角形。</summary>
    /// <remarks>
    /// 点云和自定义形状可能使用带有缝隙或面片重叠的近似表面。
    /// 更高的细节设置会增加绘制开销，且并不保证表面更精确。
    /// </remarks>
    public virtual void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        var sink = new ShapeDebugDraw.DrawerSink(drawer, JQuaternion.Identity, JVector.Zero, settings.CurrentColor);
        DebugDrawLocal(ref sink, settings);
    }

    /// <summary>以 <see cref="SupportMap"/> 所使用的坐标系追加调试三角形。</summary>
    /// <remarks>
    /// 重写此方法以提供自定义表面几何。刚体和仿射变换由调用方
    /// 应用。闭合表面请使用朝外的三角形。
    /// 若未重写，形状将绘制为近似表面。
    /// </remarks>
    /// <param name="triangleSink">生成三角形的目标。</param>
    /// <param name="settings">可视化设置。</param>
    protected internal virtual void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
        where TSink : ISink<JTriangle>
    {
        ShapeHelper.Tessellate(this, ref triangleSink, settings.ShapeSubdivisions);
    }
}
