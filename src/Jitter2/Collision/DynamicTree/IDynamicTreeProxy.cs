/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.DataStructures;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 表示可由 <see cref="DynamicTree"/> 跟踪以进行宽相碰撞检测的实体。
/// </summary>
public interface IDynamicTreeProxy : IPartitionedSetIndex
{
    /// <summary>
    /// 获取或设置树所使用的内部节点指针。
    /// </summary>
    /// <remarks>
    /// 该值由 <see cref="DynamicTree"/> 内部管理。请勿直接修改。
    /// </remarks>
    int NodePtr { get; set; }

    /// <summary>
    /// 获取实体的速度，用于扩展包围盒。
    /// </summary>
    JVector Velocity { get; }

    /// <summary>
    /// 获取实体在世界空间中的轴对齐包围盒。
    /// </summary>
    JBoundingBox WorldBoundingBox { get; }
}

/// <summary>
/// 表示可重新计算包围盒的实体。
/// </summary>
public interface IUpdatableBoundingBox
{
    /// <summary>
    /// 重新计算世界空间中的包围盒。
    /// </summary>
    /// <param name="dt">基于速度扩展所用的时间步。默认为零。</param>
    public void UpdateWorldBoundingBox(Real dt = (Real)0.0);
}

/// <summary>
/// 表示可被射线相交的实体。
/// </summary>
public interface IRayCastable
{
    /// <summary>
    /// 对该对象执行射线投射。
    /// </summary>
    /// <param name="origin">射线的起点。</param>
    /// <param name="direction">射线的方向。无需归一化。</param>
    /// <param name="normal">
    /// 交点处的表面法线；若射线起点位于对象内部，则为 <see cref="JVector.Zero"/>。
    /// 请使用返回值而非该参数来判断是否发生命中。
    /// </param>
    /// <param name="lambda">
    /// 沿射线到交点的距离：<c>hitPoint = origin + lambda * direction</c>。
    /// 当射线起点位于对象内部时为零。
    /// </param>
    /// <returns>
    /// 若射线与该对象相交则为 <c>true</c>；否则为 <c>false</c>。
    /// 当射线起点位于对象内部时同样返回 <c>true</c>，此时 <paramref name="lambda"/>
    /// 为零，<paramref name="normal"/> 为 <see cref="JVector.Zero"/>。
    /// </returns>
    public bool RayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda);
}

/// <summary>
/// 表示支持针对带支撑映射的查询形状执行最近点距离查询的实体。
/// </summary>
public interface IDistanceTestable
{
    /// <summary>
    /// 查找该对象与查询形状之间的最近点。
    /// </summary>
    /// <typeparam name="T">查询支撑映射的类型。</typeparam>
    /// <param name="support">查询形状。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="pointA">
    /// 查询形状在世界空间中的最近点。当形状重叠时无定义。
    /// </param>
    /// <param name="pointB">
    /// 该对象在世界空间中的最近点。当形状重叠时无定义。
    /// </param>
    /// <param name="normal">
    /// 从查询形状指向该对象的单位方向；当形状重叠时为 <see cref="JVector.Zero"/>。
    /// 请勿用它来判断是否找到了结果。
    /// </param>
    /// <param name="distance">形状之间的分离距离。重叠时为零。</param>
    /// <returns>若形状相互分离则为 <c>true</c>；若重叠则为 <c>false</c>。</returns>
    public bool Distance<T>(in T support, in JQuaternion orientation, in JVector position,
        out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
        where T : ISupportMappable;
}

/// <summary>
/// 表示可针对移动的、带支撑映射的查询形状执行扫掠测试的实体。
/// </summary>
public interface ISweepTestable
{
    /// <summary>
    /// 对该对象执行扫掠测试。
    /// </summary>
    /// <typeparam name="T">查询支撑映射的类型。</typeparam>
    /// <param name="support">查询形状。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="sweep">查询形状在世界空间中的平移量。</param>
    /// <param name="pointA">在扫掠起点处，查询形状在世界空间中的碰撞点。当形状已经重叠时无定义。</param>
    /// <param name="pointB">在扫掠起点处，该对象在世界空间中的碰撞点。当形状已经重叠时无定义。</param>
    /// <param name="normal">
    /// 世界空间中的碰撞法线；若形状已经重叠，则为 <see cref="JVector.Zero"/>。
    /// 请使用返回值判断是否发生命中；不要依赖该值非零。
    /// </param>
    /// <param name="lambda">以 <paramref name="sweep"/> 为单位表示的碰撞时间。若形状已经重叠则为零。</param>
    /// <returns>若查询形状命中该对象或已与其重叠则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public bool Sweep<T>(in T support, in JQuaternion orientation, in JVector position, in JVector sweep,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where T : ISupportMappable;
}
