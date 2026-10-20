/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;

namespace Jitter2.SoftBodies;

/// <summary>
/// 表示物理模拟中的软体。软体由顶点（刚体）、
/// 弹簧（约束）和形状组成。
/// </summary>
public class SoftBody
{
    /// <summary>
    /// 获取构成软体的顶点（刚体）列表。
    /// </summary>
    public List<RigidBody> Vertices { get; } = [];

    /// <summary>
    /// 获取连接软体各顶点的弹簧（约束）列表。
    /// </summary>
    public List<Constraint> Springs { get; } = [];

    /// <summary>
    /// 获取定义软体几何形状的形状列表。
    /// </summary>
    public List<SoftBodyShape> Shapes { get; } = [];

    /// <summary>
    /// 获取软体所在的世界。
    /// </summary>
    public World World { get; }

    /// <summary>
    /// 获取一个值，指示软体是否处于活动状态。当软体的第一个顶点仍然有效且活动时，
    /// 该软体被视为活动状态。
    /// </summary>
    public bool IsActive => Vertices.Count > 0 && Vertices[0].IsValid && Vertices[0].IsActive;

    /// <summary>
    /// 初始化 <see cref="SoftBody"/> 类的新实例。
    /// </summary>
    /// <param name="world">将创建软体的世界。</param>
    public SoftBody(World world)
    {
        World = world;
        world.PostStep += WorldOnPostStep;
    }

    /// <summary>
    /// 向软体添加一个形状，并将其注册到世界的动态树中。
    /// </summary>
    /// <param name="shape">要添加的形状。</param>
    public void AddShape(SoftBodyShape shape)
    {
        World.DynamicTree.AddProxy(shape);
        Shapes.Add(shape);
    }

    /// <summary>
    /// 向软体添加一个弹簧（约束）。
    /// </summary>
    /// <param name="constraint">要添加的约束。</param>
    public void AddSpring(Constraint constraint)
    {
        Springs.Add(constraint);
    }

    /// <summary>
    /// 销毁软体，将其所有组件从模拟世界中移除。
    /// 可以安全地多次调用，包括在世界被清空或释放之后。
    /// </summary>
    public virtual void Destroy()
    {
        World.PostStep -= WorldOnPostStep;

        if (!World.IsDisposed)
        {
            foreach (var shape in Shapes)
            {
                if (World.DynamicTree.Proxies.Contains(shape)) World.DynamicTree.RemoveProxy(shape);
            }

            foreach (var spring in Springs)
            {
                if (spring.IsValid) World.Remove(spring);
            }

            foreach (var point in Vertices)
            {
                if (point.IsValid) World.Remove(point);
            }
        }

        Shapes.Clear();
        Springs.Clear();
        Vertices.Clear();
        active = false;
    }

    private bool active = true;

    /// <summary>
    /// 在每个世界步之后调用，以更新软体各形状的活动状态。
    /// </summary>
    /// <param name="timeStep">完整步和子步的计时信息。</param>
    protected virtual void WorldOnPostStep(TimeStep timeStep)
    {
        // World.Clear 移除顶点时不会经过 SoftBody.Destroy。
        if (Vertices.Count > 0 && !Vertices[0].IsValid)
        {
            Destroy();
            return;
        }

        if (IsActive == active) return;
        active = IsActive;

        foreach (var shape in Shapes)
        {
            if (active)
            {
                World.DynamicTree.ActivateProxy(shape);
            }
            else
            {
                World.DynamicTree.DeactivateProxy(shape);
            }
        }
    }
}
