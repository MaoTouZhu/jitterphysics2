/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using Jitter2.DataStructures;

namespace Jitter2.Dynamics.Constraints;

/// <summary>选择创建关节时附带的可选驱动。</summary>
public enum JointDrive
{
    /// <summary>创建不带驱动的机械关节。</summary>
    None,

    /// <summary>添加驱动相对速度的马达。</summary>
    Motor,

    /// <summary>添加驱动至目标位置或目标角度的伺服。</summary>
    Servo
}

/// <summary>
/// 关节的基类，关节是由多个 <see cref="Constraint"/> 实例组合而成的复合约束。
/// </summary>
public class Joint : IDebugDrawable
{
    private readonly List<Constraint> constraints = new(2);
    public ReadOnlyList<Constraint> Constraints => new(constraints);

    /// <summary>
    /// 将约束加入内部记录。
    /// </summary>
    protected void Register(Constraint constraint) => constraints.Add(constraint);

    /// <summary>
    /// 从内部记录中移除约束。
    /// </summary>
    protected void Deregister(Constraint constraint) => constraints.Remove(constraint);

    /// <summary>
    /// 启用构成该关节的所有约束。
    /// </summary>
    public void Enable()
    {
        foreach (var constraint in constraints)
        {
            if (constraint.Handle.IsZero) continue;
            constraint.IsEnabled = true;
        }
    }

    /// <summary>
    /// 临时禁用构成该关节的所有约束。
    /// 若要彻底移除，请使用 <see cref="Joint.Remove()"/>。
    /// </summary>
    public void Disable()
    {
        foreach (var constraint in constraints)
        {
            if (constraint.Handle.IsZero) continue;
            constraint.IsEnabled = false;
        }
    }

    /// <summary>
    /// 从物理世界中移除构成该关节的所有约束。
    /// </summary>
    public void Remove()
    {
        foreach (var constraint in constraints)
        {
            if (constraint.Handle.IsZero) continue;
            constraint.Body1.World.Remove(constraint);
        }

        constraints.Clear();
    }

    public virtual void DebugDraw(IDebugDrawer drawer)
        => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <summary>绘制各组成约束；在可用时把角度引导绘制在关节锚点处。</summary>
    public virtual void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        foreach (var constraint in constraints)
        {
            if (!constraint.IsValid) continue;
            DebugDrawSettings jointSettings = settings;
            if (!settings.Origin.HasValue)
            {
                foreach (var candidate in constraints)
                {
                    if (candidate is not BallSocket socket || !socket.IsValid) continue;
                    if (socket.Body1 == constraint.Body1 && socket.Body2 == constraint.Body2)
                    {
                        jointSettings = settings with { Origin = socket.Anchor1 };
                        break;
                    }
                    if (socket.Body2 == constraint.Body1 && socket.Body1 == constraint.Body2)
                    {
                        jointSettings = settings with { Origin = socket.Anchor2 };
                        break;
                    }
                }
            }
            constraint.DebugDraw(drawer, jointSettings);
        }
    }
}
