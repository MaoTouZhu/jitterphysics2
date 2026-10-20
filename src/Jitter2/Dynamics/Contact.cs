/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

namespace Jitter2.Dynamics;

/// <summary>
/// 为单次形状对形状的碰撞保存最多四个 <see cref="Contact"/> 结构。
/// </summary>
/// <remarks>
/// <para>
/// 此结构存储在非托管内存中，并通过 <see cref="Arbiter.Handle"/> 访问。
/// <see cref="UsageMask"/> 位域指示四个接触槽中哪些处于活动状态。
/// </para>
/// <para>
/// 仅当关联的仲裁器存在时，返回的数据才有效，且不得与 <see cref="World.Step(Real, bool)"/> 并发访问。
/// 不要缓存对此结构的引用。
/// </para>
/// </remarks>
[StructLayout(LayoutKind.Sequential)]
public struct ContactData
{
    /// <summary>
    /// 指定求解器应为每个刚体更新哪些速度分量。
    /// </summary>
    [Flags]
    public enum SolveMode
    {
        /// <summary>不更新速度。</summary>
        None = 0,
        /// <summary>更新刚体 1 的线速度。</summary>
        LinearBody1 = 1 << 0,
        /// <summary>更新刚体 1 的角速度。</summary>
        AngularBody1 = 1 << 1,
        /// <summary>更新刚体 2 的线速度。</summary>
        LinearBody2 = 1 << 2,
        /// <summary>更新刚体 2 的角速度。</summary>
        AngularBody2 = 1 << 3,
        /// <summary>同时更新刚体 1 的线速度和角速度。</summary>
        FullBody1 = LinearBody1 | AngularBody1,
        /// <summary>同时更新刚体 2 的线速度和角速度。</summary>
        FullBody2 = LinearBody2 | AngularBody2,
        /// <summary>更新两个刚体的线速度。</summary>
        Linear = LinearBody1 | LinearBody2,
        /// <summary>更新两个刚体的角速度。</summary>
        Angular = AngularBody1 | AngularBody2,
        /// <summary>更新两个刚体的所有速度分量。</summary>
        Full = Linear | Angular,
    }

    /// <summary>接触槽 0 的位掩码。</summary>
    public const uint MaskContact0 = 0b0001;
    /// <summary>接触槽 1 的位掩码。</summary>
    public const uint MaskContact1 = 0b0010;
    /// <summary>接触槽 2 的位掩码。</summary>
    public const uint MaskContact2 = 0b0100;
    /// <summary>接触槽 3 的位掩码。</summary>
    public const uint MaskContact3 = 0b1000;

    /// <summary>表示全部四个接触槽都在使用的位掩码。</summary>
    public const uint MaskContactAll = MaskContact0 | MaskContact1 | MaskContact2 | MaskContact3;

    // 在 unsafe 代码中访问。
#pragma warning disable CS0649
    internal int _internal;
#pragma warning restore CS0649

    /// <summary>
    /// 低半字节表示当前哪些接触完好（位已设置）或已断开（位未设置）。
    /// 接下来的半字节表示求解阶段哪些接触完好或已断开。
    /// </summary>
    /// <example>
    /// 球体可能沿斜面下滑。在一个时间步内，Jitter 可能检测到碰撞、创建接触、求解接触、
    /// 对速度和位置进行积分，然后认为该接触已断开，因为沿与接触法线正交方向的移动超过了阈值。
    /// 因此，调用 <see cref="World.Step(Real, bool)"/> 之前和之后都没有完好的接触。
    /// 不过在此情形下，求解阶段的对应位仍会被设置。
    /// </example>
    public uint UsageMask;

    /// <summary>指向第一个刚体模拟数据的句柄。</summary>
    public JHandle<RigidBodyData> Body1;
    /// <summary>指向第二个刚体模拟数据的句柄。</summary>
    public JHandle<RigidBodyData> Body2;

    /// <summary>标识该仲裁器形状对的唯一键。</summary>
    public ArbiterKey Key;

    /// <summary>该接触对的合成恢复系数。</summary>
    public Real Restitution;
    /// <summary>该接触对的合成摩擦系数。</summary>
    public Real Friction;
    /// <summary>推测接触的松弛因子。</summary>
    public Real SpeculativeRelaxationFactor;

    /// <summary>确定求解器更新哪些速度分量。</summary>
    public SolveMode Mode;

    /// <summary>第一个接触点。</summary>
    public Contact Contact0;
    /// <summary>第二个接触点。</summary>
    public Contact Contact1;
    /// <summary>第三个接触点。</summary>
    public Contact Contact2;
    /// <summary>第四个接触点。</summary>
    public Contact Contact3;

    /// <summary>
    /// 通过计算有效质量和热启动冲量，为求解器迭代准备所有活动接触。
    /// </summary>
    /// <param name="timeStep">完整步与子步的计时信息。</param>
    public unsafe void PrepareForIteration(in TimeStep timeStep)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if (Vector.IsHardwareAccelerated)
        {
            if ((UsageMask & MaskContact0) != 0) Contact0.PrepareForIterationAccelerated(ptr, timeStep);
            if ((UsageMask & MaskContact1) != 0) Contact1.PrepareForIterationAccelerated(ptr, timeStep);
            if ((UsageMask & MaskContact2) != 0) Contact2.PrepareForIterationAccelerated(ptr, timeStep);
            if ((UsageMask & MaskContact3) != 0) Contact3.PrepareForIterationAccelerated(ptr, timeStep);
        }
        else
        {
            if ((UsageMask & MaskContact0) != 0) Contact0.PrepareForIteration(ptr, timeStep);
            if ((UsageMask & MaskContact1) != 0) Contact1.PrepareForIteration(ptr, timeStep);
            if ((UsageMask & MaskContact2) != 0) Contact2.PrepareForIteration(ptr, timeStep);
            if ((UsageMask & MaskContact3) != 0) Contact3.PrepareForIteration(ptr, timeStep);
        }
    }

    /// <summary>清除所有活动接触累积的法向冲量和摩擦冲量。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetImpulses()
    {
        var zero = Vector.Create((Real)0.0);
        if ((UsageMask & MaskContact0) != 0) Contact0.Accumulated = zero;
        if ((UsageMask & MaskContact1) != 0) Contact1.Accumulated = zero;
        if ((UsageMask & MaskContact2) != 0) Contact2.Accumulated = zero;
        if ((UsageMask & MaskContact3) != 0) Contact3.Accumulated = zero;
    }

    /// <summary>在每个活动接触的下一次准备阶段跳过重放累积冲量。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SkipWarmStart()
    {
        const Contact.Flags flag = Contact.Flags.SkipWarmStart;
        if ((UsageMask & MaskContact0) != 0) Contact0.Flag |= flag;
        if ((UsageMask & MaskContact1) != 0) Contact1.Flag |= flag;
        if ((UsageMask & MaskContact2) != 0) Contact2.Flag |= flag;
        if ((UsageMask & MaskContact3) != 0) Contact3.Flag |= flag;
    }

    /// <summary>
    /// 对所有活动接触执行一次求解器迭代，施加修正冲量。
    /// </summary>
    /// <param name="applyBias">若为 <see langword="true"/>，则施加位置修正偏置。</param>
    public unsafe void Iterate(bool applyBias)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if (Vector.IsHardwareAccelerated)
        {
            if ((UsageMask & MaskContact0) != 0) Contact0.IterateAccelerated(ptr, applyBias);
            if ((UsageMask & MaskContact1) != 0) Contact1.IterateAccelerated(ptr, applyBias);
            if ((UsageMask & MaskContact2) != 0) Contact2.IterateAccelerated(ptr, applyBias);
            if ((UsageMask & MaskContact3) != 0) Contact3.IterateAccelerated(ptr, applyBias);
        }
        else
        {
            if ((UsageMask & MaskContact0) != 0) Contact0.Iterate(ptr, applyBias);
            if ((UsageMask & MaskContact1) != 0) Contact1.Iterate(ptr, applyBias);
            if ((UsageMask & MaskContact2) != 0) Contact2.Iterate(ptr, applyBias);
            if ((UsageMask & MaskContact3) != 0) Contact3.Iterate(ptr, applyBias);
        }
    }

    internal unsafe void PrepareForIterationAccelerated(in TimeStep timeStep)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if ((UsageMask & MaskContact0) != 0) Contact0.PrepareForIterationAccelerated(ptr, timeStep);
        if ((UsageMask & MaskContact1) != 0) Contact1.PrepareForIterationAccelerated(ptr, timeStep);
        if ((UsageMask & MaskContact2) != 0) Contact2.PrepareForIterationAccelerated(ptr, timeStep);
        if ((UsageMask & MaskContact3) != 0) Contact3.PrepareForIterationAccelerated(ptr, timeStep);
    }

    internal unsafe void PrepareForIterationScalar(in TimeStep timeStep)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if ((UsageMask & MaskContact0) != 0) Contact0.PrepareForIteration(ptr, timeStep);
        if ((UsageMask & MaskContact1) != 0) Contact1.PrepareForIteration(ptr, timeStep);
        if ((UsageMask & MaskContact2) != 0) Contact2.PrepareForIteration(ptr, timeStep);
        if ((UsageMask & MaskContact3) != 0) Contact3.PrepareForIteration(ptr, timeStep);
    }

    internal unsafe void IterateAccelerated(bool applyBias)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if ((UsageMask & MaskContact0) != 0) Contact0.IterateAccelerated(ptr, applyBias);
        if ((UsageMask & MaskContact1) != 0) Contact1.IterateAccelerated(ptr, applyBias);
        if ((UsageMask & MaskContact2) != 0) Contact2.IterateAccelerated(ptr, applyBias);
        if ((UsageMask & MaskContact3) != 0) Contact3.IterateAccelerated(ptr, applyBias);
    }

    internal unsafe void IterateScalar(bool applyBias)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if ((UsageMask & MaskContact0) != 0) Contact0.Iterate(ptr, applyBias);
        if ((UsageMask & MaskContact1) != 0) Contact1.Iterate(ptr, applyBias);
        if ((UsageMask & MaskContact2) != 0) Contact2.Iterate(ptr, applyBias);
        if ((UsageMask & MaskContact3) != 0) Contact3.Iterate(ptr, applyBias);
    }

    /// <summary>
    /// 获取一个值，指示当前系统是否支持 SIMD（单指令多数据）操作的硬件加速。
    /// </summary>
    public static bool IsHardwareAccelerated => Vector.IsHardwareAccelerated;

    /// <summary>
    /// 在积分之后更新接触位置，并移除法向分离超过断开阈值十分之一、
    /// 或切向漂移超过完整阈值的接触。
    /// </summary>
    public unsafe void UpdatePosition()
    {
        UsageMask &= MaskContactAll;
        UsageMask |= UsageMask << 4;

        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if ((UsageMask & MaskContact0) != 0)
        {
            if (!Contact0.UpdatePosition(ptr)) UsageMask &= ~MaskContact0;
        }

        if ((UsageMask & MaskContact1) != 0)
        {
            if (!Contact1.UpdatePosition(ptr)) UsageMask &= ~MaskContact1;
        }

        if ((UsageMask & MaskContact2) != 0)
        {
            if (!Contact2.UpdatePosition(ptr)) UsageMask &= ~MaskContact2;
        }

        if ((UsageMask & MaskContact3) != 0)
        {
            if (!Contact3.UpdatePosition(ptr)) UsageMask &= ~MaskContact3;
        }
    }

    /// <summary>
    /// 为一对刚体初始化此接触数据。
    /// </summary>
    /// <param name="body1">第一个刚体。</param>
    /// <param name="body2">第二个刚体。</param>
    public void Init(RigidBody body1, RigidBody body2)
    {
        Body1 = body1.Handle;
        Body2 = body2.Handle;

        Friction = MathR.Max(body1.Friction, body2.Friction);
        Restitution = MathR.Max(body1.Restitution, body2.Restitution);
        SpeculativeRelaxationFactor = body1.World.SpeculativeRelaxationFactor;

        Debug.Assert(body1.World == body2.World);

        Mode = SolveMode.None;
        UsageMask = 0;
    }

    /// <summary>
    /// 根据刚体当前的运动类型重置求解模式，然后移除指定的标志。
    /// </summary>
    /// <param name="removeFlags">要从完整求解模式中移除的标志。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ResetMode(SolveMode removeFlags = SolveMode.None)
    {
        Mode = (Body1.Data.MotionType == MotionType.Dynamic) ? SolveMode.FullBody1 : 0;
        Mode |= (Body2.Data.MotionType == MotionType.Dynamic) ? SolveMode.FullBody2 : 0;
        Mode &= ~removeFlags;
    }

    // ---------------------------------------------------------------------------------------------------------
    //
    // 以下接触缓存代码在很大程度上参考/直接复制自出色的
    // Bullet 物理引擎：

    //
    // Bullet Continuous Collision Detection and Physics Library Copyright (c) 2003-2006 Erwin
    // Coumans  https://bulletphysics.org

    // This software is provided 'as-is', without any express or implied warranty. In no event will
    // the authors be held liable for any damages arising from the use of this software. Permission
    // is granted to anyone to use this software for any purpose, including commercial applications,
    // and to alter it and redistribute it freely, subject to the following restrictions:

    // 1. The origin of this software must not be misrepresented; you must not claim that you wrote
    //    the original software. If you use this software in a product, an acknowledgment in the
    //    product documentation would be appreciated but is not required.
    // 2. Altered source versions must be plainly marked as such, and must not be misrepresented as
    //    being the original software.
    // 3. This notice may not be removed or altered from any source distribution.
    //
    // https://github.com/bulletphysics/bullet3/blob/39b8de74df93721add193e5b3d9ebee579faebf8/
    // src/Bullet3OpenCL/NarrowphaseCollision/b3ContactCache.cpp

    /// <summary>
    /// 向接触流形添加新的碰撞结果。最多保留四个点。
    /// </summary>
    /// <param name="point1">世界空间中第一个刚体上的接触点。</param>
    /// <param name="point2">世界空间中第二个刚体上的接触点。</param>
    /// <param name="normal">接触法线，方向为求解器对刚体 2 施加正冲量的方向。
    /// 对于重叠的接触点，它从 <paramref name="point2"/> 指向 <paramref name="point1"/>。</param>
    public unsafe void AddContact(in JVector point1, in JVector point2, in JVector normal)
    {
        var ptr = (ContactData*)Unsafe.AsPointer(ref this);

        if ((UsageMask & MaskContactAll) == MaskContactAll)
        {
            // 全部四个接触都在使用中。找出一个可被新接触替换的候选。
            SortCachedPoints(point1, point2, normal);
            return;
        }

        // 并非所有接触都在使用中，但新接触点与已有某个点足够接近。
        // 用新点替换该点。

        Contact* closest = (Contact*)IntPtr.Zero;
        Real distanceSq = Real.MaxValue;

        JVector relP1 = point1 - Body1.Data.Position;

        if ((UsageMask & MaskContact0) != 0)
        {
            Real distSq = (Contact0.RelativePosition1 - relP1).LengthSquared();
            if (distSq < distanceSq)
            {
                distanceSq = distSq;
                closest = (Contact*)Unsafe.AsPointer(ref Contact0);
            }
        }

        if ((UsageMask & MaskContact1) != 0)
        {
            Real distSq = (Contact1.RelativePosition1 - relP1).LengthSquared();
            if (distSq < distanceSq)
            {
                distanceSq = distSq;
                closest = (Contact*)Unsafe.AsPointer(ref Contact1);
            }
        }

        if ((UsageMask & MaskContact2) != 0)
        {
            Real distSq = (Contact2.RelativePosition1 - relP1).LengthSquared();
            if (distSq < distanceSq)
            {
                distanceSq = distSq;
                closest = (Contact*)Unsafe.AsPointer(ref Contact2);
            }
        }

        if ((UsageMask & MaskContact3) != 0)
        {
            Real distSq = (Contact3.RelativePosition1 - relP1).LengthSquared();
            if (distSq < distanceSq)
            {
                distanceSq = distSq;
                closest = (Contact*)Unsafe.AsPointer(ref Contact3);
            }
        }

        if (distanceSq < Contact.BreakThreshold * Contact.BreakThreshold)
        {
            closest->Initialize(ptr, point1, point2, normal, false);
            return;
        }

        // 这是一个全新的接触。

        if ((UsageMask & MaskContact0) == 0)
        {
            Contact0.Initialize(ptr, point1, point2, normal, true);
            UsageMask |= MaskContact0;
        }
        else if ((UsageMask & MaskContact1) == 0)
        {
            Contact1.Initialize(ptr, point1, point2, normal, true);
            UsageMask |= MaskContact1;
        }
        else if ((UsageMask & MaskContact2) == 0)
        {
            Contact2.Initialize(ptr, point1, point2, normal, true);
            UsageMask |= MaskContact2;
        }
        else if ((UsageMask & MaskContact3) == 0)
        {
            Contact3.Initialize(ptr, point1, point2, normal, true);
            UsageMask |= MaskContact3;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real CalcArea4Points(in JVector p0, in JVector p1, in JVector p2, in JVector p3)
    {
        JVector a0 = p0 - p1;
        JVector a1 = p0 - p2;
        JVector a2 = p0 - p3;
        JVector b0 = p2 - p3;
        JVector b1 = p1 - p3;
        JVector b2 = p1 - p2;

        JVector tmp0 = a0 % b0;
        JVector tmp1 = a1 % b1;
        JVector tmp2 = a2 % b2;

        return MathR.Max(MathR.Max(tmp0.LengthSquared(), tmp1.LengthSquared()), tmp2.LengthSquared());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static uint SelectCachedPointReplacementMask(
        Real area0, Real area1, Real area2, Real area3)
    {
        // 始终优先选择新点
        const Real epsilon = -(Real)0.0001;

        Real biggestArea = 0;
        uint mask = 0;

        if (area0 > biggestArea + epsilon)
        {
            biggestArea = area0;
            mask = MaskContact0;
        }

        if (area1 > biggestArea + epsilon)
        {
            biggestArea = area1;
            mask = MaskContact1;
        }

        if (area2 > biggestArea + epsilon)
        {
            biggestArea = area2;
            mask = MaskContact2;
        }

        if (area3 > biggestArea + epsilon)
        {
            mask = MaskContact3;
        }

        return mask;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static uint SelectCachedPointReplacementMaskScalar(
        in JVector newPoint, in JVector contact0, in JVector contact1, in JVector contact2, in JVector contact3)
    {
        Real area0 = CalcArea4Points(newPoint, contact1, contact2, contact3);
        Real area1 = CalcArea4Points(newPoint, contact0, contact2, contact3);
        Real area2 = CalcArea4Points(newPoint, contact0, contact1, contact3);
        Real area3 = CalcArea4Points(newPoint, contact0, contact1, contact2);

        return SelectCachedPointReplacementMask(area0, area1, area2, area3);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static VectorReal CrossLengthSquared(
        VectorReal ax, VectorReal ay, VectorReal az,
        VectorReal bx, VectorReal by, VectorReal bz)
    {
        VectorReal cx = Vector.Subtract(Vector.Multiply(ay, bz), Vector.Multiply(az, by));
        VectorReal cy = Vector.Subtract(Vector.Multiply(az, bx), Vector.Multiply(ax, bz));
        VectorReal cz = Vector.Subtract(Vector.Multiply(ax, by), Vector.Multiply(ay, bx));

        return Vector.Add(Vector.Add(Vector.Multiply(cx, cx), Vector.Multiply(cy, cy)), Vector.Multiply(cz, cz));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static uint SelectCachedPointReplacementMaskVectorized(
        in JVector newPoint, in JVector contact0, in JVector contact1, in JVector contact2, in JVector contact3)
    {
        VectorReal p0X = Vector.Create(newPoint.X);
        VectorReal p0Y = Vector.Create(newPoint.Y);
        VectorReal p0Z = Vector.Create(newPoint.Z);

        VectorReal p1X = Vector.Create(contact1.X, contact0.X, contact0.X, contact0.X);
        VectorReal p1Y = Vector.Create(contact1.Y, contact0.Y, contact0.Y, contact0.Y);
        VectorReal p1Z = Vector.Create(contact1.Z, contact0.Z, contact0.Z, contact0.Z);

        VectorReal p2X = Vector.Create(contact2.X, contact2.X, contact1.X, contact1.X);
        VectorReal p2Y = Vector.Create(contact2.Y, contact2.Y, contact1.Y, contact1.Y);
        VectorReal p2Z = Vector.Create(contact2.Z, contact2.Z, contact1.Z, contact1.Z);

        VectorReal p3X = Vector.Create(contact3.X, contact3.X, contact3.X, contact2.X);
        VectorReal p3Y = Vector.Create(contact3.Y, contact3.Y, contact3.Y, contact2.Y);
        VectorReal p3Z = Vector.Create(contact3.Z, contact3.Z, contact3.Z, contact2.Z);

        VectorReal a0X = Vector.Subtract(p0X, p1X);
        VectorReal a0Y = Vector.Subtract(p0Y, p1Y);
        VectorReal a0Z = Vector.Subtract(p0Z, p1Z);

        VectorReal a1X = Vector.Subtract(p0X, p2X);
        VectorReal a1Y = Vector.Subtract(p0Y, p2Y);
        VectorReal a1Z = Vector.Subtract(p0Z, p2Z);

        VectorReal a2X = Vector.Subtract(p0X, p3X);
        VectorReal a2Y = Vector.Subtract(p0Y, p3Y);
        VectorReal a2Z = Vector.Subtract(p0Z, p3Z);

        VectorReal b0X = Vector.Subtract(p2X, p3X);
        VectorReal b0Y = Vector.Subtract(p2Y, p3Y);
        VectorReal b0Z = Vector.Subtract(p2Z, p3Z);

        VectorReal b1X = Vector.Subtract(p1X, p3X);
        VectorReal b1Y = Vector.Subtract(p1Y, p3Y);
        VectorReal b1Z = Vector.Subtract(p1Z, p3Z);

        VectorReal b2X = Vector.Subtract(p1X, p2X);
        VectorReal b2Y = Vector.Subtract(p1Y, p2Y);
        VectorReal b2Z = Vector.Subtract(p1Z, p2Z);

        VectorReal area0 = CrossLengthSquared(a0X, a0Y, a0Z, b0X, b0Y, b0Z);
        VectorReal area1 = CrossLengthSquared(a1X, a1Y, a1Z, b1X, b1Y, b1Z);
        VectorReal area2 = CrossLengthSquared(a2X, a2Y, a2Z, b2X, b2Y, b2Z);
        VectorReal areas = Vector.Max(Vector.Max(area0, area1), area2);

        return SelectCachedPointReplacementMask(
            areas.GetElement(0), areas.GetElement(1), areas.GetElement(2), areas.GetElement(3));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static uint SelectCachedPointReplacementMask(
        in JVector newPoint, in JVector contact0, in JVector contact1, in JVector contact2, in JVector contact3)
    {
        return SelectCachedPointReplacementMaskVectorized(newPoint, contact0, contact1, contact2, contact3);
    }

    private unsafe void SortCachedPoints(in JVector point1, in JVector point2, in JVector normal)
    {
        JVector.Subtract(point1, Body1.Data.Position, out JVector rp1);

        uint mask = SelectCachedPointReplacementMask(
            rp1, Contact0.RelativePosition1, Contact1.RelativePosition1, Contact2.RelativePosition1, Contact3.RelativePosition1);

        ref Contact cref = ref Contact0;

        if (mask == MaskContact1)
        {
            cref = ref Contact1;
        }
        else if (mask == MaskContact2)
        {
            cref = ref Contact2;
        }
        else if (mask == MaskContact3)
        {
            cref = ref Contact3;
        }

        var ptr = (ContactData*)Unsafe.AsPointer(ref this);
        cref.Initialize(ptr, point1, point2, normal, false);
        UsageMask |= mask;
    }

    // ---------------------------------------------------------------------------------------------------------

    /// <summary>
    /// 表示两个刚体之间的单个接触点。
    /// </summary>
    /// <remarks>
    /// <para>
    /// 此结构对布局敏感，作为 <see cref="ContactData"/> 的一部分存储在非托管内存中。
    /// 它包含几何信息（法线、切线、位置）和求解器状态（累积冲量、偏置）。
    /// </para>
    /// <para>
    /// 不要跨模拟步缓存或保存对此结构的引用。
    /// </para>
    /// </remarks>
    [StructLayout(LayoutKind.Explicit)]
    public struct Contact
    {
        /// <summary>每次迭代施加的最大位置修正偏置。</summary>
        public const Real MaximumBias = (Real)100.0;
        /// <summary>每步修正的穿透比例（Baumgarte 稳定化）。</summary>
        public const Real BiasFactor = (Real)0.2;
        /// <summary>低于该穿透深度时不进行位置修正。</summary>
        public const Real AllowedPenetration = (Real)0.01;
        /// <summary>接触断开前的最大切向漂移；法向分离被限制为该值的十分之一。</summary>
        public const Real BreakThreshold = (Real)0.02;

        /// <summary>
        /// 指示接触状态的标志。
        /// </summary>
        [Flags]
        public enum Flags
        {
            /// <summary>表示该接触是在当前步中创建的。</summary>
            NewContact = 1 << 1,
            /// <summary>仅在下次准备阶段跳过重放累积冲量。</summary>
            SkipWarmStart = 1 << 2,
        }

        /// <summary>当前接触状态标志。</summary>
        [FieldOffset(0)] public Flags Flag;

        /// <summary>用于恢复或推测分离的速度偏置。</summary>
        [FieldOffset(4)] public Real Bias;

        /// <summary>根据穿透深度计算的位置修正偏置。</summary>
        [FieldOffset(4 + 1 * sizeof(Real))] public Real PenaltyBias;

        // ―――――― 以下 4 分量向量相互重叠 ――――――――――――――――――――――――――――――

        [FieldOffset(4 + 2 * sizeof(Real))] internal VectorReal NormalTangentX;

        [FieldOffset(4 + 5 * sizeof(Real))] internal VectorReal NormalTangentY;

        [FieldOffset(4 + 8 * sizeof(Real))] internal VectorReal NormalTangentZ;

        [FieldOffset(4 + 11 * sizeof(Real))] internal VectorReal MassNormalTangent;

        [FieldOffset(4 + 14 * sizeof(Real))] internal VectorReal Accumulated;

        [FieldOffset(4 + 18 * sizeof(Real))]
        [ReferenceFrame(ReferenceFrame.Local)]
        internal JVector Position1;

        [FieldOffset(4 + 21 * sizeof(Real))]
        [ReferenceFrame(ReferenceFrame.Local)]
        internal JVector Position2;

        /// <summary>
        /// 用于施加冲量的接触点相对第一个刚体质心的世界空间偏移。
        /// 在完整模拟步的各子步期间保持不变。
        /// </summary>
        [FieldOffset(4 + 25 * sizeof(Real))]
        [ReferenceFrame(ReferenceFrame.World)] public JVector RelativePosition1;

        /// <summary>
        /// 用于施加冲量的接触点相对第二个刚体质心的世界空间偏移。
        /// 在完整模拟步的各子步期间保持不变。
        /// </summary>
        [FieldOffset(4 + 28 * sizeof(Real))]
        [ReferenceFrame(ReferenceFrame.World)] public JVector RelativePosition2;

        /// <summary>
        /// 归一化的接触方向。正的法向冲量沿此方向推动 <see cref="ContactData.Body2"/>，
        /// 并反向推动 <see cref="ContactData.Body1"/>。对于重叠的接触点，
        /// 法线从刚体 2 上的点指向刚体 1 上的点。
        /// </summary>
        [ReferenceFrame(ReferenceFrame.World)]
        public readonly JVector Normal => new(NormalTangentX.GetElement(0),
            NormalTangentY.GetElement(0), NormalTangentZ.GetElement(0));

        /// <summary>
        /// 接触 <see cref="Normal"/> 的（归一化）切线。创建时若切向相对速度非零，
        /// 则沿其方向；否则为任意切线。缓存的接触会保留其基向量。
        /// </summary>
        [ReferenceFrame(ReferenceFrame.World)]
        public readonly JVector Tangent1 => new(NormalTangentX.GetElement(1),
            NormalTangentY.GetElement(1), NormalTangentZ.GetElement(1));

        /// <summary>
        /// 第二条切线，与 <see cref="Normal"/> 和 <see cref="Tangent1"/> 构成正交基。
        /// </summary>
        [ReferenceFrame(ReferenceFrame.World)]
        public readonly JVector Tangent2 => new JVector(NormalTangentX.GetElement(2),
            NormalTangentY.GetElement(2), NormalTangentZ.GetElement(2));

        /// <summary>
        /// 获取上一个已求解子步累积的法向求解器冲量。
        /// </summary>
        public readonly Real Impulse => Accumulated.GetElement(0);

        /// <summary>
        /// 获取上一个已求解子步在第一切线方向上累积的求解器冲量。
        /// </summary>
        public readonly Real TangentImpulse1 => Accumulated.GetElement(1);

        /// <summary>
        /// 获取上一个已求解子步在第二切线方向上累积的求解器冲量。
        /// </summary>
        public readonly Real TangentImpulse2 => Accumulated.GetElement(2);

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public unsafe void Initialize(ContactData* cd, in JVector point1, in JVector point2,
            in JVector n, bool newContact)
        {
            Debug.Assert(Math.Abs(n.LengthSquared() - (Real)1.0) < 1e-3);

            ref var b1 = ref cd->Body1.Data;
            ref var b2 = ref cd->Body2.Data;

            JVector.Subtract(point1, b1.Position, out RelativePosition1);
            JVector.Subtract(point2, b2.Position, out RelativePosition2);
            JVector.ConjugatedTransform(RelativePosition1, b1.Orientation, out Position1);
            JVector.ConjugatedTransform(RelativePosition2, b2.Orientation, out Position2);

            // 材质属性
            if (!newContact) return;

            Flag = Flags.NewContact;
            Accumulated = Vector.Create((Real)0.0);

            // 仅线性的接触在恢复和摩擦基中也会忽略旋转。
            JVector dv = b2.Velocity - b1.Velocity;
            if (cd->Mode == SolveMode.None || (cd->Mode & SolveMode.Angular) != 0)
            {
                dv = b2.Velocity + b2.AngularVelocity % RelativePosition2;
                dv -= b1.Velocity + b1.AngularVelocity % RelativePosition1;
            }

            Real relNormalVel = JVector.Dot(dv, n);

            Bias = 0;

            // 模拟恢复
            if (relNormalVel < (Real)(-1.0))
            {
                Bias = -cd->Restitution * relNormalVel;
            }

            var tangent1 = dv - n * relNormalVel;

            Real num = tangent1.LengthSquared();

            if (num > (Real)1e-12)
            {
                num = (Real)1.0 / MathR.Sqrt(num);
                tangent1 *= num;
            }
            else
            {
                tangent1 = MathHelper.CreateOrthonormal(n);
            }

            var tangent2 = tangent1 % n;

            NormalTangentX = Vector.Create(n.X, tangent1.X, tangent2.X, 0);
            NormalTangentY = Vector.Create(n.Y, tangent1.Y, tangent2.Y, 0);
            NormalTangentZ = Vector.Create(n.Z, tangent1.Z, tangent2.Z, 0);
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public unsafe bool UpdatePosition(ContactData* cd)
        {
            ref var b1 = ref cd->Body1.Data;
            ref var b2 = ref cd->Body2.Data;

            // 求解已结束。刷新偏移，以便在下一步中进行接触匹配。
            JVector.Transform(Position1, b1.Orientation, out RelativePosition1);
            JVector.Add(RelativePosition1, b1.Position, out JVector p1);

            JVector.Transform(Position2, b2.Orientation, out RelativePosition2);
            JVector.Add(RelativePosition2, b2.Position, out JVector p2);

            JVector.Subtract(p1, p2, out JVector dist);

            JVector n = new JVector(NormalTangentX.GetElement(0),
                NormalTangentY.GetElement(0),
                NormalTangentZ.GetElement(0));

            Real penetration = JVector.Dot(dist, n);

            // 此处使用实际阈值值的十分之一。
            if (penetration < -BreakThreshold * (Real)0.1)
            {
                return false;
            }

            dist -= penetration * n;
            Real tangentialOffsetSq = dist.LengthSquared();

            if (tangentialOffsetSq > BreakThreshold * BreakThreshold)
            {
                return false;
            }

            return true;
        }

        // 缺少硬件加速时的回退实现
        #region public unsafe void PrepareForIteration(ContactData* cd, in TimeStep timeStep)

        public unsafe void PrepareForIteration(ContactData* cd, in TimeStep timeStep)
        {
            // 开始从 VectorReal 读取
            Real accumulatedNormalImpulse = Accumulated.GetElement(0);
            Real accumulatedTangentImpulse1 = Accumulated.GetElement(1);
            Real accumulatedTangentImpulse2 = Accumulated.GetElement(2);

            bool skipWarmStart = (Flag & Flags.SkipWarmStart) != 0;
            Flag &= ~Flags.SkipWarmStart;
            if (skipWarmStart)
            {
                // 为 Iterate 保留 Accumulated；只是不将其重放到速度中。
                accumulatedNormalImpulse = 0;
                accumulatedTangentImpulse1 = 0;
                accumulatedTangentImpulse2 = 0;
            }

            ref var b1 = ref cd->Body1.Data;
            ref var b2 = ref cd->Body2.Data;
            JVector normal = Normal;
            JVector tangent1 = Tangent1;
            JVector tangent2 = Tangent2;

            JVector.Transform(Position1, b1.Orientation, out JVector currentRelativePosition1);
            JVector.Transform(Position2, b2.Orientation, out JVector currentRelativePosition2);

            JVector inverseMassVector = JVector.Zero;
            if ((cd->Mode & SolveMode.LinearBody1) != 0) inverseMassVector = b1.InverseMassVector;
            if ((cd->Mode & SolveMode.LinearBody2) != 0) inverseMassVector += b2.InverseMassVector;
            Real inverseMass = inverseMassVector.X;

            if ((Flag & Flags.NewContact) == 0)
            {
                Bias = 0;
            }

            JVector.Add(currentRelativePosition1, b1.Position, out JVector p1);
            JVector.Add(currentRelativePosition2, b2.Position, out JVector p2);
            JVector.Subtract(p1, p2, out JVector dist);
            Real penetration = JVector.Dot(dist, normal);

            if (penetration < -BreakThreshold)
            {
                // 推测接触
                Bias = penetration * timeStep.InverseStepDt * cd->SpeculativeRelaxationFactor;
            }

            Flag &= ~Flags.NewContact;

            Real kTangent1 = inverseMass;
            Real kTangent2 = inverseMass;
            Real kNormal = inverseMass;
            if (b1.HasLinearLocks || b2.HasLinearLocks)
            {
                kNormal = JVector.Multiply(inverseMassVector, normal) * normal;
                kTangent1 = JVector.Multiply(inverseMassVector, tangent1) * tangent1;
                kTangent2 = JVector.Multiply(inverseMassVector, tangent2) * tangent2;
            }
            JVector angularNormalContribution = JVector.Zero;
            JVector angularTangent1Contribution = JVector.Zero;
            JVector angularTangent2Contribution = JVector.Zero;

            JVector impulse = accumulatedNormalImpulse * normal;
            impulse += accumulatedTangentImpulse1 * tangent1;
            impulse += accumulatedTangentImpulse2 * tangent2;

            if ((cd->Mode & SolveMode.LinearBody1) != 0)
            {
                b1.Velocity -= JVector.Multiply(impulse, b1.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody1) != 0)
            {
                JVector angularNormal = TransformSymmetricInertia(RelativePosition1 % normal, b1.InverseInertiaWorld);
                JVector angularTangent1 = TransformSymmetricInertia(RelativePosition1 % tangent1, b1.InverseInertiaWorld);
                JVector angularTangent2 = TransformSymmetricInertia(RelativePosition1 % tangent2, b1.InverseInertiaWorld);

                angularNormalContribution += angularNormal % RelativePosition1;
                angularTangent1Contribution += angularTangent1 % RelativePosition1;
                angularTangent2Contribution += angularTangent2 % RelativePosition1;

                JVector angularImpulse = accumulatedNormalImpulse * angularNormal;
                angularImpulse += accumulatedTangentImpulse1 * angularTangent1;
                angularImpulse += accumulatedTangentImpulse2 * angularTangent2;
                b1.AngularVelocity -= angularImpulse;
            }


            if ((cd->Mode & SolveMode.LinearBody2) != 0)
            {
                b2.Velocity += JVector.Multiply(impulse, b2.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody2) != 0)
            {
                JVector angularNormal = TransformSymmetricInertia(RelativePosition2 % normal, b2.InverseInertiaWorld);
                JVector angularTangent1 = TransformSymmetricInertia(RelativePosition2 % tangent1, b2.InverseInertiaWorld);
                JVector angularTangent2 = TransformSymmetricInertia(RelativePosition2 % tangent2, b2.InverseInertiaWorld);

                angularNormalContribution += angularNormal % RelativePosition2;
                angularTangent1Contribution += angularTangent1 % RelativePosition2;
                angularTangent2Contribution += angularTangent2 % RelativePosition2;

                JVector angularImpulse = accumulatedNormalImpulse * angularNormal;
                angularImpulse += accumulatedTangentImpulse1 * angularTangent1;
                angularImpulse += accumulatedTangentImpulse2 * angularTangent2;
                b2.AngularVelocity += angularImpulse;
            }

            kNormal += JVector.Dot(normal, angularNormalContribution);
            kTangent1 += JVector.Dot(tangent1, angularTangent1Contribution);
            kTangent2 += JVector.Dot(tangent2, angularTangent2Contribution);

            Real massTangent1 = kTangent1 > 0 ? (Real)1.0 / kTangent1 : 0;
            Real massTangent2 = kTangent2 > 0 ? (Real)1.0 / kTangent2 : 0;
            Real massNormal = kNormal > 0 ? (Real)1.0 / kNormal : 0;

            JVector mass = new(massNormal, massTangent1, massTangent2);
            Unsafe.CopyBlock(Unsafe.AsPointer(ref MassNormalTangent), Unsafe.AsPointer(ref mass), 3 * sizeof(Real));

            PenaltyBias = BiasFactor * timeStep.InverseStepDt * Math.Max((Real)0.0, penetration - AllowedPenetration);
            PenaltyBias = Math.Min(PenaltyBias, MaximumBias);
        }

        #endregion

        // 缺少硬件加速时的回退实现
        #region public unsafe void Iterate(ContactData* cd, bool applyBias)

        public unsafe void Iterate(ContactData* cd, bool applyBias)
        {
            // 开始从 VectorReal 读取
            Real massNormal = MassNormalTangent.GetElement(0);
            Real massTangent1 = MassNormalTangent.GetElement(1);
            Real massTangent2 = MassNormalTangent.GetElement(2);
            Real accumulatedNormalImpulse = Accumulated.GetElement(0);
            Real accumulatedTangentImpulse1 = Accumulated.GetElement(1);
            Real accumulatedTangentImpulse2 = Accumulated.GetElement(2);

            ref var b1 = ref cd->Body1.Data;
            ref var b2 = ref cd->Body2.Data;
            JVector normal = Normal;
            JVector tangent1 = Tangent1;
            JVector tangent2 = Tangent2;

            JVector dv = b2.Velocity - b1.Velocity;
            if ((cd->Mode & SolveMode.Angular) != 0)
            {
                dv = b2.Velocity + b2.AngularVelocity % RelativePosition2;
                dv -= b1.Velocity + b1.AngularVelocity % RelativePosition1;
            }

            Real bias = Bias;
            if (applyBias && PenaltyBias > 0)
            {
                bias = MathR.Max(PenaltyBias, Bias);
            }

            Real vn = JVector.Dot(normal, dv);
            Real vt1 = JVector.Dot(tangent1, dv);
            Real vt2 = JVector.Dot(tangent2, dv);

            Real normalImpulse = -vn + bias;

            normalImpulse *= massNormal;

            Real oldNormalImpulse = accumulatedNormalImpulse;
            accumulatedNormalImpulse = MathR.Max(oldNormalImpulse + normalImpulse, (Real)0.0);
            normalImpulse = accumulatedNormalImpulse - oldNormalImpulse;

            // 摩擦锥使用更新前的法向冲量，以与 SIMD 路径保持一致：
            // 在 SIMD 路径中这能避免破坏向量化钳制。
            Real maxTangentImpulse = cd->Friction * oldNormalImpulse;
            Real tangentImpulse1 = massTangent1 * -vt1;
            Real tangentImpulse2 = massTangent2 * -vt2;

            Real oldTangentImpulse1 = accumulatedTangentImpulse1;
            accumulatedTangentImpulse1 = oldTangentImpulse1 + tangentImpulse1;
            accumulatedTangentImpulse1 = Math.Clamp(accumulatedTangentImpulse1, -maxTangentImpulse, maxTangentImpulse);
            tangentImpulse1 = accumulatedTangentImpulse1 - oldTangentImpulse1;

            Real oldTangentImpulse2 = accumulatedTangentImpulse2;
            accumulatedTangentImpulse2 = oldTangentImpulse2 + tangentImpulse2;
            accumulatedTangentImpulse2 = Math.Clamp(accumulatedTangentImpulse2, -maxTangentImpulse, maxTangentImpulse);
            tangentImpulse2 = accumulatedTangentImpulse2 - oldTangentImpulse2;

            Accumulated = Vector.Create(accumulatedNormalImpulse, accumulatedTangentImpulse1, accumulatedTangentImpulse2, 0);

            JVector impulse = normalImpulse * normal;
            impulse += tangentImpulse1 * tangent1;
            impulse += tangentImpulse2 * tangent2;

            if ((cd->Mode & SolveMode.LinearBody1) != 0)
            {
                b1.Velocity -= JVector.Multiply(impulse, b1.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody1) != 0)
            {
                JVector angularImpulse = TransformSymmetricInertia(
                    RelativePosition1 % impulse, b1.InverseInertiaWorld);
                b1.AngularVelocity -= angularImpulse;
            }

            if ((cd->Mode & SolveMode.LinearBody2) != 0)
            {
                b2.Velocity += JVector.Multiply(impulse, b2.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody2) != 0)
            {
                JVector angularImpulse = TransformSymmetricInertia(
                    RelativePosition2 % impulse, b2.InverseInertiaWorld);
                b2.AngularVelocity += angularImpulse;
            }

        }

        #endregion

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Real GetSum3(VectorReal vector)
        {
            return vector.GetElement(0) + vector.GetElement(1) + vector.GetElement(2);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static JVector TransformSymmetricInertia(in JVector vector, in JSymmetricMatrix matrix)
        {
            Unsafe.SkipInit(out JVector result);
            result.X = vector.X * matrix.M11 + vector.Y * matrix.M21 + vector.Z * matrix.M31;
            result.Y = vector.X * matrix.M21 + vector.Y * matrix.M22 + vector.Z * matrix.M23;
            result.Z = vector.X * matrix.M31 + vector.Y * matrix.M23 + vector.Z * matrix.M33;
            return result;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public unsafe void PrepareForIterationAccelerated(ContactData* cd, in TimeStep timeStep)
        {
            bool skipWarmStart = (Flag & Flags.SkipWarmStart) != 0;
            Flag &= ~Flags.SkipWarmStart;
            var warmStartImpulse = skipWarmStart ? Vector.Create((Real)0.0) : Accumulated;

            ref var b1 = ref cd->Body1.Data;
            ref var b2 = ref cd->Body2.Data;

            JVector.Transform(Position1, b1.Orientation, out JVector currentRelativePosition1);
            JVector.Transform(Position2, b2.Orientation, out JVector currentRelativePosition2);

            JVector inverseMassVector = JVector.Zero;
            if ((cd->Mode & SolveMode.LinearBody1) != 0) inverseMassVector = b1.InverseMassVector;
            if ((cd->Mode & SolveMode.LinearBody2) != 0) inverseMassVector += b2.InverseMassVector;
            VectorReal kNormalTangent = Vector.Create(inverseMassVector.X);
            if (b1.HasLinearLocks || b2.HasLinearLocks)
            {
                var kx = Vector.Multiply(Vector.Multiply(NormalTangentX, Vector.Create(inverseMassVector.X)), NormalTangentX);
                var ky = Vector.Multiply(Vector.Multiply(NormalTangentY, Vector.Create(inverseMassVector.Y)), NormalTangentY);
                var kz = Vector.Multiply(Vector.Multiply(NormalTangentZ, Vector.Create(inverseMassVector.Z)), NormalTangentZ);
                kNormalTangent = Vector.Add(Vector.Add(kx, ky), kz);
            }

            if ((Flag & Flags.NewContact) == 0)
            {
                Bias = 0;
            }

            JVector.Add(currentRelativePosition1, b1.Position, out JVector r1);
            JVector.Add(currentRelativePosition2, b2.Position, out JVector r2);
            JVector.Subtract(r1, r2, out JVector dist);

            JVector n = new JVector(NormalTangentX.GetElement(0), NormalTangentY.GetElement(0), NormalTangentZ.GetElement(0));
            Real penetration = JVector.Dot(dist, n);

            if (penetration < -BreakThreshold)
            {
                // 推测接触
                Bias = penetration * timeStep.InverseStepDt * cd->SpeculativeRelaxationFactor;
            }

            Flag &= ~Flags.NewContact;

            var ktnx = Vector.Create((Real)0.0);
            var ktny = Vector.Create((Real)0.0);
            var ktnz = Vector.Create((Real)0.0);

            // 热启动，线性部分
            Unsafe.SkipInit(out JVector linearImpulse);
            linearImpulse.X = GetSum3(Vector.Multiply(warmStartImpulse, NormalTangentX));
            linearImpulse.Y = GetSum3(Vector.Multiply(warmStartImpulse, NormalTangentY));
            linearImpulse.Z = GetSum3(Vector.Multiply(warmStartImpulse, NormalTangentZ));

            if ((cd->Mode & SolveMode.LinearBody1) != 0)
            {
                b1.Velocity -= JVector.Multiply(linearImpulse, b1.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody1) != 0)
            {
                var rp1X = Vector.Create(RelativePosition1.X);
                var rp1Y = Vector.Create(RelativePosition1.Y);
                var rp1Z = Vector.Create(RelativePosition1.Z);

                var rrx = Vector.Subtract(Vector.Multiply(rp1Y, NormalTangentZ), Vector.Multiply(rp1Z, NormalTangentY));
                var rry = Vector.Subtract(Vector.Multiply(rp1Z, NormalTangentX), Vector.Multiply(rp1X, NormalTangentZ));
                var rrz = Vector.Subtract(Vector.Multiply(rp1X, NormalTangentY), Vector.Multiply(rp1Y, NormalTangentX));

                var ixx = Vector.Create(b1.InverseInertiaWorld.M11);
                var ixy = Vector.Create(b1.InverseInertiaWorld.M21);
                var ixz = Vector.Create(b1.InverseInertiaWorld.M31);
                var iyy = Vector.Create(b1.InverseInertiaWorld.M22);
                var iyz = Vector.Create(b1.InverseInertiaWorld.M23);
                var izz = Vector.Create(b1.InverseInertiaWorld.M33);

                var e1 = Vector.Add(Vector.Add(Vector.Multiply(ixx, rrx), Vector.Multiply(ixy, rry)), Vector.Multiply(ixz, rrz));
                var e2 = Vector.Add(Vector.Add(Vector.Multiply(ixy, rrx), Vector.Multiply(iyy, rry)), Vector.Multiply(iyz, rrz));
                var e3 = Vector.Add(Vector.Add(Vector.Multiply(ixz, rrx), Vector.Multiply(iyz, rry)), Vector.Multiply(izz, rrz));

                Unsafe.SkipInit(out JVector angularImpulse1);
                angularImpulse1.X = GetSum3(Vector.Multiply(warmStartImpulse, e1));
                angularImpulse1.Y = GetSum3(Vector.Multiply(warmStartImpulse, e2));
                angularImpulse1.Z = GetSum3(Vector.Multiply(warmStartImpulse, e3));

                b1.AngularVelocity -= angularImpulse1;

                ktnx = Vector.Subtract(Vector.Multiply(e2, rp1Z), Vector.Multiply(e3, rp1Y));
                ktny = Vector.Subtract(Vector.Multiply(e3, rp1X), Vector.Multiply(e1, rp1Z));
                ktnz = Vector.Subtract(Vector.Multiply(e1, rp1Y), Vector.Multiply(e2, rp1X));
            }

            if ((cd->Mode & SolveMode.LinearBody2) != 0)
            {
                b2.Velocity += JVector.Multiply(linearImpulse, b2.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody2) != 0)
            {
                var rp2X = Vector.Create(RelativePosition2.X);
                var rp2Y = Vector.Create(RelativePosition2.Y);
                var rp2Z = Vector.Create(RelativePosition2.Z);

                var rrx = Vector.Subtract(Vector.Multiply(rp2Y, NormalTangentZ), Vector.Multiply(rp2Z, NormalTangentY));
                var rry = Vector.Subtract(Vector.Multiply(rp2Z, NormalTangentX), Vector.Multiply(rp2X, NormalTangentZ));
                var rrz = Vector.Subtract(Vector.Multiply(rp2X, NormalTangentY), Vector.Multiply(rp2Y, NormalTangentX));

                var ixx = Vector.Create(b2.InverseInertiaWorld.M11);
                var ixy = Vector.Create(b2.InverseInertiaWorld.M21);
                var ixz = Vector.Create(b2.InverseInertiaWorld.M31);
                var iyy = Vector.Create(b2.InverseInertiaWorld.M22);
                var iyz = Vector.Create(b2.InverseInertiaWorld.M23);
                var izz = Vector.Create(b2.InverseInertiaWorld.M33);

                var f1 = Vector.Add(Vector.Add(Vector.Multiply(ixx, rrx), Vector.Multiply(ixy, rry)), Vector.Multiply(ixz, rrz));
                var f2 = Vector.Add(Vector.Add(Vector.Multiply(ixy, rrx), Vector.Multiply(iyy, rry)), Vector.Multiply(iyz, rrz));
                var f3 = Vector.Add(Vector.Add(Vector.Multiply(ixz, rrx), Vector.Multiply(iyz, rry)), Vector.Multiply(izz, rrz));

                Unsafe.SkipInit(out JVector angularImpulse2);
                angularImpulse2.X = GetSum3(Vector.Multiply(warmStartImpulse, f1));
                angularImpulse2.Y = GetSum3(Vector.Multiply(warmStartImpulse, f2));
                angularImpulse2.Z = GetSum3(Vector.Multiply(warmStartImpulse, f3));

                b2.AngularVelocity += angularImpulse2;

                ktnx = Vector.Add(ktnx, Vector.Subtract(Vector.Multiply(f2, rp2Z), Vector.Multiply(f3, rp2Y)));
                ktny = Vector.Add(ktny, Vector.Subtract(Vector.Multiply(f3, rp2X), Vector.Multiply(f1, rp2Z)));
                ktnz = Vector.Add(ktnz, Vector.Subtract(Vector.Multiply(f1, rp2Y), Vector.Multiply(f2, rp2X)));
            }

            var kres = Vector.Add(Vector.Add(Vector.Multiply(NormalTangentX, ktnx),
                Vector.Multiply(NormalTangentY, ktny)), Vector.Multiply(NormalTangentZ, ktnz));

            kNormalTangent = Vector.Add(kNormalTangent, kres);

            var mnt = Vector.ConditionalSelect(Vector.GreaterThan(kNormalTangent, Vector.Create((Real)0.0)),
                Vector.Divide(Vector.Create((Real)1.0), kNormalTangent), Vector.Create((Real)0.0));
            Unsafe.CopyBlock(Unsafe.AsPointer(ref MassNormalTangent), Unsafe.AsPointer(ref mnt), 3 * sizeof(Real));

            PenaltyBias = BiasFactor * timeStep.InverseStepDt * Math.Max((Real)0.0, penetration - AllowedPenetration);
            PenaltyBias = Math.Min(PenaltyBias, MaximumBias);
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public unsafe void IterateAccelerated(ContactData* cd, bool applyBias)
        {
            ref var b1 = ref cd->Body1.Data;
            ref var b2 = ref cd->Body2.Data;

            JVector dv = b2.Velocity - b1.Velocity;
            if ((cd->Mode & SolveMode.Angular) != 0)
            {
                dv = b2.Velocity + b2.AngularVelocity % RelativePosition2;
                dv -= b1.Velocity + b1.AngularVelocity % RelativePosition1;
            }

            Real bias = Bias;
            if (applyBias && PenaltyBias > 0)
            {
                bias = MathR.Max(PenaltyBias, Bias);
            }

            var vdots = Vector.Add(Vector.Add(Vector.Multiply(NormalTangentX, Vector.Create(dv.X)),
                Vector.Multiply(NormalTangentY, Vector.Create(dv.Y))), Vector.Multiply(NormalTangentZ, Vector.Create(dv.Z)));

            var impulse = Vector.Multiply(MassNormalTangent, (Vector.Subtract(Vector.Create(bias, 0, 0, 0), vdots)));
            var oldImpulse = Accumulated;

            // 摩擦锥使用更新前的法向冲量；这样可保持向量化钳制简单，
            // 并与标量路径一致。
            Real maxTangentImpulse = cd->Friction * Accumulated.GetElement(0);

            Accumulated = Vector.Add(oldImpulse, impulse);

            var minImpulse = Vector.Create(0, -maxTangentImpulse, -maxTangentImpulse, 0);
            var maxImpulse = Vector.Create(Real.MaxValue, maxTangentImpulse, maxTangentImpulse, 0);

            Accumulated = Vector.Min(Vector.Max(Accumulated, minImpulse), maxImpulse);
            impulse = Vector.Subtract(Accumulated, oldImpulse);

            Unsafe.SkipInit(out JVector linearImpulse);
            linearImpulse.X = GetSum3(Vector.Multiply(impulse, NormalTangentX));
            linearImpulse.Y = GetSum3(Vector.Multiply(impulse, NormalTangentY));
            linearImpulse.Z = GetSum3(Vector.Multiply(impulse, NormalTangentZ));

            if ((cd->Mode & SolveMode.LinearBody1) != 0)
            {
                b1.Velocity -= JVector.Multiply(linearImpulse, b1.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody1) != 0)
            {
                JVector angularImpulse = TransformSymmetricInertia(
                    RelativePosition1 % linearImpulse, b1.InverseInertiaWorld);
                b1.AngularVelocity -= angularImpulse;
            }

            if ((cd->Mode & SolveMode.LinearBody2) != 0)
            {
                b2.Velocity += JVector.Multiply(linearImpulse, b2.InverseMassVector);
            }

            if ((cd->Mode & SolveMode.AngularBody2) != 0)
            {
                JVector angularImpulse = TransformSymmetricInertia(
                    RelativePosition2 % linearImpulse, b2.InverseInertiaWorld);
                b2.AngularVelocity += angularImpulse;
            }

        }
    }
}
