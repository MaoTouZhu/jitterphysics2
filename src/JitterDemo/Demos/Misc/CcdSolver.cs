using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace JitterDemo;


/// <summary>
/// 迭代式推测接触发现（Iterative Speculative Contact Discovery，ISCD）的一个简单实现。
/// 这更像是一个概念验证，而非可用于生产的方案。它不支持多线程，
/// 求解器效率也不高。对于极少量形状（约 10 个）效果良好。
/// </summary>
public class CcdSolver
{
    private const int SelfConsistencyIterations = 4;

    private readonly World world;

    private readonly List<RigidBody> bodies = new();
    private readonly List<IDynamicTreeProxy> overlaps = new();
    private readonly List<Arbiter> discoveredArbiters = new();

    public CcdSolver(World world)
    {
        this.world = world;
        world.PreStep += PreStep;
    }

    public bool Enabled { get; set; } = true;

    private void PreStep(TimeStep timeStep)
    {
        if (!Enabled) return;

        discoveredArbiters.Clear();

        try
        {
            var spanBodies = CollectionsMarshal.AsSpan(bodies);

            for (int iter = 0; iter < SelfConsistencyIterations; iter++)
            {

                // 遍历所有已注册到 ccd-solver 的刚体。
                for (int i = 0; i < spanBodies.Length; i++)
                {
                    ref var body = ref spanBodies[i];

                    if (body.Handle.IsZero)
                    {
                        throw new InvalidOperationException("RigidBody has been removed from the world, " +
                                                            "but is still registered with the CCD solver.");
                    }

                    foreach (var shape in body.Shapes)
                    {
                        // 在宽相返回的候选中找出第一个未来碰撞，
                        // 并注册其接触。
                        CreateContact(shape, timeStep.StepDt);
                    }
                }

                // 发现阶段的求解从零累积冲量开始。后续
                // 冲量已反映在刚体速度中，因此准备阶段
                // 只重新计算接触数据。
                for (int i = 0; i < discoveredArbiters.Count; i++)
                {
                    ref var contact = ref discoveredArbiters[i].Handle.Data;
                    contact.SkipWarmStart();
                    contact.PrepareForIteration(timeStep);
                }

                for (int i = 0; i < discoveredArbiters.Count; i++)
                {
                    ref var contact = ref discoveredArbiters[i].Handle.Data;
                    contact.Iterate(false);
                }
            }
        }
        finally
        {
            // 发现阶段的冲量已反映在刚体速度中。保留它们
            // 供世界求解使用，但不要在其首次准备时重放。
            for (int i = 0; i < discoveredArbiters.Count; i++)
            {
                ref var contact = ref discoveredArbiters[i].Handle.Data;
                contact.SkipWarmStart();
            }

            discoveredArbiters.Clear();
            overlaps.Clear();
        }
    }

    private void CreateContact(RigidBodyShape shape, float dt)
    {
        JBoundingBox sweptBox = CalculateSweptBoundingBox(shape, dt, out float extentA);
        overlaps.Clear();
        world.DynamicTree.Query(overlaps, sweptBox);

        // 找出在最小碰撞时刻（TOI）与 'shape' 碰撞的候选。

        RigidBodyShape otherShape = null!;

        Unsafe.SkipInit(out JVector bestpA);
        Unsafe.SkipInit(out JVector bestpB);
        Unsafe.SkipInit(out JVector bestNormal);

        float smallestToi = float.MaxValue;

        for (int i = 0; i < overlaps.Count; i++)
        {
            if (overlaps[i] is not RigidBodyShape candidate) continue;

            if (candidate.RigidBody == shape.RigidBody) continue;
            if (world.BroadPhaseFilter != null && !world.BroadPhaseFilter.Filter(shape, candidate)) continue;

            ref var data = ref shape.RigidBody.Data;
            ref var pdata = ref candidate.RigidBody.Data;
            float extentB = CalculateAngularExtent(candidate);

            bool success = NarrowPhase.Sweep(shape, candidate, data.Orientation, pdata.Orientation,
                data.Position, pdata.Position, data.Velocity, pdata.Velocity,
                data.AngularVelocity, pdata.AngularVelocity, extentA, extentB,
                out JVector pA, out JVector pB, out JVector normal, out float toi);

            if (!success || toi > dt || toi == (float)0.0) continue;

            if (world.NarrowPhaseFilter != null)
            {
                // 过滤器约定期望的是有符号分离量，而不是碰撞时刻。
                // TOI 仅用于对扫掠命中的排序。
                float separation = JVector.Dot(normal, pA - pB) * world.SpeculativeRelaxationFactor;
                bool result = world.NarrowPhaseFilter.Filter(shape, candidate,
                    ref pA, ref pB, ref normal, ref separation);
                if (!result) continue;
            }

            if (toi < smallestToi)
            {
                smallestToi = toi;
                bestpA = pA;
                bestpB = pB;
                bestNormal = normal;
                otherShape = candidate;
            }
        }

        if (!(smallestToi < float.MaxValue)) return;

        // 创建一个仲裁器并注册该接触。所有发现的仲裁器会在
        // 本次检测遍历之后一起准备并迭代。

        Arbiter arbiter;

        if (shape.ShapeId < otherShape.ShapeId)
        {
            world.GetOrCreateArbiter(shape.ShapeId, otherShape.ShapeId, shape.RigidBody, otherShape.RigidBody, out arbiter);
            world.RegisterContact(arbiter, bestpA, bestpB, bestNormal);
        }
        else
        {
            world.GetOrCreateArbiter(otherShape.ShapeId, shape.ShapeId, otherShape.RigidBody, shape.RigidBody, out arbiter);
            world.RegisterContact(arbiter, bestpB, bestpA, -bestNormal);
        }

        // 该概念验证仅探索由接触产生的运动。在发现阶段之后，
        // 约束会故意留给常规世界求解器处理。
        if (!discoveredArbiters.Contains(arbiter))
        {
            discoveredArbiters.Add(arbiter);
            // 忽略先前世界步在发现阶段缓存的冲量。
            arbiter.Handle.Data.ResetImpulses();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static JBoundingBox CalculateSweptBoundingBox(RigidBodyShape shape, float dt, out float extent)
    {
        ref var data = ref shape.RigidBody.Data;
        shape.CalculateBoundingBox(data.Orientation, data.Position, out JBoundingBox currentBox);

        extent = CalculateAngularExtent(currentBox, data.Position);

        JVector translation = data.Velocity * dt;
        JBoundingBox endBox = new(currentBox.Min + translation, currentBox.Max + translation);
        JBoundingBox result = JBoundingBox.CreateMerged(currentBox, endBox);

        // 由于旋转，每个点最多移动 |omega| * radius * dt。
        // 将平移扫掠按该距离扩展，即可覆盖所有
        // 中间朝向，而无需使用过大的球体。
        JVector angularExpansion = new(data.AngularVelocity.Length() * extent * dt);
        result.Min -= angularExpansion;
        result.Max += angularExpansion;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float CalculateAngularExtent(RigidBodyShape shape)
    {
        ref var data = ref shape.RigidBody.Data;
        shape.CalculateBoundingBox(data.Orientation, data.Position, out JBoundingBox currentBox);

        return CalculateAngularExtent(currentBox, data.Position);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float CalculateAngularExtent(in JBoundingBox box, in JVector position)
    {
        JVector fromMin = JVector.Abs(box.Min - position);
        JVector fromMax = JVector.Abs(box.Max - position);
        return JVector.Max(fromMin, fromMax).Length();
    }

    public void Destroy()
    {
        world.PreStep -= PreStep;
        bodies.Clear();
        overlaps.Clear();
        discoveredArbiters.Clear();
    }

    public void Remove(RigidBody body) => bodies.Remove(body);

    public void Add(RigidBody body) => bodies.Add(body);
}
