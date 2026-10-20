/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.DataStructures;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由原始形状和仿射变换（平移和线性映射）定义的形状包装器。
/// </summary>
public class TransformedShape : RigidBodyShape
{
    private enum TransformationType
    {
        Identity,
        Rotation,
        General
    }

    private JVector translation;
    private JMatrix transformation;
    private TransformationType type;

    private struct TransformedSink<TSink>(TSink target, JMatrix transform, JVector offset) : ISink<JTriangle>
        where TSink : ISink<JTriangle>
    {
        public TSink Target = target;
        private readonly bool reverseWinding = transform.Determinant() < 0;

        public void Add(in JTriangle triangle)
        {
            JVector a = JVector.Transform(triangle.V0, transform) + offset;
            JVector b = JVector.Transform(triangle.V1, transform) + offset;
            JVector c = JVector.Transform(triangle.V2, transform) + offset;
            if (reverseWinding) (b, c) = (c, b);
            Target.Add(new JTriangle(a, b, c));
        }
    }

    /// <inheritdoc/>
    protected internal override void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
    {
        var sink = new TransformedSink<TSink>(triangleSink, transformation, translation);
        OriginalShape.DebugDrawLocal(ref sink, settings);
        triangleSink = sink.Target;
    }

    /// <summary>
    /// 通过由线性映射和平移定义的仿射变换构造一个变换形状。
    /// </summary>
    /// <param name="shape">要变换的形状。</param>
    /// <param name="translation">仿射变换的平移分量。</param>
    /// <param name="transform">仿射变换的线性分量。</param>
    /// <exception cref="ArgumentNullException">
    /// 当 <paramref name="shape"/> 为 <see langword="null"/> 时抛出。
    /// </exception>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="translation"/> 或 <paramref name="transform"/> 包含非有限值时抛出。
    /// </exception>
    public TransformedShape(RigidBodyShape shape, in JVector translation, in JMatrix transform)
    {
        ArgumentNullException.ThrowIfNull(shape);

        OriginalShape = shape;
        this.translation = ArgumentCheck.Finite(translation, nameof(translation));
        this.transformation = ArgumentCheck.Finite(transform, nameof(transform));

        AnalyzeTransformation();
        UpdateWorldBoundingBox();
    }

    /// <summary>
    /// 使用平移（偏移）构造变换形状，假定旋转/缩放为单位变换。
    /// </summary>
    public TransformedShape(RigidBodyShape shape, JVector translation) :
        this(shape, translation, JMatrix.Identity)
    {
    }

    /// <summary>
    /// 使用线性变换（旋转、缩放或剪切）构造变换形状，
    /// 假定平移为零。
    /// </summary>
    public TransformedShape(RigidBodyShape shape, JMatrix transform) :
        this(shape, JVector.Zero, transform)
    {
    }

    /// <summary>
    /// 获取被变换的原始形状。
    /// </summary>
    public RigidBodyShape OriginalShape { get; }

    /// <summary>
    /// 获取或设置应用于形状的平移。
    /// </summary>
    public JVector Translation
    {
        get => translation;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            translation = value;
            UpdateWorldBoundingBox();
        }
    }

    private void AnalyzeTransformation()
    {
        if (MathHelper.IsRotationMatrix(transformation))
        {
            JMatrix delta = transformation - JMatrix.Identity;
            type = MathHelper.UnsafeIsZero(ref delta) ? TransformationType.Identity : TransformationType.Rotation;
        }
        else
        {
            type = TransformationType.General;
        }
    }

    /// <summary>
    /// 获取或设置应用于形状的线性变换（旋转、缩放或剪切）。
    /// </summary>
    public JMatrix Transformation
    {
        get => transformation;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            transformation = value;
            AnalyzeTransformation();
            UpdateWorldBoundingBox();
        }
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        if (type == TransformationType.Identity)
        {
            OriginalShape.SupportMap(direction, out result);
            result += translation;
        }
        else
        {
            JVector.TransposedTransform(direction, transformation, out JVector dir);
            OriginalShape.SupportMap(dir, out JVector sm);
            JVector.Transform(sm, transformation, out result);
            result += translation;
        }
    }

    /// <inheritdoc/>
    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        if (type == TransformationType.General)
        {
            // 直接从支撑映射获取包围盒
            base.CalculateBoundingBox(orientation, position, out box);
        }
        else
        {
            JQuaternion quat = JQuaternion.CreateFromMatrix(transformation);
            OriginalShape.CalculateBoundingBox(orientation * quat,
                JVector.Transform(translation, orientation) + position, out box);
        }
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        OriginalShape.GetCenter(out point);
        point = JVector.Transform(point, transformation) + translation;
    }

    /// <inheritdoc/>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        OriginalShape.CalculateMassInertia(out JSymmetricMatrix originalInertia, out JVector originalCom, out mass);

        // 源惯性是相对于其局部原点的。在变换之前先将其移到质心，
        // 以免已有的偏移被重复计算一次。
        JSymmetricMatrix originalParallelAxis = mass *
            (JSymmetricMatrix.Identity * originalCom.LengthSquared() - JSymmetricMatrix.Outer(originalCom));
        JSymmetricMatrix centralInertia = originalInertia - originalParallelAxis;

        com = JVector.Transform(originalCom, transformation) + translation;

        Real det = MathR.Abs(transformation.Determinant());
        mass *= det;

        // 惯性张量 I 与二阶矩矩阵 C 的关系为：I = trace(C)·E - C
        // 在变换 T 下，中心二阶矩按如下方式变换：C' = |det(T)| · T · C · Tᵀ
        // 我们从 I 反推 C：C = (trace(I)/2)·E - I
        Real halfTrace = centralInertia.Trace() * (Real)0.5;
        JSymmetricMatrix secondMoment = halfTrace * JSymmetricMatrix.Identity - centralInertia;

        // 变换二阶矩矩阵
        JSymmetricMatrix transformedSecondMoment = det * JSymmetricMatrix.Transform(secondMoment, transformation);

        // 转换回惯性张量
        inertia = transformedSecondMoment.Trace() * JSymmetricMatrix.Identity - transformedSecondMoment;

        // 将变换后的惯性从其质心移到局部原点。
        JSymmetricMatrix pat = mass * (JSymmetricMatrix.Identity * com.LengthSquared() - JSymmetricMatrix.Outer(com));
        inertia += pat;
    }
}
