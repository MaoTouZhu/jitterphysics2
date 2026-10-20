/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 提供内置的图元支撑映射查询类型，以及用于构建它们的辅助方法。
/// </summary>
public static class SupportPrimitives
{
    /// <summary>
    /// 在原点处创建一个点支撑图元。
    /// </summary>
    public static Point CreatePoint() => default;

    /// <summary>
    /// 创建一个球体支撑图元。
    /// </summary>
    /// <param name="radius">球体半径。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="radius"/> 小于或等于零，或不是有限值时引发。
    /// </exception>
    public static Sphere CreateSphere(Real radius) => new(radius);

    /// <summary>
    /// 创建一个盒体支撑图元。
    /// </summary>
    /// <param name="halfExtents">盒体的正半尺寸。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="halfExtents"/> 的任一分量小于或等于零，或不是有限值时引发。
    /// </exception>
    public static Box CreateBox(JVector halfExtents) => new(halfExtents);

    /// <summary>
    /// 创建一个胶囊体支撑图元，其对称轴为 Y 轴。
    /// </summary>
    /// <param name="radius">胶囊体半径。</param>
    /// <param name="halfLength">胶囊体线段的非负半长。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="radius"/> 小于或等于零、<paramref name="halfLength"/> 为负，
    /// 或任一值不是有限值时引发。
    /// </exception>
    public static Capsule CreateCapsule(Real radius, Real halfLength) => new(radius, halfLength);

    /// <summary>
    /// 创建一个圆柱体支撑图元，其对称轴为 Y 轴。
    /// </summary>
    /// <param name="radius">圆柱体半径。</param>
    /// <param name="halfHeight">圆柱体的正半高。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="radius"/> 或 <paramref name="halfHeight"/> 小于或等于零，
    /// 或不是有限值时引发。
    /// </exception>
    public static Cylinder CreateCylinder(Real radius, Real halfHeight) => new(radius, halfHeight);

    /// <summary>
    /// 创建一个圆锥体支撑图元，其对称轴为 Y 轴。
    /// </summary>
    /// <param name="radius">圆锥体半径。</param>
    /// <param name="height">圆锥体高度。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="radius"/> 或 <paramref name="height"/> 小于或等于零，
    /// 或不是有限值时引发。
    /// </exception>
    public static Cone CreateCone(Real radius, Real height) => new(radius, height);

    /// <summary>
    /// 将点表示为轻量级的支撑映射查询图元。
    /// </summary>
    public readonly struct Point : ISupportMappable
    {
        public readonly void SupportMap(in JVector direction, out JVector result) => result = JVector.Zero;

        public readonly void GetCenter(out JVector point) => point = JVector.Zero;
    }

    /// <summary>
    /// 将球体表示为轻量级的支撑映射查询图元。
    /// </summary>
    public readonly struct Sphere(Real radius) : ISupportMappable
    {
        private readonly Real radius = ArgumentCheck.Positive(radius, nameof(radius));

        public readonly void SupportMap(in JVector direction, out JVector result) => result = JVector.Normalize(direction) * radius;

        public readonly void GetCenter(out JVector point) => point = JVector.Zero;
    }

    /// <summary>
    /// 将盒体表示为轻量级的支撑映射查询图元。
    /// </summary>
    public readonly struct Box(JVector halfExtents) : ISupportMappable
    {
        private readonly JVector halfExtents = ArgumentCheck.PositiveComponents(halfExtents, nameof(halfExtents));

        public readonly void SupportMap(in JVector direction, out JVector result)
        {
            result.X = MathHelper.SignBit(direction.X) * halfExtents.X;
            result.Y = MathHelper.SignBit(direction.Y) * halfExtents.Y;
            result.Z = MathHelper.SignBit(direction.Z) * halfExtents.Z;
        }

        public readonly void GetCenter(out JVector point) => point = JVector.Zero;
    }

    /// <summary>
    /// 将胶囊体表示为轻量级的支撑映射查询图元。其对称轴为 Y 轴。
    /// </summary>
    public readonly struct Capsule(Real radius, Real halfLength) : ISupportMappable
    {
        private readonly Real radius = ArgumentCheck.Positive(radius, nameof(radius));
        private readonly Real halfLength = ArgumentCheck.NonNegative(halfLength, nameof(halfLength));

        public readonly void SupportMap(in JVector direction, out JVector result)
        {
            result = JVector.Normalize(direction) * radius;
            result.Y += MathR.Sign(direction.Y) * halfLength;
        }

        public readonly void GetCenter(out JVector point) => point = JVector.Zero;
    }

    /// <summary>
    /// 将圆柱体表示为轻量级的支撑映射查询图元。其对称轴为 Y 轴。
    /// </summary>
    public readonly struct Cylinder(Real radius, Real halfHeight) : ISupportMappable
    {
        private readonly Real radius = ArgumentCheck.Positive(radius, nameof(radius));
        private readonly Real halfHeight = ArgumentCheck.Positive(halfHeight, nameof(halfHeight));

        public readonly void SupportMap(in JVector direction, out JVector result)
        {
            Real sigma = MathR.Sqrt(direction.X * direction.X + direction.Z * direction.Z);

            if (sigma > (Real)0.0)
            {
                result.X = direction.X / sigma * radius;
                result.Y = MathR.Sign(direction.Y) * halfHeight;
                result.Z = direction.Z / sigma * radius;
            }
            else
            {
                result.X = (Real)0.0;
                result.Y = MathR.Sign(direction.Y) * halfHeight;
                result.Z = (Real)0.0;
            }
        }

        public readonly void GetCenter(out JVector point) => point = JVector.Zero;
    }

    /// <summary>
    /// 将圆锥体表示为轻量级的支撑映射查询图元。其对称轴为 Y 轴。
    /// 圆锥体以其质心为中心：底面位于 Y = -height/4，顶点位于 Y = 3*height/4。
    /// </summary>
    public readonly struct Cone(Real radius, Real height) : ISupportMappable
    {
        private readonly Real radius = ArgumentCheck.Positive(radius, nameof(radius));
        private readonly Real height = ArgumentCheck.Positive(height, nameof(height));

        public readonly void SupportMap(in JVector direction, out JVector result)
        {
            const Real zeroEpsilon = (Real)1e-12;

            JVector baseDir = new(direction.X, (Real)0.0, direction.Z);
            baseDir = JVector.NormalizeSafe(baseDir, zeroEpsilon) * radius;
            baseDir.Y = -(Real)0.25 * height;

            if (JVector.Dot(direction, baseDir) >= direction.Y * (Real)0.75 * height)
            {
                result = baseDir;
            }
            else
            {
                result = new JVector((Real)0.0, (Real)0.75 * height, (Real)0.0);
            }
        }

        public readonly void GetCenter(out JVector point) => point = JVector.Zero;
    }
}
