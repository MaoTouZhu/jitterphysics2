/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 使用六个 <see cref="Real"/> 类型分量表示对称 3x3 矩阵。
/// </summary>
/// <remarks>
/// 仅存储上三角部分。下三角属性访问的是相同的分量。
/// </remarks>
[StructLayout(LayoutKind.Explicit, Size = 6 * sizeof(Real))]
public struct JSymmetricMatrix(
    Real m11, Real m12, Real m13,
    Real m22, Real m23,
    Real m33) : IEquatable<JSymmetricMatrix>
{
    [FieldOffset(0 * sizeof(Real))] public Real M11 = m11;
    [FieldOffset(1 * sizeof(Real))] public Real M12 = m12;
    [FieldOffset(2 * sizeof(Real))] public Real M13 = m13;
    [FieldOffset(3 * sizeof(Real))] public Real M22 = m22;
    [FieldOffset(4 * sizeof(Real))] public Real M23 = m23;
    [FieldOffset(5 * sizeof(Real))] public Real M33 = m33;

    /// <summary>与 <see cref="M12"/> 共享的分量。</summary>
    public Real M21
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => M12;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => M12 = value;
    }

    /// <summary>与 <see cref="M13"/> 共享的分量。</summary>
    public Real M31
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => M13;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => M13 = value;
    }

    /// <summary>与 <see cref="M23"/> 共享的分量。</summary>
    public Real M32
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => M23;
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => M23 = value;
    }


    /// <summary>单位矩阵。</summary>
    public static readonly JSymmetricMatrix Identity = new(1, 0, 0, 1, 0, 1);

    /// <summary>零矩阵。</summary>
    public static readonly JSymmetricMatrix Zero;

    /// <summary>将矩阵展开为完整的 3x3 矩阵。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly JMatrix ToMatrix()
    {
        return new JMatrix(M11, M12, M13, M12, M22, M23, M13, M23, M33);
    }

    /// <summary>创建对角缩放矩阵。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix CreateScale(in JVector scale)
    {
        return new JSymmetricMatrix(scale.X, 0, 0, scale.Y, 0, scale.Z);
    }

    /// <summary>创建对角缩放矩阵。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix CreateScale(Real x, Real y, Real z)
    {
        return new JSymmetricMatrix(x, 0, 0, y, 0, z);
    }

    /// <summary>计算对称外积 <c>vector * vectorᵀ</c>。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix Outer(in JVector vector)
    {
        return new JSymmetricMatrix(vector.X * vector.X, vector.X * vector.Y, vector.X * vector.Z,
            vector.Y * vector.Y, vector.Y * vector.Z, vector.Z * vector.Z);
    }

    /// <summary>计算迹（对角分量之和）。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real Trace()
    {
        return M11 + M22 + M33;
    }

    /// <summary>计算矩阵的行列式。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real Determinant()
    {
        return M11 * (M22 * M33 - M23 * M23) +
               M12 * (M23 * M13 - M12 * M33) +
               M13 * (M12 * M23 - M22 * M13);
    }

    /// <summary>计算对称矩阵的逆。</summary>
    /// <param name="matrix">要求逆的矩阵。</param>
    /// <param name="result">输出：逆矩阵，若求逆失败则为零。</param>
    /// <returns>
    /// 若可计算出有限的逆则为 <c>true</c>；否则为 <c>false</c>，
    /// 与 <see cref="JMatrix.Inverse"/> 的行为一致。
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Inverse(in JSymmetricMatrix matrix, out JSymmetricMatrix result)
    {
        if (TryInverseUnscaled(matrix, out JSymmetricMatrix inverse))
        {
            result = inverse;
            return true;
        }

        Real scale = MathR.Max(MathR.Abs(matrix.M11), MathR.Max(MathR.Abs(matrix.M12), MathR.Abs(matrix.M13)));
        scale = MathR.Max(scale, MathR.Max(MathR.Abs(matrix.M22), MathR.Max(MathR.Abs(matrix.M23), MathR.Abs(matrix.M33))));
        if (scale > 0 && Real.IsFinite(scale))
        {
            JSymmetricMatrix normalized = new(matrix.M11 / scale, matrix.M12 / scale, matrix.M13 / scale,
                matrix.M22 / scale, matrix.M23 / scale, matrix.M33 / scale);
            if (TryInverseUnscaled(normalized, out inverse))
            {
                inverse = new JSymmetricMatrix(inverse.M11 / scale, inverse.M12 / scale, inverse.M13 / scale,
                    inverse.M22 / scale, inverse.M23 / scale, inverse.M33 / scale);
                if (IsFinite(inverse))
                {
                    result = inverse;
                    return true;
                }
            }
        }

        result = Zero;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsFinite(in JSymmetricMatrix matrix)
    {
        return Real.IsFinite(matrix.M11) && Real.IsFinite(matrix.M12) && Real.IsFinite(matrix.M13) &&
               Real.IsFinite(matrix.M22) && Real.IsFinite(matrix.M23) && Real.IsFinite(matrix.M33);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryInverseUnscaled(in JSymmetricMatrix matrix, out JSymmetricMatrix result)
    {
        Real idet = (Real)1.0 / matrix.Determinant();

        if (!Real.IsNormal(idet))
        {
            result = Zero;
            return false;
        }

        // 在写入结果之前读取所有输入，从而支持就地求逆。
        Real m11 = matrix.M22 * matrix.M33 - matrix.M23 * matrix.M23;
        Real m12 = matrix.M13 * matrix.M23 - matrix.M12 * matrix.M33;
        Real m13 = matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13;
        Real m22 = matrix.M11 * matrix.M33 - matrix.M13 * matrix.M13;
        Real m23 = matrix.M13 * matrix.M12 - matrix.M23 * matrix.M11;
        Real m33 = matrix.M11 * matrix.M22 - matrix.M12 * matrix.M12;

        result = new JSymmetricMatrix(m11 * idet, m12 * idet, m13 * idet,
            m22 * idet, m23 * idet, m33 * idet);
        return IsFinite(result);
    }

    /// <summary>将两个矩阵逐分量相加。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix Add(in JSymmetricMatrix matrix1, in JSymmetricMatrix matrix2)
    {
        Add(matrix1, matrix2, out JSymmetricMatrix result);
        return result;
    }

    /// <summary>将两个矩阵逐分量相加。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Add(in JSymmetricMatrix matrix1, in JSymmetricMatrix matrix2,
        out JSymmetricMatrix result)
    {
        result = new JSymmetricMatrix(matrix1.M11 + matrix2.M11, matrix1.M12 + matrix2.M12,
            matrix1.M13 + matrix2.M13, matrix1.M22 + matrix2.M22,
            matrix1.M23 + matrix2.M23, matrix1.M33 + matrix2.M33);
    }

    /// <summary>将第二个矩阵从第一个中逐分量减去。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix Subtract(in JSymmetricMatrix matrix1, in JSymmetricMatrix matrix2)
    {
        Subtract(matrix1, matrix2, out JSymmetricMatrix result);
        return result;
    }

    /// <summary>将第二个矩阵从第一个中逐分量减去。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Subtract(in JSymmetricMatrix matrix1, in JSymmetricMatrix matrix2,
        out JSymmetricMatrix result)
    {
        result = new JSymmetricMatrix(matrix1.M11 - matrix2.M11, matrix1.M12 - matrix2.M12,
            matrix1.M13 - matrix2.M13, matrix1.M22 - matrix2.M22,
            matrix1.M23 - matrix2.M23, matrix1.M33 - matrix2.M33);
    }

    /// <summary>将矩阵乘以标量因子。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix Multiply(in JSymmetricMatrix matrix, Real scaleFactor)
    {
        Multiply(matrix, scaleFactor, out JSymmetricMatrix result);
        return result;
    }

    /// <summary>将矩阵乘以标量因子。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Multiply(in JSymmetricMatrix matrix, Real scaleFactor, out JSymmetricMatrix result)
    {
        result = new JSymmetricMatrix(matrix.M11 * scaleFactor, matrix.M12 * scaleFactor,
            matrix.M13 * scaleFactor, matrix.M22 * scaleFactor,
            matrix.M23 * scaleFactor, matrix.M33 * scaleFactor);
    }

    /// <summary>计算 <c>transform * matrix * transformᵀ</c>。</summary>
    /// <remarks>
    /// 仅计算六个独立分量。旋转可将惯性张量变换
    /// 到旋转后的坐标系；也支持一般变换。
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix Transform(in JSymmetricMatrix matrix, in JMatrix transform)
    {
        Transform(matrix, transform, out JSymmetricMatrix result);
        return result;
    }

    /// <summary>计算 <c>transform * matrix * transformᵀ</c>。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Transform(in JSymmetricMatrix matrix, in JMatrix transform,
        out JSymmetricMatrix result)
    {
        JVector row1 = new(transform.M11, transform.M12, transform.M13);
        JVector row2 = new(transform.M21, transform.M22, transform.M23);
        JVector row3 = new(transform.M31, transform.M32, transform.M33);

        JVector product1 = JVector.Transform(row1, matrix);
        JVector product2 = JVector.Transform(row2, matrix);
        JVector product3 = JVector.Transform(row3, matrix);

        result = new JSymmetricMatrix(product1 * row1, product1 * row2, product1 * row3,
            product2 * row2, product2 * row3, product3 * row3);
    }

    /// <summary>将对称矩阵展开为完整矩阵。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static explicit operator JMatrix(in JSymmetricMatrix matrix) => matrix.ToMatrix();

    /// <summary>将两个矩阵相加。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix operator +(in JSymmetricMatrix matrix1, in JSymmetricMatrix matrix2)
        => Add(matrix1, matrix2);

    /// <summary>将第二个矩阵从第一个中减去。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix operator -(in JSymmetricMatrix matrix1, in JSymmetricMatrix matrix2)
        => Subtract(matrix1, matrix2);

    /// <summary>按因子缩放矩阵。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix operator *(in JSymmetricMatrix matrix, Real factor)
        => Multiply(matrix, factor);

    /// <summary>按因子缩放矩阵。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JSymmetricMatrix operator *(Real factor, in JSymmetricMatrix matrix)
        => Multiply(matrix, factor);

    public readonly bool Equals(JSymmetricMatrix other)
    {
        return M11.Equals(other.M11) && M12.Equals(other.M12) && M13.Equals(other.M13) &&
               M22.Equals(other.M22) && M23.Equals(other.M23) && M33.Equals(other.M33);
    }

    public readonly override bool Equals(object? obj) => obj is JSymmetricMatrix other && Equals(other);

    public readonly override int GetHashCode() => HashCode.Combine(M11, M12, M13, M22, M23, M33);

    public readonly override string ToString()
    {
        return $"M11={M11:F6}, M12={M12:F6}, M13={M13:F6}, " +
               $"M22={M22:F6}, M23={M23:F6}, M33={M33:F6}";
    }

    public static bool operator ==(JSymmetricMatrix left, JSymmetricMatrix right) => left.Equals(right);

    public static bool operator !=(JSymmetricMatrix left, JSymmetricMatrix right) => !left.Equals(right);
}
