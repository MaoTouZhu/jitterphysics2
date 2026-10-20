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
/// 表示一个分量为 <see cref="Real"/> 类型的 3x3 矩阵。
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 9 * sizeof(Real))]
public struct JMatrix(
    Real m11, Real m12, Real m13,
    Real m21, Real m22, Real m23,
    Real m31, Real m32, Real m33) : IEquatable<JMatrix>
{
    [FieldOffset(0 * sizeof(Real))] public Real M11 = m11;
    [FieldOffset(1 * sizeof(Real))] public Real M21 = m21;
    [FieldOffset(2 * sizeof(Real))] public Real M31 = m31;
    [FieldOffset(3 * sizeof(Real))] public Real M12 = m12;
    [FieldOffset(4 * sizeof(Real))] public Real M22 = m22;
    [FieldOffset(5 * sizeof(Real))] public Real M32 = m32;
    [FieldOffset(6 * sizeof(Real))] public Real M13 = m13;
    [FieldOffset(7 * sizeof(Real))] public Real M23 = m23;
    [FieldOffset(8 * sizeof(Real))] public Real M33 = m33;

    /// <summary>
    /// 单位矩阵。
    /// </summary>
    public static readonly JMatrix Identity;

    /// <summary>
    /// 零矩阵。
    /// </summary>
    public static readonly JMatrix Zero;

    static JMatrix()
    {
        Zero = new JMatrix();

        Identity = new JMatrix
        {
            M11 = (Real)1.0,
            M22 = (Real)1.0,
            M33 = (Real)1.0
        };
    }

    /// <summary>
    /// 由三个列向量创建矩阵。
    /// </summary>
    /// <param name="col1">第一个列向量。</param>
    /// <param name="col2">第二个列向量。</param>
    /// <param name="col3">第三个列向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix FromColumns(in JVector col1, in JVector col2, in JVector col3)
    {
        Unsafe.SkipInit(out JMatrix result);
        result.UnsafeGet(0) = col1;
        result.UnsafeGet(1) = col2;
        result.UnsafeGet(2) = col3;
        return result;
    }

    /// <summary>
    /// 使用不安全指针运算按索引获取列向量的引用。
    /// </summary>
    /// <param name="index">列的从零开始的索引（0、1 或 2）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe ref JVector UnsafeGet(int index)
    {
        JVector* ptr = (JVector*)Unsafe.AsPointer(ref this);
        return ref ptr[index];
    }

    /// <summary>
    /// 按索引获取列向量。
    /// </summary>
    /// <param name="index">列的从零开始的索引（0、1 或 2）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe JVector GetColumn(int index)
    {
        fixed (Real* ptr = &M11)
        {
            JVector* vptr = (JVector*)ptr;
            return vptr[index];
        }
    }

    /// <summary>
    /// 将两个矩阵相乘。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <returns>两个矩阵的乘积。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix Multiply(in JMatrix matrix1, in JMatrix matrix2)
    {
        Multiply(matrix1, matrix2, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 计算 <c>matrix1 * transpose(matrix2)</c>。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵（在相乘过程中会被转置）。</param>
    /// <returns>相乘的结果。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix MultiplyTransposed(in JMatrix matrix1, in JMatrix matrix2)
    {
        MultiplyTransposed(matrix1, matrix2, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 计算 <c>transpose(matrix1) * matrix2</c>。
    /// </summary>
    /// <param name="matrix1">第一个矩阵（在相乘过程中会被转置）。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <returns>相乘的结果。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix TransposedMultiply(in JMatrix matrix1, in JMatrix matrix2)
    {
        TransposedMultiply(matrix1, matrix2, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 根据轴和角度创建旋转矩阵。
    /// </summary>
    /// <param name="axis">旋转所围绕的单位长度轴。</param>
    /// <param name="angle">以弧度表示的旋转角度。</param>
    /// <returns>旋转矩阵。</returns>
    public static JMatrix CreateRotationMatrix(JVector axis, Real angle)
    {
        (Real s, Real c) = StableMath.SinCos(angle / (Real)2.0);
        axis *= s;
        JQuaternion jq = new(axis.X, axis.Y, axis.Z, c);
        CreateFromQuaternion(in jq, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 将两个矩阵相乘。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <param name="result">输出：两个矩阵的乘积。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Multiply(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
    {
        Real num0 = matrix1.M11 * matrix2.M11 + matrix1.M12 * matrix2.M21 + matrix1.M13 * matrix2.M31;
        Real num1 = matrix1.M11 * matrix2.M12 + matrix1.M12 * matrix2.M22 + matrix1.M13 * matrix2.M32;
        Real num2 = matrix1.M11 * matrix2.M13 + matrix1.M12 * matrix2.M23 + matrix1.M13 * matrix2.M33;
        Real num3 = matrix1.M21 * matrix2.M11 + matrix1.M22 * matrix2.M21 + matrix1.M23 * matrix2.M31;
        Real num4 = matrix1.M21 * matrix2.M12 + matrix1.M22 * matrix2.M22 + matrix1.M23 * matrix2.M32;
        Real num5 = matrix1.M21 * matrix2.M13 + matrix1.M22 * matrix2.M23 + matrix1.M23 * matrix2.M33;
        Real num6 = matrix1.M31 * matrix2.M11 + matrix1.M32 * matrix2.M21 + matrix1.M33 * matrix2.M31;
        Real num7 = matrix1.M31 * matrix2.M12 + matrix1.M32 * matrix2.M22 + matrix1.M33 * matrix2.M32;
        Real num8 = matrix1.M31 * matrix2.M13 + matrix1.M32 * matrix2.M23 + matrix1.M33 * matrix2.M33;

        result.M11 = num0;
        result.M12 = num1;
        result.M13 = num2;
        result.M21 = num3;
        result.M22 = num4;
        result.M23 = num5;
        result.M31 = num6;
        result.M32 = num7;
        result.M33 = num8;
    }

    /// <summary>
    /// 将两个矩阵相加。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <returns>两个矩阵之和。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix Add(JMatrix matrix1, JMatrix matrix2)
    {
        Add(matrix1, matrix2, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 计算 <c>matrix1 * matrix2ᵀ</c>（用 matrix1 乘以 matrix2 的转置）。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵（运算过程中会被转置）。</param>
    /// <param name="result">输出：相乘的结果。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void MultiplyTransposed(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
    {
        Real num0 = matrix1.M11 * matrix2.M11 + matrix1.M12 * matrix2.M12 + matrix1.M13 * matrix2.M13;
        Real num1 = matrix1.M11 * matrix2.M21 + matrix1.M12 * matrix2.M22 + matrix1.M13 * matrix2.M23;
        Real num2 = matrix1.M11 * matrix2.M31 + matrix1.M12 * matrix2.M32 + matrix1.M13 * matrix2.M33;
        Real num3 = matrix1.M21 * matrix2.M11 + matrix1.M22 * matrix2.M12 + matrix1.M23 * matrix2.M13;
        Real num4 = matrix1.M21 * matrix2.M21 + matrix1.M22 * matrix2.M22 + matrix1.M23 * matrix2.M23;
        Real num5 = matrix1.M21 * matrix2.M31 + matrix1.M22 * matrix2.M32 + matrix1.M23 * matrix2.M33;
        Real num6 = matrix1.M31 * matrix2.M11 + matrix1.M32 * matrix2.M12 + matrix1.M33 * matrix2.M13;
        Real num7 = matrix1.M31 * matrix2.M21 + matrix1.M32 * matrix2.M22 + matrix1.M33 * matrix2.M23;
        Real num8 = matrix1.M31 * matrix2.M31 + matrix1.M32 * matrix2.M32 + matrix1.M33 * matrix2.M33;

        result.M11 = num0;
        result.M12 = num1;
        result.M13 = num2;
        result.M21 = num3;
        result.M22 = num4;
        result.M23 = num5;
        result.M31 = num6;
        result.M32 = num7;
        result.M33 = num8;
    }

    /// <summary>
    /// 创建绕 X 轴旋转的旋转矩阵。
    /// </summary>
    /// <param name="radians">以弧度表示的旋转角度。</param>
    /// <returns>旋转矩阵。</returns>
    public static JMatrix CreateRotationX(Real radians)
    {
        JMatrix result = Identity;

        (Real s, Real c) = StableMath.SinCos(radians);

        // [  1  0  0  ]
        // [  0  c -s  ]
        // [  0  s  c  ]
        result.M22 = c;
        result.M23 = -s;
        result.M32 = s;
        result.M33 = c;

        return result;
    }

    /// <summary>
    /// 创建绕 Y 轴旋转的旋转矩阵。
    /// </summary>
    /// <param name="radians">以弧度表示的旋转角度。</param>
    /// <returns>旋转矩阵。</returns>
    public static JMatrix CreateRotationY(Real radians)
    {
        JMatrix result = Identity;

        (Real s, Real c) = StableMath.SinCos(radians);

        // [  c  0  s  ]
        // [  0  1  0  ]
        // [ -s  0  c  ]
        result.M11 = c;
        result.M13 = s;
        result.M31 = -s;
        result.M33 = c;

        return result;
    }

    /// <summary>
    /// 创建绕 Z 轴旋转的旋转矩阵。
    /// </summary>
    /// <param name="radians">以弧度表示的旋转角度。</param>
    /// <returns>旋转矩阵。</returns>
    public static JMatrix CreateRotationZ(Real radians)
    {
        JMatrix result = Identity;

        (Real s, Real c) = StableMath.SinCos(radians);

        // [  c -s  0  ]
        // [  s  c  0  ]
        // [  0  0  1  ]
        result.M11 = c;
        result.M12 = -s;
        result.M21 = s;
        result.M22 = c;

        return result;
    }

    /// <summary>
    /// 创建缩放矩阵。
    /// </summary>
    /// <param name="scale">缩放向量。</param>
    /// <returns>缩放矩阵。</returns>
    public static JMatrix CreateScale(in JVector scale)
    {
        JMatrix result = Zero;

        result.M11 = scale.X;
        result.M22 = scale.Y;
        result.M33 = scale.Z;

        return result;
    }

    /// <summary>
    /// 创建缩放矩阵。
    /// </summary>
    /// <param name="x">X 轴上的缩放因子。</param>
    /// <param name="y">Y 轴上的缩放因子。</param>
    /// <param name="z">Z 轴上的缩放因子。</param>
    /// <returns>缩放矩阵。</returns>
    public static JMatrix CreateScale(Real x, Real y, Real z)
    {
        return CreateScale(new JVector(x, y, z));
    }

    /// <summary>
    /// 计算 <c>matrix1ᵀ * matrix2</c>（用 matrix1 的转置乘以 matrix2）。
    /// </summary>
    /// <param name="matrix1">第一个矩阵（运算过程中会被转置）。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <param name="result">输出：相乘的结果。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void TransposedMultiply(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
    {
        Real num0 = matrix1.M11 * matrix2.M11 + matrix1.M21 * matrix2.M21 + matrix1.M31 * matrix2.M31;
        Real num1 = matrix1.M11 * matrix2.M12 + matrix1.M21 * matrix2.M22 + matrix1.M31 * matrix2.M32;
        Real num2 = matrix1.M11 * matrix2.M13 + matrix1.M21 * matrix2.M23 + matrix1.M31 * matrix2.M33;
        Real num3 = matrix1.M12 * matrix2.M11 + matrix1.M22 * matrix2.M21 + matrix1.M32 * matrix2.M31;
        Real num4 = matrix1.M12 * matrix2.M12 + matrix1.M22 * matrix2.M22 + matrix1.M32 * matrix2.M32;
        Real num5 = matrix1.M12 * matrix2.M13 + matrix1.M22 * matrix2.M23 + matrix1.M32 * matrix2.M33;
        Real num6 = matrix1.M13 * matrix2.M11 + matrix1.M23 * matrix2.M21 + matrix1.M33 * matrix2.M31;
        Real num7 = matrix1.M13 * matrix2.M12 + matrix1.M23 * matrix2.M22 + matrix1.M33 * matrix2.M32;
        Real num8 = matrix1.M13 * matrix2.M13 + matrix1.M23 * matrix2.M23 + matrix1.M33 * matrix2.M33;

        result.M11 = num0;
        result.M12 = num1;
        result.M13 = num2;
        result.M21 = num3;
        result.M22 = num4;
        result.M23 = num5;
        result.M31 = num6;
        result.M32 = num7;
        result.M33 = num8;
    }

    /// <summary>
    /// 按分量将两个矩阵相加。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <param name="result">输出：两个矩阵之和。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Add(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
    {
        result.M11 = matrix1.M11 + matrix2.M11;
        result.M12 = matrix1.M12 + matrix2.M12;
        result.M13 = matrix1.M13 + matrix2.M13;
        result.M21 = matrix1.M21 + matrix2.M21;
        result.M22 = matrix1.M22 + matrix2.M22;
        result.M23 = matrix1.M23 + matrix2.M23;
        result.M31 = matrix1.M31 + matrix2.M31;
        result.M32 = matrix1.M32 + matrix2.M32;
        result.M33 = matrix1.M33 + matrix2.M33;
    }

    /// <summary>
    /// 按分量从第一个矩阵中减去第二个矩阵。
    /// </summary>
    /// <param name="matrix1">第一个矩阵。</param>
    /// <param name="matrix2">第二个矩阵。</param>
    /// <param name="result">输出：两个矩阵之差。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Subtract(in JMatrix matrix1, in JMatrix matrix2, out JMatrix result)
    {
        result.M11 = matrix1.M11 - matrix2.M11;
        result.M12 = matrix1.M12 - matrix2.M12;
        result.M13 = matrix1.M13 - matrix2.M13;
        result.M21 = matrix1.M21 - matrix2.M21;
        result.M22 = matrix1.M22 - matrix2.M22;
        result.M23 = matrix1.M23 - matrix2.M23;
        result.M31 = matrix1.M31 - matrix2.M31;
        result.M32 = matrix1.M32 - matrix2.M32;
        result.M33 = matrix1.M33 - matrix2.M33;
    }

    /// <summary>
    /// 计算矩阵的行列式。
    /// </summary>
    /// <returns>行列式。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real Determinant()
    {
        return M11 * M22 * M33 + M12 * M23 * M31 + M13 * M21 * M32 -
               M31 * M22 * M13 - M32 * M23 * M11 - M33 * M21 * M12;
    }

    /// <summary>
    /// 计算矩阵的逆矩阵。
    /// </summary>
    /// <param name="matrix">要求逆的矩阵。</param>
    /// <param name="result">输出：逆矩阵；若无法计算有限逆矩阵则为零矩阵。</param>
    /// <returns>若矩阵可逆则为 <c>true</c>；否则为 <c>false</c>。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Inverse(in JMatrix matrix, out JMatrix result)
    {
        if (TryInverseUnscaled(matrix, out JMatrix inverse))
        {
            result = inverse;
            return true;
        }

        // 整体过小或过大的矩阵即使在逆矩阵可表示时也可能使行列式溢出。
        // 对常见量级保留常规路径。
        Real scale = MathR.Max(MathR.Abs(matrix.M11), MathR.Max(MathR.Abs(matrix.M12), MathR.Abs(matrix.M13)));
        scale = MathR.Max(scale, MathR.Max(MathR.Abs(matrix.M21), MathR.Max(MathR.Abs(matrix.M22), MathR.Abs(matrix.M23))));
        scale = MathR.Max(scale, MathR.Max(MathR.Abs(matrix.M31), MathR.Max(MathR.Abs(matrix.M32), MathR.Abs(matrix.M33))));
        if (scale > 0 && Real.IsFinite(scale))
        {
            JMatrix normalized = new(matrix.M11 / scale, matrix.M12 / scale, matrix.M13 / scale,
                matrix.M21 / scale, matrix.M22 / scale, matrix.M23 / scale,
                matrix.M31 / scale, matrix.M32 / scale, matrix.M33 / scale);
            if (TryInverseUnscaled(normalized, out inverse))
            {
                inverse.M11 /= scale; inverse.M12 /= scale; inverse.M13 /= scale;
                inverse.M21 /= scale; inverse.M22 /= scale; inverse.M23 /= scale;
                inverse.M31 /= scale; inverse.M32 /= scale; inverse.M33 /= scale;
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
    private static bool IsFinite(in JMatrix matrix)
    {
        return Real.IsFinite(matrix.M11) && Real.IsFinite(matrix.M12) && Real.IsFinite(matrix.M13) &&
               Real.IsFinite(matrix.M21) && Real.IsFinite(matrix.M22) && Real.IsFinite(matrix.M23) &&
               Real.IsFinite(matrix.M31) && Real.IsFinite(matrix.M32) && Real.IsFinite(matrix.M33);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryInverseUnscaled(in JMatrix matrix, out JMatrix result)
    {
        Real idet = (Real)1.0 / matrix.Determinant();

        if (!Real.IsNormal(idet))
        {
            result = new JMatrix();
            return false;
        }

        Real num11 = matrix.M22 * matrix.M33 - matrix.M23 * matrix.M32;
        Real num12 = matrix.M13 * matrix.M32 - matrix.M12 * matrix.M33;
        Real num13 = matrix.M12 * matrix.M23 - matrix.M22 * matrix.M13;

        Real num21 = matrix.M23 * matrix.M31 - matrix.M33 * matrix.M21;
        Real num22 = matrix.M11 * matrix.M33 - matrix.M31 * matrix.M13;
        Real num23 = matrix.M13 * matrix.M21 - matrix.M23 * matrix.M11;

        Real num31 = matrix.M21 * matrix.M32 - matrix.M31 * matrix.M22;
        Real num32 = matrix.M12 * matrix.M31 - matrix.M32 * matrix.M11;
        Real num33 = matrix.M11 * matrix.M22 - matrix.M21 * matrix.M12;

        result.M11 = num11 * idet;
        result.M12 = num12 * idet;
        result.M13 = num13 * idet;
        result.M21 = num21 * idet;
        result.M22 = num22 * idet;
        result.M23 = num23 * idet;
        result.M31 = num31 * idet;
        result.M32 = num32 * idet;
        result.M33 = num33 * idet;

        return IsFinite(result);
    }

    /// <summary>
    /// 将矩阵乘以标量因子。
    /// </summary>
    /// <param name="matrix1">矩阵。</param>
    /// <param name="scaleFactor">标量因子。</param>
    /// <returns>缩放后的矩阵。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix Multiply(JMatrix matrix1, Real scaleFactor)
    {
        Multiply(in matrix1, scaleFactor, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 将矩阵乘以标量因子。
    /// </summary>
    /// <param name="matrix1">矩阵。</param>
    /// <param name="scaleFactor">标量因子。</param>
    /// <param name="result">输出：缩放后的矩阵。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Multiply(in JMatrix matrix1, Real scaleFactor, out JMatrix result)
    {
        result.M11 = matrix1.M11 * scaleFactor;
        result.M12 = matrix1.M12 * scaleFactor;
        result.M13 = matrix1.M13 * scaleFactor;
        result.M21 = matrix1.M21 * scaleFactor;
        result.M22 = matrix1.M22 * scaleFactor;
        result.M23 = matrix1.M23 * scaleFactor;
        result.M31 = matrix1.M31 * scaleFactor;
        result.M32 = matrix1.M32 * scaleFactor;
        result.M33 = matrix1.M33 * scaleFactor;
    }

    /// <summary>
    /// 根据四元数创建旋转矩阵。
    /// </summary>
    /// <param name="quaternion">表示旋转的单位四元数。</param>
    /// <returns>旋转矩阵。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix CreateFromQuaternion(JQuaternion quaternion)
    {
        CreateFromQuaternion(quaternion, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 创建一个矩阵，其每个分量均为输入矩阵对应分量的绝对值。
    /// </summary>
    /// <param name="matrix">输入矩阵。</param>
    /// <param name="result">输出：绝对值矩阵。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Absolute(in JMatrix matrix, out JMatrix result)
    {
        result.M11 = Math.Abs(matrix.M11);
        result.M12 = Math.Abs(matrix.M12);
        result.M13 = Math.Abs(matrix.M13);
        result.M21 = Math.Abs(matrix.M21);
        result.M22 = Math.Abs(matrix.M22);
        result.M23 = Math.Abs(matrix.M23);
        result.M31 = Math.Abs(matrix.M31);
        result.M32 = Math.Abs(matrix.M32);
        result.M33 = Math.Abs(matrix.M33);
    }

    /// <summary>
    /// 根据四元数创建旋转矩阵。
    /// </summary>
    /// <param name="quaternion">表示旋转的单位四元数。</param>
    /// <param name="result">输出：旋转矩阵。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CreateFromQuaternion(in JQuaternion quaternion, out JMatrix result)
    {
        Real r = quaternion.W;
        Real i = quaternion.X;
        Real j = quaternion.Y;
        Real k = quaternion.Z;

        result.M11 = (Real)1.0 - (Real)2.0 * (j * j + k * k);
        result.M12 = (Real)2.0 * (i * j - k * r);
        result.M13 = (Real)2.0 * (i * k + j * r);
        result.M21 = (Real)2.0 * (i * j + k * r);
        result.M22 = (Real)1.0 - (Real)2.0 * (i * i + k * k);
        result.M23 = (Real)2.0 * (j * k - i * r);
        result.M31 = (Real)2.0 * (i * k - j * r);
        result.M32 = (Real)2.0 * (j * k + i * r);
        result.M33 = (Real)1.0 - (Real)2.0 * (i * i + j * j);
    }

    /// <summary>
    /// 转置矩阵。
    /// </summary>
    /// <param name="matrix">要转置的矩阵。</param>
    /// <returns>转置后的矩阵。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix Transpose(in JMatrix matrix)
    {
        Transpose(in matrix, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 根据向量创建反对称矩阵，表示叉积运算。
    /// </summary>
    /// <remarks>
    /// 结果等价于：
    /// <code>
    /// [  0  -z   y ]
    /// [  z   0  -x ]
    /// [ -y   x   0 ]
    /// </code>
    /// </remarks>
    /// <param name="vec">向量。</param>
    /// <returns>反对称矩阵。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix CreateCrossProduct(in JVector vec)
    {
        return new JMatrix(0, -vec.Z, vec.Y, vec.Z, 0, -vec.X, -vec.Y, vec.X, 0);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Transpose(in JMatrix matrix, out JMatrix result)
    {
        result.M11 = matrix.M11;
        result.M12 = matrix.M21;
        result.M13 = matrix.M31;
        result.M21 = matrix.M12;
        result.M22 = matrix.M22;
        result.M23 = matrix.M32;
        result.M31 = matrix.M13;
        result.M32 = matrix.M23;
        result.M33 = matrix.M33;
    }

    /// <summary>
    /// 将两个矩阵相乘。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix operator *(in JMatrix matrix1, in JMatrix matrix2)
    {
        JMatrix result;
        result.M11 = matrix1.M11 * matrix2.M11 + matrix1.M12 * matrix2.M21 + matrix1.M13 * matrix2.M31;
        result.M12 = matrix1.M11 * matrix2.M12 + matrix1.M12 * matrix2.M22 + matrix1.M13 * matrix2.M32;
        result.M13 = matrix1.M11 * matrix2.M13 + matrix1.M12 * matrix2.M23 + matrix1.M13 * matrix2.M33;
        result.M21 = matrix1.M21 * matrix2.M11 + matrix1.M22 * matrix2.M21 + matrix1.M23 * matrix2.M31;
        result.M22 = matrix1.M21 * matrix2.M12 + matrix1.M22 * matrix2.M22 + matrix1.M23 * matrix2.M32;
        result.M23 = matrix1.M21 * matrix2.M13 + matrix1.M22 * matrix2.M23 + matrix1.M23 * matrix2.M33;
        result.M31 = matrix1.M31 * matrix2.M11 + matrix1.M32 * matrix2.M21 + matrix1.M33 * matrix2.M31;
        result.M32 = matrix1.M31 * matrix2.M12 + matrix1.M32 * matrix2.M22 + matrix1.M33 * matrix2.M32;
        result.M33 = matrix1.M31 * matrix2.M13 + matrix1.M32 * matrix2.M23 + matrix1.M33 * matrix2.M33;
        return result;
    }

    /// <summary>
    /// 计算矩阵的迹（对角线元素之和）。
    /// </summary>
    /// <returns>矩阵的迹。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real Trace()
    {
        return M11 + M22 + M33;
    }

    /// <summary>
    /// 按因子缩放矩阵。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix operator *(Real factor, in JMatrix matrix)
    {
        Multiply(matrix, factor, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 按因子缩放矩阵。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix operator *(in JMatrix matrix, Real factor)
    {
        Multiply(matrix, factor, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 将两个矩阵相加。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix operator +(in JMatrix value1, in JMatrix value2)
    {
        Add(value1, value2, out JMatrix result);
        return result;
    }

    /// <summary>
    /// 从第一个矩阵中减去第二个矩阵。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix operator -(in JMatrix value1, in JMatrix value2)
    {
        Subtract(value1, value2, out JMatrix result);
        return result;
    }

    public readonly bool Equals(JMatrix other)
    {
        return M11.Equals(other.M11) && M21.Equals(other.M21) &&
               M31.Equals(other.M31) && M12.Equals(other.M12) &&
               M22.Equals(other.M22) && M32.Equals(other.M32) &&
               M13.Equals(other.M13) && M23.Equals(other.M23) &&
               M33.Equals(other.M33);
    }

    public readonly override string ToString()
    {
        return $"M11={M11:F6}, M12={M12:F6}, M13={M13:F6}, " +
               $"M21={M21:F6}, M22={M22:F6}, M23={M23:F6}, " +
               $"M31={M31:F6}, M32={M32:F6}, M33={M33:F6}";
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JMatrix other && Equals(other);
    }

    public readonly override int GetHashCode()
    {
        var hc = new HashCode();
        hc.Add(M11); hc.Add(M21); hc.Add(M31);
        hc.Add(M12); hc.Add(M22); hc.Add(M32);
        hc.Add(M13); hc.Add(M23); hc.Add(M33);
        return hc.ToHashCode();
    }

    public static bool operator ==(JMatrix left, JMatrix right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(JMatrix left, JMatrix right)
    {
        return !(left == right);
    }
}
