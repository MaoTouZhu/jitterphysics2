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
/// 表示分量类型为 <see cref="Real"/> 的三维向量。
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 3 * sizeof(Real))]
public partial struct JVector(Real x, Real y, Real z) : IEquatable<JVector>
{
    internal static JVector InternalZero;
    internal static JVector Arbitrary;

    [FieldOffset(0 * sizeof(Real))] public Real X = x;
    [FieldOffset(1 * sizeof(Real))] public Real Y = y;
    [FieldOffset(2 * sizeof(Real))] public Real Z = z;

    /// <summary>零向量 (0, 0, 0)。</summary>
    public static readonly JVector Zero;

    /// <summary>沿 X 轴的单位向量 (1, 0, 0)。</summary>
    public static readonly JVector UnitX;

    /// <summary>沿 Y 轴的单位向量 (0, 1, 0)。</summary>
    public static readonly JVector UnitY;

    /// <summary>沿 Z 轴的单位向量 (0, 0, 1)。</summary>
    public static readonly JVector UnitZ;

    /// <summary>所有分量均设为一的向量 (1, 1, 1)。</summary>
    public static readonly JVector One;

    /// <summary>所有分量均设为 <see cref="Real.MinValue"/> 的向量。</summary>
    public static readonly JVector MinValue;

    /// <summary>所有分量均设为 <see cref="Real.MaxValue"/> 的向量。</summary>
    public static readonly JVector MaxValue;

    static JVector()
    {
        One = new JVector(1, 1, 1);
        Zero = new JVector(0, 0, 0);
        UnitX = new JVector(1, 0, 0);
        UnitY = new JVector(0, 1, 0);
        UnitZ = new JVector(0, 0, 1);
        MinValue = new JVector(Real.MinValue);
        MaxValue = new JVector(Real.MaxValue);
        Arbitrary = new JVector(1, 1, 1);
        InternalZero = Zero;
    }

    [Obsolete($"Do not use any longer.", true)]
    public void Set(Real x, Real y, Real z)
    {
        X = x;
        Y = y;
        Z = z;
    }

    /// <summary>
    /// 创建一个所有分量均设为同一值的向量。
    /// </summary>
    /// <param name="xyz">X、Y、Z 分量的值。</param>
    public JVector(Real xyz) : this(xyz, xyz, xyz)
    {
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public unsafe ref Real UnsafeGet(int index)
    {
        Real* ptr = (Real*)Unsafe.AsPointer(ref this);
        return ref ptr[index];
    }

    public unsafe Real this[int i]
    {
        get
        {
            fixed (Real* ptr = &X)
            {
                return ptr[i];
            }
        }
        set
        {
            fixed (Real* ptr = &X)
            {
                ptr[i] = value;
            }
        }
    }

    /// <summary>
    /// 返回 <see cref="JVector"/> 的字符串表示形式。
    /// </summary>
    public readonly override string ToString()
    {
        return $"X={X:F6}, Y={Y:F6}, Z={Z:F6}";
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JVector other && Equals(other);
    }

    public static bool operator ==(JVector value1, JVector value2)
    {
        return value1.X == value2.X && value1.Y == value2.Y && value1.Z == value2.Z;
    }

    public static bool operator !=(JVector value1, JVector value2)
    {
        if (value1.X == value2.X && value1.Y == value2.Y)
        {
            return value1.Z != value2.Z;
        }

        return true;
    }

    /// <summary>
    /// 返回一个包含指定向量各分量最小值的向量。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <returns>各分量取最小值的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Min(in JVector value1, in JVector value2)
    {
        Min(value1, value2, out JVector result);
        return result;
    }

    /// <summary>
    /// 返回一个包含指定向量各分量最小值的向量。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <param name="result">输出：各分量取最小值的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Min(in JVector value1, in JVector value2, out JVector result)
    {
        result.X = value1.X < value2.X ? value1.X : value2.X;
        result.Y = value1.Y < value2.Y ? value1.Y : value2.Y;
        result.Z = value1.Z < value2.Z ? value1.Z : value2.Z;
    }

    /// <summary>
    /// 返回一个包含指定向量各分量最大值的向量。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <returns>各分量取最大值的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Max(in JVector value1, in JVector value2)
    {
        Max(value1, value2, out JVector result);
        return result;
    }

    /// <summary>
    /// 返回一个包含指定向量各分量最大值的向量。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <param name="result">输出：各分量取最大值的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Max(in JVector value1, in JVector value2, out JVector result)
    {
        result.X = value1.X > value2.X ? value1.X : value2.X;
        result.Y = value1.Y > value2.Y ? value1.Y : value2.Y;
        result.Z = value1.Z > value2.Z ? value1.Z : value2.Z;
    }

    /// <summary>
    /// 返回一个包含指定向量各分量绝对值的向量。
    /// </summary>
    /// <param name="value1">输入向量。</param>
    /// <returns>各分量取绝对值的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Abs(in JVector value1)
    {
        return new JVector(MathR.Abs(value1.X), MathR.Abs(value1.Y), MathR.Abs(value1.Z));
    }

    /// <summary>
    /// 返回向量各分量中绝对值的最大值。
    /// </summary>
    /// <param name="value1">输入向量。</param>
    /// <returns>|X|、|Y| 和 |Z| 中的最大值。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Real MaxAbs(in JVector value1)
    {
        JVector abs = Abs(value1);
        return MathR.Max(MathR.Max(abs.X, abs.Y), abs.Z);
    }

    /// <summary>
    /// 计算 <c>matrix * vector</c>（矩阵乘以列向量）。
    /// </summary>
    /// <param name="vector">列向量。</param>
    /// <param name="matrix">矩阵。</param>
    /// <returns>变换后的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Transform(in JVector vector, in JMatrix matrix)
    {
        Transform(vector, matrix, out JVector result);
        return result;
    }

    /// <summary>
    /// 使用对称矩阵计算 <c>matrix * vector</c>，且不展开其存储。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Transform(in JVector vector, in JSymmetricMatrix matrix)
    {
        Transform(vector, matrix, out JVector result);
        return result;
    }

    /// <summary>
    /// 使用对称矩阵计算 <c>matrix * vector</c>，且不展开其存储。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Transform(in JVector vector, in JSymmetricMatrix matrix, out JVector result)
    {
        Real x = vector.X * matrix.M11 + vector.Y * matrix.M12 + vector.Z * matrix.M13;
        Real y = vector.X * matrix.M12 + vector.Y * matrix.M22 + vector.Z * matrix.M23;
        Real z = vector.X * matrix.M13 + vector.Y * matrix.M23 + vector.Z * matrix.M33;

        result.X = x;
        result.Y = y;
        result.Z = z;
    }

    /// <summary>
    /// 通过四元数旋转对向量进行变换。
    /// </summary>
    /// <param name="vector">要变换的向量。</param>
    /// <param name="quat">表示旋转的单位四元数。</param>
    /// <returns>旋转后的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Transform(in JVector vector, in JQuaternion quat)
    {
        Transform(vector, quat, out JVector result);
        return result;
    }

    /// <summary>
    /// 计算 <c>matrixᵀ * vector</c>（转置矩阵乘以列向量）。
    /// </summary>
    /// <param name="vector">列向量。</param>
    /// <param name="matrix">矩阵（运算时转置）。</param>
    /// <returns>变换后的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector TransposedTransform(in JVector vector, in JMatrix matrix)
    {
        TransposedTransform(vector, matrix, out JVector result);
        return result;
    }

    /// <summary>
    /// 通过四元数的共轭对向量进行变换（对单位四元数而言即逆旋转）。
    /// </summary>
    /// <param name="vector">要变换的向量。</param>
    /// <param name="quat">四元数（运算时取共轭）。</param>
    /// <returns>逆旋转后的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector ConjugatedTransform(in JVector vector, in JQuaternion quat)
    {
        ConjugatedTransform(vector, quat, out JVector result);
        return result;
    }

    /// <summary>
    /// 计算 <c>matrix * vector</c>（矩阵乘以列向量）。
    /// </summary>
    /// <param name="vector">列向量。</param>
    /// <param name="matrix">矩阵。</param>
    /// <param name="result">输出：变换后的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Transform(in JVector vector, in JMatrix matrix, out JVector result)
    {
        Real num0 = vector.X * matrix.M11 + vector.Y * matrix.M12 + vector.Z * matrix.M13;
        Real num1 = vector.X * matrix.M21 + vector.Y * matrix.M22 + vector.Z * matrix.M23;
        Real num2 = vector.X * matrix.M31 + vector.Y * matrix.M32 + vector.Z * matrix.M33;

        result.X = num0;
        result.Y = num1;
        result.Z = num2;
    }

    /// <summary>
    /// 计算 <c>matrixᵀ * vector</c>（转置矩阵乘以列向量）。
    /// </summary>
    /// <param name="vector">列向量。</param>
    /// <param name="matrix">矩阵（运算时转置）。</param>
    /// <param name="result">输出：变换后的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void TransposedTransform(in JVector vector, in JMatrix matrix, out JVector result)
    {
        Real num0 = vector.X * matrix.M11 + vector.Y * matrix.M21 + vector.Z * matrix.M31;
        Real num1 = vector.X * matrix.M12 + vector.Y * matrix.M22 + vector.Z * matrix.M32;
        Real num2 = vector.X * matrix.M13 + vector.Y * matrix.M23 + vector.Z * matrix.M33;

        result.X = num0;
        result.Y = num1;
        result.Z = num2;
    }

    /// <summary>
    /// 通过四元数旋转对向量进行变换。
    /// </summary>
    /// <param name="vector">要变换的向量。</param>
    /// <param name="quaternion">表示旋转的单位四元数。</param>
    /// <param name="result">输出：旋转后的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Transform(in JVector vector, in JQuaternion quaternion, out JVector result)
    {
        Real num0 = (Real)2.0 * (quaternion.Y * vector.Z - quaternion.Z * vector.Y);
        Real num1 = (Real)2.0 * (quaternion.Z * vector.X - quaternion.X * vector.Z);
        Real num2 = (Real)2.0 * (quaternion.X * vector.Y - quaternion.Y * vector.X);

        Real num00 = quaternion.Y * num2 - quaternion.Z * num1;
        Real num11 = quaternion.Z * num0 - quaternion.X * num2;
        Real num22 = quaternion.X * num1 - quaternion.Y * num0;

        result.X = vector.X + quaternion.W * num0 + num00;
        result.Y = vector.Y + quaternion.W * num1 + num11;
        result.Z = vector.Z + quaternion.W * num2 + num22;
    }

    /// <summary>
    /// 通过单位四元数的共轭对向量进行变换（逆旋转）。
    /// </summary>
    /// <param name="vector">要变换的向量。</param>
    /// <param name="quaternion">四元数（运算时取共轭）。</param>
    /// <param name="result">输出：逆旋转后的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ConjugatedTransform(in JVector vector, in JQuaternion quaternion, out JVector result)
    {
        Real num0 = (Real)2.0 * (quaternion.Z * vector.Y - quaternion.Y * vector.Z);
        Real num1 = (Real)2.0 * (quaternion.X * vector.Z - quaternion.Z * vector.X);
        Real num2 = (Real)2.0 * (quaternion.Y * vector.X - quaternion.X * vector.Y);

        Real num00 = quaternion.Z * num1 - quaternion.Y * num2;
        Real num11 = quaternion.X * num2 - quaternion.Z * num0;
        Real num22 = quaternion.Y * num0 - quaternion.X * num1;

        result.X = vector.X + quaternion.W * num0 + num00;
        result.Y = vector.Y + quaternion.W * num1 + num11;
        result.Z = vector.Z + quaternion.W * num2 + num22;
    }

    /// <summary>
    /// 计算两个向量的外积（张量积）。
    /// </summary>
    /// <remarks>
    /// 结果为矩阵 <c>u * vᵀ</c>。
    /// </remarks>
    /// <param name="u">第一个向量。</param>
    /// <param name="v">第二个向量。</param>
    /// <returns>表示外积的 3x3 矩阵。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JMatrix Outer(in JVector u, in JVector v)
    {
        JMatrix result;
        result.M11 = u.X * v.X;
        result.M12 = u.X * v.Y;
        result.M13 = u.X * v.Z;
        result.M21 = u.Y * v.X;
        result.M22 = u.Y * v.Y;
        result.M23 = u.Y * v.Z;
        result.M31 = u.Z * v.X;
        result.M32 = u.Z * v.Y;
        result.M33 = u.Z * v.Z;
        return result;
    }

    /// <summary>
    /// 计算两个向量的点积（<c>u · v</c>）。
    /// </summary>
    /// <param name="vector1">第一个向量。</param>
    /// <param name="vector2">第二个向量。</param>
    /// <returns>点积。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Real Dot(in JVector vector1, in JVector vector2)
    {
        return vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z;
    }

    /// <summary>
    /// 将两个向量相加。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <returns>两个向量之和。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Add(in JVector value1, in JVector value2)
    {
        Add(value1, value2, out JVector result);
        return result;
    }

    /// <summary>
    /// 将两个向量相加。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <param name="result">输出：两个向量之和。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Add(in JVector value1, in JVector value2, out JVector result)
    {
        result.X = value1.X + value2.X;
        result.Y = value1.Y + value2.Y;
        result.Z = value1.Z + value2.Z;
    }

    /// <summary>
    /// 从第一个向量中减去第二个向量。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <returns>两个向量之差。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Subtract(JVector value1, JVector value2)
    {
        Subtract(value1, value2, out JVector result);
        return result;
    }

    /// <summary>
    /// 从第一个向量中减去第二个向量。
    /// </summary>
    /// <param name="value1">第一个向量。</param>
    /// <param name="value2">第二个向量。</param>
    /// <param name="result">输出：两个向量之差。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Subtract(in JVector value1, in JVector value2, out JVector result)
    {
        Real num0 = value1.X - value2.X;
        Real num1 = value1.Y - value2.Y;
        Real num2 = value1.Z - value2.Z;

        result.X = num0;
        result.Y = num1;
        result.Z = num2;
    }

    /// <summary>
    /// 计算两个向量的叉积（<c>u × v</c>）。
    /// </summary>
    /// <param name="vector1">第一个向量。</param>
    /// <param name="vector2">第二个向量。</param>
    /// <returns>叉积。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Cross(in JVector vector1, in JVector vector2)
    {
        Cross(vector1, vector2, out JVector result);
        return result;
    }

    /// <summary>
    /// 计算两个向量的叉积（<c>u × v</c>）。
    /// </summary>
    /// <param name="vector1">第一个向量。</param>
    /// <param name="vector2">第二个向量。</param>
    /// <param name="result">输出：叉积。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Cross(in JVector vector1, in JVector vector2, out JVector result)
    {
        Real num0 = vector1.Y * vector2.Z - vector1.Z * vector2.Y;
        Real num1 = vector1.Z * vector2.X - vector1.X * vector2.Z;
        Real num2 = vector1.X * vector2.Y - vector1.Y * vector2.X;

        result.X = num0;
        result.Y = num1;
        result.Z = num2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly override int GetHashCode() => HashCode.Combine(X, Y, Z);

    [Obsolete($"Use static {nameof(NegateInPlace)} instead.", true)]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Negate()
    {
        X = -X;
        Y = -Y;
        Z = -Z;
    }

    /// <summary>
    /// 就地取反向量。
    /// </summary>
    /// <param name="vector">要取反的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void NegateInPlace(ref JVector vector)
    {
        vector.X = -vector.X;
        vector.Y = -vector.Y;
        vector.Z = -vector.Z;
    }

    /// <summary>
    /// 返回向量的取反副本。
    /// </summary>
    /// <param name="value">要取反的向量。</param>
    /// <returns>取反后的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Negate(in JVector value)
    {
        Negate(value, out JVector result);
        return result;
    }

    /// <summary>
    /// 返回向量的取反副本。
    /// </summary>
    /// <param name="value">要取反的向量。</param>
    /// <param name="result">输出：取反后的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Negate(in JVector value, out JVector result)
    {
        Real num0 = -value.X;
        Real num1 = -value.Y;
        Real num2 = -value.Z;

        result.X = num0;
        result.Y = num1;
        result.Z = num2;
    }

    /// <summary>
    /// 返回归一化的单位向量。
    /// </summary>
    /// <param name="value">要归一化的向量。</param>
    /// <returns>归一化的单位向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Normalize(in JVector value)
    {
        Normalize(value, out JVector result);
        return result;
    }

    /// <summary>
    /// 归一化 <paramref name="value"/>；当其平方长度低于 <paramref name="epsilonSquared"/> 时返回 <see cref="JVector.Zero"/>。
    /// </summary>
    /// <param name="value">要归一化的向量。</param>
    /// <param name="epsilonSquared"><c>‖value‖²</c> 的截断阈值；默认为 <c>1 × 10⁻¹⁶</c>。</param>
    /// <returns>单位向量或零向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector NormalizeSafe(in JVector value, Real epsilonSquared = (Real)1e-16)
    {
        Real len2 = value.X * value.X + value.Y * value.Y + value.Z * value.Z;
        if (len2 < epsilonSquared) return JVector.Zero;

        return ((Real)1.0 / MathR.Sqrt(len2)) * value;
    }

    [Obsolete($"In-place Normalize() is deprecated; " +
              $"use the static {nameof(JVector.Normalize)} method or {nameof(JVector.NormalizeInPlace)}.", true)]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Normalize()
    {
        Real lengthSquared = X * X + Y * Y + Z * Z;
        Real invLength = (Real)1.0 / MathR.Sqrt(lengthSquared);
        X *= invLength;
        Y *= invLength;
        Z *= invLength;
    }

    /// <summary>
    /// 就地归一化向量。
    /// </summary>
    /// <param name="toNormalize">要归一化的向量。</param>
    public static void NormalizeInPlace(ref JVector toNormalize)
    {
        Real lengthSquared = toNormalize.LengthSquared();
        Real invLength = (Real)1.0 / MathR.Sqrt(lengthSquared);
        toNormalize.X *= invLength;
        toNormalize.Y *= invLength;
        toNormalize.Z *= invLength;
    }

    /// <summary>
    /// 返回归一化的单位向量。
    /// </summary>
    /// <param name="value">要归一化的向量。</param>
    /// <param name="result">输出：归一化的单位向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Normalize(in JVector value, out JVector result)
    {
        Real num2 = value.X * value.X + value.Y * value.Y + value.Z * value.Z;
        Real num = (Real)1.0 / MathR.Sqrt(num2);
        result.X = value.X * num;
        result.Y = value.Y * num;
        result.Z = value.Z * num;
    }

    /// <summary>
    /// 计算向量的平方长度（<c>‖v‖²</c>）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real LengthSquared()
    {
        return X * X + Y * Y + Z * Z;
    }

    /// <summary>
    /// 计算向量的长度（<c>‖v‖</c>）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real Length()
    {
        return MathR.Sqrt(X * X + Y * Y + Z * Z);
    }

    /// <summary>
    /// 交换两个向量的值。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Swap(ref JVector vector1, ref JVector vector2)
    {
        (vector2, vector1) = (vector1, vector2);
    }

    /// <summary>
    /// 将向量乘以一个标量。
    /// </summary>
    /// <param name="value1">向量。</param>
    /// <param name="scaleFactor">标量因子。</param>
    /// <returns>缩放后的向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Multiply(in JVector value1, Real scaleFactor)
    {
        Multiply(value1, scaleFactor, out JVector result);
        return result;
    }

    /// <summary>将两个向量按分量相乘。</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector Multiply(in JVector value1, in JVector value2)
    {
        return new JVector(value1.X * value2.X, value1.Y * value2.Y, value1.Z * value2.Z);
    }

    /// <summary>
    /// 将向量乘以一个标量。
    /// </summary>
    /// <param name="value1">向量。</param>
    /// <param name="scaleFactor">标量因子。</param>
    /// <param name="result">输出：缩放后的向量。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Multiply(in JVector value1, Real scaleFactor, out JVector result)
    {
        result.X = value1.X * scaleFactor;
        result.Y = value1.Y * scaleFactor;
        result.Z = value1.Z * scaleFactor;
    }

    /// <summary>
    /// 计算两个向量的叉积（<c>u × v</c>）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator %(in JVector vector1, in JVector vector2)
    {
        JVector result;
        result.X = vector1.Y * vector2.Z - vector1.Z * vector2.Y;
        result.Y = vector1.Z * vector2.X - vector1.X * vector2.Z;
        result.Z = vector1.X * vector2.Y - vector1.Y * vector2.X;
        return result;
    }

    /// <summary>
    /// 计算两个向量的点积（<c>u · v</c>）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Real operator *(in JVector vector1, in JVector vector2)
    {
        return vector1.X * vector2.X + vector1.Y * vector2.Y + vector1.Z * vector2.Z;
    }

    /// <summary>
    /// 将向量乘以一个标量。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator *(in JVector value1, Real value2)
    {
        JVector result;
        result.X = value1.X * value2;
        result.Y = value1.Y * value2;
        result.Z = value1.Z * value2;
        return result;
    }

    /// <summary>
    /// 将向量乘以一个标量。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator *(Real value1, in JVector value2)
    {
        JVector result;
        result.X = value2.X * value1;
        result.Y = value2.Y * value1;
        result.Z = value2.Z * value1;
        return result;
    }

    /// <summary>
    /// 从第一个向量中减去第二个向量。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator -(in JVector value1, in JVector value2)
    {
        JVector result;
        result.X = value1.X - value2.X;
        result.Y = value1.Y - value2.Y;
        result.Z = value1.Z - value2.Z;
        return result;
    }

    /// <summary>
    /// 对向量取反。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator -(in JVector left)
    {
        return Multiply(left, -(Real)1.0);
    }

    /// <summary>
    /// 返回向量本身。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator +(in JVector left)
    {
        return left;
    }

    /// <summary>
    /// 将两个向量相加。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector operator +(in JVector value1, in JVector value2)
    {
        JVector result;
        result.X = value1.X + value2.X;
        result.Y = value1.Y + value2.Y;
        result.Z = value1.Z + value2.Z;

        return result;
    }

    public readonly bool Equals(JVector other)
    {
        return X.Equals(other.X) && Y.Equals(other.Y) && Z.Equals(other.Z);
    }
}
