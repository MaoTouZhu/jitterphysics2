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
/// 表示四元数 <c>Q = x*i + y*j + z*k + w</c> 的结构。
/// <br/>
/// 采用 Hamilton 约定，其中 <c>i² = j² = k² = ijk = -1</c>。
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 4 * sizeof(Real))]
public partial struct JQuaternion(Real x, Real y, Real z, Real w) : IEquatable<JQuaternion>
{
    [FieldOffset(0 * sizeof(Real))] public Real X = x;
    [FieldOffset(1 * sizeof(Real))] public Real Y = y;
    [FieldOffset(2 * sizeof(Real))] public Real Z = z;
    [FieldOffset(3 * sizeof(Real))] public Real W = w;

    /// <summary>
    /// 获取四元数的向量部分 <c>(x, y, z)</c>。
    /// </summary>
    public JVector Vector
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Unsafe.As<JQuaternion, JVector>(ref this);
    }

    /// <summary>
    /// 获取四元数的标量部分 <c>(w)</c>。
    /// </summary>
    public Real Scalar
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => W;
    }

    /// <summary>
    /// 获取单位四元数 <c>(0, 0, 0, 1)</c>。
    /// </summary>
    public static JQuaternion Identity => new(0, 0, 0, 1);

    /// <summary>
    /// 初始化 <see cref="JQuaternion"/> 结构的新实例。
    /// </summary>
    /// <param name="w">标量（W）分量。</param>
    /// <param name="v">向量（X、Y、Z）分量。</param>
    public JQuaternion(Real w, in JVector v) : this(v.X, v.Y, v.Z, w)
    {
    }

    /// <summary>
    /// 将两个四元数逐分量相加。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <returns>两个四元数之和。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Add(in JQuaternion quaternion1, in JQuaternion quaternion2)
    {
        Add(in quaternion1, in quaternion2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 创建一个四元数，将单位向量 <paramref name="from"/> 旋转到单位向量 <paramref name="to"/>。
    /// </summary>
    /// <remarks>
    /// 此计算基于半角公式。
    /// 若两向量平行（<c>dot ≈ 1</c>），返回 Identity。
    /// 若两向量相反（<c>dot ≈ -1</c>），返回绕任意正交轴旋转 180°（π 弧度）的旋转。
    /// </remarks>
    /// <param name="from">源方向（必须是单位长度）。</param>
    /// <param name="to">目标方向（必须是单位长度）。</param>
    /// <returns>表示最短旋转的单位四元数。</returns>
    public static JQuaternion CreateFromToRotation(JVector from, JVector to)
    {
        const Real epsilon = (Real)1e-6;
        Real dot = JVector.Dot(from, to);

        // 向量方向相反（奇异情形）
        if (dot < -(Real)1.0 + epsilon)
        {
            JVector axis = MathHelper.CreateOrthonormal(from);
            return new JQuaternion(axis.X, axis.Y, axis.Z, 0);
        }

        Real s = MathR.Sqrt(((Real)1.0 + dot) * (Real)2.0);
        Real invS = (Real)1.0 / s;

        JVector c = JVector.Cross(from, to);
        return new JQuaternion(s * (Real)0.5, c * invS);
    }

    /// <summary>
    /// 将两个四元数逐分量相加。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <param name="result">输出：两个四元数之和。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Add(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
    {
        result.X = quaternion1.X + quaternion2.X;
        result.Y = quaternion1.Y + quaternion2.Y;
        result.Z = quaternion1.Z + quaternion2.Z;
        result.W = quaternion1.W + quaternion2.W;
    }

    /// <summary>
    /// 返回四元数的共轭。
    /// </summary>
    /// <remarks>
    /// 共轭定义为 <c>(-x, -y, -z, w)</c>。
    /// 对于单位四元数，共轭等价于逆。
    /// </remarks>
    /// <param name="value">要求共轭的四元数。</param>
    /// <returns>该四元数的共轭。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Conjugate(in JQuaternion value)
    {
        JQuaternion quaternion;
        quaternion.X = -value.X;
        quaternion.Y = -value.Y;
        quaternion.Z = -value.Z;
        quaternion.W = value.W;
        return quaternion;
    }

    /// <summary>
    /// 返回该四元数的共轭。
    /// </summary>
    /// <remarks>
    /// 共轭定义为 <c>(-x, -y, -z, w)</c>。
    /// </remarks>
    /// <returns>该四元数的共轭。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly JQuaternion Conjugate()
    {
        JQuaternion quaternion;
        quaternion.X = -X;
        quaternion.Y = -Y;
        quaternion.Z = -Z;
        quaternion.W = W;
        return quaternion;
    }

    /// <summary>
    /// 返回以 <c>X=..., Y=..., Z=..., W=...</c> 格式表示该四元数的字符串。
    /// </summary>
    public readonly override string ToString()
    {
        return $"X={X:F6}, Y={Y:F6}, Z={Z:F6}, W={W:F6}";
    }

    /// <summary>
    /// 将第二个四元数从第一个中逐分量减去。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <returns>两个四元数之差。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Subtract(in JQuaternion quaternion1, in JQuaternion quaternion2)
    {
        Subtract(quaternion1, quaternion2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 将第二个四元数从第一个中逐分量减去。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <param name="result">输出：两个四元数之差。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Subtract(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
    {
        result.X = quaternion1.X - quaternion2.X;
        result.Y = quaternion1.Y - quaternion2.Y;
        result.Z = quaternion1.Z - quaternion2.Z;
        result.W = quaternion1.W - quaternion2.W;
    }

    /// <summary>
    /// 将两个四元数相乘（Hamilton 积）。
    /// </summary>
    /// <remarks>
    /// 不可交换。<c>Q1 * Q2</c> 表示先旋转 Q2 再旋转 Q1（局部坐标系），或先旋转 Q1 再旋转 Q2（全局坐标系）。
    /// </remarks>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <returns>两个四元数之积。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Multiply(in JQuaternion quaternion1, in JQuaternion quaternion2)
    {
        Multiply(quaternion1, quaternion2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 计算该四元数对 X 轴 <c>(1, 0, 0)</c> 的变换。
    /// </summary>
    /// <remarks>
    /// 对于单位四元数，数学上等价于 <c>q · (1,0,0) · q⁻¹</c>。<br/>
    /// 结果：<c>[1 - 2(y² + z²), 2(xy + zw), 2(xz - yw)]</c>
    /// </remarks>
    /// <returns>变换后的向量。</returns>
    public readonly JVector GetBasisX()
    {
        JVector result;
        result.X = (Real)1.0 - (Real)2.0 * (Y * Y + Z * Z);
        result.Y = (Real)2.0 * (X * Y + Z * W);
        result.Z = (Real)2.0 * (X * Z - Y * W);
        return result;
    }

    /// <summary>
    /// 计算该四元数对 Y 轴 <c>(0, 1, 0)</c> 的变换。
    /// </summary>
    /// <remarks>
    /// 对于单位四元数，数学上等价于 <c>q · (0,1,0) · q⁻¹</c>。<br/>
    /// 结果：<c>[2(xy - zw), 1 - 2(x² + z²), 2(yz + xw)]</c>
    /// </remarks>
    /// <returns>变换后的向量。</returns>
    public readonly JVector GetBasisY()
    {
        JVector result;
        result.X = (Real)2.0 * (X * Y - Z * W);
        result.Y = (Real)1.0 - (Real)2.0 * (X * X + Z * Z);
        result.Z = (Real)2.0 * (Y * Z + X * W);
        return result;
    }

    /// <summary>
    /// 计算该四元数对 Z 轴 <c>(0, 0, 1)</c> 的变换。
    /// </summary>
    /// <remarks>
    /// 对于单位四元数，数学上等价于 <c>q · (0,0,1) · q⁻¹</c>。<br/>
    /// 结果：<c>[2(xz + yw), 2(yz - xw), 1 - 2(x² + y²)]</c>
    /// </remarks>
    /// <returns>变换后的向量。</returns>
    public readonly JVector GetBasisZ()
    {
        JVector result;
        result.X = (Real)2.0 * (X * Z + Y * W);
        result.Y = (Real)2.0 * (Y * Z - X * W);
        result.Z = (Real)1.0 - (Real)2.0 * (X * X + Y * Y);
        return result;
    }

    /// <summary>
    /// 创建一个表示绕 X 轴旋转的四元数。
    /// </summary>
    /// <param name="radians">以弧度表示的旋转角。</param>
    /// <returns>得到的四元数。</returns>
    public static JQuaternion CreateRotationX(Real radians)
    {
        Real halfAngle = radians * (Real)0.5;
        (Real sha, Real cha) = StableMath.SinCos(halfAngle);
        return new JQuaternion(sha, 0, 0, cha);
    }

    /// <summary>
    /// 创建一个表示绕 Y 轴旋转的四元数。
    /// </summary>
    /// <param name="radians">以弧度表示的旋转角。</param>
    /// <returns>得到的四元数。</returns>
    public static JQuaternion CreateRotationY(Real radians)
    {
        Real halfAngle = radians * (Real)0.5;
        (Real sha, Real cha) = StableMath.SinCos(halfAngle);
        return new JQuaternion(0, sha, 0, cha);
    }

    /// <summary>
    /// 创建一个表示绕 Z 轴旋转的四元数。
    /// </summary>
    /// <param name="radians">以弧度表示的旋转角。</param>
    /// <returns>得到的四元数。</returns>
    public static JQuaternion CreateRotationZ(Real radians)
    {
        Real halfAngle = radians * (Real)0.5;
        (Real sha, Real cha) = StableMath.SinCos(halfAngle);
        return new JQuaternion(0, 0, sha, cha);
    }

    /// <summary>
    /// 根据单位轴和角度创建一个四元数。
    /// </summary>
    /// <remarks>
    /// <paramref name="axis"/> 必须已归一化。
    /// </remarks>
    /// <param name="axis">要绕其旋转的单位向量（必须已归一化）。</param>
    /// <param name="angle">以弧度表示的旋转角。</param>
    /// <returns>表示该旋转的单位四元数。</returns>
    public static JQuaternion CreateFromAxisAngle(in JVector axis, Real angle)
    {
        Real halfAngle = angle * (Real)0.5;
        (Real s, Real c) = StableMath.SinCos(halfAngle);
        return new JQuaternion(axis.X * s, axis.Y * s, axis.Z * s, c);
    }

    /// <summary>
    /// 将单位四元数分解为轴和角度。
    /// </summary>
    /// <remarks>
    /// 假定 <paramref name="quaternion"/> 已归一化。
    /// 返回最短弧（角度在 [0, π] 范围内）。
    /// </remarks>
    /// <param name="quaternion">要分解的单位四元数。</param>
    /// <param name="axis">输出：单位旋转轴。</param>
    /// <param name="angle">输出：旋转角（弧度）。</param>
    public static void ToAxisAngle(JQuaternion quaternion, out JVector axis, out Real angle)
    {
        Real s = MathR.Sqrt(MathR.Max((Real)0.0, (Real)1.0 - quaternion.W * quaternion.W));

        const Real epsilonSingularity = (Real)1e-6;

        if (s < epsilonSingularity)
        {
            angle = (Real)0.0;
            axis = JVector.UnitX;
            return;
        }

        Real invS = (Real)1.0 / s;
        axis = new JVector(quaternion.X * invS, quaternion.Y * invS, quaternion.Z * invS);
        angle = (Real)2.0 * StableMath.Acos(quaternion.W);

        // 强制采用最短弧表示（角度在 0 到 PI 之间）
        if (angle > MathR.PI)
        {
            angle = (Real)2.0 * MathR.PI - angle;
            axis = -axis;
        }
    }

    /// <summary>
    /// 将两个四元数相乘。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <param name="result">输出：两个四元数之积。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Multiply(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
    {
        Real r1 = quaternion1.W;
        Real i1 = quaternion1.X;
        Real j1 = quaternion1.Y;
        Real k1 = quaternion1.Z;

        Real r2 = quaternion2.W;
        Real i2 = quaternion2.X;
        Real j2 = quaternion2.Y;
        Real k2 = quaternion2.Z;

        result.W = r1 * r2 - (i1 * i2 + j1 * j2 + k1 * k2);
        result.X = r1 * i2 + r2 * i1 + j1 * k2 - k1 * j2;
        result.Y = r1 * j2 + r2 * j1 + k1 * i2 - i1 * k2;
        result.Z = r1 * k2 + r2 * k1 + i1 * j2 - j1 * i2;
    }

    /// <summary>
    /// 计算 <c>conjugate(Q1) * Q2</c>。
    /// </summary>
    /// <param name="quaternion1">先求共轭再参与相乘的四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <param name="result">输出：得到的四元数。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ConjugateMultiply(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
    {
        Real r1 = quaternion1.W;
        Real i1 = -quaternion1.X;
        Real j1 = -quaternion1.Y;
        Real k1 = -quaternion1.Z;

        Real r2 = quaternion2.W;
        Real i2 = quaternion2.X;
        Real j2 = quaternion2.Y;
        Real k2 = quaternion2.Z;

        result.W = r1 * r2 - (i1 * i2 + j1 * j2 + k1 * k2);
        result.X = r1 * i2 + r2 * i1 + j1 * k2 - k1 * j2;
        result.Y = r1 * j2 + r2 * j1 + k1 * i2 - i1 * k2;
        result.Z = r1 * k2 + r2 * k1 + i1 * j2 - j1 * i2;
    }

    /// <inheritdoc cref="ConjugateMultiply(in JQuaternion, in JQuaternion, out JQuaternion)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion ConjugateMultiply(in JQuaternion quaternion1, in JQuaternion quaternion2)
    {
        ConjugateMultiply(quaternion1, quaternion2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 计算 <c>Q1 * conjugate(Q2)</c>。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">要求共轭的四元数。</param>
    /// <param name="result">输出：得到的四元数。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void MultiplyConjugate(in JQuaternion quaternion1, in JQuaternion quaternion2, out JQuaternion result)
    {
        Real r1 = quaternion1.W;
        Real i1 = quaternion1.X;
        Real j1 = quaternion1.Y;
        Real k1 = quaternion1.Z;

        Real r2 = quaternion2.W;
        Real i2 = -quaternion2.X;
        Real j2 = -quaternion2.Y;
        Real k2 = -quaternion2.Z;

        result.W = r1 * r2 - (i1 * i2 + j1 * j2 + k1 * k2);
        result.X = r1 * i2 + r2 * i1 + j1 * k2 - k1 * j2;
        result.Y = r1 * j2 + r2 * j1 + k1 * i2 - i1 * k2;
        result.Z = r1 * k2 + r2 * k1 + i1 * j2 - j1 * i2;
    }

    /// <inheritdoc cref="MultiplyConjugate(in JQuaternion, in JQuaternion, out JQuaternion)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion MultiplyConjugate(in JQuaternion quaternion1, in JQuaternion quaternion2)
    {
        MultiplyConjugate(quaternion1, quaternion2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 将四元数乘以标量因子。
    /// </summary>
    /// <param name="quaternion1">四元数。</param>
    /// <param name="scaleFactor">标量因子。</param>
    /// <returns>缩放后的四元数。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Multiply(in JQuaternion quaternion1, Real scaleFactor)
    {
        Multiply(in quaternion1, scaleFactor, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 将四元数乘以标量因子。
    /// </summary>
    /// <param name="quaternion1">四元数。</param>
    /// <param name="scaleFactor">标量因子。</param>
    /// <param name="result">输出：缩放后的四元数。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Multiply(in JQuaternion quaternion1, Real scaleFactor, out JQuaternion result)
    {
        result.W = quaternion1.W * scaleFactor;
        result.X = quaternion1.X * scaleFactor;
        result.Y = quaternion1.Y * scaleFactor;
        result.Z = quaternion1.Z * scaleFactor;
    }

    /// <summary>
    /// 计算四元数的欧几里得长度。
    /// </summary>
    /// <returns>长度（模）。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real Length()
    {
        return (Real)Math.Sqrt(X * X + Y * Y + Z * Z + W * W);
    }

    /// <summary>
    /// 计算四元数欧几里得长度的平方。
    /// </summary>
    /// <returns>长度的平方。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real LengthSquared()
    {
        return X * X + Y * Y + Z * Z + W * W;
    }

    /// <summary>
    /// 将四元数归一化为单位长度。
    /// </summary>
    [Obsolete($"In-place Normalize() is deprecated; " +
              $"use the static {nameof(JQuaternion.Normalize)} method or {nameof(JQuaternion.NormalizeInPlace)}.", true)]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Normalize()
    {
        Real lengthSquared = X * X + Y * Y + Z * Z + W * W;
        Real invLength = (Real)1.0 / MathR.Sqrt(lengthSquared);
        X *= invLength;
        Y *= invLength;
        Z *= invLength;
        W *= invLength;
    }

    /// <summary>
    /// 就地归一化给定的四元数结构。
    /// </summary>
    /// <param name="quaternion">要归一化的四元数。</param>
    public static void NormalizeInPlace(ref JQuaternion quaternion)
    {
        Real lengthSquared = quaternion.LengthSquared();
        Real invLength = (Real)1.0 / MathR.Sqrt(lengthSquared);
        quaternion.X *= invLength;
        quaternion.Y *= invLength;
        quaternion.Z *= invLength;
        quaternion.W *= invLength;
    }

    /// <summary>
    /// 返回四元数的归一化版本。
    /// </summary>
    /// <param name="value">源四元数。</param>
    /// <param name="result">输出：归一化后的单位四元数。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Normalize(in JQuaternion value, out JQuaternion result)
    {
        Real num2 = value.X * value.X + value.Y * value.Y + value.Z * value.Z + value.W * value.W;
        Real num = (Real)1.0 / MathR.Sqrt(num2);
        result.X = value.X * num;
        result.Y = value.Y * num;
        result.Z = value.Z * num;
        result.W = value.W * num;
    }

    /// <inheritdoc cref="Normalize(in JQuaternion, out JQuaternion)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Normalize(in JQuaternion value)
    {
        Normalize(value, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 从旋转矩阵创建四元数。
    /// </summary>
    /// <param name="matrix">旋转矩阵。</param>
    /// <returns>表示该旋转的四元数。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion CreateFromMatrix(in JMatrix matrix)
    {
        CreateFromMatrix(matrix, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 从旋转矩阵创建四元数。
    /// </summary>
    /// <param name="matrix">旋转矩阵。</param>
    /// <param name="result">输出：表示该旋转的四元数。</param>
    public static void CreateFromMatrix(in JMatrix matrix, out JQuaternion result)
    {
        Real t;

        if (matrix.M33 < 0)
        {
            if (matrix.M11 > matrix.M22)
            {
                t = (Real)1.0 + matrix.M11 - matrix.M22 - matrix.M33;
                result = new JQuaternion(t, matrix.M21 + matrix.M12, matrix.M31 + matrix.M13, matrix.M32 - matrix.M23);
            }
            else
            {
                t = (Real)1.0 - matrix.M11 + matrix.M22 - matrix.M33;
                result = new JQuaternion(matrix.M21 + matrix.M12, t, matrix.M32 + matrix.M23, matrix.M13 - matrix.M31);
            }
        }
        else
        {
            if (matrix.M11 < -matrix.M22)
            {
                t = (Real)1.0 - matrix.M11 - matrix.M22 + matrix.M33;
                result = new JQuaternion(matrix.M13 + matrix.M31, matrix.M32 + matrix.M23, t, matrix.M21 - matrix.M12);
            }
            else
            {
                t = (Real)1.0 + matrix.M11 + matrix.M22 + matrix.M33;
                result = new JQuaternion(matrix.M32 - matrix.M23, matrix.M13 - matrix.M31, matrix.M21 - matrix.M12, t);
            }
        }

        t = (Real)(0.5d / Math.Sqrt(t));
        result.X *= t;
        result.Y *= t;
        result.Z *= t;
        result.W *= t;
    }

    /// <summary>
    /// 计算两个四元数的点积。
    /// </summary>
    /// <param name="quaternion1">第一个四元数。</param>
    /// <param name="quaternion2">第二个四元数。</param>
    /// <returns>点积。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Real Dot(in JQuaternion quaternion1, in JQuaternion quaternion2)
    {
        return quaternion1.X * quaternion2.X +
               quaternion1.Y * quaternion2.Y +
               quaternion1.Z * quaternion2.Z +
               quaternion1.W * quaternion2.W;
    }

    /// <summary>
    /// 返回四元数的逆。
    /// </summary>
    /// <remarks>
    /// 与 <see cref="Conjugate(in JQuaternion)"/> 不同，此方法通过除以长度平方正确处理非单位四元数。
    /// 当长度平方低于 1e-12 时返回 <see cref="Identity"/>。
    /// </remarks>
    public static JQuaternion Inverse(in JQuaternion value)
    {
        Real lengthSq = value.LengthSquared();
        if (lengthSq < (Real)1e-12) return Identity;

        Real invLengthSq = (Real)1.0 / lengthSq;
        return new JQuaternion(
            -value.X * invLengthSq,
            -value.Y * invLengthSq,
            -value.Z * invLengthSq,
            value.W * invLengthSq);
    }

    /// <summary>
    /// 在两个四元数之间进行线性插值并归一化结果。
    /// </summary>
    /// <param name="quaternion1">源四元数。</param>
    /// <param name="quaternion2">目标四元数。</param>
    /// <param name="amount">插值权重。</param>
    /// <returns>插值后的单位四元数。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion Lerp(in JQuaternion quaternion1, in JQuaternion quaternion2, Real amount)
    {
        Real inverse = (Real)1.0 - amount;
        JQuaternion result;

        if (Dot(quaternion1, quaternion2) >= (Real)0.0)
        {
            result.X = inverse * quaternion1.X + amount * quaternion2.X;
            result.Y = inverse * quaternion1.Y + amount * quaternion2.Y;
            result.Z = inverse * quaternion1.Z + amount * quaternion2.Z;
            result.W = inverse * quaternion1.W + amount * quaternion2.W;
        }
        else
        {
            // 翻转第二个四元数的符号以确保走最短路径
            result.X = inverse * quaternion1.X - amount * quaternion2.X;
            result.Y = inverse * quaternion1.Y - amount * quaternion2.Y;
            result.Z = inverse * quaternion1.Z - amount * quaternion2.Z;
            result.W = inverse * quaternion1.W - amount * quaternion2.W;
        }

        Normalize(result, out result);
        return result;
    }

    /// <summary>
    /// 使用球面线性插值（SLERP）在两个四元数之间进行插值。
    /// </summary>
    /// <param name="quaternion1">源四元数。</param>
    /// <param name="quaternion2">目标四元数。</param>
    /// <param name="amount">插值权重。</param>
    /// <returns>插值后的四元数。</returns>
    public static JQuaternion Slerp(in JQuaternion quaternion1, in JQuaternion quaternion2, Real amount)
    {
        Real dot = Dot(quaternion1, quaternion2);
        JQuaternion target = quaternion2;

        // 如果点积为负，则两个四元数指向相反方向。
        // 反转其中一个可确保沿球面走最短路径。
        if (dot < (Real)0.0)
        {
            dot = -dot;
            target = -target;
        }

        const Real epsilon = (Real)1e-6;
        Real scale0, scale1;

        // 如果两个四元数非常接近，线性插值更快且安全。
        if (dot > (Real)1.0 - epsilon)
        {
            scale0 = (Real)1.0 - amount;
            scale1 = amount;
        }
        else
        {
            Real omega = StableMath.Acos(dot);
            Real invSinOmega = (Real)1.0 / StableMath.Sin(omega);
            scale0 = StableMath.Sin(((Real)1.0 - amount) * omega) * invSinOmega;
            scale1 = StableMath.Sin(amount * omega) * invSinOmega;
        }

        return new JQuaternion(
            scale0 * quaternion1.X + scale1 * target.X,
            scale0 * quaternion1.Y + scale1 * target.Y,
            scale0 * quaternion1.Z + scale1 * target.Z,
            scale0 * quaternion1.W + scale1 * target.W);
    }

    /// <summary>
    /// 翻转四元数各分量的符号。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion operator -(in JQuaternion value)
    {
        return new JQuaternion(-value.X, -value.Y, -value.Z, -value.W);
    }

    /// <summary>
    /// 将两个四元数相乘（Hamilton 积）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion operator *(in JQuaternion value1, in JQuaternion value2)
    {
        Multiply(value1, value2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 按因子缩放四元数。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion operator *(Real value1, in JQuaternion value2)
    {
        Multiply(value2, value1, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 按因子缩放四元数。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion operator *(in JQuaternion value1, Real value2)
    {
        Multiply(value1, value2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 将两个四元数逐分量相加。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion operator +(in JQuaternion value1, in JQuaternion value2)
    {
        Add(value1, value2, out JQuaternion result);
        return result;
    }

    /// <summary>
    /// 将第二个四元数从第一个中逐分量减去。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion operator -(in JQuaternion value1, in JQuaternion value2)
    {
        Subtract(value1, value2, out JQuaternion result);
        return result;
    }

    public readonly bool Equals(JQuaternion other)
    {
        return X.Equals(other.X) && Y.Equals(other.Y) && Z.Equals(other.Z) && W.Equals(other.W);
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JQuaternion other && Equals(other);
    }

    public readonly override int GetHashCode() => HashCode.Combine(X, Y, Z, W);

    public static bool operator ==(JQuaternion left, JQuaternion right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(JQuaternion left, JQuaternion right)
    {
        return !(left == right);
    }
}
