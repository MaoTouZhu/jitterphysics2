/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 为线性代数与物理计算提供数学辅助方法。
/// </summary>
public static class MathHelper
{
    /// <summary>
    /// 仅根据 IEEE-754 符号位获取 <paramref name="value"/> 的符号。
    /// </summary>
    /// <param name="value">要测试的数值。</param>
    /// <returns>
    /// 当符号位为 0 时返回 <c>+1</c>（正数、+0 或正号 NaN），
    /// 当符号位为 1 时返回 <c>-1</c>（负数、−0 或负号 NaN）。
    /// 与 <see cref="Math.Sign(float)"/> 不同，该方法从不返回 <c>0</c>。
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int SignBit(float value)
    {
        return 1 | (BitConverter.SingleToInt32Bits(value) >> 31);
    }

    /// <inheritdoc cref="SignBit(float)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int SignBit(double value)
    {
        return 1 | (int)(BitConverter.DoubleToInt64Bits(value) >> 63);
    }

    /// <summary>
    /// 使用确定性的三角函数近似，计算与给定角速度对应的旋转四元数。
    /// </summary>
    /// <param name="omega">角速度向量，单位为弧度每秒。</param>
    /// <param name="dt">时间步，单位为秒。</param>
    /// <returns>表示该旋转的单位四元数。</returns>
    public static JQuaternion RotationQuaternion(in JVector omega, Real dt)
    {
        Real angle = omega.Length();
        Real theta = angle * dt;

        if (theta < (Real)1e-3)
        {
            // 精确旋转：(omega * sin(theta/2) / |omega|, cos(theta/2))。
            // 对于较小的 theta，向量缩放为 dt/2 * (1 - theta^2/24)，
            // 标量部分为 1 - theta^2/8（泰勒展开）。
            // 使用 theta^2 可以避免构造 dt^3，后者在长扫掠中可能溢出。
            Real theta2 = theta * theta;
            Real smallAngleScale = (Real)0.5 * dt * ((Real)1.0 - theta2 / (Real)24.0);
            JVector.Multiply(omega, smallAngleScale, out var smallAngleAxis);

            Real cos = (Real)1.0 - ((Real)1.0 / (Real)8.0) * theta2;

            JQuaternion smallAngleResult = new JQuaternion(smallAngleAxis.X, smallAngleAxis.Y, smallAngleAxis.Z, cos);
            Debug.Assert(MathHelper.IsZero(smallAngleResult.Length() - 1, (Real)1e-2));
            return smallAngleResult;
        }

        Real halfAngleDt = (Real)0.5 * angle * dt;
        (Real sinD, Real cosD) = StableMath.SinCos(halfAngleDt);

        Real scale = sinD / angle;
        JVector.Multiply(omega, scale, out var axis);

        JQuaternion result = new JQuaternion(axis.X, axis.Y, axis.Z, cosD);
        Debug.Assert(MathHelper.IsZero(result.Length() - 1, (Real)1e-2));
        return result;
    }

    /// <summary>
    /// 检查矩阵是否为纯旋转矩阵。
    /// </summary>
    /// <param name="matrix">要检查的矩阵。</param>
    /// <param name="epsilon">浮点比较的容差。</param>
    /// <returns>如果矩阵是行列式为 1 的正交矩阵，则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public static bool IsRotationMatrix(in JMatrix matrix, Real epsilon = (Real)1e-06)
    {
        JMatrix delta = JMatrix.MultiplyTransposed(matrix, matrix) - JMatrix.Identity;

        if (!UnsafeIsZero(ref delta, epsilon))
        {
            return false;
        }

        return MathR.Abs(matrix.Determinant() - (Real)1.0) < epsilon;
    }

    /// <summary>
    /// 检查向量的所有分量是否都接近零。
    /// </summary>
    /// <param name="vector">要检查的向量。</param>
    /// <param name="epsilon">每个分量的容差。</param>
    /// <returns>如果没有任何分量的绝对值大于或等于 epsilon，则返回 <see langword="true"/>；
    /// NaN 分量也满足该测试。</returns>
    public static bool IsZero(in JVector vector, Real epsilon = (Real)1e-6)
    {
        return !(MathR.Abs(vector.X) >= epsilon) &&
               !(MathR.Abs(vector.Y) >= epsilon) &&
               !(MathR.Abs(vector.Z) >= epsilon);
    }

    /// <summary>
    /// 检查数值是否接近零。
    /// </summary>
    /// <param name="value">要检查的数值。</param>
    /// <param name="epsilon">容差。</param>
    /// <returns>如果绝对值小于 epsilon，则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public static bool IsZero(Real value, Real epsilon = (Real)1e-6)
    {
        return MathR.Abs(value) < epsilon;
    }

    /// <summary>
    /// 检查矩阵的所有元素是否都接近零。
    /// </summary>
    /// <param name="matrix">要检查的矩阵。</param>
    /// <param name="epsilon">每个元素的容差。</param>
    /// <returns>如果所有行都满足 <see cref="IsZero(in JVector, Real)"/>（包括包含 NaN 分量的行），则返回 <see langword="true"/>。</returns>
    public static bool UnsafeIsZero(ref JMatrix matrix, Real epsilon = (Real)1e-6)
    {
        if (!IsZero(matrix.UnsafeGet(0), epsilon)) return false;
        if (!IsZero(matrix.UnsafeGet(1), epsilon)) return false;
        if (!IsZero(matrix.UnsafeGet(2), epsilon)) return false;
        return true;
    }

    /// <summary>
    /// 使用 Jacobi 迭代近似计算对称正定矩阵的 <c>M^(-1/2)</c>。
    /// </summary>
    /// <param name="m">对称正定输入矩阵。</param>
    /// <param name="sweeps">Jacobi 迭代次数。</param>
    /// <returns><c>M</c> 的近似逆平方根。</returns>
    public static JMatrix InverseSquareRoot(JMatrix m, int sweeps = 2)
    {
        Unsafe.SkipInit(out JMatrix r);

        JMatrix rotation = JMatrix.Identity;

        for (int i = 0; i < sweeps; i++)
        {
            Real phi, cp, sp;

            // M23
            if (MathR.Abs(m.M23) > (Real)1e-6)
            {
                phi = StableMath.Atan2((Real)1.0, (m.M33 - m.M22) / ((Real)2.0 * m.M23)) / (Real)2.0;
                (sp, cp) = StableMath.SinCos(phi);
                r = new JMatrix(1, 0, 0, 0, cp, sp, 0, -sp, cp);
                JMatrix.Multiply(m, r, out m);
                JMatrix.TransposedMultiply(r, m, out m);
                JMatrix.Multiply(rotation, r, out rotation);
            }

            // M21
            if (MathR.Abs(m.M21) > (Real)1e-6)
            {
                phi = StableMath.Atan2((Real)1.0, (m.M22 - m.M11) / ((Real)2.0 * m.M21)) / (Real)2.0;
                (sp, cp) = StableMath.SinCos(phi);
                r = new JMatrix(cp, sp, 0, -sp, cp, 0, 0, 0, 1);
                JMatrix.Multiply(m, r, out m);
                JMatrix.TransposedMultiply(r, m, out m);
                JMatrix.Multiply(rotation, r, out rotation);
            }

            // M31
            if (MathR.Abs(m.M31) > (Real)1e-6)
            {
                phi = StableMath.Atan2((Real)1.0, (m.M33 - m.M11) / ((Real)2.0 * m.M31)) / (Real)2.0;
                (sp, cp) = StableMath.SinCos(phi);
                r = new JMatrix(cp, 0, sp, 0, 1, 0, -sp, 0, cp);
                JMatrix.Multiply(m, r, out m);
                JMatrix.TransposedMultiply(r, m, out m);
                JMatrix.Multiply(rotation, r, out rotation);
            }
        }

        JMatrix d = new((Real)1.0 / MathR.Sqrt(m.M11), 0, 0,
            0, (Real)1.0 / MathR.Sqrt(m.M22), 0,
            0, 0, (Real)1.0 / MathR.Sqrt(m.M33));

        return rotation * d * JMatrix.Transpose(rotation);
    }

    /// <summary>
    /// 计算与给定向量正交的单位向量。
    /// </summary>
    /// <remarks>
    /// 输入向量必须非零。调试版本会断言该条件。
    /// </remarks>
    /// <param name="vec">输入向量（必须非零，无需归一化）。</param>
    /// <returns>与输入正交的单位向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector CreateOrthonormal(in JVector vec)
    {
        Debug.Assert(!CloseToZero(vec), "Cannot create orthonormal of a zero vector");

        Real ax = MathR.Abs(vec.X);
        Real ay = MathR.Abs(vec.Y);
        Real az = MathR.Abs(vec.Z);

        JVector r;

        if (ax <= ay && ax <= az)
        {
            // (0, z, -y)
            Real y = vec.Z;
            Real z = -vec.Y;

            // invLen = 1 / sqrt(y*y + z*z)
            Real invLen = (Real)1.0 / MathR.Sqrt(y * y + z * z);

            r.X = 0;
            r.Y = y * invLen;
            r.Z = z * invLen;
        }
        else if (ay <= az)
        {
            // (-z, 0, x)
            Real x = -vec.Z;
            Real z = vec.X;

            Real invLen = (Real)1.0 / MathR.Sqrt(x * x + z * z);

            r.X = x * invLen;
            r.Y = 0;
            r.Z = z * invLen;
        }
        else
        {
            // (y, -x, 0)
            Real x = vec.Y;
            Real y = -vec.X;

            Real invLen = (Real)1.0 / MathR.Sqrt(x * x + y * y);

            r.X = x * invLen;
            r.Y = y * invLen;
            r.Z = 0;
        }

        Debug.Assert(MathR.Abs(JVector.Dot(r, vec)) < (Real)1e-6);
        return r;
    }

    /// <summary>
    /// 验证给定矩阵的各列是否构成正交基。
    /// </summary>
    /// <param name="matrix">要检查的输入矩阵。</param>
    /// <param name="epsilon">浮点比较的容差。</param>
    /// <returns>如果各列相互垂直且长度均为单位长度，则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public static bool CheckOrthonormalBasis(in JMatrix matrix, Real epsilon = (Real)1e-6)
    {
        JMatrix delta = JMatrix.MultiplyTransposed(matrix, matrix) - JMatrix.Identity;
        return UnsafeIsZero(ref delta, epsilon);
    }

    /// <summary>
    /// 计算对称半正定 3x3 矩阵（例如关节的逆有效质量）的近似
    /// Moore-Penrose 伪逆。
    /// </summary>
    /// <param name="matrix">一个有限、对称、半正定的矩阵。</param>
    /// <returns>可响应方向上的逆，被舍弃方向上的响应为零。</returns>
    /// <remarks>
    /// 当归一化行列式足够大时，使用普通逆。
    /// 否则，Jacobi 对角化会保留大于归一化容差的特征值，单精度为 1e-6，
    /// 双精度为 1e-12。归一化使用最大的绝对对角元素。零矩阵产生零伪逆。
    /// 对称性与半正定性是假定条件，不做校验。
    /// 请在约束准备阶段计算该值，并缓存结果供求解器迭代使用。
    /// </remarks>
    public static JMatrix PseudoInverseSymmetric(in JMatrix matrix)
    {
        Real scale = MathR.Max(MathR.Abs(matrix.M11),
            MathR.Max(MathR.Abs(matrix.M22), MathR.Abs(matrix.M33)));
        if (!(scale > 0) || !Real.IsFinite(scale)) return JMatrix.Zero;

        JSymmetricMatrix normalized = new(matrix.M11 / scale, matrix.M12 / scale, matrix.M13 / scale,
            matrix.M22 / scale, matrix.M23 / scale, matrix.M33 / scale);
        Real tolerance = Precision.IsDoublePrecision ? (Real)1e-12 : (Real)1e-6;
        if (MathR.Abs(normalized.Determinant()) > tolerance && JMatrix.Inverse(matrix, out JMatrix inverse))
        {
            return inverse;
        }

        Span<Real> a = stackalloc Real[9]
        {
            normalized.M11, normalized.M12, normalized.M13,
            normalized.M12, normalized.M22, normalized.M23,
            normalized.M13, normalized.M23, normalized.M33
        };
        Span<Real> basis = stackalloc Real[9] { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

        // Jacobi 旋转可以在不使用三角函数的情况下对角化对称矩阵。
        for (int sweep = 0; sweep < 8; sweep++)
        {
            bool changed = JacobiRotate(a, basis, 0, 1, tolerance);
            changed |= JacobiRotate(a, basis, 0, 2, tolerance);
            changed |= JacobiRotate(a, basis, 1, 2, tolerance);
            if (!changed) break;
        }

        JSymmetricMatrix result = JSymmetricMatrix.Zero;
        for (int i = 0; i < 3; i++)
        {
            Real eigenvalue = a[3 * i + i];
            if (!(eigenvalue > tolerance)) continue;
            Real weight = ((Real)1.0 / eigenvalue) / scale;
            JVector axis = new(basis[i], basis[3 + i], basis[6 + i]);
            result += weight * new JSymmetricMatrix(axis.X * axis.X, axis.X * axis.Y, axis.X * axis.Z,
                axis.Y * axis.Y, axis.Y * axis.Z, axis.Z * axis.Z);
        }

        return result.ToMatrix();
    }

    /// <summary>求约束响应块的逆，并以不同尺度保留相互独立的行。</summary>
    internal static JMatrix InverseSymmetric(in JMatrix matrix)
    {
        Real tolerance = Precision.IsDoublePrecision ? (Real)1e-12 : (Real)1e-6;
        Real diagonalProduct = matrix.M11 * matrix.M22 * matrix.M33;
        if (matrix.Determinant() / diagonalProduct > tolerance &&
            JMatrix.Inverse(matrix, out JMatrix inverse)) return inverse;

        // 在识别相关方向之前先对各行做均衡。一个数值很小但相互独立的行
        // 不应仅仅因为另一行强得多就消失。
        Real x = matrix.M11 > 0 ? MathR.Sqrt(matrix.M11) : 0;
        Real y = matrix.M22 > 0 ? MathR.Sqrt(matrix.M22) : 0;
        Real z = matrix.M33 > 0 ? MathR.Sqrt(matrix.M33) : 0;
        if (x == 0 && y == 0 && z == 0) return JMatrix.Zero;
        Real xy = x > 0 && y > 0 ? matrix.M12 / x / y : 0;
        Real xz = x > 0 && z > 0 ? matrix.M13 / x / z : 0;
        Real yz = y > 0 && z > 0 ? matrix.M23 / y / z : 0;
        // 零行没有耦合。单位对角元让独立的双边
        // 块可以使用普通逆；其人为响应会在下面被移除。
        JMatrix normalized = new(1, xy, xz, xy, 1, yz, xz, yz, 1);
        JMatrix response = PseudoInverseSymmetric(normalized);
        xy = x > 0 && y > 0 ? response.M12 / x / y : 0;
        xz = x > 0 && z > 0 ? response.M13 / x / z : 0;
        yz = y > 0 && z > 0 ? response.M23 / y / z : 0;

        // 这是均衡行度量下的广义逆；公开的
        // 伪逆在原始度量下保持其 Moore-Penrose 定义。
        JMatrix result = new(
            x > 0 ? response.M11 / x / x : 0,
            xy, xz, xy,
            y > 0 ? response.M22 / y / y : 0,
            yz, xz, yz,
            z > 0 ? response.M33 / z / z : 0);
        // 无法表示的逆不得向求解器引入无穷大。
        return Real.IsFinite(result.M11) && Real.IsFinite(result.M12) && Real.IsFinite(result.M13) &&
               Real.IsFinite(result.M22) && Real.IsFinite(result.M23) && Real.IsFinite(result.M33)
            ? result : JMatrix.Zero;
    }

    /// <summary>将前两个响应行的逆打包为 (M11, M12, M22)。</summary>
    internal static JVector InverseBilateralBlock(in JMatrix matrix)
    {
        JMatrix block = new(matrix.M11, matrix.M12, 0, matrix.M12, matrix.M22, 0, 0, 0, 0);
        JMatrix inverse = InverseSymmetric(block);
        return new JVector(inverse.M11, inverse.M12, inverse.M22);
    }

    private static bool JacobiRotate(Span<Real> a, Span<Real> basis, int p, int q, Real tolerance)
    {
        Real offDiagonal = a[3 * p + q];
        if (MathR.Abs(offDiagonal) <= tolerance) return false;

        Real tau = (a[3 * q + q] - a[3 * p + p]) / ((Real)2.0 * offDiagonal);
        Real tangent = MathR.CopySign((Real)1.0, tau) /
                       (MathR.Abs(tau) + MathR.Sqrt((Real)1.0 + tau * tau));
        Real cosine = (Real)1.0 / MathR.Sqrt((Real)1.0 + tangent * tangent);
        Real sine = tangent * cosine;

        a[3 * p + p] -= tangent * offDiagonal;
        a[3 * q + q] += tangent * offDiagonal;
        a[3 * p + q] = a[3 * q + p] = 0;

        for (int k = 0; k < 3; k++)
        {
            if (k != p && k != q)
            {
                Real akp = a[3 * k + p];
                Real akq = a[3 * k + q];
                a[3 * k + p] = a[3 * p + k] = cosine * akp - sine * akq;
                a[3 * k + q] = a[3 * q + k] = sine * akp + cosine * akq;
            }

            Real vkp = basis[3 * k + p];
            Real vkq = basis[3 * k + q];
            basis[3 * k + p] = cosine * vkp - sine * vkq;
            basis[3 * k + q] = sine * vkp + cosine * vkq;
        }

        return true;
    }

    /// <summary>
    /// 判断给定向量的长度是否为零或接近零。
    /// </summary>
    /// <param name="v">要评估的向量。</param>
    /// <param name="epsilonSq">平方幅值的阈值。</param>
    /// <returns>如果平方长度小于 <paramref name="epsilonSq"/>，则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public static bool CloseToZero(in JVector v, Real epsilonSq = (Real)1e-16)
    {
        return v.LengthSquared() < epsilonSq;
    }
}
