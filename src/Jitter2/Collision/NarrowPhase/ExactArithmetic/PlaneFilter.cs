/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

// 用于 dot(cross(a-b, a-c), p-a) 的缓存浮点过滤器。
// 它只接受能够被证明为非零的符号。不确定的符号（包括精确的零）
// 必须由调用方用精确算术求值。
// 调用方在构造或查询它之前，用 IsSafe 校验所有坐标。
internal struct PlaneFilter
{
    private const double UnitRoundoff = 1.1102230246251565e-16; // 2^-53，而不是 double.Epsilon。
    private const double ErrorFactor = 16 * UnitRoundoff;
    private readonly double errorX, errorY, errorZ;
    public double NormalX, NormalY, NormalZ;

    // 几何计算仍使用近似法线。对于条件较差的叉积，在转换为
    // double 之前通过精确展开重新求值。
    public readonly bool ReliableNormal => Math.Max(Math.Abs(NormalX), Math.Max(Math.Abs(NormalY), Math.Abs(NormalZ))) >
        1048576 * Math.Max(errorX, Math.Max(errorY, errorZ));

    public PlaneFilter(in JVector a, in JVector b, in JVector c)
    {
        double ux = (double)a.X - b.X, uy = (double)a.Y - b.Y, uz = (double)a.Z - b.Z;
        double vx = (double)a.X - c.X, vy = (double)a.Y - c.Y, vz = (double)a.Z - c.Z;
        double yz = uy * vz, zy = uz * vy;
        double zx = uz * vx, xz = ux * vz;
        double xy = ux * vy, yx = uy * vx;
        NormalX = yz - zy;
        NormalY = zx - xz;
        NormalZ = xy - yx;

        // 两个舍入的坐标差、一次乘积和一次减法
        // 给出小于 5u 乘以舍入乘积绝对值之和的误差。16u 为
        // 计算误差界本身时的舍入留出余量。
        errorX = ErrorFactor * (Math.Abs(yz) + Math.Abs(zy));
        errorY = ErrorFactor * (Math.Abs(zx) + Math.Abs(xz));
        errorZ = ErrorFactor * (Math.Abs(xy) + Math.Abs(yx));
    }

    public void Negate()
    {
        NormalX = -NormalX;
        NormalY = -NormalY;
        NormalZ = -NormalZ;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly bool TrySide(in JVector anchor, in JVector point, out int sign)
    {
        sign = 0;
        double dx = (double)point.X - anchor.X, dy = (double)point.Y - anchor.Y, dz = (double)point.Z - anchor.Z;
        double x = NormalX * dx, y = NormalY * dy, z = NormalZ * dz;
        double value = (x + y) + z;
        double roundoff = ErrorFactor * ((Math.Abs(x) + Math.Abs(y)) + Math.Abs(z));
        double coefficientError = (errorX * Math.Abs(dx) + errorY * Math.Abs(dy)) + errorZ * Math.Abs(dz);
        // 16u 覆盖舍入的位移与点积（< 8u）。
        // 因子二还覆盖系数误差求和的舍入。
        double bound = roundoff + 2 * coefficientError;
        if (Math.Abs(value) <= bound) return false;
        sign = value > 0 ? 1 : -1;
        return true;
    }

    // 位于 [2^-200, 2^200] 内的非零坐标可确保坐标差、
    // 所有乘积和误差界在 double 中保持正规且有限。
    // 此窗口之外的值不受支持，包括 double 次正规数。
    public static bool IsSafe(in JVector point) => SafeCoordinate(point.X) && SafeCoordinate(point.Y) && SafeCoordinate(point.Z);

    private static bool SafeCoordinate(double value) => value == 0 ||
        (Math.Abs(value) >= 6.223015277861142e-61 && Math.Abs(value) <= 1.6069380442589903e60);
}
