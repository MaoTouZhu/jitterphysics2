/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 内部三角函数辅助方法，用于在不同平台上获得逐位相同的结果（Math 与 MathF 不保证这一点）。
/// </summary>
/// <remarks>
/// 在 BCL 中的标量三角函数既具有确定性、又在各目标平台上由托管源码固定之前，
/// 该辅助类保留自己的一套微型近似流程，使物理引擎掌控其行为。
/// </remarks>
internal static class StableMath
{
    internal const Real Pi = (Real)3.141592653589793238462643383279502884;
    internal const Real HalfPi = (Real)1.570796326794896619231321691639751442;
    internal const Real QuarterPi = (Real)0.785398163397448309615660845819875721;
    internal const Real TwoPi = (Real)6.283185307179586476925286766559005768;

    // 供 atan 的角度加法恒等式使用：atan(x) = pi/4 + atan((x - 1) / (x + 1))。
    private const Real TanPiOver8 = (Real)0.414213562373095048801688724209698079;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int FloorToInt(Real value)
    {
        // 强制转换会向零截断。归约器需要数学意义上的 floor，以便负
        // 角度在每个平台上落入相同的桶，且象限边界保持对称。
        int integer = (int)value;
        return value < integer ? integer - 1 : integer;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real ReduceAngle(Real angle)
    {
        // 这里有意采用“朴素”归约器：以当前 Real 精度做一次对 2*pi 取模，将角度归入 [-pi, pi]。
        // 它不是 Payne-Hanek/Cody-Waite 风格的高精度归约器。
        // 这意味着极大的输入在多项式运行之前就可能丢失低位，但对于
        // 求解器中出现的角度量级来说，这是一个不错的权衡：代码极小、数据流确定，
        // 且不依赖平台 libm 的内部实现。
        int periods = FloorToInt((angle + Pi) / TwoPi);
        angle -= periods * TwoPi;

        if (angle > Pi) angle -= TwoPi;
        else if (angle <= -Pi) angle += TwoPi;

        return angle;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ReduceToQuadrant(Real angle, out int quadrant, out Real reduced)
    {
        angle = ReduceAngle(angle);

        // 再折叠一次到 [-pi/4, pi/4]。返回的象限保存了在低阶多项式求值之后
        // 重建原始正弦/余弦对所需的交换与符号翻转。
        int nearestQuarterTurn = FloorToInt((angle + QuarterPi) / HalfPi);

        reduced = angle - nearestQuarterTurn * HalfPi;
        quadrant = nearestQuarterTurn & 3;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real SinPolynomial(Real x)
    {
        // sin(x) 的 13 阶 Maclaurin 近似，以 Horner 形式求值。
        //
        // 上面的归约正是其可用性的关键。在精确算术下，[-pi/4, pi/4] 上的截断余项
        // 以 |x|^15 / 15! 为界，在端点处约为 2.04e-14。
        // 如果跳过归约，直接在 [-pi, pi] 上运行同一多项式，密集 float 扫描
        // 的最大绝对误差约为 2.14e-5。因此象限折叠并非可选的细节；
        // 它正是这个低阶泰勒多项式能用于该引擎的原因。
        Real x2 = x * x;
        Real poly = -(Real)(1.0 / 6227020800.0);
        poly = poly * x2 + (Real)(1.0 / 39916800.0);
        poly = poly * x2 - (Real)(1.0 / 362880.0);
        poly = poly * x2 + (Real)(1.0 / 5040.0);
        poly = poly * x2 - (Real)(1.0 / 120.0);
        poly = poly * x2 + (Real)(1.0 / 6.0);
        poly = poly * x2 - (Real)1.0;
        return -x * poly;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real CosPolynomial(Real x)
    {
        // cos(x) 的 12 阶 Maclaurin 近似，同样采用 Horner 形式。
        //
        // 在 [-pi/4, pi/4] 上，精确级数的截断余项以 |x|^14 / 14! 为界，端点处约为
        // 3.90e-13。若不做归约，同样的系数就远没那么可接受：
        // 在 [-pi, pi] 上做密集 float 扫描，最大绝对误差约为 1.01e-4。
        Real x2 = x * x;
        Real poly = -(Real)(1.0 / 479001600.0);
        poly = poly * x2 + (Real)(1.0 / 3628800.0);
        poly = poly * x2 - (Real)(1.0 / 40320.0);
        poly = poly * x2 + (Real)(1.0 / 720.0);
        poly = poly * x2 - (Real)(1.0 / 24.0);
        poly = poly * x2 + (Real)(1.0 / 2.0);
        poly = poly * x2 - (Real)1.0;
        return -poly;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (Real sin, Real cos) ApplyQuadrant(int quadrant, Real sin, Real cos)
    {
        // 撤销 ReduceToQuadrant 中的八分圆/象限折叠。
        return quadrant switch
        {
            0 => (sin, cos),
            1 => (cos, -sin),
            2 => (-sin, -cos),
            _ => (-cos, sin)
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real ApplyQuadrantSin(int quadrant, Real sin, Real cos)
    {
        return quadrant switch
        {
            0 => sin,
            1 => cos,
            2 => -sin,
            _ => -cos
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real ApplyQuadrantCos(int quadrant, Real sin, Real cos)
    {
        return quadrant switch
        {
            0 => cos,
            1 => -sin,
            2 => -cos,
            _ => sin
        };
    }

    internal static (Real sin, Real cos) SinCos(Real angle)
    {
        if (angle >= -QuarterPi && angle <= QuarterPi)
        {
            return (SinPolynomial(angle), CosPolynomial(angle));
        }

        // 位于最小多项式区间之外的所有输入都会经过常规范围归约器。
        ReduceToQuadrant(angle, out int quadrant, out Real reduced);

        Real sin = SinPolynomial(reduced);
        Real cos = CosPolynomial(reduced);

        return ApplyQuadrant(quadrant, sin, cos);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Real Sin(Real angle)
    {
        if (angle >= -QuarterPi && angle <= QuarterPi)
        {
            return SinPolynomial(angle);
        }

        ReduceToQuadrant(angle, out int quadrant, out Real reduced);

        // 单输出路径保留相同的归约逻辑，但避免计算
        // 所选象限不需要的多项式。
        return quadrant switch
        {
            0 => SinPolynomial(reduced),
            1 => CosPolynomial(reduced),
            2 => -SinPolynomial(reduced),
            _ => -CosPolynomial(reduced)
        };
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Real Cos(Real angle)
    {
        if (angle >= -QuarterPi && angle <= QuarterPi)
        {
            return CosPolynomial(angle);
        }

        ReduceToQuadrant(angle, out int quadrant, out Real reduced);

        return quadrant switch
        {
            0 => CosPolynomial(reduced),
            1 => -SinPolynomial(reduced),
            2 => -CosPolynomial(reduced),
            _ => SinPolynomial(reduced)
        };
    }

    private static Real AtanTaylor(Real value)
    {
        // atan(x) 的 17 阶奇泰勒级数，采用 Horner 形式。调用方保证 |x| 足够小，
        // 使 x = 1 附近出了名缓慢的收敛不会主导误差。
        Real x2 = value * value;
        Real poly = (Real)(1.0 / 17.0);
        poly = poly * x2 - (Real)(1.0 / 15.0);
        poly = poly * x2 + (Real)(1.0 / 13.0);
        poly = poly * x2 - (Real)(1.0 / 11.0);
        poly = poly * x2 + (Real)(1.0 / 9.0);
        poly = poly * x2 - (Real)(1.0 / 7.0);
        poly = poly * x2 + (Real)(1.0 / 5.0);
        poly = poly * x2 - (Real)(1.0 / 3.0);
        poly = poly * x2 + (Real)1.0;
        return value * poly;
    }

    private static Real Atan(Real value)
    {
        if (value < (Real)0.0) return -Atan(-value);

        if (value > (Real)1.0)
        {
            // atan(x) = pi/2 - atan(1/x)
            return HalfPi - Atan((Real)1.0 / value);
        }

        if (value > TanPiOver8)
        {
            // atan(x) = pi/4 + atan((x - 1) / (x + 1))。经过该变换后，泰勒级数
            // 只需处理最大到 tan(pi/8) ~= 0.4142 的值，而不必直接硬扛 x ~= 1 的缓慢情形。
            Real reduced = (value - (Real)1.0) / (value + (Real)1.0);
            return QuarterPi + AtanTaylor(reduced);
        }

        return AtanTaylor(value);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Real AsinTaylor(Real value)
    {
        // asin(x) 的 15 阶 Maclaurin 近似。我们只直接向它输入 |x| <= 0.5；
        // 在端点附近，Asin/Acos 会切换到半角形式，使多项式仍然看到较小的 x。
        Real x2 = value * value;
        Real poly = (Real)(143.0 / 10240.0);
        poly = poly * x2 + (Real)(231.0 / 13312.0);
        poly = poly * x2 + (Real)(63.0 / 2816.0);
        poly = poly * x2 + (Real)(35.0 / 1152.0);
        poly = poly * x2 + (Real)(5.0 / 112.0);
        poly = poly * x2 + (Real)(3.0 / 40.0);
        poly = poly * x2 + (Real)(1.0 / 6.0);
        poly = poly * x2 + (Real)1.0;
        return value * poly;
    }

    internal static Real Atan2(Real y, Real x)
    {
        // 围绕上述标量 atan 近似进行经典的象限重建。
        if (x > (Real)0.0)
        {
            return Atan(y / x);
        }

        if (x < (Real)0.0)
        {
            return y >= (Real)0.0
                ? Atan(y / x) + Pi
                : Atan(y / x) - Pi;
        }

        if (y > (Real)0.0) return HalfPi;
        if (y < (Real)0.0) return -HalfPi;

        return (Real)0.0;
    }

    internal static Real Acos(Real value)
    {
        value = Math.Clamp(value, (Real)(-1.0), (Real)1.0);

        if (value > (Real)0.5)
        {
            // acos(x) = 2 * asin(sqrt((1 - x) / 2))
            Real reduced = MathR.Sqrt(MathR.Max((Real)0.0, ((Real)1.0 - value) * (Real)0.5));
            return (Real)2.0 * AsinTaylor(reduced);
        }

        if (value < (Real)(-0.5))
        {
            // acos(x) = pi - 2 * asin(sqrt((1 + x) / 2))
            Real reduced = MathR.Sqrt(MathR.Max((Real)0.0, ((Real)1.0 + value) * (Real)0.5));
            return Pi - (Real)2.0 * AsinTaylor(reduced);
        }

        return HalfPi - AsinTaylor(value);
    }

    internal static Real Asin(Real value)
    {
        value = Math.Clamp(value, (Real)(-1.0), (Real)1.0);
        Real absValue = MathR.Abs(value);

        if (absValue <= (Real)0.5)
        {
            return AsinTaylor(value);
        }

        // asin(x) = pi/2 - 2 * asin(sqrt((1 - |x|) / 2))
        Real reduced = MathR.Sqrt(MathR.Max((Real)0.0, ((Real)1.0 - absValue) * (Real)0.5));
        Real angle = HalfPi - (Real)2.0 * AsinTaylor(reduced);

        return value < (Real)0.0 ? -angle : angle;
    }
}
