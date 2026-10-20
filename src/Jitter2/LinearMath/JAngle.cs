/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.InteropServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 一个表示角度的 <see cref="Real"/> 类型浮点变量。此结构的存在是为了消除
/// Jitter API 中弧度与角度之间的歧义。
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 1 * sizeof(Real))]
public struct JAngle : IEquatable<JAngle>
{
    /// <summary>
    /// 获取或设置以弧度为单位的角度值。
    /// </summary>
    [field: FieldOffset(0 * sizeof(Real))]
    public Real Radian { get; set; }

    /// <summary>
    /// 返回 <see cref="JAngle"/> 的字符串表示。
    /// </summary>
    public readonly override string ToString()
    {
        return $"Radian={Radian}, Degree={Degree}";
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JAngle other && Equals(other);
    }

    public readonly bool Equals(JAngle p)
    {
        return p.Radian == Radian;
    }

    public readonly override int GetHashCode()
    {
        return Radian.GetHashCode();
    }

    /// <summary>
    /// 获取或设置以度为单位的角度值。
    /// </summary>
    public Real Degree
    {
        readonly get => Radian / MathR.PI * (Real)180.0;
        set => Radian = value / (Real)180.0 * MathR.PI;
    }

    /// <summary>
    /// 从以弧度表示的值创建 <see cref="JAngle"/>。
    /// </summary>
    public static JAngle FromRadian(Real rad)
    {
        return new JAngle { Radian = rad };
    }

    /// <summary>
    /// 从以度表示的值创建 <see cref="JAngle"/>。
    /// </summary>
    public static JAngle FromDegree(Real deg)
    {
        return new JAngle { Degree = deg };
    }

    public static explicit operator JAngle(Real angle)
    {
        return FromRadian(angle);
    }

    public static JAngle operator -(JAngle a)
    {
        return FromRadian(-a.Radian);
    }

    public static JAngle operator +(JAngle a, JAngle b)
    {
        return FromRadian(a.Radian + b.Radian);
    }

    public static JAngle operator -(JAngle a, JAngle b)
    {
        return FromRadian(a.Radian - b.Radian);
    }

    public static bool operator ==(JAngle l, JAngle r)
    {
        return (Real)l == (Real)r;
    }

    public static bool operator !=(JAngle l, JAngle r)
    {
        return (Real)l != (Real)r;
    }

    public static bool operator <(JAngle l, JAngle r)
    {
        return (Real)l < (Real)r;
    }

    public static bool operator >(JAngle l, JAngle r)
    {
        return (Real)l > (Real)r;
    }

    public static bool operator >=(JAngle l, JAngle r)
    {
        return (Real)l >= (Real)r;
    }

    public static bool operator <=(JAngle l, JAngle r)
    {
        return (Real)l <= (Real)r;
    }

    public static explicit operator Real(JAngle angle)
    {
        return angle.Radian;
    }
}
