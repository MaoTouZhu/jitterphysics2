/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Diagnostics;
using Jitter2.LinearMath;

namespace Jitter2;

/// <summary>
/// 在 Debug 构建中运行额外的输入健全性检查。
/// </summary>
/// <remarks>
/// 仅当定义了 <c>DEBUG</c> 符号时，才会编译对此辅助类的调用。它用于
/// 运行时状态变更路径，这些路径中的无效值应在开发期间被捕获，而不会
/// 增加 Release 构建的开销。
/// </remarks>
internal static class DebugCheck
{
    [Conditional("DEBUG")]
    public static void IsFinite(Real value, string paramName) => ArgumentCheck.Finite(value, paramName);

    [Conditional("DEBUG")]
    public static void IsFinite(JAngle value, string paramName) => ArgumentCheck.Finite(value, paramName);

    [Conditional("DEBUG")]
    public static void IsNotNaN(Real value, string paramName) => ArgumentCheck.NotNaN(value, paramName);

    [Conditional("DEBUG")]
    public static void IsNotNaN(JAngle value, string paramName) => ArgumentCheck.NotNaN(value, paramName);

    [Conditional("DEBUG")]
    public static void IsFinite(in JVector value, string paramName) => ArgumentCheck.Finite(value, paramName);

    [Conditional("DEBUG")]
    public static void IsFinite(in JQuaternion value, string paramName) => ArgumentCheck.Finite(value, paramName);

    [Conditional("DEBUG")]
    public static void IsFinite(in JMatrix value, string paramName) => ArgumentCheck.Finite(value, paramName);

    [Conditional("DEBUG")]
    public static void IsNonNegative(Real value, string paramName) => ArgumentCheck.NonNegative(value, paramName);

    [Conditional("DEBUG")]
    public static void IsPositive(Real value, string paramName) => ArgumentCheck.Positive(value, paramName);

    [Conditional("DEBUG")]
    public static void IsInRange(Real value, Real min, Real max, string paramName) => ArgumentCheck.InRange(value, min, max, paramName);

    [Conditional("DEBUG")]
    public static void IsNonZero(in JVector value, string paramName) => ArgumentCheck.NonZero(value, paramName);

    [Conditional("DEBUG")]
    public static void IsUnitVector(in JVector value, string paramName) => ArgumentCheck.UnitVector(value, paramName);

    [Conditional("DEBUG")]
    public static void IsUnitQuaternion(in JQuaternion value, string paramName) => ArgumentCheck.UnitQuaternion(value, paramName);
}
