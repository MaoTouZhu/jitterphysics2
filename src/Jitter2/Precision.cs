/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

// 在此取消注释即可使用双精度构建 Jitter
// --------------------------------------------------------
// #define USE_DOUBLE_PRECISION
// --------------------------------------------------------
// 或使用命令行选项，例如，
// dotnet build -c Release -p:DoublePrecision=true

#if USE_DOUBLE_PRECISION

global using Real = System.Double;
global using MathR = System.Math;
global using Vector = System.Runtime.Intrinsics.Vector256;
global using VectorReal = System.Runtime.Intrinsics.Vector256<System.Double>;

#else

global using Real = System.Single;
global using MathR = System.MathF;
global using Vector = System.Runtime.Intrinsics.Vector128;
global using VectorReal = System.Runtime.Intrinsics.Vector128<System.Single>;

#endif

namespace Jitter2;

/// <summary>
/// 提供与浮点精度配置相关的常量和工具。
/// 通过定义 <c>USE_DOUBLE_PRECISION</c> 符号，可以将该库编译为
/// 单精度（float）或双精度（double）。
/// </summary>
public static class Precision
{
#if USE_DOUBLE_PRECISION
    /// <summary>
    /// 完整约束数据结构的字节大小。
    /// </summary>
    public const int ConstraintSizeFull = 512;

    /// <summary>
    /// 小型约束数据结构的字节大小。
    /// </summary>
    public const int ConstraintSizeSmall = 256;

    /// <summary>
    /// <see cref="Dynamics.RigidBodyData"/> 结构的字节大小。
    /// </summary>
    public const int RigidBodyDataSize = 256;
#else
    /// <summary>
    /// 完整约束数据结构的字节大小。
    /// </summary>
    public const int ConstraintSizeFull = 256;

    /// <summary>
    /// 小型约束数据结构的字节大小。
    /// </summary>
    public const int ConstraintSizeSmall = 128;

    /// <summary>
    /// <see cref="Dynamics.RigidBodyData"/> 结构的字节大小。
    /// </summary>
    public const int RigidBodyDataSize = 128;
#endif

    /// <summary>
    /// 获取一个值，指示引擎是否配置为使用双精度浮点数。
    /// </summary>
    public static bool IsDoublePrecision => sizeof(Real) == sizeof(double);
}
