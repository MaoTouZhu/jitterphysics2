/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Runtime.CompilerServices;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 提供计算两个凸形状 Minkowski 差上各点的方法。
/// </summary>
/// <remarks>
/// Minkowski 差 A - B 定义为所有点 (a - b) 的集合，其中 a 属于 A，b 属于 B。
/// 这是 GJK 和 EPA 算法用于检测碰撞的基本构造。
/// </remarks>
public static class MinkowskiDifference
{
    /// <summary>
    /// 表示两个形状 Minkowski 差上的一个顶点。
    /// </summary>
    public struct Vertex
    {
        /// <summary>Minkowski 差上的点：<c>V = A - B</c>。</summary>
        public JVector V;

        /// <summary>形状 A 在世界空间中的支撑点。</summary>
        public JVector A;

        /// <summary>形状 B 在世界空间中的支撑点。</summary>
        public JVector B;

        /// <summary>
        /// 创建一个仅设置差值的顶点。
        /// </summary>
        /// <param name="v">Minkowski 差上的点。</param>
        public Vertex(JVector v)
        {
            V = v;
        }
    }

    /// <summary>
    /// 计算 Minkowski 差的支撑函数 S_{A-B}(d) = S_A(d) - S_B(-d)。
    /// </summary>
    /// <typeparam name="Ta">支撑形状 A 的类型。</typeparam>
    /// <typeparam name="Tb">支撑形状 B 的类型。</typeparam>
    /// <param name="supportA">形状 A 的支撑函数（位于原点，未旋转）。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationB">形状 B 的朝向。</param>
    /// <param name="positionB">形状 B 的位置。</param>
    /// <param name="direction">搜索方向。</param>
    /// <param name="v">包含两个形状支撑点的结果顶点。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void Support<Ta, Tb>(in Ta supportA, in Tb supportB, in JQuaternion orientationB,
        in JVector positionB, in JVector direction, out Vertex v) where Ta : ISupportMappable where Tb : ISupportMappable
    {
        JVector.Negate(direction, out JVector tmp);
        supportA.SupportMap(direction, out v.A);

        JVector.ConjugatedTransform(tmp, orientationB, out JVector tmp2);
        supportB.SupportMap(tmp2, out v.B);
        JVector.Transform(v.B, orientationB, out v.B);
        JVector.Add(v.B, positionB, out v.B);

        JVector.Subtract(v.A, v.B, out v.V);
    }

    /// <summary>
    /// 计算一个保证位于 Minkowski 差内部的点。
    /// </summary>
    /// <typeparam name="Ta">支撑形状 A 的类型。</typeparam>
    /// <typeparam name="Tb">支撑形状 B 的类型。</typeparam>
    /// <param name="supportA">形状 A 的支撑函数（位于原点，未旋转）。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationB">形状 B 的朝向。</param>
    /// <param name="positionB">形状 B 的位置。</param>
    /// <param name="center">表示 Minkowski 差中心的结果顶点。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void GetCenter<Ta, Tb>(in Ta supportA, in Tb supportB, in JQuaternion orientationB, in JVector positionB,
        out Vertex center) where Ta : ISupportMappable where Tb : ISupportMappable
    {
        supportA.GetCenter(out center.A);
        supportB.GetCenter(out center.B);
        JVector.Transform(center.B, orientationB, out center.B);
        JVector.Add(positionB, center.B, out center.B);
        JVector.Subtract(center.A, center.B, out center.V);
    }
}
