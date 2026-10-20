/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 定义一种方法，用于创建形状的新实例以便与另一个刚体配合使用。
/// </summary>
/// <typeparam name="T">
/// 实现此接口的具体形状类型。
/// </typeparam>
public interface ICloneableShape<out T> where T : Shape
{
    /// <summary>
    /// 创建当前形状实例的副本，该副本共享底层几何数据。
    /// </summary>
    /// <returns>
    /// 类型为 <typeparamref name="T"/> 的新形状实例，它与原始实例共享不可变数据，
    /// 但拥有自己的实例状态。
    /// </returns>
    T Clone();
}
