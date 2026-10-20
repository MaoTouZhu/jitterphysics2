/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.Collision;
using Jitter2.Collision.Shapes;

namespace Jitter2.SoftBodies;

/// <summary>
/// 提供一个碰撞过滤器，用于拒绝属于同一软体的软体形状对，
/// 以及附着在同一刚体上的刚体形状对。混合形状对则通过。
/// </summary>
public static class DynamicTreeCollisionFilter
{
    /// <summary>
    /// 过滤拥有相同所有者的软体与刚体形状对。
    /// </summary>
    /// <param name="proxyA">第一个代理。</param>
    /// <param name="proxyB">第二个代理。</param>
    /// <returns>
    /// 如果该形状对应进行碰撞处理则为 <c>true</c>；如果应跳过则为 <c>false</c>。
    /// </returns>
    public static bool Filter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        if (proxyA is RigidBodyShape rbsA && proxyB is RigidBodyShape rbsB)
        {
            if (rbsA.RigidBody == rbsB.RigidBody) return false;
        }
        else if (proxyA is SoftBodyShape softBodyShapeA &&
                 proxyB is SoftBodyShape softBodyShapeB)
        {
            SoftBody softBodyA = softBodyShapeA.SoftBody;
            SoftBody softBodyB = softBodyShapeB.SoftBody;
            return softBodyA != softBodyB;
        }

        return true;
    }
}
