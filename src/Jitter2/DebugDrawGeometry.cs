/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2;

/// <summary>通过 <see cref="IDebugDrawer"/> 发出的与渲染器无关的绘制几何体。</summary>
/// <remarks>方向必须有限且非零。参考方向会被投影到绘制平面上。</remarks>
public static class DebugDrawGeometry
{
    /// <summary>在世界空间中两点之间绘制一条白色线段。</summary>
    public static void DrawSegment(this IDebugDrawer drawer, in JVector from, in JVector to) =>
        drawer.DrawSegment(from, to, DebugDrawColors.White);

    /// <summary>绘制一个球面点标记。</summary>
    /// <remarks>半径为零时不绘制任何内容。</remarks>
    /// <param name="drawer">接收三角形的调试绘制器。</param>
    /// <param name="point">标记在世界空间中的中心。</param>
    /// <param name="radius">球体半径，以世界单位计。必须有限且非负。</param>
    /// <param name="color">打包为 <c>0xRRGGBBAA</c> 的颜色。</param>
    public static void DrawPoint(this IDebugDrawer drawer, JVector point, Real radius, uint color)
    {
        ArgumentCheck.NonNegative(radius, nameof(radius));
        ArgumentCheck.Finite(point, nameof(point));
        if (radius == 0) return;
        foreach (ref readonly JTriangle triangle in PointMarkerMesh.Triangles)
        {
            JVector a = point + radius * triangle.V0;
            JVector b = point + radius * triangle.V1;
            JVector c = point + radius * triangle.V2;
            drawer.DrawTriangle(a, b, c, color);
        }
    }

    private static class PointMarkerMesh
    {
        private static readonly JTriangle[] triangles = CreateTriangles();
        public static ReadOnlySpan<JTriangle> Triangles => triangles;

        private static JTriangle[] CreateTriangles()
        {
            Real t = (1 + MathR.Sqrt(5)) * (Real)0.5;
            Span<JVector> vertices = stackalloc JVector[]
            {
                new(-1, t, 0), new(1, t, 0), new(-1, -t, 0), new(1, -t, 0),
                new(0, -1, t), new(0, 1, t), new(0, -1, -t), new(0, 1, -t),
                new(t, 0, -1), new(t, 0, 1), new(-t, 0, -1), new(-t, 0, 1)
            };
            for (int i = 0; i < vertices.Length; i++) vertices[i] = JVector.Normalize(vertices[i]);

            // 朝外的二十面体面。细分会保持其绕序。
            ReadOnlySpan<int> faces =
            [
                0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
                1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
                3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
                4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1
            ];
            var result = new JTriangle[80];
            int next = 0;
            for (int i = 0; i < faces.Length; i += 3)
            {
                JVector a = vertices[faces[i]], b = vertices[faces[i + 1]], c = vertices[faces[i + 2]];
                JVector ab = JVector.Normalize(a + b), bc = JVector.Normalize(b + c), ca = JVector.Normalize(c + a);
                result[next++] = new JTriangle(a, ab, ca);
                result[next++] = new JTriangle(ab, b, bc);
                result[next++] = new JTriangle(ca, bc, c);
                result[next++] = new JTriangle(ab, bc, ca);
            }
            return result;
        }
    }

    private static void Basis(JVector axis, JVector reference, out JVector first, out JVector second)
    {
        ArgumentCheck.NonZero(axis, nameof(axis));
        ArgumentCheck.Finite(reference, nameof(reference));
        axis = JVector.Normalize(axis);
        first = reference - axis * JVector.Dot(axis, reference);
        first = first.LengthSquared() > (Real)1e-12 ? JVector.Normalize(first) : MathHelper.CreateOrthonormal(axis);
        second = axis % first;
    }

    private static void ValidateCurve(Real radius, int segments)
    {
        ArgumentCheck.NonNegative(radius, nameof(radius));
        if (segments < 3 || segments > 4096) throw new ArgumentOutOfRangeException(nameof(segments));
    }

    /// <summary>绘制一段圆弧。角度从参考方向开始绕轴测量。</summary>
    public static void DrawArc(this IDebugDrawer drawer, JVector center, JVector axis, JVector reference,
        JAngle from, JAngle to, Real radius, uint color, int segments = 48)
    {
        ValidateCurve(radius, segments);
        ArgumentCheck.Finite(from, nameof(from));
        ArgumentCheck.Finite(to, nameof(to));
        Basis(axis, reference, out JVector first, out JVector second);
        Real start = from.Radian, end = to.Radian;
        JVector previous = center + radius * (MathR.Cos(start) * first + MathR.Sin(start) * second);
        for (int i = 1; i <= segments; i++)
        {
            Real angle = start * (1 - (Real)i / segments) + end * ((Real)i / segments);
            JVector point = center + radius * (MathR.Cos(angle) * first + MathR.Sin(angle) * second);
            drawer.DrawSegment(previous, point, color);
            previous = point;
        }
    }

    /// <summary>在垂直于轴的平面内绘制一个圆。</summary>
    public static void DrawCircle(this IDebugDrawer drawer, JVector center, JVector axis, JVector reference,
        Real radius, uint color, int segments = 48) =>
        drawer.DrawArc(center, axis, reference, (JAngle)0, (JAngle)(2 * MathR.PI), radius, color, segments);

    /// <summary>绘制三个互相垂直的大圆。</summary>
    public static void DrawSphere(this IDebugDrawer drawer, JVector center, Real radius, uint color, int segments = 48)
    {
        drawer.DrawCircle(center, JVector.UnitX, JVector.UnitY, radius, color, segments);
        drawer.DrawCircle(center, JVector.UnitY, JVector.UnitZ, radius, color, segments);
        drawer.DrawCircle(center, JVector.UnitZ, JVector.UnitX, radius, color, segments);
    }

    /// <summary>在球面上绘制一个椭圆锥，沿两个互相垂直的方向使用角半径。</summary>
    public static void DrawCone(this IDebugDrawer drawer, JVector origin, JVector axis, JVector reference,
        JAngle alongReference, JAngle alongPerpendicular, Real radius, uint color, int segments = 48, int spokeOffset = 0)
    {
        ValidateCurve(radius, segments);
        ArgumentCheck.NonNegative(alongReference.Radian, nameof(alongReference));
        ArgumentCheck.NonNegative(alongPerpendicular.Radian, nameof(alongPerpendicular));
        Basis(axis, reference, out JVector first, out JVector second);
        axis = JVector.Normalize(axis);
        JVector previous = default;
        int spokeStep = Math.Max(1, segments / 8);
        for (int i = 0; i <= segments; i++)
        {
            Real phase = 2 * MathR.PI * i / segments;
            Real x = alongReference.Radian * MathR.Cos(phase), y = alongPerpendicular.Radian * MathR.Sin(phase);
            Real angle = MathR.Sqrt(x * x + y * y);
            Real sinc = angle == 0 ? 1 : MathR.Sin(angle) / angle;
            JVector point = origin + radius * (MathR.Cos(angle) * axis + sinc * (x * first + y * second));
            if (i > 0) drawer.DrawSegment(previous, point, color);
            if (i < segments && i % spokeStep == spokeOffset % spokeStep) drawer.DrawSegment(origin, point, color);
            previous = point;
        }
    }

    /// <summary>绘制表示无限平面的有限网格。</summary>
    public static void DrawPlane(this IDebugDrawer drawer, JVector center, JVector normal, JVector reference,
        Real halfExtent, uint color)
    {
        ArgumentCheck.NonNegative(halfExtent, nameof(halfExtent));
        Basis(normal, reference, out JVector first, out JVector second);
        for (int i = -3; i <= 3; i++)
        {
            Real offset = halfExtent * i / 3;
            drawer.DrawSegment(center + first * offset - second * halfExtent,
                center + first * offset + second * halfExtent, color);
            drawer.DrawSegment(center + second * offset - first * halfExtent,
                center + second * offset + first * halfExtent, color);
        }
    }

    /// <summary>使用默认可视化设置绘制一个箭头。</summary>
    public static void DrawArrow(this IDebugDrawer drawer, JVector origin, JVector vector, uint color)
        => drawer.DrawArrow(origin, vector, color, DebugDrawSettings.Default);

    /// <summary>使用默认箭头头部大小绘制一个线框箭头，每个底部顶点都连接到尖端。</summary>
    /// <param name="segments">头部底部周围的段数，从 3 到 4096。默认为 12。</param>
    public static void DrawWireArrow(this IDebugDrawer drawer, JVector origin, JVector vector, uint color,
        int segments = 12)
        => DrawWireArrow(drawer, origin, vector, color, DebugDrawSettings.Default.ArrowHeadSize, segments);

    private static void DrawWireArrow(IDebugDrawer drawer, JVector origin, JVector vector, uint color,
        Real headSize, int segments)
    {
        Real length = vector.Length();
        ValidateCurve(length, segments);
        JVector tip = origin + vector;
        drawer.DrawSegment(origin, tip, color);
        if (length < (Real)1e-12) return;
        headSize = MathR.Min(headSize, length * (Real)0.5);
        JVector direction = vector * (1 / length);
        JVector sideways = MathHelper.CreateOrthonormal(direction) * (headSize * (Real)0.4);
        JVector up = direction % sideways;
        JVector root = tip - direction * headSize;
        JVector first = root + sideways, previous = first;
        for (int i = 1; i <= segments; i++)
        {
            Real angle = 2 * MathR.PI * i / segments;
            JVector point = i == segments ? first : root + MathR.Cos(angle) * sideways + MathR.Sin(angle) * up;
            drawer.DrawSegment(previous, point, color);
            drawer.DrawSegment(tip, previous, color);
            previous = point;
        }
    }

    /// <summary>使用设置中的样式、杆身粗细、箭头头部大小和细节绘制一个箭头。</summary>
    public static void DrawArrow(this IDebugDrawer drawer, JVector origin, JVector vector, uint color,
        in DebugDrawSettings settings)
    {
        if (settings.SolidArrows)
            DrawSolidArrowCore(drawer, origin, vector, settings.ArrowShaftRadius,
                settings.ArrowHeadSize, color, settings.ArrowSegments);
        else
            DrawWireArrow(drawer, origin, vector, color, settings.ArrowHeadSize,
                settings.ArrowSegments);
    }

    /// <summary>绘制一个闭合的实心箭头，带有圆柱形杆身和锥形头部。</summary>
    /// <remarks>
    /// 三角形朝外，因此可以使用背面剔除来渲染箭头。
    /// 零半径和极短的箭头用线段表示。
    /// </remarks>
    /// <param name="radius">杆身半径，以世界单位计，限制为箭头长度的 5% 和头部底部半径的一半。必须有限且非负。</param>
    /// <param name="segments">杆身和头部周围的边数，从 3 到 4096。默认为 12。</param>
    public static void DrawSolidArrow(this IDebugDrawer drawer, JVector origin, JVector vector, Real radius,
        uint color, int segments = 12)
        => DrawSolidArrowCore(drawer, origin, vector, radius, DebugDrawSettings.Default.ArrowHeadSize, color, segments);

    private static void DrawSolidArrowCore(IDebugDrawer drawer, JVector origin, JVector vector, Real radius,
        Real headSize, uint color, int segments)
    {
        Real length = vector.Length();
        ValidateCurve(radius, segments);
        ArgumentCheck.NonNegative(length, nameof(vector));
        ArgumentCheck.Finite(origin, nameof(origin));
        if (length < (Real)1e-12 || radius == 0)
        {
            drawer.DrawSegment(origin, origin + vector, color);
            return;
        }

        headSize = MathR.Min(headSize, length * (Real)0.5);
        Real headRadius = headSize * (Real)0.4;
        radius = MathR.Min(radius, MathR.Min(length * (Real)0.05, headRadius * (Real)0.5));
        JVector axis = vector * (1 / length);
        JVector first = MathHelper.CreateOrthonormal(axis), second = axis % first;
        JVector tip = origin + vector;
        JVector shoulder = tip - axis * headSize;
        JVector previous = first;
        for (int i = 1; i <= segments; i++)
        {
            Real angle = 2 * MathR.PI * i / segments;
            JVector next = i == segments ? first : MathR.Cos(angle) * first + MathR.Sin(angle) * second;
            JVector a = origin + radius * previous, b = origin + radius * next;
            JVector c = shoulder + radius * previous, d = shoulder + radius * next;
            JVector e = shoulder + headRadius * previous, f = shoulder + headRadius * next;

            drawer.DrawTriangle(a, b, c, color);
            drawer.DrawTriangle(b, d, c, color);
            drawer.DrawTriangle(origin, b, a, color);
            drawer.DrawTriangle(c, d, e, color);
            drawer.DrawTriangle(d, f, e, color);
            drawer.DrawTriangle(e, f, tip, color);
            previous = next;
        }
    }

    /// <summary>使用默认可视化设置绘制一个朝向的三个轴。</summary>
    public static void DrawFrame(this IDebugDrawer drawer, JVector center, JQuaternion orientation)
        => drawer.DrawFrame(center, orientation, DebugDrawSettings.Default);

    /// <summary>使用请求的大小和单一颜色绘制一个朝向坐标系。</summary>
    public static void DrawFrame(this IDebugDrawer drawer, JVector center, JQuaternion orientation, Real size, uint color)
        => drawer.DrawFrame(center, orientation, size, color, color, color);

    /// <summary>使用请求的大小和轴颜色绘制一个朝向坐标系。</summary>
    public static void DrawFrame(this IDebugDrawer drawer, JVector center, JQuaternion orientation, Real size,
        uint xColor, uint yColor, uint zColor)
        => drawer.DrawFrame(center, orientation, DebugDrawSettings.Default with
        {
            FrameSize = size,
            AxisXColor = xColor,
            AxisYColor = yColor,
            AxisZColor = zColor
        });

    /// <summary>使用单一颜色绘制一个线框朝向坐标系。</summary>
    /// <param name="segments">每个箭头头部底部周围的段数，从 3 到 4096。默认为 12。</param>
    public static void DrawWireFrame(this IDebugDrawer drawer, JVector center, JQuaternion orientation, Real size, uint color,
        int segments = 12)
        => drawer.DrawWireFrame(center, orientation, size, color, color, color, segments);

    /// <summary>
    /// 使用为每个局部轴单独指定的颜色绘制线框朝向坐标系。
    /// 颜色会随朝向变化而跟随各轴。
    /// </summary>
    /// <param name="xColor">局部 X 轴的颜色，打包为 <c>0xRRGGBBAA</c>。</param>
    /// <param name="yColor">局部 Y 轴的颜色，打包为 <c>0xRRGGBBAA</c>。</param>
    /// <param name="zColor">局部 Z 轴的颜色，打包为 <c>0xRRGGBBAA</c>。</param>
    /// <param name="segments">每个箭头头部底部周围的段数，从 3 到 4096。默认为 12。</param>
    public static void DrawWireFrame(this IDebugDrawer drawer, JVector center, JQuaternion orientation, Real size,
        uint xColor, uint yColor, uint zColor, int segments = 12)
    {
        drawer.DrawWireArrow(center, orientation.GetBasisX() * size, xColor, segments);
        drawer.DrawWireArrow(center, orientation.GetBasisY() * size, yColor, segments);
        drawer.DrawWireArrow(center, orientation.GetBasisZ() * size, zColor, segments);
    }

    /// <summary>
    /// 使用设置中的坐标系大小、轴颜色和箭头头部细节绘制一个朝向坐标系。
    /// 轴长度使用 FrameSize；杆身半径和箭头头部大小保持以世界单位固定。
    /// </summary>
    public static void DrawFrame(this IDebugDrawer drawer, JVector center, JQuaternion orientation,
        in DebugDrawSettings settings)
    {
        drawer.DrawArrow(center, orientation.GetBasisX() * settings.FrameSize, settings.AxisXColor, settings);
        drawer.DrawArrow(center, orientation.GetBasisY() * settings.FrameSize, settings.AxisYColor, settings);
        drawer.DrawArrow(center, orientation.GetBasisZ() * settings.FrameSize, settings.AxisZColor, settings);
    }

    /// <summary>绘制一个垂直于轴的十字，用于标记线性限位。</summary>
    public static void DrawStop(this IDebugDrawer drawer, JVector center, JVector axis, Real size, uint color)
    {
        Basis(axis, JVector.Zero, out JVector first, out JVector second);
        drawer.DrawSegment(center - first * size, center + first * size, color);
        drawer.DrawSegment(center - second * size, center + second * size, color);
    }

    /// <summary>
    /// 绘制一个弹簧。为每个弹簧保留单独的参考方向，并在每次调用时传回，
    /// 以便线圈的朝向在锚点移动时保持连续。
    /// </summary>
    public static void DrawSpring(this IDebugDrawer drawer, JVector from, JVector to, ref JVector reference,
        Real radius, uint color, int segments = 96)
    {
        ValidateCurve(radius, segments);
        JVector displacement = to - from;
        if (displacement.LengthSquared() < (Real)1e-20) return;
        Basis(JVector.Normalize(displacement), reference, out JVector first, out JVector second);
        reference = first;
        JVector previous = from;
        for (int i = 1; i <= segments; i++)
        {
            Real t = (Real)i / segments;
            Real phase = t * 20 * MathR.PI;
            Real taper = radius * MathR.Sin(t * MathR.PI);
            JVector point = i == segments ? to : from + displacement * t +
                taper * (MathR.Cos(phase) * first + MathR.Sin(phase) * second);
            drawer.DrawSegment(previous, point, color);
            previous = point;
        }
    }

    internal static void AngularRange(IDebugDrawer drawer, JVector origin, JVector axis, JVector reference,
        Real minimum, Real maximum, Real current, in DebugDrawSettings settings)
    {
        Real radius = settings.GuideSize;
        JVector other = axis % reference;
        bool fixedAngle = minimum >= maximum;
        bool fullRange = minimum <= -MathR.PI && maximum >= MathR.PI;
        if (fixedAngle)
        {
            Real angle = minimum * (Real)0.5 + maximum * (Real)0.5;
            drawer.DrawArrow(origin, radius * (MathR.Cos(angle) * reference + MathR.Sin(angle) * other),
                settings.UpperLimitColor, settings);
        }
        else
        {
            Real from = Real.Clamp(minimum, -MathR.PI, MathR.PI), to = Real.Clamp(maximum, -MathR.PI, MathR.PI);
            drawer.DrawArc(origin, axis, reference, (JAngle)from, (JAngle)to, radius, settings.UpperLimitColor, settings.Segments);
            if (!fullRange)
            {
                drawer.DrawSegment(origin, origin + radius * (MathR.Cos(from) * reference + MathR.Sin(from) * other), settings.LowerLimitColor);
                drawer.DrawSegment(origin, origin + radius * (MathR.Cos(to) * reference + MathR.Sin(to) * other), settings.UpperLimitColor);
            }
        }
        drawer.DrawArrow(origin, axis * (radius * (Real)0.4), settings.ReferenceColor, settings);
        drawer.DrawArrow(origin, radius * (MathR.Cos(current) * reference + MathR.Sin(current) * other),
            settings.CurrentColor, settings);
    }
}
