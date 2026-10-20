/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;

namespace Jitter2;

/// <summary>模拟步及其求解器子步的计时信息。</summary>
/// <remarks>
/// 力和力矩限值在 <see cref="SubstepDt"/> 上积分。位置修正
/// 偏置使用 <see cref="InverseStepDt"/>，因此其速率按完整步长计算。
/// </remarks>
public readonly struct TimeStep
{
    /// <summary>完整模拟步的时长（秒）。</summary>
    public Real StepDt { get; }

    /// <summary>一个求解器子步的时长（秒）。</summary>
    public Real SubstepDt { get; }

    /// <summary>完整步时长的倒数；当时长为零时为零。</summary>
    public Real InverseStepDt { get; }

    /// <summary>子步时长的倒数；当时长为零时为零。</summary>
    public Real InverseSubstepDt { get; }

    /// <summary>模拟步中的子步数量。</summary>
    public int SubstepCount { get; }

    /// <summary>创建一个完整步被划分为若干子步的计时信息。</summary>
    /// <param name="dt">完整步时长（秒），必须为非负有限值。</param>
    /// <param name="substepCount">求解器子步的数量，必须为正数。</param>
    public TimeStep(Real dt, int substepCount = 1)
    {
        ArgumentCheck.NonNegative(dt, nameof(dt));
        if (substepCount < 1) throw new ArgumentOutOfRangeException(nameof(substepCount));

        StepDt = dt;
        SubstepCount = substepCount;
        SubstepDt = dt / substepCount;
        InverseStepDt = dt > 0 ? (Real)1.0 / dt : 0;
        InverseSubstepDt = SubstepDt > 0 ? (Real)1.0 / SubstepDt : 0;
    }
}
