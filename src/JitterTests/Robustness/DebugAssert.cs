#if DEBUG

using System.Diagnostics;
using System.Reflection;

namespace JitterTests.Robustness;

internal static class DebugAssert
{
    private static readonly FieldInfo FailCore = typeof(Debug).Assembly
        .GetType("System.Diagnostics.DebugProvider", throwOnError: true)!
        .GetField("s_FailCore", BindingFlags.Static | BindingFlags.NonPublic)!;
    private static readonly MethodInfo SetProvider = typeof(Debug)
        .GetMethod("SetProvider", BindingFlags.Static | BindingFlags.Public)!;
    private static readonly object DefaultProvider = Activator.CreateInstance(FailCore.DeclaringType!)!;

    public static void ExpectFailure(TestDelegate action, string? operationName = null)
    {
        object? previous = FailCore.GetValue(null);
        object? previousProvider = null;

        try
        {
            // 在断言处停止，避免操作访问无效的原生数据。
            FailCore.SetValue(null, (Action<string, string?, string?, string>)
                ((_, _, _, _) => throw new DebugAssertionException()));
            previousProvider = SetProvider.Invoke(null, [DefaultProvider]);
            Assert.Throws<DebugAssertionException>(action, operationName ?? string.Empty);
        }
        finally
        {
            if (previousProvider != null) SetProvider.Invoke(null, [previousProvider]);
            FailCore.SetValue(null, previous);
        }
    }

    private sealed class DebugAssertionException : Exception
    {
    }
}

#endif
