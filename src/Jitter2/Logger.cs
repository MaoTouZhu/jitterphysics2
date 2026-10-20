/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;

namespace Jitter2;

/// <summary>
/// 提供日志功能。
/// </summary>
public static class Logger
{
    /// <summary>
    /// 获取或设置接收日志消息的监听器。
    /// </summary>
    public static Action<LogLevel, string>? Listener { get; set; }

    /// <summary>
    /// 定义日志消息的严重级别。
    /// </summary>
    public enum LogLevel
    {
        Information,
        Warning,
        Error
    }

    /// <summary>
    /// 记录一条信息性消息。
    /// </summary>
    /// <param name="format">要记录的消息。</param>
    public static void Information(scoped ReadOnlySpan<char> format) => Log(LogLevel.Information, format);

    /// <inheritdoc cref="Information"/>
    public static void Information<T1>(scoped ReadOnlySpan<char> format, T1 arg1) => LogFormat(LogLevel.Information, format, arg1);

    /// <inheritdoc cref="Information"/>
    public static void Information<T1, T2>(scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2) => LogFormat(LogLevel.Information, format, arg1, arg2);

    /// <inheritdoc cref="Information"/>
    public static void Information<T1, T2, T3>(scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2, T3 arg3) => LogFormat(LogLevel.Information, format, arg1, arg2, arg3);

    /// <summary>
    /// 记录一条警告消息。
    /// </summary>
    /// <param name="format">要记录的消息。</param>
    public static void Warning(scoped ReadOnlySpan<char> format) => Log(LogLevel.Warning, format);

    /// <inheritdoc cref="Warning"/>
    public static void Warning<T1>(scoped ReadOnlySpan<char> format, T1 arg1) => LogFormat(LogLevel.Warning, format, arg1);

    /// <inheritdoc cref="Warning"/>
    public static void Warning<T1, T2>(scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2) => LogFormat(LogLevel.Warning, format, arg1, arg2);

    /// <inheritdoc cref="Warning"/>
    public static void Warning<T1, T2, T3>(scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2, T3 arg3) => LogFormat(LogLevel.Warning, format, arg1, arg2, arg3);

    /// <summary>
    /// 记录一条错误消息。
    /// </summary>
    /// <param name="format">要记录的消息。</param>
    public static void Error(scoped ReadOnlySpan<char> format) => Log(LogLevel.Error, format);

    /// <inheritdoc cref="Error"/>
    public static void Error<T1>(scoped ReadOnlySpan<char> format, T1 arg1) => LogFormat(LogLevel.Error, format, arg1);

    /// <inheritdoc cref="Error"/>
    public static void Error<T1, T2>(scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2) => LogFormat(LogLevel.Error, format, arg1, arg2);

    /// <inheritdoc cref="Error"/>
    public static void Error<T1, T2, T3>(scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2, T3 arg3) => LogFormat(LogLevel.Error, format, arg1, arg2, arg3);

    /// <summary>
    /// 内部日志方法，使用给定消息调用已注册的监听器。
    /// </summary>
    /// <param name="level">消息的日志级别。</param>
    /// <param name="format">要记录的消息。</param>
    private static void Log(LogLevel level, scoped ReadOnlySpan<char> format)
    {
        Listener?.Invoke(level, format.ToString());
    }

    /// <summary>
    /// 使用一个参数格式化日志消息，并调用已注册的监听器。
    /// </summary>
    private static void LogFormat<T1>(LogLevel level, scoped ReadOnlySpan<char> format, T1 arg1)
    {
        Listener?.Invoke(level, string.Format(format.ToString(), arg1));
    }

    /// <summary>
    /// 使用两个参数格式化日志消息，并调用监听器。
    /// </summary>
    private static void LogFormat<T1, T2>(LogLevel level, scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2)
    {
        Listener?.Invoke(level, string.Format(format.ToString(), arg1, arg2));
    }

    /// <summary>
    /// 使用三个参数格式化日志消息，并调用监听器。
    /// </summary>
    private static void LogFormat<T1, T2, T3>(LogLevel level, scoped ReadOnlySpan<char> format, T1 arg1, T2 arg2, T3 arg3)
    {
        Listener?.Invoke(level, string.Format(format.ToString(), arg1, arg2, arg3));
    }
}
