## 浏览器中的 Jitter2

**浏览器中 raylib-cs 及本 README 的致谢：https://github.com/disketteman/DotnetRaylibWasm 和 https://github.com/Kiriller12/RaylibWasm**

## 环境准备

确保你安装了最新版本的 .NET 9。

安装官方 wasm 工具链：

```
dotnet workload install wasm-tools
dotnet workload install wasm-experimental
```

安装一个工具，用于创建临时 HTTP 服务器来提供 `application/wasm`：

```
dotnet tool install --global dotnet-serve
```

## 运行

`publish` 该解决方案。不要使用 `build`。发布可能需要一段时间。

```
dotnet publish -c Release
```

要提供这些文件，请使用以下命令：

```
dotnet serve --mime .wasm=application/wasm --mime .js=text/javascript --mime .json=application/json --directory ./bin/Release/net9.0/browser-wasm/AppBundle
```
