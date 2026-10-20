# 你好，世界

接下来我们为场景添加物理效果。具体方式是创建 World 类的新实例，并向其中添加若干刚体。

将 `Program.cs` 的内容替换为以下代码（高亮行表示对源代码的新增部分）：

[!code-csharp[](snippets/hello-world.cs?highlight=3-6,9-18,28,41-63,88-94)]

运行程序后，你应该能看到几个箱子在重力作用下落到地面上。

![平面](images/raylibjitter.gif)
