using System;
using System.IO;

namespace JitterDemo.Renderer;

public class Image
{
    public int Width { private set; get; }
    public int Height { private set; get; }

    private readonly byte[] argbData;

    private Image(byte[] argbData, int width, int height)
    {
        this.argbData = argbData;
        Width = width;
        Height = height;
    }

    /// <summary>
    /// 极简的 *.tga（Truevision TGA）加载器，支持带游程编码的真彩色图像。
    /// </summary>
    public static Image LoadImage(string filename)
    {
        const int dataOffset = 18;

        var data = File.ReadAllBytes(filename).AsSpan();

        int imageType = data[2];
        int imageWidth = (data[13] << 8) | data[12];
        int imageHeight = (data[15] << 8) | data[14];
        int bitPerPixel = data[16];
        int descriptor = data[17];

        // 图像类型 2 和 10：图像数据直接表示像素颜色。
        // 对于 15 位和 16 位的像素深度，每个像素的每种颜色占 5 位。
        // 如果像素深度为 16 位，最高位保留用于透明度。对于
        // 24 位像素深度，每个像素的每种颜色占 8 位。32 位像素
        // 深度则额外定义了一个 8 位 alpha 通道。
        // [https://en.wikipedia.org/wiki/Truevision_TGA]

        if (!(imageType is 2 or 10 && bitPerPixel is 24 or 32))
        {
            throw new Exception("Only 24bit and 32bit encoded *.tga-files supported!");
        }

        var colorData = data[dataOffset..data.Length];

        int bytesPerPixel = bitPerPixel / 8;

        bool hl = (descriptor & 0x20) == 0;
        bool vl = (descriptor & 0x10) != 0;

        int FlipImage(int index)
        {
            int row = index / imageWidth;
            int col = index - row * imageWidth;
            if (hl) row = imageHeight - 1 - row;
            if (vl) col = imageWidth - 1 - col;
            return row * imageWidth + col;
        }

        int pixelCount = imageWidth * imageHeight;

        byte[] decoded = new byte[4 * pixelCount];
        int pixelIndex = 0;

        int pos = 0;
        while (pixelIndex < pixelCount)
        {
            int skip = 0;
            int count = pixelCount;

            if (imageType == 10)
            {
                skip = (colorData[pos] & 0x80) != 0 ? 1 : 0;
                count = (colorData[pos] & 0x7F) + 1;
                pos += 1;
            }

            for (int i = 0; i < count; i++)
            {
                int idx = 4 * FlipImage(pixelIndex++);
                decoded[idx + 0] = colorData[pos + 0];
                decoded[idx + 1] = colorData[pos + 1];
                decoded[idx + 2] = colorData[pos + 2];
                decoded[idx + 3] = bytesPerPixel == 4 ? colorData[pos + 3] : (byte)0;
                pos += bytesPerPixel * (1 - skip);
            }

            pos += bytesPerPixel * skip;
        }

        return new Image(decoded, imageWidth, imageHeight);
    }

    public unsafe void FixedData(Action<Image, IntPtr> action)
    {
        fixed (void* ptr = argbData)
        {
            action(this, (IntPtr)ptr);
        }
    }
}
