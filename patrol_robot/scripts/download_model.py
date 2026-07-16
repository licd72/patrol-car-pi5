#!/usr/bin/env python3
"""
下载并转换 YOLOv5n 模型为 ONNX 格式 (在本地 PC 或 RPi5 上运行)

用法:
  # PC 端下载, 然后 scp 到 RPi5
  python download_model.py

  # RPi5 端直接运行
  python download_model.py --device cpu
"""

import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="下载 YOLOv5n ONNX 模型")
    parser.add_argument("--output", default="models/yolov5n.onnx",
                        help="输出路径")
    parser.add_argument("--size", type=int, default=640,
                        help="模型输入尺寸")
    parser.add_argument("--device", default="cpu",
                        help="导出设备 (cpu / cuda)")
    args = parser.parse_args()

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    print(f"下载 YOLOv5n 并导出 ONNX (imgsz={args.size})...")
    print("首次运行会下载 ~4MB 权重文件\n")

    try:
        from ultralytics import YOLO
    except ImportError:
        print("请先安装 ultralytics: pip install ultralytics")
        return

    model = YOLO("yolov5n.pt")
    model.export(format="onnx", imgsz=args.size, device=args.device)

    # 移动到目标路径
    src = Path("yolov5n.onnx")
    if src.exists() and src != output:
        shutil.move(str(src), str(output))

    size_mb = output.stat().st_size / 1024 / 1024
    print(f"\n✅ 完成: {output} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
