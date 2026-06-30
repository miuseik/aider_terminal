"""
STL 网格压缩模块。
在仿真启动时自动检测并压缩过大的 STL 文件，使 PyBullet 能够正常加载。
"""

import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

import trimesh


# ---- 默认阈值 ----
MAX_SINGLE_FILE_MB = 5.0       # 单文件超过此值触发压缩
TARGET_FACES = 20000           # 压缩目标面数
TOTAL_BUDGET_MB = 200.0        # 总网格大小上限


def _get_file_size_mb(path: str) -> float:
    return os.path.getsize(path) / (1024 * 1024)


def _scan_stl_files(mesh_dir: str) -> Dict[str, float]:
    """扫描目录下所有 STL 文件及其大小。"""
    result: Dict[str, float] = {}
    for root, _, files in os.walk(mesh_dir):
        for f in files:
            if f.lower().endswith('.stl'):
                full = os.path.join(root, f)
                result[full] = _get_file_size_mb(full)
    return result


def compress_single(source_path: str, target_faces: int = TARGET_FACES,
                    overwrite: bool = False,
                    output_dir: Optional[str] = None) -> Optional[Tuple[int, int, float, float]]:
    """对单个 STL 文件进行二次误差减面压缩。

    Args:
        source_path: 原始 STL 路径
        target_faces: 目标面数
        overwrite: True 覆盖原文件, False 输出到 output_dir 或同目录 .compressed 后缀

    Returns:
        (original_faces, simplified_faces, original_mb, new_mb) 或 None（跳过/失败）
    """
    orig_mb = _get_file_size_mb(source_path)
    if orig_mb < 1.0:
        return None  # 太小，不需要压缩

    try:
        mesh = trimesh.load(source_path)
    except Exception as e:
        print(f"  ⚠️  加载失败 {os.path.basename(source_path)}: {e}")
        return None

    if not isinstance(mesh, trimesh.Trimesh) or len(mesh.faces) == 0:
        print(f"  ⚠️  {os.path.basename(source_path)} 不是有效三角网格")
        return None

    orig_faces = len(mesh.faces)
    tgt = min(target_faces, max(int(orig_faces * 0.05), 500))
    if orig_faces <= tgt:
        return None  # 已经在目标以下

    t0 = time.time()
    try:
        simplified = mesh.simplify_quadric_decimation(face_count=tgt)
    except Exception as e:
        print(f"  ✗ 减面失败 {os.path.basename(source_path)}: {e}")
        return None
    elapsed = time.time() - t0

    # 确定输出路径
    if overwrite:
        dest = source_path
    elif output_dir:
        name = os.path.basename(source_path)
        dest = os.path.join(output_dir, name)
    else:
        base, ext = os.path.splitext(source_path)
        dest = f"{base}.compressed{ext}"

    os.makedirs(os.path.dirname(dest), exist_ok=True)
    simplified.export(dest)
    new_mb = _get_file_size_mb(dest)

    print(f"  ✓ {os.path.basename(source_path)}: {orig_faces:,}→{len(simplified.faces):,}面 "
          f"({orig_mb:.1f}MB→{new_mb:.1f}MB, {elapsed:.1f}s)")

    return (orig_faces, len(simplified.faces), orig_mb, new_mb)


def compress_directory(mesh_dir: str, target_faces: int = TARGET_FACES,
                       max_single_mb: float = MAX_SINGLE_FILE_MB,
                       total_budget_mb: float = TOTAL_BUDGET_MB,
                       overwrite: bool = False) -> Tuple[float, str]:
    """扫描并压缩目录下所有过大的 STL 文件。

    Returns:
        (saved_mb, summary_message)
    """
    if not os.path.isdir(mesh_dir):
        return 0.0, f"目录不存在: {mesh_dir}"

    # 排除 compressed/ 子目录和已压缩文件
    compressed_dir = os.path.join(mesh_dir, "compressed")
    raw_files = {k: v for k, v in _scan_stl_files(mesh_dir).items()
                 if not k.startswith(compressed_dir)}
    total_mb = sum(raw_files.values())

    if total_mb < total_budget_mb and all(v < max_single_mb for v in raw_files.values()):
        return 0.0, f"所有 STL 已在阈值内 (总计 {total_mb:.1f}MB, 单文件 <{max_single_mb}MB)，无需压缩"

    # 需要压缩的大文件（跳过已有压缩版本的）
    big_files = {}
    for path, size_mb in sorted(raw_files.items(), key=lambda x: -x[1]):
        if size_mb < max_single_mb:
            continue
        name = os.path.basename(path)
        if os.path.exists(os.path.join(compressed_dir, name)):
            continue  # 已有压缩版
        big_files[path] = size_mb

    if not big_files:
        existing = sum(1 for p in raw_files
                       if os.path.exists(os.path.join(compressed_dir, os.path.basename(p))))
        msg = f"无需压缩（{existing} 个文件已有缓存, 原始总计 {total_mb:.1f}MB）"
        print(f"✅ {msg}")
        return 0.0, msg

    print(f"🔄 STL 压缩: {len(big_files)} 个文件过大 (总计 {total_mb:.1f}MB, "
          f"单文件阈值 {max_single_mb}MB)")

    count = 0
    out_dir = os.path.join(mesh_dir, "compressed") if not overwrite else None

    for path, size_mb in big_files.items():
        result = compress_single(path, target_faces=target_faces,
                                 overwrite=overwrite, output_dir=out_dir)
        if result:
            count += 1

    # 计算压缩后的总大小（优先用压缩版，无压缩版则用原版）
    compressed_set = set(os.listdir(compressed_dir)) if os.path.isdir(compressed_dir) else set()
    new_total_mb = sum(
        _get_file_size_mb(os.path.join(compressed_dir, os.path.basename(p)))
        if os.path.basename(p) in compressed_set else v
        for p, v in raw_files.items()
    )
    saved_mb = total_mb - new_total_mb
    summary = (f"压缩完成: {count}/{len(big_files)} 文件, "
               f"{total_mb:.1f}MB → {new_total_mb:.1f}MB (节省 {saved_mb:.1f}MB)")
    print(f"✅ {summary}")
    return saved_mb, summary


# ======================== 脚本入口 ========================
if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="压缩超大 STL 网格文件")
    p.add_argument("path", help="STL 文件或目录路径")
    p.add_argument("--max-mb", type=float, default=MAX_SINGLE_FILE_MB, help="单文件压缩阈值 (MB)")
    p.add_argument("--target-faces", type=int, default=TARGET_FACES, help="目标面数")
    p.add_argument("--overwrite", action="store_true", help="覆盖原文件，不创建副本")
    p.add_argument("--dry-run", action="store_true", help="仅扫描，不压缩")
    args = p.parse_args()

    if os.path.isfile(args.path):
        info = _scan_stl_files(os.path.dirname(args.path)) if args.dry_run else {}
        if args.dry_run:
            for f, s in sorted(info.items(), key=lambda x: -x[1]):
                tag = " 🔴" if s >= args.max_mb else ""
                print(f"  {s:8.1f}MB  {f}{tag}")
        else:
            compress_single(args.path, target_faces=args.target_faces)
    elif os.path.isdir(args.path):
        if args.dry_run:
            info = _scan_stl_files(args.path)
            total = sum(info.values())
            print(f"总计 {total:.1f}MB, {len(info)} 个文件:")
            for f, s in sorted(info.items(), key=lambda x: -x[1]):
                tag = " 🔴" if s >= args.max_mb else ""
                print(f"  {s:8.1f}MB  {f}{tag}")
        else:
            compress_directory(args.path, target_faces=args.target_faces,
                               max_single_mb=args.max_mb, overwrite=args.overwrite)
    else:
        print(f"路径不存在: {args.path}")
        sys.exit(1)
