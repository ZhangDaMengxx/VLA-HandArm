"""列出重定向 URDF 的所有 link、有没有 visual、网格文件在不在磁盘上(Linux 区分大小写)。"""
from pathlib import Path
import xml.etree.ElementTree as ET
import tyro
from dex_retargeting.retargeting_config import RetargetingConfig


def resolve(fn, urdf_path, pkg_dirs):
    stripped = fn.replace("package://", "")
    cands = [urdf_path.parent / fn, urdf_path.parent / stripped]
    for d in pkg_dirs:
        cands += [Path(d) / stripped, Path(d) / fn]
    for c in cands:
        if c.exists():
            return fn, True
    return fn, False


def main(config_path: str):
    robot_dir = Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands"
    RetargetingConfig.set_default_urdf_dir(str(robot_dir))
    config = RetargetingConfig.load_from_file(config_path)
    urdf_path = Path(config.urdf_path)
    if not urdf_path.is_absolute():
        urdf_path = robot_dir / urdf_path
    urdf_path = urdf_path.resolve()
    pkg_dirs = [str(urdf_path.parent), str(robot_dir), str(robot_dir.parent), str(robot_dir.parent.parent)]
    print("URDF:", urdf_path, "\n")
    root = ET.parse(urdf_path).getroot()
    print(f"{'link':<30}{'visual?':<9}{'mesh?':<12} filename")
    for link in root.findall("link"):
        name = link.get("name")
        vis = link.find("visual")
        if vis is None:
            print(f"{name:<30}{'no':<9}{'-':<12}")
            continue
        mesh = vis.find("geometry/mesh")
        if mesh is None:
            print(f"{name:<30}{'yes':<9}{'primitive':<12}")
            continue
        fn = mesh.get("filename")
        _, ok = resolve(fn, urdf_path, pkg_dirs)
        print(f"{name:<30}{'yes':<9}{('YES' if ok else 'MISSING!'):<12} {fn}")


if __name__ == "__main__":
    tyro.cli(main)
