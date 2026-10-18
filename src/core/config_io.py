"""pipeline.yaml 读写：持久化效果器编排（效果器 + 连线 + 通道 + 样条 + 绑定）。

参数输出层声明不在此文件中（保留在 servo_control/slots.py）。
"""

from pathlib import Path

import yaml

_SRC_DIR = Path(__file__).resolve().parent.parent
PIPELINE_PATH = _SRC_DIR / "pipeline.yaml"


class _FlowList(list):
    """YAML 行内流式列表标记：point_set 按 [[x,y],...] 写出。"""


def _flow_style_list(dumper, data):
    """把 _FlowList 序列化为行内流式风格（point_set: [[0,0],[1,1]]）。"""
    return dumper.represent_sequence("tag:yaml.org,2002:seq",
                                     list(data), flow_style=True)


yaml.SafeDumper.add_representer(_FlowList, _flow_style_list)


def _wrap_channel(cfg):
    """把通道配置里的 point_set 转成行内流式列表。"""
    out = dict(cfg)
    pts = out.get("point_set")
    if pts is not None:
        out["point_set"] = _FlowList([list(map(float, p)) for p in pts])
    return out


def read_pipeline(path):
    """读取 pipeline.yaml，返回 dict；文件不存在 / 损坏 / 顶层非字典返回 None。"""
    path = Path(path)
    if not path.exists():
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as exc:  # noqa: BLE001  文件损坏 / 权限等异常按缺失处理
        print(f"[config_io] 读取 {path.name} 失败（{exc}），将按默认编排新建。")
        return None
    return data if isinstance(data, dict) else None


def write_pipeline(path, data):
    """写出 pipeline.yaml；通道的 point_set 用行内流式。"""
    payload = {}
    for key, value in (data or {}).items():
        if key == "channels" and isinstance(value, dict):
            payload[key] = {ch: _wrap_channel(cfg) for ch, cfg in value.items()}
        else:
            payload[key] = value
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
