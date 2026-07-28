# eye_tracker_main.py 代码审查

## 一、整体评价（Good）

模块职责划分清晰，类结构分明（数据模型 → 持久化 → 调试UI → 管线 → 编排），符合单一职责原则。对外接口简洁，`EyeTrackingModule` 提供了合理的 `start/stop/get_normalized_eye_state` 三大方法，使用方无需了解内部细节。`__main__` 独立入口也便于调试。总体质量中上，存在一些可改进点。

---

## 二、严重问题

### 1. 默认相机索引冲突

```python
@dataclass
class AppConfig:
    left: CameraConfig = field(default_factory=CameraConfig)
    right: CameraConfig = field(default_factory=lambda: CameraConfig(index=0))  # ❌ 默认也是 0
```

`left` 和 `right` 的默认 `index` 都是 `0`，首次启动时双眼会竞争同一设备。建议右侧默认 `index=1` 或从配置中读取。

### 2. `_constrain_rect` 宽度/高度可能为负

```python
new_h = int(new_w / ratio)  # dx 为负时 new_h 也为负
x2 = x1 + new_w if dx >= 0 else x1 - new_w
```

当鼠标从右向左/从下向上拖拽时，`dx`/`dy` 为负，计算得到的 `new_h` 或 `new_w` 符号相反，最后 `min/max` 虽然能修正最终结果，但中间计算会产生负值。建议先取绝对值再计算，最后归一化坐标。

### 3. 闭包捕获循环变量

在 `open_crop_window` 中：

```python
def _on_right_close():
    if self._crop_window_right is not None:
        self._crop_window_right._on_close()  # ❌ 访问私有方法
    self._crop_window_right = None
```

虽然这里写法正确，但如果后续修改为 lambda 或引入循环（如列表生成），会出现经典闭包陷阱。此外 `_on_close` 是 `CropDebugWindow` 的私有方法，外部直接调用违反了封装约定。

---

## 三、设计缺陷

### 4. 依赖 `tk._default_root`

多处使用 `tk._default_root` 获取父窗口，这是 tkinter 内部实现细节，非公开 API。在 Tk 被销毁或被 GC 后可能为 `None`，行为未定义。建议显式维护一个 `Tk` 实例或由调用方传入 `master` 参数。

### 5. 按钮状态管理脆弱

`ControlPanel._make_button` 使用 `getattr(self, lock_attr)` 反射来读写锁定状态，字符串属性名与代码逻辑紧耦合，不易静态检查。建议改用显式的 `dict` 或枚举管理：

```python
self._locks = {"left_radius": False, "left_center": False, ...}
```

### 6. `GazeConsumer` 忙等轮询

```python
while not self._stop_event.is_set():
    try:
        while True: data = self._queue.get_nowait()
    except queue.Empty: pass
    self._stop_event.wait(0.001)  # 1ms 轮询
```

每秒约 1000 次循环，CPU 空转明显。更高效的方式：

- 使用 `queue.get(timeout=0.001)` 阻塞获取，替代 `get_nowait`+`wait` 组合
- 或使用带超时的 `Condition` 变量

### 7. 文件名拼写错误

`extream_vectors.yaml` → 应为 `extreme_vectors.yaml`

```python
def __init__(self, extreme_file: str = "extream_vectors.yaml"):
```

此错误贯穿整个文件，会误导开发者。

### 8. 配置键名不一致

`ConfigPersistence` 的键名混合了 snake_case 和 camelCase：

```python
if 'left_cam_i' in data:       # snake_case
    config.left.index = int(data['left_cam_i'])
```

建议统一为 `left_cam_index` 等更清晰的命名，并加上版本号以支持向后兼容。

---

## 四、健壮性问题

### 9. 未验证子进程启动成功

```python
self._process_left.start()
self._process_right.start()
```

没有检查 `GazeVectorTracker` 是否成功初始化相机。如果相机被占用或不存在，子进程可能静默退出，而 `is_running()` 此时报错。建议：

- 等待一段时间后检查进程是否存活
- 或通过 `result_queue` 传回初始化结果

### 10. 子进程 `start_tracking` 可能阻塞

```python
def _run_tracker_in_process(...):
    tracker = GazeVectorTracker(...)
    tracker.start_tracking(...)  # ❌ 如果这是阻塞调用，进程永远不会结束
```

如果 `start_tracking` 是阻塞循环（处理视频帧直到收到停止信号），则设计正确；否则会导致进程立即退出。需确认内部实现。

### 11. `CameraConfig.crop` 索引含义不明确

```python
crop: List[int] = field(default_factory=lambda: [0, 640, 0, 480])
```

注释只说 `[x1, x2, y1, y2]`，但实际 `_save_crop_config` 中赋值顺序为：

```python
self._cam_config.crop = [x1, x2, y1, y2]  # 注意是 x1,x2,y1,y2 而非 x1,y1,x2,y2
```

与 `CropDebugWindow` 文档注释的 `[x1, y1, x2, y2]` 不一致。建议统一语义并添加验证。

---

## 五、代码风格与可维护性

### 12. `_crop_rect` 类型定义模糊

类型标注写的是 `Optional[List[int]]`，但从上下文看其真实形状是 `Optional[List[int]]` where `len() == 4`。建议使用 `Tuple[int, int, int, int]` 或自定义 NamedTuple，提高可读性。

### 13. `on_config_changed` 回调被忽略

```python
self._crop_window_left = CropDebugWindow(
    parent, ..., on_config_changed=lambda: None,  # ❌ 空实现
)
```

配置变更回调未连接到真正的保存逻辑，用户手动修改后不会被持久化。建议连接至 `module.save_config()`。

### 14. 重复的 `cv2.VideoCapture` 设置

```python
self._cap.set(cv2.CAP_PROP_FPS, 30)  # 在 CropDebugWindow 和 detect_cameras 中重复
```

可提取为工具函数。

### 15. 缺少 `__all__` 或接口文档

模块顶部虽然写了职责分层和对外接口，但没有 `__all__` 列表，外部用户可能误导入内部类（如 `ConfigPersistence`、`Normalizer`）。

---

## 六、改进建议优先级

| 优先级 | 问题 | 建议 |
|--------|------|------|
| 🔴 P0 | 默认相机索引冲突 | 改为不同默认值 |
| 🔴 P0 | 文件名拼写错误 | 修复为 `extreme_vectors.yaml` |
| 🟠 P1 | `_constrain_rect` 负值 | 先取 abs 再计算 |
| 🟠 P1 | 依赖 `tk._default_root` | 改为显式传入 master |
| 🟠 P1 | 子进程启动验证 | 增加存活检查 |
| 🟡 P2 | `crop` 赋值顺序歧义 | 统一语义并加验证 |
| 🟡 P2 | 按钮状态反射 | 改为字典管理 |
| 🟡 P2 | 轮询忙等 | 改用阻塞 get |
| 🟢 P3 | 缺少 `__all__` | 补齐 |
| 🟢 P3 | 配置键名统一 | 重构持久化键名 |

---

## 七、总结

模块整体架构清晰，分层合理，`EyeTrackingModule` 的 API 设计简洁易用。主要问题集中在 **默认配置的合理性**、**GUI 资源管理的健壮性** 以及 **部分实现细节的准确性与一致性** 上。修复上述 P0/P1 问题后，代码质量将显著提升。