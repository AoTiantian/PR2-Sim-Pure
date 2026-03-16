# PR2 MuJoCo Simulator

该目录仅保留 PR2 所需仿真组件：

- `simulate`：PR2 专用通信桥
- `unitree_robots/pr2`：PR2 MJCF 场景与网格

## 编译运行

```bash
cd simulate
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./unitree_mujoco
```

默认配置位于 `simulate/config.yaml`，已设置：

- `robot: pr2`
- `robot_scene: scene.xml`
- `domain_id: 1`
- `interface: lo`
