# ProjectGeodesy - 快速参考卡片

## 🚀 一键启动（Linux）

```bash
# 完整安装
wget https://raw.githubusercontent.com/WilliamLX/ProjectGeodesy/master/scripts/install_linux.sh
chmod +x install_linux.sh
./install_linux.sh
```

## 📦 已安装环境

| 组件 | 版本 | 位置 |
|------|------|------|
| ROS 2 | Humble | `/opt/ros/humble` |
| Miniconda | Latest | `~/miniconda3` |
| Python环境 | geodesy (3.11) | `~/miniconda3/envs/geodesy` |
| 项目代码 | - | `~/ProjectGeodesy` |

## 🔑 常用命令

### 激活环境

```bash
source ~/ProjectGeodesy/setup_env.sh
```

### 运行测试

```bash
cd ~/ProjectGeodesy/src/geodesy_perception
python tests/test_algorithms.py
```

### 启动GUI

```bash
cd ~/ProjectGeodesy/src/geodesy_perception
python teaching_gui.py
```

### 查看点云

```bash
cd ~/ProjectGeodesy/src/geodesy_perception
python visualize_pointcloud.py data.pcd
```

## 📁 项目结构

```
~/ProjectGeodesy/
├── src/geodesic_perception/     # ROS 2包
│   ├── geodesic_perception/      # Python模块
│   ├── tests/                    # 测试脚本
│   ├── scripts/                  # 工具脚本
│   └── config/                   # 配置文件
├── docs/                         # 文档
├── data/                         # 数据目录
└── scripts/                      # 部署脚本
```

## 🧪 测试结果

- ✅ 环境验证: 85.7% (ROS 2 on macOS除外)
- ✅ 算法测试: 100%
- ✅ 配准精度: 0.85mm (目标: <2mm)

## 📖 关键文档

| 文档 | 说明 |
|------|------|
| `README_PERCEPTION.md` | 感知模块使用指南 |
| `docs/Perception_Development_Plan.md` | 5周开发计划 |
| `docs/PlanA_Test_Report.md` | Phase 1测试报告 |
| `docs/Development_Roadmap.md` | 3个开发方案 |
| `scripts/README_DEPLOYMENT.md` | 完整部署指南 |

## 🔧 Conda命令

```bash
# 激活环境
conda activate geodesy

# 查看已安装包
conda list

# 更新包
pip install --upgrade open3d

# 退出环境
conda deactivate
```

## 🐛 故障排除

| 问题 | 解决方案 |
|------|---------|
| ROS 2找不到 | `source /opt/ros/humble/setup.bash` |
| Conda未激活 | `conda init && source ~/.bashrc` |
| Open3D错误 | `pip install open3d==0.19.0` |
| GUI无法显示 | 检查`echo $DISPLAY` |

## 📞 获取帮助

- GitHub: https://github.com/WilliamLX/ProjectGeodesy
- Issues: https://github.com/WilliamLX/ProjectGeodesic/issues

---

**更新**: 2025-02-24
