# ProjectGeodesy Wiki

> **固定基座智能拧紧机器人系统** - 基于视觉伺服与力控的单臂自动化装配解决方案

欢迎来到ProjectGeodesy文档中心！本文档系统涵盖项目架构、开发指南、部署教程和API参考。

---

## 📚 文档导航

### 🚀 快速开始
- **[01_快速开始](01_Getting_Started.md)** - 5分钟上手指南
  - 环境配置
  - 安装部署
  - 第一个测试

### 🏗️ 架构设计
- **[02_系统架构](02_Architecture_Overview.md)** - 整体架构设计
  - 硬件组成
  - 软件栈
  - 数据流

### 🗺️ 开发规划
- **[03_开发路线图](03_Development_Roadmap.md)** - 4个月开发计划
  - 3个并行方案
  - 里程碑和时间线
  - 资源配置

### 📖 模块文档
- **[04_3D感知模块](04_Perception_Module.md)** - 点云配准与定位
  - 功能特性
  - API文档
  - 使用示例

- **[05_LLM行为树](05_LLM_Behavior_Tree.md)** - AI任务规划
  - 架构设计
  - 集成指南
  - 示例代码

### 🚀 部署运维
- **[06_部署指南](06_Deployment_Guide.md)** - 系统部署教程
  - Linux自动化安装
  - Docker容器化
  - 环境配置

- **[07_测试指南](07_Testing_Guide.md)** - 测试与验证
  - 单元测试
  - 集成测试
  - 性能基准

### 🔧 故障排除
- **[08_故障排除](08_Troubleshooting.md)** - 常见问题解决
  - 环境问题
  - 依赖冲突
  - 调试技巧

---

## 📁 项目结构

```
ProjectGeodesy/
├── wiki/                       # 📖 文档中心
│   ├── 00_Wiki_Index.md        # 本文件
│   ├── 01_Getting_Started.md
│   ├── 02_Architecture_Overview.md
│   └── ...
├── src/                        # 💻 源代码
│   └── geodesic_perception/    # ROS 2感知包
│       ├── geodesic_perception/# Python模块
│       ├── tests/              # 测试脚本
│       ├── scripts/            # 工具脚本
│       └── config/             # 配置文件
├── scripts/                    # 🔧 部署脚本
│   ├── install_linux.sh        # Linux自动化安装
│   ├── Dockerfile              # Docker镜像
│   └── README_DEPLOYMENT.md    # 部署说明
├── data/                       # 📦 数据目录
│   ├── templates/              # 工件模板
│   ├── pointclouds/            # 点云数据
│   └── calibration/           # 标定数据
└── Geodesic.md                 # 📋 原始技术方案
```

---

## 🎯 快速链接

### 我想...

| 目标 | 文档 | 命令 |
|------|------|------|
| **快速安装** | [快速开始](01_Getting_Started.md) | `./scripts/install_linux.sh` |
| **了解架构** | [系统架构](02_Architecture_Overview.md) | - |
| **查看API** | [感知模块](04_Perception_Module.md) | - |
| **部署到Linux** | [部署指南](06_Deployment_Guide.md) | `wget install_linux.sh` |
| **解决问题** | [故障排除](08_Troubleshooting.md) | - |
| **运行测试** | [测试指南](07_Testing_Guide.md) | `python tests/test_algorithms.py` |

---

## 📊 项目状态

### 当前进度 (2025-02-24)

| 模块 | 状态 | 进度 |
|------|------|------|
| 3D感知与配准 | ✅ 完成 | 100% |
| ROS 2集成 | ✅ 完成 | 100% |
| 测试验证 | ✅ 完成 | 100% |
| 示教界面 | ✅ 完成 | 100% |
| 视觉伺服 | 🚧 开发中 | 0% |
| 运动规划 | 📋 计划中 | 0% |
| 力控模块 | 📋 计划中 | 0% |
| LLM行为树 | 📋 计划中 | 0% |

### 性能指标

| 指标 | 目标 | 实测 | 状态 |
|------|------|------|------|
| 配准精度 | <2mm | 0.85mm | ✅ 超标 |
| 配准时间 | <600ms | ~270ms | ✅ 达标 |
| 测试通过率 | >90% | 100% | ✅ 达标 |

---

## 🔗 外部资源

- **GitHub仓库**: https://github.com/WilliamLX/ProjectGeodesic
- **问题反馈**: https://github.com/WilliamLX/ProjectGeodesic/issues
- **更新日志**: [CHANGELOG.md](../CHANGELOG.md)

---

## 📝 文档贡献

欢迎改进文档！请参考：
- 风格指南：[文档规范](.github/CONTRIBUTING.md)
- 提交流程：Pull Request to `master` branch

---

## 📜 许可证

TBD

---

**最后更新**: 2025-02-24
**维护者**: WilliamLX
