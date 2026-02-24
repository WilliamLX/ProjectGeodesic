# 📖 ProjectGeodesy Wiki系统使用指南

## 系统概览

ProjectGeodesy现在拥有一个完整的、系统化的Wiki文档系统，方便查阅文档和调用脚本。

---

## 📂 目录结构

```
ProjectGeodesy/
├── wiki/                          # 📖 Wiki文档中心（主入口）
│   ├── 00_Wiki_Index.md          # 总索引 - 从这里开始！
│   ├── 01_Getting_Started.md      # 快速开始
│   ├── 02_Architecture_Overview.md # 系统架构
│   ├── 03_Development_Roadmap.md  # 开发路线图
│   ├── 04_Perception_Module.md    # 3D感知模块
│   ├── 05_LLM_Behavior_Tree.md    # LLM行为树
│   ├── 06_Deployment_Guide.md     # 部署指南
│   ├── 07_Testing_Guide.md        # 测试指南
│   └── 08_Troubleshooting.md      # 故障排除
│
├── scripts/                       # 🔧 可执行脚本
│   ├── install_linux.sh          # Linux一键安装
│   ├── Dockerfile                # Docker镜像
│   ├── README_DEPLOYMENT.md      # 部署说明
│   └── reorganize_docs.sh        # 文档维护脚本
│
├── docs/                          # 📋 归档的旧文档
│   └── legacy/                    # 保留的历史文档
│       ├── README_PERCEPTION.md
│       ├── Geodesic.md
│       ├── Development_Roadmap.md
│       └── ... (其他已归档文档)
│
├── README.md                      # 📌 项目主页
├── CHANGELOG.md                   # 📝 更新日志
├── LICENSE                        # ⚖️ MIT License
└── .env.example                   # 🔑 环境配置示例
```

---

## 🚀 快速导航

### 我想...

| 目标 | 文档 | 命令/操作 |
|------|------|----------|
| **了解项目** | README.md | 查看项目主页 |
| **快速上手** | wiki/01_Getting_Started.md | 5分钟入门 |
| **部署系统** | wiki/06_Deployment_Guide.md | 运行scripts/install_linux.sh |
| **查看API** | wiki/04_Perception_Module.md | 阅读模块文档 |
| **解决问题** | wiki/08_Troubleshooting.md | 查找错误信息 |
| **运行测试** | wiki/07_Testing_Guide.md | python tests/test_algorithms.py |

---

## 📖 Wiki文档说明

### 00_Wiki_Index.md - 总索引
**用途**: 文档导航中心
**包含**: 所有Wiki文档的链接和说明
**从这里开始**:

```bash
cat wiki/00_Wiki_Index.md
```

### 01_Getting_Started.md - 快速开始
**用途**: 5分钟上手指南
**包含**:
- Linux一键安装
- Docker部署
- 手动安装步骤
- 验证安装

### 02_Architecture_Overview.md - 系统架构
**用途**: 了解整体设计
**包含**:
- 硬件架构图
- 软件分层架构
- 数据流
- 技术栈
- 关键算法

### 03_Development_Roadmap.md - 开发路线图
**用途**: 4个月开发计划
**包含**:
- 方案A: 无硬件验证
- 方案B: 真实相机集成
- 方案C: Isaac Sim仿真
- 里程碑和时间线

### 04_Perception_Module.md - 3D感知模块
**用途**: 核心功能文档
**包含**:
- 模块功能
- API文档
- 使用示例
- 数据格式

### 05_LLM_Behavior_Tree.md - LLM行为树
**用途**: AI任务规划系统
**包含**:
- 架构设计
- LLM集成
- 行为树编译
- 使用示例

### 06_Deployment_Guide.md - 部署指南
**用途**: 系统部署教程
**包含**:
- Linux自动化安装
- Docker容器化
- 环境配置
- 故障排除

### 07_Testing_Guide.md - 测试指南
**用途**: 测试与验证
**包含**:
- 环境测试
- 算法测试
- 性能基准
- 报告模板

### 08_Troubleshooting.md - 故障排除
**用途**: 解决常见问题
**包含**:
- 环境问题
- 依赖问题
- 运行时问题
- 平台特定问题

---

## 🔧 脚本说明

### scripts/install_linux.sh
**用途**: Linux系统一键安装
**使用**:

```bash
# 下载并运行
wget https://raw.githubusercontent.com/WilliamLX/ProjectGeodesy/master/scripts/install_linux.sh
chmod +x install_linux.sh
./install_linux.sh
```

**功能**:
- 检测系统
- 安装ROS 2 Humble
- 安装Miniconda
- 创建Python环境
- 克隆并构建项目
- 运行测试验证

### scripts/Dockerfile
**用途**: Docker镜像定义
**使用**:

```bash
# 构建镜像
docker build -f scripts/Dockerfile -t projectgeodesy:dev .

# 运行容器
docker run -it --volume $(pwd):/root/ProjectGeodesy projectgeodesy:dev
```

### scripts/README_DEPLOYMENT.md
**用途**: 详细部署说明
**包含**:
- 所有部署方式
- 环境要求
- 验证步骤

---

## 📝 文档维护

### 添加新Wiki文档

1. 在wiki/目录创建新文件
2. 命名格式: `XX_Document_Name.md`
3. 在wiki/00_Wiki_Index.md添加链接
4. 提交更改

### 更新文档

```bash
# 运行重组脚本（可选）
bash scripts/reorganize_docs.sh

# 提交更改
git add wiki/
git commit -m "docs: Update XX document"
git push origin master
```

### 文档风格指南

- 使用清晰的标题层次
- 添加代码示例
- 包含使用场景
- 链接相关文档

---

## 🔍 文档搜索

### 本地搜索

```bash
# 搜索关键词
grep -r "ICP" wiki/

# 在所有文档中搜索
grep -r "配准" wiki/ docs/
```

### GitHub搜索

在GitHub仓库页面使用搜索功能，或：
```
https://github.com/WilliamLX/ProjectGeodesy/search?q=ICP
```

---

## 📖 推荐阅读顺序

### 新手入门
1. README.md - 了解项目
2. wiki/01_Getting_Started.md - 快速安装
3. wiki/02_Architecture_Overview.md - 理解架构
4. wiki/07_Testing_Guide.md - 运行测试

### 开发者
1. wiki/02_Architecture_Overview.md - 架构设计
2. wiki/04_Perception_Module.md - 模块API
3. wiki/03_Development_Roadmap.md - 开发计划
4. Geodesic.md (docs/legacy/) - 完整技术方案

### 运维部署
1. wiki/06_Deployment_Guide.md - 部署教程
2. wiki/08_Troubleshooting.md - 故障排除
3. scripts/install_linux.sh - 自动化脚本

---

## 📊 文档状态

| 类型 | 数量 | 位置 |
|------|------|------|
| Wiki文档 | 8个 | wiki/ |
| 脚本文件 | 3个 | scripts/ |
| 归档文档 | 8个 | docs/legacy/ |
| 配置文件 | 3个 | .env.example, LICENSE, CHANGELOG |

---

## 🎯 常见任务

### 安装部署
```bash
# Linux
wget https://raw.githubusercontent.com/.../install_linux.sh
./install_linux.sh

# 查看详细步骤
cat wiki/06_Deployment_Guide.md
```

### 运行测试
```bash
# 激活环境
source ~/ProjectGeodesy/setup_env.sh

# 运行测试
cd ~/ProjectGeodesy/src/geodesic_perception
python tests/test_algorithms.py

# 查看更多测试
cat wiki/07_Testing_Guide.md
```

### 解决问题
```bash
# 查看故障排除
cat wiki/08_Troubleshooting.md

# 或搜索归档文档
grep -r "错误关键词" docs/legacy/
```

---

## 🔄 更新日志

所有重大更新记录在CHANGELOG.md中：

```bash
cat CHANGELOG.md
```

---

## 📞 反馈

如果您发现文档问题或有改进建议：

1. 查看现有文档: wiki/
2. 搜索归档: docs/legacy/
3. 提交Issue: https://github.com/WilliamLX/ProjectGeodesy/issues
4. 提交PR: 改进并推送

---

**最后更新**: 2025-02-24
**维护者**: WilliamLX
