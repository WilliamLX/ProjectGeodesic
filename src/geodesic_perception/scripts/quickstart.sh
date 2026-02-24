#!/bin/bash
# Quick Start Script for Geodesic Perception
# 快速启动测试脚本

set -e  # 遇到错误立即退出

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR"))"

echo "=================================================="
echo "ProjectGeodesic - 快速启动测试"
echo "=================================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 步骤1: 环境验证
echo -e "${YELLOW}步骤1: 环境验证${NC}"
echo "----------------------------------------"
cd "$PROJECT_ROOT/src/geodesic_perception"
python3 tests/test_environment.py
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ 环境验证失败，请先安装依赖${NC}"
    exit 1
fi
echo ""

# 步骤2: 下载示例数据
echo -e "${YELLOW}步骤2: 下载示例数据${NC}"
echo "----------------------------------------"
python3 scripts/download_sample_data.py
echo ""

# 步骤3: 运行算法测试
echo -e "${YELLOW}步骤3: 运行算法单元测试${NC}"
echo "----------------------------------------"
python3 tests/test_algorithms.py
echo ""

# 步骤4: 运行配准演示
echo -e "${YELLOW}步骤4: 运行配准演示${NC}"
echo "----------------------------------------"
read -p "是否运行可视化演示？(y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python3 scripts/demo_registration.py
else
    echo -e "${YELLOW}跳过可视化演示${NC}"
fi
echo ""

# 完成
echo "=================================================="
echo -e "${GREEN}✅ 所有测试完成！${NC}"
echo "=================================================="
echo ""
echo "下一步："
echo "  1. 查看示例数据: ls $PROJECT_ROOT/data/sample_data/"
echo "  2. 启动示教界面: ros2 run geodesic_perception teaching_gui"
echo "  3. 查看点云: ros2 run geodesic_perception visualize_pointcloud <file.pcd>"
echo ""
echo "文档："
echo "  - README_PERCEPTION.md"
echo "  - docs/Perception_Development_Plan.md"
echo "  - docs/Development_Roadmap.md"
echo ""
