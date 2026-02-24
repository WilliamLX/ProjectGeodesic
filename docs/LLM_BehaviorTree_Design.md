# LLM + Behavior Tree 架构设计

## 概述

本文档描述 ProjectGeodesic 中使用大语言模型（LLM）动态生成拧紧顺序，并通过 Behavior Tree CPP 执行的完整技术方案。

---

## 1. 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        Perception Layer                         │
│  - ICP Point Cloud Registration                                 │
│  - Hole Detection (YOLOv8 / Traditional)                        │
│  - Occlusion & Reachability Analysis                            │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    LLM Planning Layer                           │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Input Builder                                          │   │
│  │  收集工件状态、约束条件，构建 LLM Prompt                 │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              ↓                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  LLM Inference                                          │   │
│  │  模型: Qwen 2.5 7B / GPT-4                              │   │
│  │  输出: 拧紧序列 (JSON格式)                               │   │
│  └─────────────────────────────────────────────────────────┘   │
│                              ↓                                  │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  Safety Validator                                       │   │
│  │  验证序列完整性、约束合规性                              │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                    BT Compiler Layer                           │
│  将 LLM 输出的拧紧序列编译成 Behavior Tree CPP XML 格式         │
└─────────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────────┐
│                   Execution Layer (BT CPP)                     │
│  - GlobalAlignment  - ApproachMove  - VisualServo              │
│  - ScrewIn         - ErrorRecovery - SkipHole                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. LLM 输入设计

### 2.1 输入数据结构

```json
{
  "task_description": "为汽车前顶板生成最优螺丝拧紧顺序",
  "workpiece_info": {
    "name": "NIO Front Roof Panel",
    "total_holes": 13,
    "hole_positions": [
      {"id": 1, "position": [100, 200, 50], "type": "corner"},
      {"id": 2, "position": [150, 200, 50], "type": "edge"},
      {"id": 3, "position": [200, 200, 50], "type": "standard"},
      ...
    ]
  },
  "current_state": {
    "accessible_holes": [1, 2, 3, 5, 7, 8, 10, 11, 12, 13],
    "occluded_holes": [4, 6],
    "blocked_holes": [9],
    "robot_current_pose": "home",
    "completed_holes": []
  },
  "constraints": {
    "sequence_constraints": [
      {"type": "before", "hole_id": 1, "must_precede": [2, 5]},  // 孔1必须在孔2、5之前
      {"type": "avoid", "sequence": [7, 8]}  // 避免按7→8的顺序
    ],
    "priority_holes": [1, 13],  // 四角优先固定
    "optimization_goal": "minimize_robot_movement"
  },
  "robot_capabilities": {
    "max_reach_radius": 800,
    "tool_type": "atlas_copco_screwdriver"
  }
}
```

### 2.2 Prompt 模板

```
你是一个工业机器人任务规划专家。根据以下工件状态和约束条件，生成最优的螺丝拧紧顺序。

## 任务信息
- 工件: {workpiece_info.name}
- 总孔数: {workpiece_info.total_holes}
- 优化目标: {constraints.optimization_goal}

## 当前工件状态
- 可访问孔位: {current_state.accessible_holes}
- 被遮挡孔位: {current_state.occluded_holes} (需要调整工件或跳过)
- 被阻挡孔位: {current_state.blocked_holes} (不可达)

## 约束条件
1. 优先处理孔位: {constraints.priority_holes}
2. 顺序约束:
   - 孔 {hole_id} 必须在 {must_precede} 之前完成
   - 避免序列: {avoid_sequence}

## 要求
1. 输出完整的拧紧顺序（包含所有可访问孔位）
2. 优化机器人移动路径，减少不必要的行程
3. 遵守所有顺序约束
4. 对于被遮挡/阻挡的孔位，放在序列末尾或标记为 SKIP

请以 JSON 格式输出:
{{
  "reasoning": "简要说明你的规划思路",
  "sequence": [1, 13, 5, 2, 3, 7, 10, 11, 12, 8, 4, 6, 9],
  "skipped_holes": [9],  // 无法到达的孔
  "warnings": ["孔4、6被遮挡，可能需要人工干预"]
}}
```

---

## 3. LLM 输出设计

### 3.1 输出格式

```json
{
  "reasoning": "优先固定四角(1,13)以稳定工件，然后按从左到右、从上到下的顺序处理中间孔位，减少机械臂移动距离。被遮挡的孔4、6放在后面，孔9不可达标记为跳过。",
  "sequence": [1, 13, 5, 2, 3, 7, 10, 11, 12, 8, 4, 6],
  "skipped_holes": [9],
  "warnings": ["孔4、6被遮挡，可能需要调整工件位置"]
}
```

### 3.2 输出验证规则

Safety Validator 会检查:

```python
def validate_sequence(output, constraints):
    errors = []

    # 1. 完整性检查
    accessible = set(constraints['current_state']['accessible_holes'])
    sequenced = set(output['sequence'])
    if not accessible.issubset(sequenced):
        errors.append("未包含所有可访问孔位")

    # 2. 顺序约束检查
    for constraint in constraints['sequence_constraints']:
        if constraint['type'] == 'before':
            hole_idx = output['sequence'].index(constraint['hole_id'])
            for must_after in constraint['must_precede']:
                if must_after in output['sequence']:
                    after_idx = output['sequence'].index(must_after)
                    if hole_idx > after_idx:
                        errors.append(f"违反约束: 孔{constraint['hole_id']}应在孔{must_after}之前")

    # 3. 重复检查
    if len(output['sequence']) != len(set(output['sequence'])):
        errors.append("序列中存在重复孔位")

    # 4. 跳过孔位合理性
    for hole_id in output['skipped_holes']:
        if hole_id in constraints['current_state']['accessible_holes']:
            errors.append(f"孔{hole_id}可访问但被标记为跳过")

    return {
        "valid": len(errors) == 0,
        "errors": errors
    }
```

---

## 4. BT Compiler 设计

### 4.1 行为树节点定义

| 节点类型 | 名称 | 功能 | 返回值 |
|---------|------|------|--------|
| **Action** | `GlobalAlignment` | ICP点云配准，获取工件位姿 | SUCCESS/FAILURE |
| **Action** | `ApproachHole` | 移动到指定孔的进场位置 | SUCCESS/FAILURE |
| **Action** | `VisualServo` | IBVS视觉伺服，精对准孔位 | SUCCESS/FAILURE |
| **Action** | `ScrewIn` | 螺旋搜索入孔 + 拧紧 | SUCCESS/FAILURE |
| **Action** | `SkipHole` | 记录跳过某孔，继续下一个 | SUCCESS |
| **Action** | `UpdateState` | 更新工件状态（已完成某孔） | SUCCESS |
| **Condition** | `IsHoleAccessible` | 检查孔是否可访问 | TRUE/FALSE |
| **Control** | `Sequence` | 顺序执行子节点，全部成功才返回SUCCESS | - |
| **Control** | `Selector` | 顺序尝试子节点，首个成功即返回SUCCESS | - |
| **Control** | `ReactiveSequence` | 条件节点失败时重新评估 | - |
| **Decorator** | `RetryUntilSuccessful` | 失败时重试N次 | - |

### 4.2 编译后的 BT XML 示例

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="ScrewTighteningTask">
      <!-- 步骤1: 全局定位 -->
      <GlobalAlignment/>

      <!-- 步骤2: 按LLM生成的顺序拧紧每个孔 -->
      <Sequence name="ScrewHole_1">
        <ApproachHole hole_id="1"/>
        <VisualServo hole_id="1"/>
        <ScrewIn hole_id="1"/>
        <UpdateState hole_id="1" status="completed"/>
      </Sequence>

      <Sequence name="ScrewHole_13">
        <ApproachHole hole_id="13"/>
        <VisualServo hole_id="13"/>
        <ScrewIn hole_id="13"/>
        <UpdateState hole_id="13" status="completed"/>
      </Sequence>

      <Sequence name="ScrewHole_5">
        <ApproachHole hole_id="5"/>
        <VisualServo hole_id="5"/>
        <ScrewIn hole_id="5"/>
        <UpdateState hole_id="5" status="completed"/>
      </Sequence>

      <!-- ... 其他孔位 ... -->

      <!-- 处理被遮挡的孔（带重试） -->
      <Selector name="HandleOccludedHole_4">
        <Sequence name="TryScrewHole_4">
          <ApproachHole hole_id="4"/>
          <VisualServo hole_id="4"/>
          <ScrewIn hole_id="4"/>
          <UpdateState hole_id="4" status="completed"/>
        </Sequence>
        <SkipHole hole_id="4" reason="occluded"/>
      </Selector>

      <!-- 被阻挡的孔直接跳过 -->
      <SkipHole hole_id="9" reason="blocked"/>

    </Sequence>
  </BehaviorTree>
</root>
```

### 4.3 Python 编译器实现

```python
class BTCompiler:
    """将 LLM 输出的拧紧序列编译成 Behavior Tree XML"""

    HOLE_TEMPLATE = """
    <Sequence name="ScrewHole_{hole_id}">
      <ApproachHole hole_id="{hole_id}"/>
      <VisualServo hole_id="{hole_id}"/>
      <RetryUntilSuccessful num_attempts="3">
        <ScrewIn hole_id="{hole_id}"/>
      </RetryUntilSuccessful>
      <UpdateState hole_id="{hole_id}" status="completed"/>
    </Sequence>
    """

    OCCLUDED_TEMPLATE = """
    <Selector name="HandleOccludedHole_{hole_id}">
      <Sequence name="TryScrewHole_{hole_id}">
        <ApproachHole hole_id="{hole_id}"/>
        <VisualServo hole_id="{hole_id}"/>
        <ScrewIn hole_id="{hole_id}"/>
        <UpdateState hole_id="{hole_id}" status="completed"/>
      </Sequence>
      <SkipHole hole_id="{hole_id}" reason="occluded"/>
    </Selector>
    """

    def compile(self, llm_output, constraints):
        """编译 LLM 输出为 BT XML"""
        xml_parts = ["<root main_tree_to_execute=\"MainTree\">",
                     "<BehaviorTree ID=\"MainTree\">",
                     "<Sequence name=\"ScrewTighteningTask\">",
                     "<GlobalAlignment/>"]

        # 添加正常孔位序列
        for hole_id in llm_output['sequence']:
            if hole_id in constraints['current_state']['occluded_holes']:
                xml_parts.append(self.OCCLUDED_TEMPLATE.format(hole_id=hole_id))
            else:
                xml_parts.append(self.HOLE_TEMPLATE.format(hole_id=hole_id))

        # 添加跳过的孔位
        for hole_id in llm_output['skipped_holes']:
            reason = "blocked" if hole_id in constraints['current_state']['blocked_holes'] else "unknown"
            xml_parts.append(f"<SkipHole hole_id=\"{hole_id}\" reason=\"{reason}\"/>")

        xml_parts.extend(["</Sequence>", "</BehaviorTree>", "</root>"])

        return "\n".join(xml_parts)
```

---

## 5. ROS 2 节点集成

### 5.1 节点架构

```
┌─────────────────────────────────────────────────────────────┐
│                  llm_planner_node                            │
│  - 订阅: /perception/workpiece_state                         │
│  - 服务: /llm_planner/generate_sequence                     │
│  - 发布: /bt_compiler/compiled_tree                         │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                  bt_executor_node                            │
│  - 订阅: /bt_compiler/compiled_tree                         │
│  - 行为树引擎: BehaviorTree CPP                             │
│  - 发布: /robot/command                                     │
│  - 订阅: /robot/feedback                                    │
└─────────────────────────────────────────────────────────────┘
```

### 5.2 消息定义

```python
# WorkpieceState.msg
string workpiece_name
int32 total_holes
int32[] accessible_holes
int32[] occluded_holes
int32[] blocked_holes
int32[] completed_holes

# ScrewSequence.msg
int32[] sequence
int32[] skipped_holes
string[] warnings

# BTTree.msg
string xml_tree
```

---

## 6. 错误处理与重试策略

### 6.1 行为树级别的错误处理

```xml
<!-- 带重试的拧紧节点 -->
<Sequence name="ScrewHole_WithRetry">
  <RetryUntilSuccessful num_attempts="3">
    <Sequence name="ScrewHole">
      <ApproachHole hole_id="1"/>
      <VisualServo hole_id="1"/>
      <ScrewIn hole_id="1"/>
    </Sequence>
  </RetryUntilSuccessful>

  <!-- 3次失败后触发 -->
  <Fallback name="HandleFailure">
    <Sequence name="RetryNextTime">
      <SkipHole hole_id="1" reason="failed_3_times"/>
      <Log message="Hole 1 failed after 3 attempts, will retry later"/>
    </Sequence>
  </Fallback>
</Sequence>
```

### 6.2 动态重规划触发条件

| 触发条件 | 行为 |
|---------|------|
| 单孔拧紧失败3次 | 跳过该孔，继续下一个 |
| 3个以上孔失败 | 暂停，请求人工介入 |
| 工件位姿发生意外变化 | 重新执行全局定位 |
| 发现新的遮挡 | 请求 LLM 重新规划剩余序列 |

---

## 7. 实施计划

### Phase 1: 基础框架 (Week 1-2)
- [ ] 定义消息格式（.msg）
- [ ] 实现 LLM Planner Node（先硬编码序列）
- [ ] 实现 BT Executor Node（先执行固定序列）
- [ ] 集成基础 BT 节点

### Phase 2: LLM 集成 (Week 3-4)
- [ ] 实现 Prompt Builder
- [ ] 集成 LLM 推理（Qwen 2.5 或 API）
- [ ] 实现 Safety Validator
- [ ] 实现 BT Compiler

### Phase 3: 测试与优化 (Week 5-6)
- [ ] 仿真环境测试（Isaac Sim）
- [ ] 不同遮挡场景测试
- [ ] 性能优化（LLM 推理延迟）
- [ ] 异常处理测试

---

## 8. 技术选型建议

### LLM 模型选择

| 场景 | 推荐模型 | 部署方式 |
|------|---------|---------|
| 快速原型开发 | GPT-4 / Claude 3.5 Sonnet | API |
| 生产环境（低延迟） | Qwen 2.5 7B Instruct | 本地部署 (RTX 5090) |
| 离线部署 | Llama 3.1 8B Instruct | 本地部署 |
| 微调需求 | Qwen 2.5 7B + LoRA | 本地部署 |

**推荐**: 先用 GPT-4 API 快速验证可行性，再迁移到本地 Qwen 2.5。

### 依赖库

```cmake
# package.xml
<depend>behavior_tree_cpp</depend>
<depend>behaviortree_cpp_v3</depend>
<depend>openai_ros2</depend>  # 如果使用 OpenAI API
<depend>llama_cpp_ros2</depend>  # 如果使用本地模型
```

```python
# requirements.txt
openai>=1.0.0          # OpenAI API
langchain>=0.1.0       # Prompt 管理
pyyaml>=6.0            # 配置文件
behavior-tree-cpp>=4.0 # Python bindings
```

---

## 9. 参考资源

- [Behavior Tree CPP 文档](https://behaviorTreeCPP.readthedocs.io/)
- [ROS 2 Behavior Tree Tutorial](https://github.com/BehaviorTree/BehaviorTree.ROS2)
- [GPT Contract Design Pattern](https://lilianweng.github.io/posts/2023-03-15-prompt-engineering/#output-enforcing)
- [Qwen 2.5 技术报告](https://arxiv.org/abs/2309.16609)
