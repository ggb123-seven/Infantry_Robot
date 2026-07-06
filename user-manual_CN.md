# 🌿 auto-embedded 实用使用手册

> 版本：发布整理版  
> 日期：2026-07-05  
> 维护者：DuncanY  
> 目标：把**当前仓库已验证事实**、**auto-embedded 的通用能力**、**本机/平台差异**分开说清楚，避免把示例、通用机制和本机经验混写。

### 快速导航

- 想知道 **auto-embedded 到底是什么**：看 **第 2 章**
- 想知道 **第一次怎么装进固件工程**：看 **第 4 章**
- 想知道 **开工、续工、收工怎么做**：看 **第 5 章**
- 想查 **各平台命令写法**：看 **第 7 章**
- 想理解 **RIPER-5、硬件锁、记忆机制**：看 **第 8 章**
- 想做 **升级、卸载、排障**：看 **第 10 章** 和 **第 11 章**
- 想避免误用：重点看 **1.1**、**4.4**、**5.4**、**附录 B**

---

## 1. 阅读说明：这份手册怎么读

这份手册统一使用 4 种事实标签：

- **[已验证]**：已经直接核对过当前仓库文件、源码或文档的事实。
- **[通用]**：auto-embedded 的常规工作方式，适用于大多数接入后的固件项目。
- **[平台差异]**：不同 AI 编码工具的命令、钩子、子 Agent 能力不一样，不能混用。
- **[本机特例]**：只对某台机器、某个 shell 或某套本地环境成立，不应写死成通用规则。

### 1.1 证据优先级

安装、续工、排障、写文档时，统一按这个顺序判断：

1. **当前固件工程里实际存在的 `.auto-embedded/` 内容**
2. **当前仓库源码里的单一事实源**，例如 `src/types/ai-tools.ts`
3. **当前仓库中文文档**，例如 `README.md`、`docs/quick-start_CN.md`、`docs/concepts_CN.md`
4. **CLI 实际输出或脚本检查结果**，例如 `aemb doctor`、`aemb status`、`aemb check`
5. **旧会话、口头描述、示例命令**

> 结论：**工程事实优先，示例只能辅助。** 别把“某个例子这么写”误当成“所有项目都必须这么写”。

### 1.2 这份手册和快速开始的区别

[已验证] 当前仓库已经有：

- `README.md`
- `docs/quick-start_CN.md`
- `docs/concepts_CN.md`
- `docs/architecture_CN.md`
- `docs/user-manual_CN.md`

[通用] `quick-start_CN.md` 适合第一次快速跑通；本手册适合放在旁边长期查阅，重点说明：

- 什么时候用 `init`
- 什么时候用 `start`
- 什么时候用 `continue`
- 哪些平台命令不同
- 升级、卸载、排障时先看什么

---

## 2. 当前仓库现状总览

这一章只写**当前 auto-embedded 仓库已经确认存在**的事实。

### 2.1 当前仓库身份

[已验证] `auto-embedded` 是一个面向嵌入式固件开发的 AI 工程框架。

它不是：

- MCU SDK
- 编译器
- 烧录器
- 原理图或 PCB 工具

它做的是把一套项目级工程纪律装进固件工程：

- RIPER-5 五阶段流程
- `.auto-embedded/` 本地运行时
- 多平台 AI 工具接线
- 硬件资源锁 `hw-lock.yaml`
- 任务记忆、journal、spec 规范库
- 编译、烧录、调试、串口、总线、静态分析等工具技能
- 离线嵌入式知识库和专项流程

一句话版：

> **auto-embedded 把“让 AI 帮你写固件”变成有流程、有状态、有门禁、有证据的项目级工作方式。**

### 2.2 当前仓库已验证的入口文件

[已验证] 当前源码仓库中，至少存在这些关键入口：

- `package.json`
- `README.md`
- `README_EN.md`
- `INSTALL.md`
- `SKILL.md`
- `docs/quick-start_CN.md`
- `docs/concepts_CN.md`
- `docs/architecture_CN.md`
- `src/cli/index.ts`
- `src/types/ai-tools.ts`
- `src/commands/`
- `src/channel/`
- `templates/`
- `tests/test-auto-embedded.sh`

### 2.3 当前版本和运行要求

[已验证] `package.json` 当前记录：

- 包名：`auto-embedded`
- 版本：`1.1.3`
- CLI 名称：`aemb`
- Node.js 要求：`>=18`
- 支持系统：Windows、Linux、macOS
- 测试脚本：`bash tests/test-auto-embedded.sh`

[通用] 使用者至少需要：

| 依赖 | 要求 | 用途 |
|---|---|---|
| Node.js | `>= 18` | 安装和运行 `aemb` CLI |
| Python | `>= 3.9` | 运行项目钩子和 `.auto-embedded/scripts/` |
| AI 编码工具 | 至少一个 | Claude Code / Cursor / Codex / OpenCode / Copilot / Gemini CLI / Windsurf |
| git | 推荐 | 追踪变更、保存快照、定位项目状态 |

---

## 3. auto-embedded 的通用工作模型

这一章说的是 **auto-embedded 一般怎么组织 AI 固件开发**，不是说每个项目都已经拥有所有产物。

### 3.1 它主要解决什么问题

[通用] auto-embedded 主要解决 6 类嵌入式 AI 开发问题：

| 问题 | 通用做法 |
|---|---|
| AI 容易没调研就改代码 | 用 RIPER-5 强制先 RESEARCH，再 PLAN，再 EXECUTE |
| AI 容易乱猜硬件资源 | 用 `hw-lock.yaml` 冻结引脚、DMA、IRQ、定时器 |
| 长任务容易断上下文 | 用 `.auto-embedded/tasks/`、`workspace/`、journal 落盘 |
| AI 容易空口宣称完成 | 用 `aemb check`、编译输出、串口日志、手册页码做 evidence |
| 多平台规则不一致 | 用平台 configurator 生成各工具自己的命令、hooks、skills |
| 项目经验容易蒸发 | 用 `promote` 把经验沉淀进 `.auto-embedded/spec/` |

### 3.2 装进项目后会出现什么

[通用] 对某个固件工程执行 `aemb init` 后，目标项目通常会出现：

```text
.auto-embedded/
├── scripts/       # task/get_context/check 等运行时脚本
├── tools/         # build/flash/debug/serial/CAN 等工具技能
├── refs/          # 离线嵌入式知识库
├── modes/         # 专项流程
├── spec/          # 项目规范库和硬件资源锁
├── tasks/         # 当前和历史任务状态
├── workspace/     # 工作区状态和 journal
├── workflow.md    # RIPER-5 流程定义
└── config.yaml    # 注入预算、规范层等配置
```

[已验证] 当前仓库文档说明，`init` 还会为所选平台写入各自的接线文件，例如 `.claude/`、`.cursor/`、`.codex/`、`.opencode/`、`.github/copilot/`、`.gemini/`、`.windsurf/` 等。

### 3.3 不要把通用能力写成绝对事实

下面这些说法容易误导：

- “所有平台都支持同一种 `/aemb:start` 写法”
- “所有平台都有同等级 hook 能力”
- “执行 `aemb init` 后 AI 一定能自动实测硬件”
- “AI 说完成就说明固件已经可用”
- “`aemb update` 会覆盖所有旧文件”

更稳妥的写法是：

- [平台差异] 各平台触发命令和 hook 能力不同，应按 `src/types/ai-tools.ts` 和 init 输出确认。
- [通用] `aemb init` 会安装运行时和平台接线，但真实硬件验证仍要人提供设备、日志和测试条件。
- [通用] `aemb update` 主要更新 managed 内容；用户改过的受管文件可能生成 `.new`，需要人工合并。

---

## 4. 首次接入 vs 日常开工

这一章最重要的结论只有一个：

> **`aemb init` 是首次接入或增量接线命令，不是每天开工命令。**

### 4.1 首次接入时做什么

[通用] 第一次把 auto-embedded 装进某个固件工程，通常是：

```bash
npm install -g auto-embedded
aemb init /path/to/firmware-project -u 你的名字 --platforms claude,codex
aemb doctor /path/to/firmware-project
```

适用场景：

- 第一次给固件工程接入 auto-embedded
- 旧工程第一次补装某个 AI 平台
- 明确要重建或修复平台接线
- 升级 CLI 后，需要把新运行时同步进目标项目

### 4.2 日常开工时做什么

[通用] 项目已经 init 过以后，日常开工不是再跑 `init`，而是：

1. 进入目标固件项目
2. 打开对应 AI 编码工具的新会话
3. 用 `start` 开新任务，或用 `continue` 续旧任务

示例：

```text
/aemb:start 给主板增加 SHT30 温湿度读取
```

或：

```text
/aemb:continue
```

### 4.3 为什么不能把 `aemb init` 当作“启动按钮”

[通用] 因为 `init` 的职责是**安装运行时和接线平台**，不是**恢复今天的任务现场**。

重复乱跑的风险包括：

- 误把续工变成重新建现场
- 让新旧平台配置混在一起难排查
- 把本该由 `continue` 读取的任务状态绕过去
- 在不清楚目标目录时，把运行时装进错误项目

### 4.4 重复执行 `init` 什么时候是合理的

[通用] 这些场景可以重复执行 `aemb init`：

- 给已接入项目新增平台：

```bash
aemb init ./firmware -u zhangsan --codex
```

- 一次性补齐所有已实现平台：

```bash
aemb init ./firmware -u zhangsan --all
```

- 平台接线坏了，需要重建：

```bash
aemb init ./firmware -u zhangsan --platforms claude,codex --force
```

[关键规则] 重复执行前先确认：

```bash
aemb doctor ./firmware
```

---

## 5. 日常操作 SOP：安装 / 开工 / 续工 / 收工

这一章只保留一套主流程，避免“想到哪跑哪”。

### 5.1 第一次安装到固件工程

#### 第一步：确认目标是固件工程根目录

[通用] 目标目录通常能看到这些文件或目录中的一部分：

```text
Core/
Drivers/
MDK-ARM/
User/
*.ioc
*.uvprojx
CMakeLists.txt
platformio.ini
Makefile
```

#### 第二步：安装 CLI

```bash
npm install -g auto-embedded
aemb
```

#### 第三步：初始化项目

```bash
aemb init ./firmware -u zhangsan --platforms claude,codex
```

或安装全部已实现平台：

```bash
aemb init ./firmware -u zhangsan --all
```

#### 第四步：体检

```bash
aemb doctor ./firmware
```

[关键规则] `doctor` 不通过时，先不要让 AI 改代码。优先修：

- Python 是否可用
- `.auto-embedded/` 是否完整
- 平台配置是否写入
- 当前 AI 工具是否从项目根目录打开

### 5.2 开新任务

[通用] 新需求用 `start <标题>`。

Claude Code / OpenCode / Gemini CLI：

```text
/aemb:start 给 STM32F407 工程补全底盘 CAN 电机反馈检查
```

Cursor / Windsurf / Copilot：

```text
/aemb-start 给 STM32F407 工程补全底盘 CAN 电机反馈检查
```

Codex：

```text
$aemb-start 给 STM32F407 工程补全底盘 CAN 电机反馈检查
```

[通用] 开工后，AI 应先进入 `RESEARCH`，查项目结构、芯片、外设、硬件资源和相关知识库，不应马上修改代码。

### 5.3 进行中

#### 需求还不清楚

```text
/aemb:brainstorm 设计一个低功耗采样方案
```

或按平台语法改成：

```text
/aemb-brainstorm 设计一个低功耗采样方案
```

#### 计划要写代码

[通用] 计划里如果包含代码修改，应该先让人确认，再进入 `EXECUTE`。

你可以要求：

```text
先给出文件路径、函数签名、硬件资源占用和验证方式，我确认后再执行。
```

### 5.4 续工

这是最容易断上下文和串任务的环节，所以只保留一套规则：

```text
/aemb:continue
```

或：

```text
$aemb-continue
```

[通用] 续工时，AI 应先恢复“五问重启”：

1. 当前在哪个 RIPER 阶段？
2. 最近完成了什么？
3. 硬件资源锁状态如何？
4. 之前 RESEARCH 有哪些发现？
5. 应该以 Scout、Builder 还是 Verifier 继续？

> 结论：**续工不是重新 start，也不是靠 AI 回忆，而是读取 `.auto-embedded/` 里的现场。**

### 5.5 收工

任务完成时使用：

```text
/aemb:finish-work
```

或按平台语法：

```text
$aemb-finish-work
```

[通用] 收工应完成：

- 执行机械检查
- 给出编译、烧录、串口、总线、测试或手册页码证据
- 写 journal
- 把长期经验 promote 到 `spec/`
- 归档任务

---

## 6. CLI 命令速查

### 6.1 顶层命令

[已验证] `src/cli/index.ts` 暴露的主要命令包括：

| 命令 | 用途 |
|---|---|
| `aemb init [工程]` | 把运行时和平台接线装进工程 |
| `aemb update [工程]` | 更新项目内 managed 内容 |
| `aemb status [工程]` | 打印当前项目现场 |
| `aemb doctor [工程]` | 检查安装和平台接线 |
| `aemb check [工程]` | 执行机械门禁 |
| `aemb backup [工程]` | 备份 `.auto-embedded/` |
| `aemb uninstall [工程]` | 按 manifest 剥离 auto-embedded |
| `aemb upgrade` | 升级全局 CLI |
| `aemb workflow` | 查看或切换工作流模板 |
| `aemb mem` | 检索本机 Claude / Codex 历史会话 |
| `aemb channel` | 本地多 Agent 协作通道 |

### 6.2 安装和维护命令

```bash
# 安装 CLI
npm install -g auto-embedded

# 初始化项目
aemb init ./firmware -u zhangsan --platforms claude,cursor,codex
aemb init ./firmware -u zhangsan --all

# 体检和状态
aemb doctor ./firmware
aemb status ./firmware

# 检查
aemb check ./firmware
aemb check ./firmware --arch
aemb check ./firmware --hw
aemb check ./firmware --spec
aemb check ./firmware --json

# 备份、更新、卸载
aemb backup ./firmware
aemb update ./firmware
aemb uninstall ./firmware

# 升级 CLI
aemb upgrade
aemb upgrade --tag beta
aemb upgrade --tag rc
aemb upgrade --tag latest
aemb upgrade --dry-run
```

### 6.3 本地脚本兜底

[通用] 如果平台命令暂时不可用，可以直接调用目标项目里的脚本。

```bash
python .auto-embedded/scripts/task.py start "任务标题"
python .auto-embedded/scripts/task.py phase PLAN
python .auto-embedded/scripts/task.py journal "完成 USART1 中断问题复盘"
python .auto-embedded/scripts/task.py promote conventions "UART 中断初始化必须检查 GPIO 复用、NVIC 和向量表"
python .auto-embedded/scripts/task.py archive
```

[本机特例] Windows 环境里如果 `python3` 不可用，优先试：

```bash
python --version
py --version
```

---

## 7. 平台命令与能力差异

这一章只说当前仓库注册表里已经确认的内容。

### 7.1 已打通的 7 个核心平台

[已验证] `src/types/ai-tools.ts` 当前把这些平台标为 `stable`：

| 平台 ID | 展示名 | 触发形式 | 注入分级 |
|---|---|---|---|
| `claude` | Claude Code | `/aemb:` | push |
| `cursor` | Cursor | `/aemb-` | push |
| `codex` | Codex | `$aemb-` | pull |
| `opencode` | OpenCode | `/aemb:` | push |
| `copilot` | GitHub Copilot | `/aemb-` | pull |
| `gemini` | Gemini CLI | `/aemb:` | pull |
| `windsurf` | Windsurf | `/aemb-` | command |

### 7.2 预留但未实现的平台

[已验证] 当前仓库还注册了 7 个 `reserved` 平台：

- `kilo`
- `kiro`
- `antigravity`
- `qoder`
- `codebuddy`
- `droid`
- `pi`

[关键规则] 预留平台只是注册位，不等于已经能安装。`init` 选到 reserved 平台时应拒绝或提示当前不可用。

### 7.3 常用命令换写表

| 目的 | Claude / OpenCode / Gemini | Cursor / Windsurf / Copilot | Codex |
|---|---|---|---|
| 开新任务 | `/aemb:start 标题` | `/aemb-start 标题` | `$aemb-start 标题` |
| 续工 | `/aemb:continue` | `/aemb-continue` | `$aemb-continue` |
| 梳理需求 | `/aemb:brainstorm 标题` | `/aemb-brainstorm 标题` | `$aemb-brainstorm 标题` |
| 检查 | `/aemb:check` | `/aemb-check` | `$aemb-check` |
| 收工 | `/aemb:finish-work` | `/aemb-finish-work` | `$aemb-finish-work` |
| 状态 | `/aemb:status` | `/aemb-status` | `$aemb-status` |

### 7.4 push / pull / command 是什么意思

[平台差异] 当前注册表使用 3 种注入分级：

| 分级 | 含义 | 典型平台 |
|---|---|---|
| `push` | hook 能主动把上下文推给主会话或子 Agent | Claude、Cursor、OpenCode |
| `pull` | 主会话可注入，子 Agent 需要主动读取 prelude 或技能上下文 | Codex、Copilot、Gemini |
| `command` | 无完整 hook，主要靠命令、工作流或技能触发 | Windsurf |

> 结论：**同样叫 auto-embedded，平台能力也不完全一样。** 写文档或排障时要说清楚平台。

---

## 8. RIPER-5、硬件锁与证据门禁

### 8.1 RIPER-5 是什么

[通用] auto-embedded 的核心流程是：

```text
RESEARCH -> INNOVATE -> PLAN -> EXECUTE -> REVIEW
```

| 阶段 | 做什么 | 严禁 |
|---|---|---|
| RESEARCH | 查芯片、手册、项目结构、已有驱动、硬件资源 | 未查证就改代码 |
| INNOVATE | 比较方案和取舍 | 直接定死实现 |
| PLAN | 形成文件、函数、资源、验证方式清单 | 含糊写“改一下 main.c” |
| EXECUTE | 按计划逐项实现，并给 evidence | 顺手扩大范围 |
| REVIEW | 机械检查、实测验证、经验回流 | 用“应该可以”当完成 |

### 8.2 硬件资源锁 `hw-lock.yaml`

[通用] 硬件资源锁位于目标项目：

```text
.auto-embedded/spec/hardware/hw-lock.yaml
```

它用于冻结：

- GPIO 引脚
- 外设实例
- DMA 通道
- IRQ 优先级
- 定时器用途
- 通信总线地址

使用顺序：

1. RESEARCH 阶段确认资源
2. PLAN 前冻结关键资源
3. EXECUTE 时对照资源锁编码
4. REVIEW 阶段运行硬件冲突检查

手动检查：

```bash
aemb check ./firmware --hw
```

### 8.3 什么算 evidence

[通用] 这些可以算证据：

- 编译输出
- 测试输出
- 烧录日志
- 串口日志
- CAN / Modbus / VISA 抓取结果
- 数据手册页码
- 静态分析结果
- `aemb check` 输出

这些不算证据：

- “理论上可以”
- “应该没问题”
- “我已经修好了”
- “这段代码看起来对”

---

## 9. 记忆、规范和多 Agent 协作

### 9.1 各类文件分别管什么

| 位置 | 作用 | 备注 |
|---|---|---|
| `.auto-embedded/workflow.md` | RIPER-5 流程定义 | hook 从这里取阶段约束 |
| `.auto-embedded/config.yaml` | 注入预算、规范层配置 | seed，用户配置优先 |
| `.auto-embedded/spec/` | 项目级规范库 | 经验回流位置 |
| `.auto-embedded/spec/hardware/hw-lock.yaml` | 硬件资源锁 | 编码前先冻结 |
| `.auto-embedded/tasks/` | 当前和历史任务状态 | 续工第一信源之一 |
| `.auto-embedded/workspace/` | journal 和工作区状态 | 跨会话恢复现场 |
| `.auto-embedded/refs/` | 上游离线知识库 | managed，随框架升级 |
| `.auto-embedded/modes/` | 专项流程 | managed，按需读取 |
| `.auto-embedded/tools/` | 工具技能脚本 | build/flash/debug/serial 等 |

### 9.2 spec 应该写什么

[通用] 适合沉淀进 `spec/` 的内容：

- 项目长期架构约定
- 硬件资源分配和限制
- ISR、临界区、错误处理等编码纪律
- 某类 bug 的防复发规则
- 团队确认过的设计决策

不适合写进 `spec/` 的内容：

- 一次性的排查流水账
- 未验证的猜测
- 某次聊天里的临时想法
- 可以从 git 历史直接看到的普通提交摘要

### 9.3 `aemb mem` 怎么用

[已验证] `aemb mem` 用于本地检索 Claude / Codex 历史会话，不上传，不建立远程索引。

常用命令：

```bash
aemb mem projects
aemb mem list --global --platform claude --since 2026-06-01
aemb mem search "USART 中断" --global
aemb mem context <会话id> --grep "hw-lock"
aemb mem extract <会话id> --phase implement
```

[关键规则] 历史会话只能作为线索，不能覆盖当前项目里的 `.auto-embedded/` 事实。

### 9.4 `aemb channel` 怎么用

[已验证] `aemb channel` 是本地多 Agent 协作通道，基于文件事件存储。

常用命令：

```bash
aemb channel create bringup --task "主板 bring-up"
aemb channel send bringup "请检查 IMU 初始化顺序" --as arch --to drv
aemb channel messages bringup --last 20
aemb channel wait bringup --from drv --timeout 10m
aemb channel spawn bringup --as drv --provider claude --cwd ./firmware
aemb channel kill bringup --as drv
```

[通用] 多 Agent 协作适合长任务、比赛模式、驱动/算法/验证分工。小改动不一定需要启用。

---

## 10. 升级、更新、卸载

### 10.1 升级全局 CLI

```bash
aemb upgrade
```

指定通道：

```bash
aemb upgrade --tag latest
aemb upgrade --tag beta
aemb upgrade --tag rc
```

只看将执行什么：

```bash
aemb upgrade --dry-run
```

### 10.2 更新项目运行时

[通用] 升级 CLI 后，如果要同步某个固件项目里的运行时内容，还需要：

```bash
aemb update ./firmware
```

[已验证] 当前架构文档把项目内容分成：

| 类别 | 内容 | `aemb update` 行为 |
|---|---|---|
| managed | `scripts/`、`tools/`、`refs/`、`modes/`、`workflow.md`、平台接线文件 | hash 比对升级；用户改过则写 `.new` |
| seed | `config.yaml`、`spec/**`、`tasks/`、`workspace/` | 不主动覆盖，仅缺失时补种 |

### 10.3 出现 `.new` 怎么办

[通用] 出现 `.new` 说明你改过受管文件，auto-embedded 为了保护本地修改，没有直接覆盖。

处理顺序：

1. 打开原文件和 `.new`
2. 手动合并需要的新内容
3. 删除 `.new`
4. 运行：

```bash
aemb doctor ./firmware
aemb check ./firmware
```

### 10.4 卸载

```bash
aemb uninstall ./firmware
```

[通用] 卸载会：

- 先备份 `.auto-embedded/`
- 按 manifest 删除 auto-embedded 写入的独占文件
- 从共享配置中剥离 auto-embedded 片段
- 删除 `.auto-embedded/`
- 尽量保留用户自己的配置和固件源码

[关键规则] 卸载不是源码回滚工具。AI 或用户改过的业务代码不会因为 `aemb uninstall` 自动还原。

---

## 11. 常见问题排查

### 11.1 平台命令没有出现

先跑：

```bash
aemb doctor ./firmware
```

重点看：

- `.auto-embedded/.platforms` 是否记录了该平台
- 平台独占配置目录是否存在
- 共享配置是否接入 auto-embedded
- 是否需要重启 AI 工具或新开会话

### 11.2 会话没有自动注入上下文

先跑：

```bash
aemb status ./firmware
aemb doctor ./firmware
```

重点看：

- Python 是否可运行
- 当前 AI 工具是否从项目根目录打开
- 平台是否支持 hook
- 是否设置了禁用注入的环境变量

### 11.3 `aemb check` 失败

[通用] 不要先改检查脚本，先看失败类别：

- `--arch`：架构分层或依赖方向问题
- `--hw`：硬件资源冲突或资源锁缺失
- `--spec`：规范文件不完整
- 普通失败：先看完整输出和 exit code

### 11.4 Python 找不到或版本不对

检查：

```bash
python --version
py --version
python3 --version
```

然后重新体检：

```bash
aemb doctor ./firmware
```

### 11.5 AI 声称完成但没有证据

直接要求补 evidence：

```text
请补齐完成证据：编译输出、检查输出、串口/总线日志或手册页码。没有证据不要宣称完成。
```

### 11.6 选到了预留平台

[已验证] 当前有 7 个 reserved 平台。它们不能当作 stable 平台使用。

处理方式：

```bash
aemb init ./firmware -u zhangsan --platforms claude,codex
```

不要把 `kilo`、`kiro`、`qoder` 等预留平台写进生产安装命令。

---

## 12. FAQ 与边界说明

### Q1：`aemb init` 能不能当每天启动命令？

不能。

- [通用] `init` 是安装运行时和平台接线的命令
- [通用] 日常开新任务用 `start`
- [通用] 中断后续工用 `continue`

### Q2：不指定平台时会怎样？

[已验证] CLI 帮助说明：`init` 不指定平台时默认 `claude`。

示例：

```bash
aemb init ./firmware -u zhangsan
```

等价于只安装 Claude Code 接线。

### Q3：`--all` 会安装哪些平台？

[已验证] `--all` 表示安装全部已实现平台，也就是当前 `stable` 的 7 个核心平台。

不会安装 reserved 平台。

### Q4：auto-embedded 会动我现有固件源码吗？

[通用] `init` 主要写入 `.auto-embedded/` 和平台配置接线，不应主动改业务源码。

但任务执行阶段，AI 可能按你确认的计划修改固件代码。这个修改不属于 `init` 行为。

### Q5：它能保证 AI 不犯错吗？

不能。

它能做的是：

- 限制流程顺序
- 要求先查证再编码
- 冻结硬件资源
- 用机械检查拦截一部分错误
- 要求 evidence 才能收尾

硬件实测、示波器/逻辑分析仪验证、焊接、PCB、量产认证仍然靠人。

### Q6：支持哪些芯片？

[通用] 当前文档列出的覆盖方向包括：

- STM32
- ESP32
- Arduino AVR
- GD32 / CH32 / AT32 / APM32
- NXP
- TI MSP430 / MSPM0

[关键规则] “支持”不等于每个芯片都有完整板级模板。具体项目仍要先 RESEARCH。

### Q7：`aemb mem` 能不能当第一信源？

不能。

正确顺序永远是：

1. 当前项目 `.auto-embedded/`
2. 当前任务和 journal
3. 当前 `spec/` 和 `hw-lock.yaml`
4. CLI 检查输出
5. 历史会话检索

### Q8：什么时候需要比赛模式？

[通用] 比赛模式适合：

- 电赛
- 智能车
- 西门子杯
- 平衡车
- 运动控制
- 多模块并行攻坚

小型 bug 修复、单文件驱动适配、简单文档整理，一般不需要比赛模式。

---

## 附录 A：推荐完整流程

### A.1 从零接入项目

```bash
npm install -g auto-embedded
aemb init ./firmware -u zhangsan --platforms claude,codex
aemb doctor ./firmware
```

然后在目标项目里打开 AI 工具新会话：

```text
/aemb:start 给主板增加 SHT30 温湿度读取
```

### A.2 标准任务推进

1. `start <任务标题>`：创建任务，进入 RESEARCH
2. RESEARCH：确认芯片、工程结构、外设资源、相关源码和知识库
3. 确认或补充 `hw-lock.yaml`
4. PLAN：列出文件、函数、资源、验证方式
5. 人工确认计划
6. EXECUTE：按计划逐项实现
7. REVIEW：执行 `aemb check` 和真实验证
8. `finish-work`：写 journal、promote 经验、归档任务

### A.3 最短备忘

```text
第一次装项目：aemb init -> aemb doctor -> 新开 AI 会话
新任务开工：start <任务标题>
中断后续工：continue
任务收尾：finish-work
框架升级：aemb upgrade -> aemb update <项目>
出问题先查：aemb doctor -> aemb status -> aemb check
```

---

## 附录 B：常见但不要脑补的内容

下面这些说法很容易写错：

- “所有平台都有自动注入”
- “所有平台都能派子 Agent”
- “`aemb init` 后不用再验证硬件”
- “`aemb update` 会覆盖用户全部配置”
- “历史会话里提到过的结论就是当前项目事实”
- “`--all` 包含预留平台”

更好的写法：

- [平台差异] 不同平台注入能力分为 `push`、`pull`、`command`。
- [通用] 硬件相关结论必须经过项目代码、资源锁、手册或实测证据确认。
- [已验证] 当前仓库有 7 个 stable 平台和 7 个 reserved 平台。
- [通用] 升级 managed 内容时，用户改过的文件可能生成 `.new` 等待人工合并。

### B.1 本手册底线

**只把仓库里核对到的写成事实，只把框架规律写成通用，只把平台差别写成平台差异，只把本地经验写成本机特例。**
