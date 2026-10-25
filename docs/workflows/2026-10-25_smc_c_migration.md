# SMC 转为 C 语言

- 用户确认保留原类全部字段后进行 C 语言迁移
- `User/component/SMC/slidingmodec.cpp` 改为 `slidingmodec.c`，原类改为 `SMC_t`
- 保留全部 19 个 float 字段的名称和顺序，包括原 private 状态及暂未使用的 error_last
- 构造函数替换为 SMC_Init，参数顺序与原构造函数一致，初始化清零历史状态
- 成员计算函数替换为 SMC_Tick，新增首参数为实例指针；原控制律、差分顺序、死区返回和限幅顺序保留
- 浮点绝对值及幂运算使用 C 的 fabsf、powf，移除 HAL 与 C++ 头文件依赖
- 更新 C 使用示例，将 .c 源文件加入 CMake 和 Keil，Keil FileType 为 1
- 当前业务模块尚未调用 SMC，本次未接入电机控制链

## 验证命令及结果

1. `gcc -std=c99 -Wall -Wextra -Werror -pedantic -fsyntax-only User/component/SMC/slidingmodec.h`
   - 通过，头文件可独立作为 C 编译，无 HAL 依赖
2. `gcc -std=c99 -Wall -Wextra -Werror -pedantic -c User/component/SMC/slidingmodec.c -o build/smc-c-verification/smc.o`
   - 通过，严格 C99 编译无警告
3. 使用 MinGW g++ 将迁移前保存的原 C++ 源码与 C 对象文件链接，对照运行 20000 个连续周期
   - 两组增益覆盖正负误差、分数误差、零与死区边界、一度切换边界、目标变化和速度变化
   - 对照输出及历史状态，允许浮点数学函数差异，误差阈值为 2e-5 * (1 + abs(reference))，全部通过
   - 额外检查全部参数及历史状态初始化、小数误差输出 -480.4、正负限幅、多实例互不干扰和空指针，全部通过
4. `arm-none-eabi-gcc -std=c99 -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Werror -pedantic -c User/component/SMC/slidingmodec.c -o build/smc-c-verification/smc-arm.o`
   - 使用 STM32Cube Bundle GNU Arm 14.3.1，通过，目标 ARM 单文件编译无警告
5. `cmake --build --preset Debug --parallel 2`
   - 首次沙箱内构建长时间无进展，中止后在沙箱外重试通过
   - 新 SMC C 对象编译及固件链接均成功，无编译警告
   - RAM 42544 B，FLASH 110804 B；SMC 暂未被调用，其未使用函数会被链接器回收
6. `python .auto-embedded/scripts/check.py`
   - ARCH、HW、SPEC 全部通过
7. 自动核对与人工复核
   - 19 个原字段及顺序完全一致，UTF-8、120 列、Allman 花括号、控制块括号、注释和行尾空白检查通过
   - Keil XML 解析、C 文件类型及源文件路径检查通过
   - git diff --check 通过

## 验证边界

- 本机未找到可用 UV4 或 armcc，未实际执行 Keil ARMCC 5 编译
- GNU Arm C99 编译与 Keil 工程文件检查不能替代 ARMCC 实机编译结果
- 未烧录、未运行真实电机，原控制律的物理效果及采样周期语义未在本次修改中重新验证
- 临时原版副本、对照程序、测试对象和可执行文件在记录结果后清理
- 用户已有 config.c 修改及文档删除不纳入本次提交
