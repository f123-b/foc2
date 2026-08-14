# 回归矩阵与当前基线

## 2026-08-12 快照结果

| 检查 | 结果 | 证据/限制 |
|---|---|---|
| Git baseline/branch/history | 阻塞 | 工作区无 `.git`；不能确认 `2f979e7`、比较历史或生成 commit。 |
| Host protocol | 通过 | `node host/foc-studio/test_protocol.mjs` 输出 `ASCII protocol tests passed`。 |
| Portable core | 通过 | `cmake -S firmware -B firmware/build-host`、build、`ctest --test-dir firmware/build-host --output-on-failure`：1/1 passed。 |
| Root test script（初始） | 失败 | `tools/test-project.ps1` 原先查找不存在的 `firmware/core_tests.exe`，未自行构建其依赖。 |
| Root test script（本阶段后） | 部分通过 / 阻塞 | 现已改为 CMake configure/build/CTest；host 与 portable core 可通过，随后 ARM smoke 因缺少工具链而阻塞。 |
| ARM firmware | 阻塞 | `tools/arm-gnu-toolchain/bin/arm-none-eabi-gcc.exe` 与 `tools/tup/bin/tup.exe` 不在快照中；完整 Tup build 未启动。 |
| ELF/HEX/BIN SHA-256 | 不可得 | 不存在可信的本次 ARM 构建产物，故不伪造哈希。 |

`firmware/build-host` 是本次只读审查期间生成的 host CMake 输出，不是 STM32 固件，不能烧录。

## 必备回归覆盖

| 域 | 已覆盖 | 必须补充 |
|---|---|---|
| Encoder | 基础 CPR 常量 | 50 ms rolling window、4000 CPR quantization、reversal、int32 overflow、reset、invalid estimate。 |
| Filter | init/noise/step | bandwidth transition、NaN/Inf、period invalid。 |
| Low speed | progress/stall/dither/recovery/reversal/zero/overspeed | overflow、timeout、mode exit、position activation/deactivation。 |
| Controller | portable state machine only | estimator/gain/I-limit/compensation/final-torque continuity、saturation、reversal。 |
| Protocol/safety | host serialization/parser | invalid axis/argument/mode, `g/j` watchdog isolation, `x` re-arm state reset, watchdog timeout, calibration/clear-error restrictions。 |
| HIL | 无 | HIL_TEST_PLAN 中的全部正负速度、堵转、释放和急停场景。 |

边界断言的目标点为 0.50、0.75、1.00、1.50、1.75、2.00 turn/s；应直接测量 effective Kp/Ki、blend、filter bandwidth、I limit、compensation scale 与最终 torque 的左右连续性。
