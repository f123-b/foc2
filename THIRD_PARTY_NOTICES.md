# 第三方组件与归属

FOC Studio 的产品配置、ABZ 控制增强、主机应用、构建脚本、测试与文档由本项目维护。

本仓库的 `firmware-target/Firmware` 与 `firmware/vendor/upstream-v0.5.1` 包含来自 ODrive v0.5.1 的修改版源码，并保留其 MIT 许可证和版权声明。STM32 HAL、FreeRTOS、CMSIS、USB Device Library、Fibre 及 Python 依赖也各自保留其原有许可证。它们是第三方基础组件；不能删除版权、许可证或将这些组件描述为从零自主实现。

对外材料可以将产品称为“FOC Studio”，并准确表述为“包含经修改的 MIT-licensed ODrive v0.5.1 firmware components”。
