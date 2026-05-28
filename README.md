# ch390 SPI 以太网驱动

该驱动仅适用于以下 WCH 设备中的 SPI 以太网功能：

- CH390H
- CH390D

## 集成到系统的方法 1

如果你使用 dts 设备树来配置 SPI 和驱动，可以参考此方法；否则请参考方法 2。

1. 请将驱动文件复制到用于添加额外驱动的软件包目录中。

2. 请像其他驱动一样添加相应的 `Makefile` 和 `Kconfig`。通常可以从其他驱动复制一份，然后再进行修改。

3. 运行 `make menuconfig`，并在 `"modules"` 项中选择 `ch390 ethernet support`。

4. 在你的 dts 文件中定义类似如下的 SPI 结构：

```dts
#include <dt-bindings/interrupt-controller/irq.h>

spidev@1 {
	#address-cells = <1>;
	#size-cells = <1>;
	compatible = "ch390_ethernet";
	reg = <1 0>;
	spi-max-frequency = <5000000>;
	interrupt-parent = <&gpio0>;
	interrupts = <0 IRQ_TYPE_LEVEL_LOW>;
}
```

`interrupts` 的触发类型需要根据硬件连接配置为电平触发。驱动支持通过设备树设置为低电平有效或高电平有效：

```dts
interrupts = <0 IRQ_TYPE_LEVEL_LOW>;
interrupts = <0 IRQ_TYPE_LEVEL_HIGH>;
```

## 集成到系统的方法 2

1. 请将驱动文件复制到内核目录：`$kernel_src/drivers/net/ethernet`

2. 请将以下文本添加到内核文件：`$kernel_src/drivers/net/ethernet/Kconfig`

```text
config ETHERNET_CH390
	tristate "ETHERNET_CH390 ethernet support"
	depends on SPI
	select SERIAL_CORE
	help
	  This selects support for ch390 ethernet.
```

3. 将以下定义添加到 `$kernel_src/drivers/net/ethernet/Makefile`，用于编译该驱动。

```makefile
obj-$(CONFIG_SERIAL_CH390) += ch390.o
```

4. 运行 `make menuconfig`，并在 `driver/net/ethernet` 中选择 `ch390 ethernet support`，然后保存配置。

5. 在你的板级文件中定义类似如下的 `spi0_board_info` 对象：

```c
static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias = "ch390_ethernet",
		.platform_data = NULL,
		.max_speed_hz = 100 * 1000,
		.bus_num = 0,
		.chip_select = 0,
		.mode = SPI_MODE_0,
		.controller_data = &spi0_csi[0],
		.irq = IRQ_EINT(25),
	}
};
```

## 注意

如有任何问题，可以发送反馈邮件至：tech@wch.cn
