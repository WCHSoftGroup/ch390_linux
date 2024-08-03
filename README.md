ch390 SPI Ethernet Driver
===========================
This driver can only work with SPI Ethernet function in these WCH devices:
CH390H, CH390D

Integrated into your system method1
---------------------------------------
If you are using dts device tree to set up spi and driver, you can read this method, otherwise
please refer to method2.

1. Please copy the driver file to the package directory which be used to add additional drivers.

2. Please add the relevant Makefile and Kconfig like other drivers, generally you can copy one
from other driver then modify it.

3. Run the make menuconfig and select the ch390 ethernet support at "modules" item.

4. Define the spi structure on your dts file similar the follow: 
	spidev@1 {
		#address-cells = <1>;
		#size-cells = <1>;
		compatible = "ch390_ethernet";
		reg = <1 0>;
		spi-max-frequency = <5000000>;
		interrupt-parent = <&gpio0>;
		interrupts = <0 2>;
	}
	Notice that the irq request method cannot be supported in this way in some platforms.
	You should modify it in ch390.c in method ch390_open.

Integrated into your system method2
---------------------------------------
1. Please copy the driver file to the kernel directory:$kernel_src/drivers/net/ethernet

2. Please add the followed txt into the kernel file:$kernel_src/drivers/net/ethernet/Konfig
config ETHERNET_CH390
	tristate "ETHERNET_CH390 ethernet support"
	depends on SPI
	select SERIAL_CORE
	help
	  This selects support for ch390 ethernet.
	
3. Add the follow define into the $kernel_src\drivers\net\ethernet\Makefile for compile the driver.
obj-$(CONFIG_SERIAL_CH390) += ch390.o

4. Run the make menuconfig and select the ch390 ethernet support at the driver/net/ethernet and save the config.

5. Define the spi0_board_info object on your board file similar the follow:
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

**Note**

Any question, you can send feedback to mail: tech@wch.cn
