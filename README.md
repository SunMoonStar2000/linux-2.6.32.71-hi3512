# linux-2.6.32.71-hi3512
linux-2.6.32.71 hi3512 port<br>
这是我首次尝试github。<br>
这是早些年移植2.6.32到hi3512项目。当时，迅雷远程(xware)还比较好用。手上这块板子，有256M的RAM，hi3512有pci，可以接sata芯片，接硬盘，有usb，可以接无线网卡，也可以接有线网，本身功耗低，当挂机神器是非常不错的。<br>
但xware是eabi的abi，hi3512默认的2.6.14本身并不支持eabi，虽然有补丁可以打，但打完之后发现xware是跑不起来的。用file命令可以看到，xware本身支持的linux版本为2.6.21，所以索性就移植到了2.6的最后一个版本上，跑了几年，还是很稳定的，跑xware，btsync，没有问题。<br>
当时，网络mdio接口退出部分移植没有完成。所以eth不能编译为模块。<br>
我用来当下载器用的配置文件名是hi3512_defconfig。<br>
这个代码我也好久没有维护了，现在我把它上传到github上，如果您需要，请拿走它。<br>
如果这里面的代码有您的版权，请告诉我，我将删除它。<br>
