# Analysis of Arduino core lib

Author: TzeHim Sung   Date: 04/12/2019  

## Preface

本文所依赖的系统环境如下：

- OS: Manjaro 18.1.3 Juhraya  
- Kernel version: x86_64 Linux 4.19.85-1-MANJARO  
- Arduino IDE version: 1.8.9  
- Openjdk version: 11.0.5, 2019-10-15  

其中，操作系统和内核版本无要求。`Arduino IDE`版本和`JDK`版本不能太老。

## Description about Arduino

`Arduino`是一家制作开源硬件和开源软件的公司，该公司负责设计和制造单板微控制器和微控制器包，用于构建数字设备和交互式对象，以便在物理和数字世界中感知和控制对象。  

Arduino电路板设计使用各种微处理器和控制器。这些电路板配有一组数字和模拟I/O引脚，可以连接各种扩展板、面包板和其他电路。这些电路板具有串行通信接口，包括某些型号上的通用串行总线，也用于从计算机加载程序。微控制器通常使用`C/C++`编程语言。除了使用传统的编译工具链之外，Arduino项目还提供了一个基于`Processing`语言项目的集成开发环境。  

Arduino项目始于2003年，作为意大利伊夫雷亚地区伊夫雷亚交互设计研究所的学生项目，目的是为新手和专业人员提供一种低成本且简单的方法，以创建使用传感器与环境相互作用的设备执行器。适用于初学者爱好者的此类设备的常见示例包括简单机器人、恒温器和运动检测器。  

## Writing purpose

本文主要目的为介绍并分析Arduino源码库的`core`部分(以下简称“core目录”)。在`Manjaro`系统中，通过执行`yay -S arduino`安装Arduino IDE后，源代码存放在`~/.arduino15/packages/arduino/hardware/avr/1.8.1`下。其中，`core`部分的目录树如下：  

<img src="/home/jhseng/ros_experiment/Course_report/res/Screenshot_20191210_165556.png" style="zoom: 50%;" />

共44个文件。我们先从最易理解的、且为程序的入口`main.cpp`开始分析。

## Analysis of main.cpp

当我们在编写Arduino程序时，我们只能看见`void setup()`和`void loop()`两个函数，main函数去哪里了？实际上，

`main.cpp`全文如下(删去了无用的注释)：

```c++
#include <Arduino.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }

int main(void)
{
	init();

	initVariant();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

```

`main.cpp`include了`Arduino.h`。在`int main(void)`中，经过了`init()`和`USBDevice.attach()`之后，Arduino板便开始执行`setup()`、`loop()`和`serialEventRun()`(如果有串口通信的话)。

## Analysis of Arduino.h





## Reference

[1]. [Arduino官方网站](https://www.arduino.cc/)
[2]. [Arduino维基百科页面](https://en.wikipedia.org/wiki/Arduino)