# Inverter Project Code for ECEn 446

### Commands to run the code
Build the code: `make -j`\
Download it to board: `STM32_Programmer_CLI -c port=SWD -w build\Inverter.elf -v -rst`

### PWM Pinout
PWM 1: PA8 - D7 - CH10 23\
PWM 2: PA9 - D8 - CH10 21\
PWM 3: PA10 - D2 - CH10 33

PWM 1N: PB13 - CH10 30\
PWM 2N: PB14 - CH10 28\
PWM 3N: PB15 - CH10 26

GND: CN7 19, 20, 22; CN5 7

### Usage
To run it on the board, plug in PWM 1 and PWM 1N (CH10 pin 23 and 30, on the back) and any of the GND pins. I usually just flip the board and plug it in there.