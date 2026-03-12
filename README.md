# Inverter Project Code for ECEn 446

### Commands to run the code
Build the code: `make -j`
Download it to board: `STM32_Programmer_CLI -c port=SWD -w build\Inverter.elf -v -rst`

### PWM Pinout
PWM 1: PA8\
PWM 2: PA9\
PWM 3: PA10

PWM 1N: PB13\
PWM 2N: PB14\
PWM 3N: PB15