# Inverter Project Code for ECEn 446

### Commands to run the code
Build the code: `make -j`\
Download it to board bare metal: `STM32_Programmer_CLI -c port=SWD -w build\Inverter.elf -v -rst`
Or via USB: `STM32_Programmer_CLI -c port=USB1 -w build\Inverter.elf -v`

### PWM Pinout
PWM 1: PA8 - D7 - CH10 23, CH9 8\
PWM 2: PA9 - D8 - CH10 21\
PWM 3: PA10 - D2 - CH10 33

PWM 1N: PB13 - CH10 30\
PWM 2N: PB14 - CH10 28\
PWM 3N: PB15 - CH10 26

RCC_MCO: CN7 14, CN6 3 (NRST/RESET)

ADC1_IN1: CN7 28

GND: CN7 19, 20, 22; CN5 7

### Usage
To run it on the board, plug in PWM 1 and PWM 1N (CH10 pin 23 and 30, on the back) and any of the GND pins. I usually just flip the board and plug it in there.

### Notes
ADC handler has ( (ADC_BUFFER_SIZE / 2) / Sample_rate ) seconds to use ADC data 
ADC read takes 2.5 ADC clock cycles to charge the input cap for reading.
Then it takes 12.5 ADC clock cycles to convert captured Voltage into 12-bit num
So ADC sample rate is ADC_CLOCK / ADC_prescaler / 15
ADC_CLOCK can be set in Clock Configuration, it uses SYS_CLK or PLLP (PLLP preferred for more control)

The RCC_MCO clock is a sin wave. 1MHz, it oscillates between 1.3 to 1.6 Volts (must be read Analog). I think
the peak-to-peak Voltage changes with frequency due to an RC constant.