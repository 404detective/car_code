################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AppSw/Tricore/Driver/LQ_ADC.c \
../src/AppSw/Tricore/Driver/LQ_CCU6.c \
../src/AppSw/Tricore/Driver/LQ_DMA.c \
../src/AppSw/Tricore/Driver/LQ_EEPROM.c \
../src/AppSw/Tricore/Driver/LQ_EMEM.c \
../src/AppSw/Tricore/Driver/LQ_FFT.c \
../src/AppSw/Tricore/Driver/LQ_GPIO.c \
../src/AppSw/Tricore/Driver/LQ_GPSR.c \
../src/AppSw/Tricore/Driver/LQ_GPT12_ENC.c \
../src/AppSw/Tricore/Driver/LQ_GTM.c \
../src/AppSw/Tricore/Driver/LQ_QSPI.c \
../src/AppSw/Tricore/Driver/LQ_SOFTI2C.c \
../src/AppSw/Tricore/Driver/LQ_SPI.c \
../src/AppSw/Tricore/Driver/LQ_STM.c \
../src/AppSw/Tricore/Driver/LQ_UART.c 

OBJS += \
./src/AppSw/Tricore/Driver/LQ_ADC.o \
./src/AppSw/Tricore/Driver/LQ_CCU6.o \
./src/AppSw/Tricore/Driver/LQ_DMA.o \
./src/AppSw/Tricore/Driver/LQ_EEPROM.o \
./src/AppSw/Tricore/Driver/LQ_EMEM.o \
./src/AppSw/Tricore/Driver/LQ_FFT.o \
./src/AppSw/Tricore/Driver/LQ_GPIO.o \
./src/AppSw/Tricore/Driver/LQ_GPSR.o \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.o \
./src/AppSw/Tricore/Driver/LQ_GTM.o \
./src/AppSw/Tricore/Driver/LQ_QSPI.o \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.o \
./src/AppSw/Tricore/Driver/LQ_SPI.o \
./src/AppSw/Tricore/Driver/LQ_STM.o \
./src/AppSw/Tricore/Driver/LQ_UART.o 

COMPILED_SRCS += \
./src/AppSw/Tricore/Driver/LQ_ADC.src \
./src/AppSw/Tricore/Driver/LQ_CCU6.src \
./src/AppSw/Tricore/Driver/LQ_DMA.src \
./src/AppSw/Tricore/Driver/LQ_EEPROM.src \
./src/AppSw/Tricore/Driver/LQ_EMEM.src \
./src/AppSw/Tricore/Driver/LQ_FFT.src \
./src/AppSw/Tricore/Driver/LQ_GPIO.src \
./src/AppSw/Tricore/Driver/LQ_GPSR.src \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.src \
./src/AppSw/Tricore/Driver/LQ_GTM.src \
./src/AppSw/Tricore/Driver/LQ_QSPI.src \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.src \
./src/AppSw/Tricore/Driver/LQ_SPI.src \
./src/AppSw/Tricore/Driver/LQ_STM.src \
./src/AppSw/Tricore/Driver/LQ_UART.src 

C_DEPS += \
./src/AppSw/Tricore/Driver/LQ_ADC.d \
./src/AppSw/Tricore/Driver/LQ_CCU6.d \
./src/AppSw/Tricore/Driver/LQ_DMA.d \
./src/AppSw/Tricore/Driver/LQ_EEPROM.d \
./src/AppSw/Tricore/Driver/LQ_EMEM.d \
./src/AppSw/Tricore/Driver/LQ_FFT.d \
./src/AppSw/Tricore/Driver/LQ_GPIO.d \
./src/AppSw/Tricore/Driver/LQ_GPSR.d \
./src/AppSw/Tricore/Driver/LQ_GPT12_ENC.d \
./src/AppSw/Tricore/Driver/LQ_GTM.d \
./src/AppSw/Tricore/Driver/LQ_QSPI.d \
./src/AppSw/Tricore/Driver/LQ_SOFTI2C.d \
./src/AppSw/Tricore/Driver/LQ_SPI.d \
./src/AppSw/Tricore/Driver/LQ_STM.d \
./src/AppSw/Tricore/Driver/LQ_UART.d 


# Each subdirectory must supply rules for building sources it contributes
src/AppSw/Tricore/Driver/%.src: ../src/AppSw/Tricore/Driver/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING C/C++ Compiler'
	cctc -D__CPU__=tc26xb -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gpt12" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/Driver" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/Main" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/User" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/src/AppSw/Tricore/APP" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/src/AppSw" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra/Platform/Tricore/Compilers" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Multican/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra/Platform" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cif/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Hssl/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/Trap" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/If/Ccu6If" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc/Dsadc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Port" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Stm/Timer" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dts/Dts" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eth" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Flash" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Vadc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Msc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi/SpiMaster" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Scu/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Comm" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Math" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra/Platform/Tricore" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Trig" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tim" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/TimerWithTrigger" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Emem" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Mtu" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fft" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/I2c/I2c" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Asc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Flash/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/If" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fce/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Stm/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Msc/Msc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Vadc/Adc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Pwm" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Port/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5/Psi5" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eray" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi/SpiSlave" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/Icu" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/CStart" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Hssl" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cif" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eth/Phy_Pef7071" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Hssl/Hssl" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Iom/Driver" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Multican/Can" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5s/Psi5s" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fft/Fft" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmHl" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Iom/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Lib" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/Timer" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Sent" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eray/Eray" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gpt12/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dma" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fce/Crc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra/Sfr" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra/Sfr/TC26B" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Bsp" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/General" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dts" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Src" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dma/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cif/Cam" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Src/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/I2c/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Configurations" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Lib/DataHandling" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Sent/Sent" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/Timer" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5s" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Emem/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/PwmBc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Iom" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/TPwm" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Multican" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Mtu/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Infra/Sfr/TC26B/_Reg" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom/PwmHl" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dma/Dma" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Timer" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/SysSe/Time" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Cpu/Irq" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Ccu6" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gpt12/IncrEnc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Psi5s/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Scu" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Lib/InternalMux" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Stm" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc/Rdc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Vadc/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dts/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eth/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Smu" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_PinMap" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Lin" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/StdIf" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Dsadc" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fce" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/PwmHl" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Qspi/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tom" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Tim/In" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Msc/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Fft/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Atom/Pwm" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/Service/CpuGeneric/_Utilities" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Gtm/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Smu/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/I2c" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Asclin/Spi" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Eray/Std" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Port/Io" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/_Impl" -I"E:/Desktop/AURIX-workspace/LQ_TC26xB_LIB_ADS/Libraries/iLLD/TC26B/Tricore/Sent/Std" --iso=99 --c++14 --language=+volatile --anachronisms --fp-model=3 --fp-model=c --fp-model=f --fp-model=l --fp-model=n --fp-model=r --fp-model=z -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc26xb -o "$@"  "$<"  -cs --dep-file=$(@:.src=.d) --misrac-version=2012 -N0 -Z0 -Y0 2>&1;
	@echo 'Finished building: $<'
	@echo ' '

src/AppSw/Tricore/Driver/%.o: ./src/AppSw/Tricore/Driver/%.src
	@echo 'Building file: $<'
	@echo 'Invoking: TASKING Assembler'
	astc -o  "$@" "$<" --list-format=L1 --optimize=gs
	@echo 'Finished building: $<'
	@echo ' '


