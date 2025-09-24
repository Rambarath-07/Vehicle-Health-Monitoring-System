################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/IR.c \
../Src/RFID.c \
../Src/cmn.c \
../Src/dh11.c \
../Src/lcd.c \
../Src/level.c \
../Src/main.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32f4xx.c \
../Src/vibration.c 

OBJS += \
./Src/IR.o \
./Src/RFID.o \
./Src/cmn.o \
./Src/dh11.o \
./Src/lcd.o \
./Src/level.o \
./Src/main.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32f4xx.o \
./Src/vibration.o 

C_DEPS += \
./Src/IR.d \
./Src/RFID.d \
./Src/cmn.d \
./Src/dh11.d \
./Src/lcd.d \
./Src/level.d \
./Src/main.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32f4xx.d \
./Src/vibration.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F4 -DSTM32F405RGTx -c -I../Inc -I"C:/Users/BFR3KOR/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Include" -I"C:/Users/BFR3KOR/STM32Cube/Repository/STM32Cube_FW_F4_V1.27.1/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/IR.d ./Src/IR.o ./Src/IR.su ./Src/RFID.d ./Src/RFID.o ./Src/RFID.su ./Src/cmn.d ./Src/cmn.o ./Src/cmn.su ./Src/dh11.d ./Src/dh11.o ./Src/dh11.su ./Src/lcd.d ./Src/lcd.o ./Src/lcd.su ./Src/level.d ./Src/level.o ./Src/level.su ./Src/main.d ./Src/main.o ./Src/main.su ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32f4xx.d ./Src/system_stm32f4xx.o ./Src/system_stm32f4xx.su ./Src/vibration.d ./Src/vibration.o ./Src/vibration.su

.PHONY: clean-Src

