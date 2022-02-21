################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/GUI.c \
../Core/main.c \
../Core/stm32h7xx_hal_msp.c \
../Core/stm32h7xx_it.c \
../Core/system_stm32h7xx.c \
../Core/test.c 

OBJS += \
./Core/GUI.o \
./Core/main.o \
./Core/stm32h7xx_hal_msp.o \
./Core/stm32h7xx_it.o \
./Core/system_stm32h7xx.o \
./Core/test.o 

C_DEPS += \
./Core/GUI.d \
./Core/main.d \
./Core/stm32h7xx_hal_msp.d \
./Core/stm32h7xx_it.d \
./Core/system_stm32h7xx.d \
./Core/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/%.o: ../Core/%.c Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../HARDWARE/LED -I../HARDWARE/SDRAM -I../HARDWARE/KEY -I../HARDWARE/TOUCH -I../HARDWARE/LCD -I../SYSTEM/delay -I../SYSTEM/sys -I../CMSIS -I../HALLIB/STM32H7xx_HAL_Driver/inc -I../Core -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

