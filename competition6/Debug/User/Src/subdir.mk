################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/Src/ad9833.c \
../User/Src/edgesync.c \
../User/Src/encoder.c \
../User/Src/font.c \
../User/Src/mainoop.c \
../User/Src/mydraw.c \
../User/Src/oled.c 

OBJS += \
./User/Src/ad9833.o \
./User/Src/edgesync.o \
./User/Src/encoder.o \
./User/Src/font.o \
./User/Src/mainoop.o \
./User/Src/mydraw.o \
./User/Src/oled.o 

C_DEPS += \
./User/Src/ad9833.d \
./User/Src/edgesync.d \
./User/Src/encoder.d \
./User/Src/font.d \
./User/Src/mainoop.d \
./User/Src/mydraw.d \
./User/Src/oled.d 


# Each subdirectory must supply rules for building sources it contributes
User/Src/%.o User/Src/%.su User/Src/%.cyclo: ../User/Src/%.c User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -IC:/Users/qipif/Desktop/StmProject/Multiconfiger/competition6/Drivers/CMSIS/Include -IC:/Users/qipif/Desktop/StmProject/Multiconfiger/competition6/Drivers/CMSIS/DSP/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/qipif/Desktop/StmProject/Multiconfiger/competition6/User/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User-2f-Src

clean-User-2f-Src:
	-$(RM) ./User/Src/ad9833.cyclo ./User/Src/ad9833.d ./User/Src/ad9833.o ./User/Src/ad9833.su ./User/Src/edgesync.cyclo ./User/Src/edgesync.d ./User/Src/edgesync.o ./User/Src/edgesync.su ./User/Src/encoder.cyclo ./User/Src/encoder.d ./User/Src/encoder.o ./User/Src/encoder.su ./User/Src/font.cyclo ./User/Src/font.d ./User/Src/font.o ./User/Src/font.su ./User/Src/mainoop.cyclo ./User/Src/mainoop.d ./User/Src/mainoop.o ./User/Src/mainoop.su ./User/Src/mydraw.cyclo ./User/Src/mydraw.d ./User/Src/mydraw.o ./User/Src/mydraw.su ./User/Src/oled.cyclo ./User/Src/oled.d ./User/Src/oled.o ./User/Src/oled.su

.PHONY: clean-User-2f-Src

