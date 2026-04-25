################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/Src/ad9833.c \
../User/Src/ad9959.c \
../User/Src/apll.c \
../User/Src/encoder.c \
../User/Src/font.c \
../User/Src/mainoop.c \
../User/Src/mydds.c \
../User/Src/mydraw.c \
../User/Src/myfft.c \
../User/Src/oled.c \
../User/Src/pid.c \
../User/Src/store.c \
../User/Src/xform.c 

OBJS += \
./User/Src/ad9833.o \
./User/Src/ad9959.o \
./User/Src/apll.o \
./User/Src/encoder.o \
./User/Src/font.o \
./User/Src/mainoop.o \
./User/Src/mydds.o \
./User/Src/mydraw.o \
./User/Src/myfft.o \
./User/Src/oled.o \
./User/Src/pid.o \
./User/Src/store.o \
./User/Src/xform.o 

C_DEPS += \
./User/Src/ad9833.d \
./User/Src/ad9959.d \
./User/Src/apll.d \
./User/Src/encoder.d \
./User/Src/font.d \
./User/Src/mainoop.d \
./User/Src/mydds.d \
./User/Src/mydraw.d \
./User/Src/myfft.d \
./User/Src/oled.d \
./User/Src/pid.d \
./User/Src/store.d \
./User/Src/xform.d 


# Each subdirectory must supply rules for building sources it contributes
User/Src/%.o User/Src/%.su User/Src/%.cyclo: ../User/Src/%.c User/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DARM_MATH_CM7 -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -IC:/Users/qipif/Desktop/StmProject/Multiconfiger/competition/Drivers/CMSIS/Include -IC:/Users/qipif/Desktop/StmProject/Multiconfiger/competition/Drivers/CMSIS/DSP/Include -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/qipif/Desktop/StmProject/Multiconfiger/competition/User/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-User-2f-Src

clean-User-2f-Src:
	-$(RM) ./User/Src/ad9833.cyclo ./User/Src/ad9833.d ./User/Src/ad9833.o ./User/Src/ad9833.su ./User/Src/ad9959.cyclo ./User/Src/ad9959.d ./User/Src/ad9959.o ./User/Src/ad9959.su ./User/Src/apll.cyclo ./User/Src/apll.d ./User/Src/apll.o ./User/Src/apll.su ./User/Src/encoder.cyclo ./User/Src/encoder.d ./User/Src/encoder.o ./User/Src/encoder.su ./User/Src/font.cyclo ./User/Src/font.d ./User/Src/font.o ./User/Src/font.su ./User/Src/mainoop.cyclo ./User/Src/mainoop.d ./User/Src/mainoop.o ./User/Src/mainoop.su ./User/Src/mydds.cyclo ./User/Src/mydds.d ./User/Src/mydds.o ./User/Src/mydds.su ./User/Src/mydraw.cyclo ./User/Src/mydraw.d ./User/Src/mydraw.o ./User/Src/mydraw.su ./User/Src/myfft.cyclo ./User/Src/myfft.d ./User/Src/myfft.o ./User/Src/myfft.su ./User/Src/oled.cyclo ./User/Src/oled.d ./User/Src/oled.o ./User/Src/oled.su ./User/Src/pid.cyclo ./User/Src/pid.d ./User/Src/pid.o ./User/Src/pid.su ./User/Src/store.cyclo ./User/Src/store.d ./User/Src/store.o ./User/Src/store.su ./User/Src/xform.cyclo ./User/Src/xform.d ./User/Src/xform.o ./User/Src/xform.su

.PHONY: clean-User-2f-Src

