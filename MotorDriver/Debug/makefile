################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./empty_cpp.o" \
"./ti_msp_dl_config.o" \
"./startup_mspm0g351x_ticlang.o" \
"./app/motor_app.o" \
"./board/board_encoders.o" \
"./board/board_i2c_target.o" \
"./board/board_motor_outputs.o" \
"./board/board_uart.o" \
"./control/motor_control.o" \
"./drivers/drv8701.o" \
"./drivers/quadrature_encoder.o" \
"./drivers/uart_link.o" \
"./protocol/register_map.o" \
"./protocol/serial_protocol.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include app/subdir_vars.mk
-include board/subdir_vars.mk
-include control/subdir_vars.mk
-include drivers/subdir_vars.mk
-include protocol/subdir_vars.mk
-include subdir_rules.mk
-include app/subdir_rules.mk
-include board/subdir_rules.mk
-include control/subdir_rules.mk
-include drivers/subdir_rules.mk
-include protocol/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
MotorDriver.out 

EXE_OUTPUTS__QUOTED += \
"MotorDriver.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "MotorDriver.out"

# Tool invocations
MotorDriver.out: $(OBJS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O2 -gdwarf-3 -Wall -Wl,-m"MotorDriver.map" -Wl,-i"C:/ti/mspm0_sdk_2_10_00_04/source" -Wl,-i"C:/Users/DdXd/workspace_ccstheia/MotorDriver" -Wl,-i"C:/Users/DdXd/workspace_ccstheia/MotorDriver/Debug/syscfg" -Wl,-i"C:/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="MotorDriver_linkInfo.xml" -Wl,--rom_model -o "MotorDriver.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "empty_cpp.o" "ti_msp_dl_config.o" "startup_mspm0g351x_ticlang.o" "app\motor_app.o" "board\board_encoders.o" "board\board_i2c_target.o" "board\board_motor_outputs.o" "board\board_uart.o" "control\motor_control.o" "drivers\drv8701.o" "drivers\quadrature_encoder.o" "drivers\uart_link.o" "protocol\register_map.o" "protocol\serial_protocol.o" 
	-$(RM) "ti_msp_dl_config.d" "startup_mspm0g351x_ticlang.d" 
	-$(RM) "empty_cpp.d" "app\motor_app.d" "board\board_encoders.d" "board\board_i2c_target.d" "board\board_motor_outputs.d" "board\board_uart.d" "control\motor_control.d" "drivers\drv8701.d" "drivers\quadrature_encoder.d" "drivers\uart_link.d" "protocol\register_map.d" "protocol\serial_protocol.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

