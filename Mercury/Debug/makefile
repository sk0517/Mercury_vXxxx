################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./pinout.obj" \
"./tm4c1290nczad_startup_ccs.obj" \
"./MAIN/COM.obj" \
"./MAIN/MAIN.obj" \
"./MAIN/MES.obj" \
"./MAIN/SV_def.obj" \
"./MAIN/ctlioport.obj" \
"./MAIN/cunet.obj" \
"./MAIN/defLED.obj" \
"./MAIN/defLOG.obj" \
"./MAIN/defMAIN.obj" \
"./MAIN/defMES.obj" \
"./MAIN/defSAVE.obj" \
"./MAIN/disp.obj" \
"./MAIN/download.obj" \
"./MAIN/eeprom_spi.obj" \
"./MAIN/int_flow.obj" \
"./MAIN/int_prg.obj" \
"./MAIN/log.obj" \
"./MAIN/onewire.obj" \
"./MAIN/protocol.obj" \
"./MAIN/resetprg.obj" \
"./MAIN/sbrk.obj" \
"./MAIN/util.obj" \
"./MAIN/vecttbl.obj" \
"../tm4c1290nczad.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \
-l"C:/ti/tivaware_c_series_2_1_4_178/driverlib/ccs/Debug/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include MAIN/subdir_vars.mk
-include subdir_rules.mk
-include MAIN/subdir_rules.mk
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
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
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
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
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
Mercury.out \

EXE_OUTPUTS__QUOTED += \
"Mercury.out" \

BIN_OUTPUTS += \
Mercury.Mot \

BIN_OUTPUTS__QUOTED += \
"Mercury.Mot" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "Mercury.out" secondary-outputs

# Tool invocations
Mercury.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: Arm Linker'
	"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --define=ccs="ccs" --define=PART_TM4C1290NCZAD -g --gcc --diag_suppress=43 --diag_suppress=112 --diag_suppress=129 --diag_suppress=163 --diag_suppress=179 --diag_suppress=225 --diag_suppress=552 --diag_suppress=1047 --diag_wrap=off --display_error_number --abi=eabi -z -m"Mercury.map" --heap_size=0 --stack_size=2048 -i"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/lib" -i"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="Mercury_linkInfo.xml" --rom_model -o "Mercury.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

Mercury.Mot: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: Arm Hex Utility'
	"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/bin/armhex" --memwidth=8 --romwidth=32 --diag_wrap=off --motorola=3 -o "Mercury.Mot" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "pinout.obj" "tm4c1290nczad_startup_ccs.obj" "MAIN\COM.obj" "MAIN\MAIN.obj" "MAIN\MES.obj" "MAIN\SV_def.obj" "MAIN\ctlioport.obj" "MAIN\cunet.obj" "MAIN\defLED.obj" "MAIN\defLOG.obj" "MAIN\defMAIN.obj" "MAIN\defMES.obj" "MAIN\defSAVE.obj" "MAIN\disp.obj" "MAIN\download.obj" "MAIN\eeprom_spi.obj" "MAIN\int_flow.obj" "MAIN\int_prg.obj" "MAIN\log.obj" "MAIN\onewire.obj" "MAIN\protocol.obj" "MAIN\resetprg.obj" "MAIN\sbrk.obj" "MAIN\util.obj" "MAIN\vecttbl.obj" 
	-$(RM) "pinout.d" "tm4c1290nczad_startup_ccs.d" "MAIN\COM.d" "MAIN\MAIN.d" "MAIN\MES.d" "MAIN\SV_def.d" "MAIN\ctlioport.d" "MAIN\cunet.d" "MAIN\defLED.d" "MAIN\defLOG.d" "MAIN\defMAIN.d" "MAIN\defMES.d" "MAIN\defSAVE.d" "MAIN\disp.d" "MAIN\download.d" "MAIN\eeprom_spi.d" "MAIN\int_flow.d" "MAIN\int_prg.d" "MAIN\log.d" "MAIN\onewire.d" "MAIN\protocol.d" "MAIN\resetprg.d" "MAIN\sbrk.d" "MAIN\util.d" "MAIN\vecttbl.d" 
	-@echo 'Finished clean'
	-@echo ' '

post-build:
	-"C:/ti/ccs1031/ccs/utils/tiobj2bin/tiobj2bin" "Mercury.out" "Mercury.bin" "C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/bin/armofd" "C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/bin/armhex" "C:/ti/ccs1031/ccs/utils/tiobj2bin/mkhex4bin"
	-@echo ' '

secondary-outputs: $(BIN_OUTPUTS)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

