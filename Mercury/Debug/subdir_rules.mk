################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me -Ooff --include_path="C:/WorkSpace/Mercury/Mercury_vD197/Mercury/MAIN" --include_path="C:/WorkSpace/Mercury/Mercury_vD197/common" --include_path="C:/WorkSpace/Mercury/Mercury_vD197/dummy" --include_path="C:/ti/ccs1031/ccs/tools/compiler/ti-cgt-arm_20.2.4.LTS/include" --include_path="C:/ti/tivaware_c_series_2_1_4_178" --define=ccs="ccs" --define=PART_TM4C1290NCZAD -g --gcc --diag_suppress=43 --diag_suppress=112 --diag_suppress=129 --diag_suppress=163 --diag_suppress=179 --diag_suppress=225 --diag_suppress=552 --diag_suppress=1047 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


