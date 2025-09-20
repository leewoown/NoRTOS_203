################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"./C2806XSrc/F2806x_Adc.obj" \
"./C2806XSrc/F2806x_CSMPasswords.obj" \
"./C2806XSrc/F2806x_CodeStartBranch.obj" \
"./C2806XSrc/F2806x_CpuTimers.obj" \
"./C2806XSrc/F2806x_DBGIER.obj" \
"./C2806XSrc/F2806x_DefaultIsr.obj" \
"./C2806XSrc/F2806x_Dma.obj" \
"./C2806XSrc/F2806x_ECan.obj" \
"./C2806XSrc/F2806x_EPwm.obj" \
"./C2806XSrc/F2806x_EQep.obj" \
"./C2806XSrc/F2806x_GlobalVariableDefs.obj" \
"./C2806XSrc/F2806x_Gpio.obj" \
"./C2806XSrc/F2806x_I2C.obj" \
"./C2806XSrc/F2806x_Mcbsp.obj" \
"./C2806XSrc/F2806x_MemCopy.obj" \
"./C2806XSrc/F2806x_OscComp.obj" \
"./C2806XSrc/F2806x_PieCtrl.obj" \
"./C2806XSrc/F2806x_PieVect.obj" \
"./C2806XSrc/F2806x_Sci.obj" \
"./C2806XSrc/F2806x_Spi.obj" \
"./C2806XSrc/F2806x_SysCtrl.obj" \
"./C2806XSrc/F2806x_usDelay.obj" \
"./SysSoure/BATAlgorithm.obj" \
"./SysSoure/BAT_LTC6802.obj" \
"./SysSoure/DSP28x_Project.obj" \
"./SysSoure/ProtectRelay.obj" \
"./SysSoure/main.obj" \
$(GEN_CMDS__FLAG) \
-l"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/lib/rts2800_fpu32.lib" \
-l"D:/203 14S1P_Gitia_PackBMS/00 C2806X/cmd/F28069.cmd" \
-l"D:/203 14S1P_Gitia_PackBMS/00 C2806X/cmd/F2806x_Headers_nonBIOS.cmd" \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include C2806XSrc/subdir_vars.mk
-include SysInclude/subdir_vars.mk
-include SysSoure/subdir_vars.mk
-include C2806XSrc/subdir_rules.mk
-include SysInclude/subdir_rules.mk
-include SysSoure/subdir_rules.mk
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
F28069PackBMS.out 

EXE_OUTPUTS__QUOTED += \
"F28069PackBMS.out" 


# All Target
all: $(OBJS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "F28069PackBMS.out"

# Tool invocations
F28069PackBMS.out: $(OBJS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: C2000 Linker'
	"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla0 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --opt_for_speed=2 --fp_mode=strict --fp_reassoc=on --advice:performance=all -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi -z -m"F28069PackBMS.map" --heap_size=0 --stack_size=0x0050 --warn_sections -i"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/lib" -i"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" -i"C:/ti/ccs1271/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/CMD" -i"D:/203 14S1P_Gitia_PackBMS/00 C2806X/cmd" --reread_libs --diag_wrap=off --display_error_number --xml_link_info="F28069PackBMS_linkInfo.xml" -o "F28069PackBMS.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)
	-$(RM) "C2806XSrc\F2806x_Adc.obj" "C2806XSrc\F2806x_CSMPasswords.obj" "C2806XSrc\F2806x_CodeStartBranch.obj" "C2806XSrc\F2806x_CpuTimers.obj" "C2806XSrc\F2806x_DBGIER.obj" "C2806XSrc\F2806x_DefaultIsr.obj" "C2806XSrc\F2806x_Dma.obj" "C2806XSrc\F2806x_ECan.obj" "C2806XSrc\F2806x_EPwm.obj" "C2806XSrc\F2806x_EQep.obj" "C2806XSrc\F2806x_GlobalVariableDefs.obj" "C2806XSrc\F2806x_Gpio.obj" "C2806XSrc\F2806x_I2C.obj" "C2806XSrc\F2806x_Mcbsp.obj" "C2806XSrc\F2806x_MemCopy.obj" "C2806XSrc\F2806x_OscComp.obj" "C2806XSrc\F2806x_PieCtrl.obj" "C2806XSrc\F2806x_PieVect.obj" "C2806XSrc\F2806x_Sci.obj" "C2806XSrc\F2806x_Spi.obj" "C2806XSrc\F2806x_SysCtrl.obj" "C2806XSrc\F2806x_usDelay.obj" "SysSoure\BATAlgorithm.obj" "SysSoure\BAT_LTC6802.obj" "SysSoure\DSP28x_Project.obj" "SysSoure\ProtectRelay.obj" "SysSoure\main.obj" 
	-$(RM) "C2806XSrc\F2806x_Adc.d" "C2806XSrc\F2806x_CpuTimers.d" "C2806XSrc\F2806x_DefaultIsr.d" "C2806XSrc\F2806x_Dma.d" "C2806XSrc\F2806x_ECan.d" "C2806XSrc\F2806x_EPwm.d" "C2806XSrc\F2806x_EQep.d" "C2806XSrc\F2806x_GlobalVariableDefs.d" "C2806XSrc\F2806x_Gpio.d" "C2806XSrc\F2806x_I2C.d" "C2806XSrc\F2806x_Mcbsp.d" "C2806XSrc\F2806x_MemCopy.d" "C2806XSrc\F2806x_OscComp.d" "C2806XSrc\F2806x_PieCtrl.d" "C2806XSrc\F2806x_PieVect.d" "C2806XSrc\F2806x_Sci.d" "C2806XSrc\F2806x_Spi.d" "C2806XSrc\F2806x_SysCtrl.d" "SysSoure\BATAlgorithm.d" "SysSoure\BAT_LTC6802.d" "SysSoure\DSP28x_Project.d" "SysSoure\ProtectRelay.d" "SysSoure\main.d" 
	-$(RM) "C2806XSrc\F2806x_CSMPasswords.d" "C2806XSrc\F2806x_CodeStartBranch.d" "C2806XSrc\F2806x_DBGIER.d" "C2806XSrc\F2806x_usDelay.d" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

