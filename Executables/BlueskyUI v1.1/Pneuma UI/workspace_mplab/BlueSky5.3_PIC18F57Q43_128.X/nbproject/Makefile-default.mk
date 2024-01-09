#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=application/hal_board_cfg.c application/app_task.c application/hal_adc.c application/hal_i2c.c application/hal_key.c application/hal_led.c application/hal_pwm.c application/hal_sdp.c application/hal_virtual_i2c.c hal_flash.c mcc_generated_files/adc/src/adcc.c mcc_generated_files/clkref/src/clkref.c mcc_generated_files/cwg/src/cwg1.c mcc_generated_files/fvr/src/fvr.c mcc_generated_files/nco/src/nco1.c mcc_generated_files/nvm/src/nvm.c mcc_generated_files/system/src/interrupt.c mcc_generated_files/system/src/system.c mcc_generated_files/system/src/config_bits.c mcc_generated_files/system/src/watchdog.c mcc_generated_files/system/src/clock.c mcc_generated_files/system/src/pins.c mcc_generated_files/timer/src/tmr1.c mcc_generated_files/timer/src/delay.c mcc_generated_files/uart/src/uart3.c mcc_generated_files/uart/src/uart2.c main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/application/hal_board_cfg.p1 ${OBJECTDIR}/application/app_task.p1 ${OBJECTDIR}/application/hal_adc.p1 ${OBJECTDIR}/application/hal_i2c.p1 ${OBJECTDIR}/application/hal_key.p1 ${OBJECTDIR}/application/hal_led.p1 ${OBJECTDIR}/application/hal_pwm.p1 ${OBJECTDIR}/application/hal_sdp.p1 ${OBJECTDIR}/application/hal_virtual_i2c.p1 ${OBJECTDIR}/hal_flash.p1 ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1 ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1 ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1 ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1 ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1 ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1 ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1 ${OBJECTDIR}/mcc_generated_files/system/src/system.p1 ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1 ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1 ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1 ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1 ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1 ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1 ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1 ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1 ${OBJECTDIR}/main.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/application/hal_board_cfg.p1.d ${OBJECTDIR}/application/app_task.p1.d ${OBJECTDIR}/application/hal_adc.p1.d ${OBJECTDIR}/application/hal_i2c.p1.d ${OBJECTDIR}/application/hal_key.p1.d ${OBJECTDIR}/application/hal_led.p1.d ${OBJECTDIR}/application/hal_pwm.p1.d ${OBJECTDIR}/application/hal_sdp.p1.d ${OBJECTDIR}/application/hal_virtual_i2c.p1.d ${OBJECTDIR}/hal_flash.p1.d ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d ${OBJECTDIR}/main.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/application/hal_board_cfg.p1 ${OBJECTDIR}/application/app_task.p1 ${OBJECTDIR}/application/hal_adc.p1 ${OBJECTDIR}/application/hal_i2c.p1 ${OBJECTDIR}/application/hal_key.p1 ${OBJECTDIR}/application/hal_led.p1 ${OBJECTDIR}/application/hal_pwm.p1 ${OBJECTDIR}/application/hal_sdp.p1 ${OBJECTDIR}/application/hal_virtual_i2c.p1 ${OBJECTDIR}/hal_flash.p1 ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1 ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1 ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1 ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1 ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1 ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1 ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1 ${OBJECTDIR}/mcc_generated_files/system/src/system.p1 ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1 ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1 ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1 ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1 ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1 ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1 ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1 ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1 ${OBJECTDIR}/main.p1

# Source Files
SOURCEFILES=application/hal_board_cfg.c application/app_task.c application/hal_adc.c application/hal_i2c.c application/hal_key.c application/hal_led.c application/hal_pwm.c application/hal_sdp.c application/hal_virtual_i2c.c hal_flash.c mcc_generated_files/adc/src/adcc.c mcc_generated_files/clkref/src/clkref.c mcc_generated_files/cwg/src/cwg1.c mcc_generated_files/fvr/src/fvr.c mcc_generated_files/nco/src/nco1.c mcc_generated_files/nvm/src/nvm.c mcc_generated_files/system/src/interrupt.c mcc_generated_files/system/src/system.c mcc_generated_files/system/src/config_bits.c mcc_generated_files/system/src/watchdog.c mcc_generated_files/system/src/clock.c mcc_generated_files/system/src/pins.c mcc_generated_files/timer/src/tmr1.c mcc_generated_files/timer/src/delay.c mcc_generated_files/uart/src/uart3.c mcc_generated_files/uart/src/uart2.c main.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F57Q43
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/application/hal_board_cfg.p1: application/hal_board_cfg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_board_cfg.p1 application/hal_board_cfg.c 
	@-${MV} ${OBJECTDIR}/application/hal_board_cfg.d ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_board_cfg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/app_task.p1: application/app_task.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/app_task.p1.d 
	@${RM} ${OBJECTDIR}/application/app_task.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/app_task.p1 application/app_task.c 
	@-${MV} ${OBJECTDIR}/application/app_task.d ${OBJECTDIR}/application/app_task.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/app_task.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_adc.p1: application/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_adc.p1 application/hal_adc.c 
	@-${MV} ${OBJECTDIR}/application/hal_adc.d ${OBJECTDIR}/application/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_i2c.p1: application/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_i2c.p1 application/hal_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_i2c.d ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_key.p1: application/hal_key.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_key.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_key.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_key.p1 application/hal_key.c 
	@-${MV} ${OBJECTDIR}/application/hal_key.d ${OBJECTDIR}/application/hal_key.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_key.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_led.p1: application/hal_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_led.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_led.p1 application/hal_led.c 
	@-${MV} ${OBJECTDIR}/application/hal_led.d ${OBJECTDIR}/application/hal_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_pwm.p1: application/hal_pwm.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_pwm.p1 application/hal_pwm.c 
	@-${MV} ${OBJECTDIR}/application/hal_pwm.d ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_pwm.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_sdp.p1: application/hal_sdp.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_sdp.p1 application/hal_sdp.c 
	@-${MV} ${OBJECTDIR}/application/hal_sdp.d ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_sdp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_virtual_i2c.p1: application/hal_virtual_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_virtual_i2c.p1 application/hal_virtual_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_virtual_i2c.d ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/hal_flash.p1: hal_flash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/hal_flash.p1.d 
	@${RM} ${OBJECTDIR}/hal_flash.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/hal_flash.p1 hal_flash.c 
	@-${MV} ${OBJECTDIR}/hal_flash.d ${OBJECTDIR}/hal_flash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/hal_flash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1: mcc_generated_files/adc/src/adcc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/adc/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1 mcc_generated_files/adc/src/adcc.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.d ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1: mcc_generated_files/clkref/src/clkref.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/clkref/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1 mcc_generated_files/clkref/src/clkref.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.d ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1: mcc_generated_files/cwg/src/cwg1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cwg/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1 mcc_generated_files/cwg/src/cwg1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.d ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1: mcc_generated_files/fvr/src/fvr.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/fvr/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1 mcc_generated_files/fvr/src/fvr.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.d ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1: mcc_generated_files/nco/src/nco1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/nco/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1 mcc_generated_files/nco/src/nco1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.d ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1: mcc_generated_files/nvm/src/nvm.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/nvm/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1 mcc_generated_files/nvm/src/nvm.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.d ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1: mcc_generated_files/system/src/interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1 mcc_generated_files/system/src/interrupt.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.d ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/system.p1: mcc_generated_files/system/src/system.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/system.p1 mcc_generated_files/system/src/system.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/system.d ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1: mcc_generated_files/system/src/config_bits.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1 mcc_generated_files/system/src/config_bits.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.d ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1: mcc_generated_files/system/src/watchdog.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1 mcc_generated_files/system/src/watchdog.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.d ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/clock.p1: mcc_generated_files/system/src/clock.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1 mcc_generated_files/system/src/clock.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/clock.d ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/pins.p1: mcc_generated_files/system/src/pins.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1 mcc_generated_files/system/src/pins.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/pins.d ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1: mcc_generated_files/timer/src/tmr1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/timer/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1 mcc_generated_files/timer/src/tmr1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.d ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1: mcc_generated_files/timer/src/delay.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/timer/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1 mcc_generated_files/timer/src/delay.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.d ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1: mcc_generated_files/uart/src/uart3.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/uart/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1 mcc_generated_files/uart/src/uart3.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.d ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1: mcc_generated_files/uart/src/uart2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/uart/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1 mcc_generated_files/uart/src/uart2.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.d ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/application/hal_board_cfg.p1: application/hal_board_cfg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_board_cfg.p1 application/hal_board_cfg.c 
	@-${MV} ${OBJECTDIR}/application/hal_board_cfg.d ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_board_cfg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/app_task.p1: application/app_task.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/app_task.p1.d 
	@${RM} ${OBJECTDIR}/application/app_task.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/app_task.p1 application/app_task.c 
	@-${MV} ${OBJECTDIR}/application/app_task.d ${OBJECTDIR}/application/app_task.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/app_task.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_adc.p1: application/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_adc.p1 application/hal_adc.c 
	@-${MV} ${OBJECTDIR}/application/hal_adc.d ${OBJECTDIR}/application/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_i2c.p1: application/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_i2c.p1 application/hal_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_i2c.d ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_key.p1: application/hal_key.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_key.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_key.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_key.p1 application/hal_key.c 
	@-${MV} ${OBJECTDIR}/application/hal_key.d ${OBJECTDIR}/application/hal_key.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_key.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_led.p1: application/hal_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_led.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_led.p1 application/hal_led.c 
	@-${MV} ${OBJECTDIR}/application/hal_led.d ${OBJECTDIR}/application/hal_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_pwm.p1: application/hal_pwm.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_pwm.p1 application/hal_pwm.c 
	@-${MV} ${OBJECTDIR}/application/hal_pwm.d ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_pwm.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_sdp.p1: application/hal_sdp.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_sdp.p1 application/hal_sdp.c 
	@-${MV} ${OBJECTDIR}/application/hal_sdp.d ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_sdp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_virtual_i2c.p1: application/hal_virtual_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/application/hal_virtual_i2c.p1 application/hal_virtual_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_virtual_i2c.d ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/hal_flash.p1: hal_flash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/hal_flash.p1.d 
	@${RM} ${OBJECTDIR}/hal_flash.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/hal_flash.p1 hal_flash.c 
	@-${MV} ${OBJECTDIR}/hal_flash.d ${OBJECTDIR}/hal_flash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/hal_flash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1: mcc_generated_files/adc/src/adcc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/adc/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1 mcc_generated_files/adc/src/adcc.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.d ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/adc/src/adcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1: mcc_generated_files/clkref/src/clkref.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/clkref/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1 mcc_generated_files/clkref/src/clkref.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.d ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/clkref/src/clkref.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1: mcc_generated_files/cwg/src/cwg1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/cwg/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1 mcc_generated_files/cwg/src/cwg1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.d ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/cwg/src/cwg1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1: mcc_generated_files/fvr/src/fvr.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/fvr/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1 mcc_generated_files/fvr/src/fvr.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.d ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/fvr/src/fvr.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1: mcc_generated_files/nco/src/nco1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/nco/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1 mcc_generated_files/nco/src/nco1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.d ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/nco/src/nco1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1: mcc_generated_files/nvm/src/nvm.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/nvm/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1 mcc_generated_files/nvm/src/nvm.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.d ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/nvm/src/nvm.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1: mcc_generated_files/system/src/interrupt.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1 mcc_generated_files/system/src/interrupt.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.d ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/interrupt.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/system.p1: mcc_generated_files/system/src/system.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/system.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/system.p1 mcc_generated_files/system/src/system.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/system.d ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/system.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1: mcc_generated_files/system/src/config_bits.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1 mcc_generated_files/system/src/config_bits.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.d ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/config_bits.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1: mcc_generated_files/system/src/watchdog.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1 mcc_generated_files/system/src/watchdog.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.d ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/watchdog.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/clock.p1: mcc_generated_files/system/src/clock.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1 mcc_generated_files/system/src/clock.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/clock.d ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/clock.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/system/src/pins.p1: mcc_generated_files/system/src/pins.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/system/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1 mcc_generated_files/system/src/pins.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/system/src/pins.d ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/system/src/pins.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1: mcc_generated_files/timer/src/tmr1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/timer/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1 mcc_generated_files/timer/src/tmr1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.d ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/timer/src/tmr1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1: mcc_generated_files/timer/src/delay.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/timer/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1 mcc_generated_files/timer/src/delay.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.d ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/timer/src/delay.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1: mcc_generated_files/uart/src/uart3.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/uart/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1 mcc_generated_files/uart/src/uart3.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.d ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/uart/src/uart3.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1: mcc_generated_files/uart/src/uart2.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files/uart/src" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1 mcc_generated_files/uart/src/uart2.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.d ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/uart/src/uart2.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     -o ${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.map  -D__DEBUG=1  -mdebugger=pickit4  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto        $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} ${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.hex 
	
else
${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -memi=wordwrite -O0 -fasmfile -maddrqual=require -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=-3 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-download -mdefault-config-bits -std=c99 -gdwarf-3 -mstack=compiled:auto:auto:auto     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/BlueSky5.3_PIC18F57Q43_128.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${OBJECTDIR}
	${RM} -r ${DISTDIR}

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
