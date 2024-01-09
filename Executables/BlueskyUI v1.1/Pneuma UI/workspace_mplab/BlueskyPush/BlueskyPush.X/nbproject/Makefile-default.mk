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
FINAL_IMAGE=${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
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
SOURCEFILES_QUOTED_IF_SPACED=application/hal_board_cfg.c application/hal_led.c application/hal_pwm.c application/hal_adc.c application/hal_key.c application/hal_i2c.c application/hal_virtual_i2c.c application/hal_sdp.c application/app_task.c application/hal_flash.c mcc_generated_files/device_config.c mcc_generated_files/tmr1.c mcc_generated_files/eusart.c mcc_generated_files/cwg1.c mcc_generated_files/pin_manager.c mcc_generated_files/nco1.c mcc_generated_files/fvr.c mcc_generated_files/mcc.c mcc_generated_files/adcc.c mcc_generated_files/interrupt_manager.c mcc_generated_files/memory.c mcc_generated_files/clkref.c main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/application/hal_board_cfg.p1 ${OBJECTDIR}/application/hal_led.p1 ${OBJECTDIR}/application/hal_pwm.p1 ${OBJECTDIR}/application/hal_adc.p1 ${OBJECTDIR}/application/hal_key.p1 ${OBJECTDIR}/application/hal_i2c.p1 ${OBJECTDIR}/application/hal_virtual_i2c.p1 ${OBJECTDIR}/application/hal_sdp.p1 ${OBJECTDIR}/application/app_task.p1 ${OBJECTDIR}/application/hal_flash.p1 ${OBJECTDIR}/mcc_generated_files/device_config.p1 ${OBJECTDIR}/mcc_generated_files/tmr1.p1 ${OBJECTDIR}/mcc_generated_files/eusart.p1 ${OBJECTDIR}/mcc_generated_files/cwg1.p1 ${OBJECTDIR}/mcc_generated_files/pin_manager.p1 ${OBJECTDIR}/mcc_generated_files/nco1.p1 ${OBJECTDIR}/mcc_generated_files/fvr.p1 ${OBJECTDIR}/mcc_generated_files/mcc.p1 ${OBJECTDIR}/mcc_generated_files/adcc.p1 ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1 ${OBJECTDIR}/mcc_generated_files/memory.p1 ${OBJECTDIR}/mcc_generated_files/clkref.p1 ${OBJECTDIR}/main.p1
POSSIBLE_DEPFILES=${OBJECTDIR}/application/hal_board_cfg.p1.d ${OBJECTDIR}/application/hal_led.p1.d ${OBJECTDIR}/application/hal_pwm.p1.d ${OBJECTDIR}/application/hal_adc.p1.d ${OBJECTDIR}/application/hal_key.p1.d ${OBJECTDIR}/application/hal_i2c.p1.d ${OBJECTDIR}/application/hal_virtual_i2c.p1.d ${OBJECTDIR}/application/hal_sdp.p1.d ${OBJECTDIR}/application/app_task.p1.d ${OBJECTDIR}/application/hal_flash.p1.d ${OBJECTDIR}/mcc_generated_files/device_config.p1.d ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d ${OBJECTDIR}/mcc_generated_files/eusart.p1.d ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d ${OBJECTDIR}/mcc_generated_files/nco1.p1.d ${OBJECTDIR}/mcc_generated_files/fvr.p1.d ${OBJECTDIR}/mcc_generated_files/mcc.p1.d ${OBJECTDIR}/mcc_generated_files/adcc.p1.d ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d ${OBJECTDIR}/mcc_generated_files/memory.p1.d ${OBJECTDIR}/mcc_generated_files/clkref.p1.d ${OBJECTDIR}/main.p1.d

# Object Files
OBJECTFILES=${OBJECTDIR}/application/hal_board_cfg.p1 ${OBJECTDIR}/application/hal_led.p1 ${OBJECTDIR}/application/hal_pwm.p1 ${OBJECTDIR}/application/hal_adc.p1 ${OBJECTDIR}/application/hal_key.p1 ${OBJECTDIR}/application/hal_i2c.p1 ${OBJECTDIR}/application/hal_virtual_i2c.p1 ${OBJECTDIR}/application/hal_sdp.p1 ${OBJECTDIR}/application/app_task.p1 ${OBJECTDIR}/application/hal_flash.p1 ${OBJECTDIR}/mcc_generated_files/device_config.p1 ${OBJECTDIR}/mcc_generated_files/tmr1.p1 ${OBJECTDIR}/mcc_generated_files/eusart.p1 ${OBJECTDIR}/mcc_generated_files/cwg1.p1 ${OBJECTDIR}/mcc_generated_files/pin_manager.p1 ${OBJECTDIR}/mcc_generated_files/nco1.p1 ${OBJECTDIR}/mcc_generated_files/fvr.p1 ${OBJECTDIR}/mcc_generated_files/mcc.p1 ${OBJECTDIR}/mcc_generated_files/adcc.p1 ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1 ${OBJECTDIR}/mcc_generated_files/memory.p1 ${OBJECTDIR}/mcc_generated_files/clkref.p1 ${OBJECTDIR}/main.p1

# Source Files
SOURCEFILES=application/hal_board_cfg.c application/hal_led.c application/hal_pwm.c application/hal_adc.c application/hal_key.c application/hal_i2c.c application/hal_virtual_i2c.c application/hal_sdp.c application/app_task.c application/hal_flash.c mcc_generated_files/device_config.c mcc_generated_files/tmr1.c mcc_generated_files/eusart.c mcc_generated_files/cwg1.c mcc_generated_files/pin_manager.c mcc_generated_files/nco1.c mcc_generated_files/fvr.c mcc_generated_files/mcc.c mcc_generated_files/adcc.c mcc_generated_files/interrupt_manager.c mcc_generated_files/memory.c mcc_generated_files/clkref.c main.c



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
	${MAKE}  -f nbproject/Makefile-default.mk ${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=16F18877
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/application/hal_board_cfg.p1: application/hal_board_cfg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_board_cfg.p1 application/hal_board_cfg.c 
	@-${MV} ${OBJECTDIR}/application/hal_board_cfg.d ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_board_cfg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_led.p1: application/hal_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_led.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_led.p1 application/hal_led.c 
	@-${MV} ${OBJECTDIR}/application/hal_led.d ${OBJECTDIR}/application/hal_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_pwm.p1: application/hal_pwm.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_pwm.p1 application/hal_pwm.c 
	@-${MV} ${OBJECTDIR}/application/hal_pwm.d ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_pwm.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_adc.p1: application/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_adc.p1 application/hal_adc.c 
	@-${MV} ${OBJECTDIR}/application/hal_adc.d ${OBJECTDIR}/application/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_key.p1: application/hal_key.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_key.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_key.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_key.p1 application/hal_key.c 
	@-${MV} ${OBJECTDIR}/application/hal_key.d ${OBJECTDIR}/application/hal_key.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_key.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_i2c.p1: application/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_i2c.p1 application/hal_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_i2c.d ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_virtual_i2c.p1: application/hal_virtual_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_virtual_i2c.p1 application/hal_virtual_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_virtual_i2c.d ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_sdp.p1: application/hal_sdp.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_sdp.p1 application/hal_sdp.c 
	@-${MV} ${OBJECTDIR}/application/hal_sdp.d ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_sdp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/app_task.p1: application/app_task.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/app_task.p1.d 
	@${RM} ${OBJECTDIR}/application/app_task.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/app_task.p1 application/app_task.c 
	@-${MV} ${OBJECTDIR}/application/app_task.d ${OBJECTDIR}/application/app_task.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/app_task.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_flash.p1: application/hal_flash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_flash.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_flash.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_flash.p1 application/hal_flash.c 
	@-${MV} ${OBJECTDIR}/application/hal_flash.d ${OBJECTDIR}/application/hal_flash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_flash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/device_config.p1: mcc_generated_files/device_config.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/device_config.p1 mcc_generated_files/device_config.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/device_config.d ${OBJECTDIR}/mcc_generated_files/device_config.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/device_config.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/tmr1.p1: mcc_generated_files/tmr1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/tmr1.p1 mcc_generated_files/tmr1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/tmr1.d ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/eusart.p1: mcc_generated_files/eusart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/eusart.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/eusart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/eusart.p1 mcc_generated_files/eusart.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/eusart.d ${OBJECTDIR}/mcc_generated_files/eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/cwg1.p1: mcc_generated_files/cwg1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/cwg1.p1 mcc_generated_files/cwg1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/cwg1.d ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/pin_manager.p1: mcc_generated_files/pin_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/pin_manager.p1 mcc_generated_files/pin_manager.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/pin_manager.d ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/nco1.p1: mcc_generated_files/nco1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/nco1.p1 mcc_generated_files/nco1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/nco1.d ${OBJECTDIR}/mcc_generated_files/nco1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/nco1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/fvr.p1: mcc_generated_files/fvr.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/fvr.p1 mcc_generated_files/fvr.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/fvr.d ${OBJECTDIR}/mcc_generated_files/fvr.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/fvr.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/mcc.p1: mcc_generated_files/mcc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/mcc.p1 mcc_generated_files/mcc.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/mcc.d ${OBJECTDIR}/mcc_generated_files/mcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/mcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/adcc.p1: mcc_generated_files/adcc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adcc.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adcc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/adcc.p1 mcc_generated_files/adcc.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/adcc.d ${OBJECTDIR}/mcc_generated_files/adcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/adcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1: mcc_generated_files/interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1 mcc_generated_files/interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.d ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/memory.p1: mcc_generated_files/memory.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/memory.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/memory.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/memory.p1 mcc_generated_files/memory.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/memory.d ${OBJECTDIR}/mcc_generated_files/memory.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/memory.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/clkref.p1: mcc_generated_files/clkref.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/clkref.p1 mcc_generated_files/clkref.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/clkref.d ${OBJECTDIR}/mcc_generated_files/clkref.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/clkref.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c  -D__DEBUG=1  -mdebugger=pickit4   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/main.p1 main.c 
	@-${MV} ${OBJECTDIR}/main.d ${OBJECTDIR}/main.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/main.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/application/hal_board_cfg.p1: application/hal_board_cfg.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_board_cfg.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_board_cfg.p1 application/hal_board_cfg.c 
	@-${MV} ${OBJECTDIR}/application/hal_board_cfg.d ${OBJECTDIR}/application/hal_board_cfg.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_board_cfg.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_led.p1: application/hal_led.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_led.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_led.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_led.p1 application/hal_led.c 
	@-${MV} ${OBJECTDIR}/application/hal_led.d ${OBJECTDIR}/application/hal_led.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_led.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_pwm.p1: application/hal_pwm.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_pwm.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_pwm.p1 application/hal_pwm.c 
	@-${MV} ${OBJECTDIR}/application/hal_pwm.d ${OBJECTDIR}/application/hal_pwm.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_pwm.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_adc.p1: application/hal_adc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_adc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_adc.p1 application/hal_adc.c 
	@-${MV} ${OBJECTDIR}/application/hal_adc.d ${OBJECTDIR}/application/hal_adc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_adc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_key.p1: application/hal_key.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_key.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_key.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_key.p1 application/hal_key.c 
	@-${MV} ${OBJECTDIR}/application/hal_key.d ${OBJECTDIR}/application/hal_key.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_key.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_i2c.p1: application/hal_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_i2c.p1 application/hal_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_i2c.d ${OBJECTDIR}/application/hal_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_virtual_i2c.p1: application/hal_virtual_i2c.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_virtual_i2c.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_virtual_i2c.p1 application/hal_virtual_i2c.c 
	@-${MV} ${OBJECTDIR}/application/hal_virtual_i2c.d ${OBJECTDIR}/application/hal_virtual_i2c.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_virtual_i2c.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_sdp.p1: application/hal_sdp.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_sdp.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_sdp.p1 application/hal_sdp.c 
	@-${MV} ${OBJECTDIR}/application/hal_sdp.d ${OBJECTDIR}/application/hal_sdp.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_sdp.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/app_task.p1: application/app_task.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/app_task.p1.d 
	@${RM} ${OBJECTDIR}/application/app_task.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/app_task.p1 application/app_task.c 
	@-${MV} ${OBJECTDIR}/application/app_task.d ${OBJECTDIR}/application/app_task.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/app_task.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/application/hal_flash.p1: application/hal_flash.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/application" 
	@${RM} ${OBJECTDIR}/application/hal_flash.p1.d 
	@${RM} ${OBJECTDIR}/application/hal_flash.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/application/hal_flash.p1 application/hal_flash.c 
	@-${MV} ${OBJECTDIR}/application/hal_flash.d ${OBJECTDIR}/application/hal_flash.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/application/hal_flash.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/device_config.p1: mcc_generated_files/device_config.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/device_config.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/device_config.p1 mcc_generated_files/device_config.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/device_config.d ${OBJECTDIR}/mcc_generated_files/device_config.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/device_config.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/tmr1.p1: mcc_generated_files/tmr1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/tmr1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/tmr1.p1 mcc_generated_files/tmr1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/tmr1.d ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/tmr1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/eusart.p1: mcc_generated_files/eusart.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/eusart.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/eusart.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/eusart.p1 mcc_generated_files/eusart.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/eusart.d ${OBJECTDIR}/mcc_generated_files/eusart.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/eusart.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/cwg1.p1: mcc_generated_files/cwg1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/cwg1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/cwg1.p1 mcc_generated_files/cwg1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/cwg1.d ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/cwg1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/pin_manager.p1: mcc_generated_files/pin_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/pin_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/pin_manager.p1 mcc_generated_files/pin_manager.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/pin_manager.d ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/pin_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/nco1.p1: mcc_generated_files/nco1.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco1.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/nco1.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/nco1.p1 mcc_generated_files/nco1.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/nco1.d ${OBJECTDIR}/mcc_generated_files/nco1.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/nco1.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/fvr.p1: mcc_generated_files/fvr.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/fvr.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/fvr.p1 mcc_generated_files/fvr.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/fvr.d ${OBJECTDIR}/mcc_generated_files/fvr.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/fvr.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/mcc.p1: mcc_generated_files/mcc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/mcc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/mcc.p1 mcc_generated_files/mcc.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/mcc.d ${OBJECTDIR}/mcc_generated_files/mcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/mcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/adcc.p1: mcc_generated_files/adcc.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adcc.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/adcc.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/adcc.p1 mcc_generated_files/adcc.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/adcc.d ${OBJECTDIR}/mcc_generated_files/adcc.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/adcc.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1: mcc_generated_files/interrupt_manager.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1 mcc_generated_files/interrupt_manager.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.d ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/interrupt_manager.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/memory.p1: mcc_generated_files/memory.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/memory.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/memory.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/memory.p1 mcc_generated_files/memory.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/memory.d ${OBJECTDIR}/mcc_generated_files/memory.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/memory.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/mcc_generated_files/clkref.p1: mcc_generated_files/clkref.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}/mcc_generated_files" 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref.p1.d 
	@${RM} ${OBJECTDIR}/mcc_generated_files/clkref.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/mcc_generated_files/clkref.p1 mcc_generated_files/clkref.c 
	@-${MV} ${OBJECTDIR}/mcc_generated_files/clkref.d ${OBJECTDIR}/mcc_generated_files/clkref.p1.d 
	@${FIXDEPS} ${OBJECTDIR}/mcc_generated_files/clkref.p1.d $(SILENT) -rsi ${MP_CC_DIR}../  
	
${OBJECTDIR}/main.p1: main.c  nbproject/Makefile-${CND_CONF}.mk 
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.p1.d 
	@${RM} ${OBJECTDIR}/main.p1 
	${MP_CC} $(MP_EXTRA_CC_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -c   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -DXPRJ_default=$(CND_CONF)  -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits $(COMPARISON_BUILD)  -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     -o ${OBJECTDIR}/main.p1 main.c 
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
${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.map  -D__DEBUG=1  -mdebugger=pickit4  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits -std=c99 -gdwarf-3 -mstack=compiled:auto:auto        $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	@${RM} ${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.hex 
	
else
${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} ${DISTDIR} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -mcpu=$(MP_PROCESSOR_OPTION) -Wl,-Map=${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.map  -DXPRJ_default=$(CND_CONF)  -Wl,--defsym=__MPLAB_BUILD=1   -mdfp="${DFP_DIR}/xc8"  -fno-short-double -fno-short-float -O0 -fasmfile -maddrqual=ignore -DBREATH_ON_TIME=${BREATH_ON_TIME} -DSTOP_PCT=${STOP_PCT} -DRES_FREQ=${RES_FREQ} -DPROG_TYPE=${PROG_TYPE} -DON_TIME=${ON_TIME} -DOFF_TIME=${OFF_TIME} -DLP_TIME=${LP_TIME} -DAMOUNT_OF_SHOTS=${AMOUNT_OF_SHOTS} -DIDLE_TIMEOUT=${IDLE_TIMEOUT} -DPRESSURE_THRESHOLD=${PRESSURE_THRESHOLD} -DSPRAY_DELAY=${SPRAY_DELAY} -DBRIDGE_VT=${BRIDGE_VT} -DVOLTAGE_REG_EN=${VOLTAGE_REG_EN} -DCURRENT_PROTECTION_EN=${CURRENT_PROTECTION_EN} -DCURRENT_TARGET=${CURRENT_TARGET} -xassembler-with-cpp -mwarn=0 -Wa,-a -msummary=-psect,-class,+mem,-hex,-file  -ginhx32 -Wl,--data-init -mno-keep-startup -mno-osccal -mno-resetbits -mno-save-resetbits -mno-download -mno-stackcall -mdefault-config-bits -std=c99 -gdwarf-3 -mstack=compiled:auto:auto     $(COMPARISON_BUILD) -Wl,--memorysummary,${DISTDIR}/memoryfile.xml -o ${DISTDIR}/BlueskyPush.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}     
	
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
