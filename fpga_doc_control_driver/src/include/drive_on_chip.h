// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#define DEBUG_ADDR_SPACE_PER_AXIS           66      // Size (32-bit words) of debug memory for each axis

// General Drive Status (Write only)
#define DOC_DBG_DRIVE_STATE                 0
#define DOC_DBG_RUNTIME                     1
#define DOC_DBG_DSP_MODE                    2
#define DOC_DBG_APP_STATE                   3
#define DOC_DBG_LATENCY1                    4
#define DOC_DBG_DUMP_MODE                   5
#define DOC_DBG_TRIG_SEL                    6
#define DOC_DBG_TRIG_EDGE                   7
#define DOC_DBG_TRIG_VALUE                  8

// Drive Performance Status (Write only)
#define DOC_DBG_SPEED                       10      // Speed feedback (read)
#define DOC_DBG_POSITION                    12      // Position feedback (read)
#define DOC_DBG_BUTTON_DSP_MODE             13      // Mode  Software/DSP BA hardware
#define DOC_DBG_BUTTON_DRIVE_RESET          14
#define DOC_DBG_DEMO_MODE                   15      // Demo mode

#define DOC_DBG_I_PI_KP                     18      // Current control Kp
#define DOC_DBG_I_PI_KI                     19      // Current control Ki
#define DOC_DBG_SPEED_PI_KP                 20      // Speed control Kp
#define DOC_DBG_SPEED_PI_KI                 21      // Speed control Ki
#define DOC_DBG_SPEED_SETP0                 22      // Speed set point or command
#define DOC_DBG_AXIS_SELECT                 23      // Select to write on different axis

#define DOC_DBG_POS_SETP0                   25      // Position set point or command

#define DOC_DBG_WAVE_DEMO_MODE              29      // Speed/position waveform demo mode
#define DOC_DBG_POS_SPEED_LIMIT             30      // Speed limit for position mode demos
#define DOC_DBG_POS_PI_KP                   31      // Speed control Kp

#define DOC_DBG_WAVE_DEMO_PERIOD            33      // Speed/position demo waveform period
#define DOC_DBG_WAVE_DEMO_OFFSET            34      // Speed/position demo waveform time offset
#define DOC_DBG_WAVE_DEMO_WAVEFORM          35      // Speed/position demo waveform shape
#define DOC_DBG_WAVE_DEMO_AMP_F             36      // Speed/position demo waveform amplitude

#define DOC_DBG_DCDC_V_SETP0                44
#define DOC_DBG_DC_DC_V_LINK                54
#define DOC_DBG_ADC_TYPE                    55
#define DOC_DBG_LATENCY2                    56
#define DOC_DBG_LATENCY3                    57
#define DOC_DBG_TRACE_DEPTH                 58      // Waveform trace depth
#define DOC_DBG_TRIGGER_POS                 59      // Waveform trigger position
#define DOC_DBG_TRACE_SAMPLES               60      // Waveform trace sample skip
#define DOC_DBG_DEMO_UPDATE				    61      // Waveform demo update flag
