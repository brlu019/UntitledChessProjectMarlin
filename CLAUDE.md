# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

### Primary Build System - PlatformIO
- `platformio run`: Build firmware for default environment (STM32G0B1RE_btt)
- `platformio run -e <environment>`: Build for specific environment
- `platformio run -t upload`: Build and upload firmware
- `platformio run -t monitor`: Start serial monitor

### Alternative Build System - Make
- `make marlin`: Build Marlin for the configured board using ./buildroot/bin/mftest -a
- `make help`: Show all available make targets

### Testing Commands
- `make tests-single-local TEST_TARGET=<target>`: Run single test locally
- `make tests-all-local`: Run all tests locally  
- `make unit-test-all-local`: Run all unit tests using `platformio run -t test-marlin -e linux_native_test`
- `make unit-test-single-local UNIT_TEST_CONFIG=<config>`: Run single unit test

### Code Quality
- `make format-pins`: Reformat all pins files
- `make validate-pins`: Validate pins files formatting
- `make validate-boards`: Validate boards.h compliance

## Architecture Overview

### Core Structure
This is a Marlin 3D printer firmware fork configured for an untitled chess project robot. The codebase is organized around these key components:

- **HAL (Hardware Abstraction Layer)**: Platform-specific code in `Marlin/src/HAL/` supporting multiple architectures
- **Core Motion Control**: `Marlin/src/module/` contains motion planning, stepper control, and kinematics
- **G-code Processing**: `Marlin/src/gcode/` organized by command categories (calibrate, config, control, motion, etc.)
- **Features**: `Marlin/src/feature/` contains modular features like bed leveling, thermal control, etc.

### Configuration System
- **Primary Config**: `Marlin/Configuration.h` - basic hardware and feature settings
- **Advanced Config**: `Marlin/Configuration_adv.h` - advanced feature configuration
- **Current Target**: STM32G0B1RE_btt (BTT SKR Mini E3 V3.0 board)

### Key Modules
- **Motion Control**: `motion.cpp/h` - coordinate transformations and movement execution
- **Planner**: `planner.cpp/h` - trajectory planning and acceleration management  
- **Stepper**: `stepper.cpp/h` - low-level stepper motor control
- **Temperature**: `temperature.cpp/h` - thermal management and PID control
- **Endstops**: `endstops.cpp/h` - limit switches and homing logic

### Custom Modifications
- **XYZI Axis System**: Modified from standard XYZ to support I as fourth axis (see git history)
- **Custom G8 Code**: Round-robin incremental cable tensioning with EEPROM persistence in `Marlin/src/gcode/calibrate/G8.cpp`
- **EEPROM Cable Storage**: Persistent cable length storage across power cycles in `settings.cpp`
- **Optimized Motor Currents**: Specialized current settings for cable-driven robot tensioning
- **Current Branch**: `skr-mini-e3-robot` - specialized for robot control rather than 3D printing

## G8 Cable Tensioning System

### Overview
**G8** is a custom G-code command that implements round-robin incremental cable tensioning for cable-driven robot systems. It uses stallguard detection to automatically tension cables and stores progress in EEPROM for persistence across power cycles.

### Key Features
- **Round-Robin Algorithm**: Always tensions the longest cable first for even tensioning
- **EEPROM Persistence**: Cable lengths survive power cycles and resume from last state
- **Stallguard Detection**: Uses TMC stepper driver stallguard for automatic tension detection
- **Safety Limits**: Maximum movement and iteration limits prevent damage
- **Smart Recovery**: Loads stored cable lengths and continues from previous state

### Usage Examples
```gcode
G8           # Tension cables using stored lengths as starting point
G8 R         # Reset stored cable lengths to zero and save to EEPROM  
G8 S5 M30    # Custom step size (5mm) and max movement (30mm per axis)
G8 F15       # Custom tensioning feedrate (15mm/min)
```

### Parameters
- `S<steps>`: Steps per increment (default: 20, range: 1-50)
- `F<rate>`: Feedrate for tensioning moves (default: 60 mm/min)  
- `M<mm>`: Maximum delta allowed per cable (default: 20mm, range: 5-200mm)
- `R`: Reset cable lengths to zero and save to EEPROM

### Algorithm Details
1. **Load stored cable lengths** from EEPROM (X:0mm Y:5mm Z:10mm I:0mm)
2. **Find longest cable** that isn't tensioned yet
3. **Tension by one increment** (5mm step) in home direction
4. **Check for stallguard** detection during movement
5. **Update cable length** or mark as tensioned
6. **Repeat** until all axes tensioned or limits reached
7. **Save final state** to EEPROM for next session

### EEPROM Integration
- **Storage Location**: `settings.cpp` line 254 in `SettingsDataStruct`
- **Global Variable**: `cable_lengths[4]` array for XYZI axes
- **Save Function**: Automatic save after successful tensioning
- **Load Function**: Automatic load on firmware startup
- **Reset Function**: `G8 R` clears stored lengths

## Motor Current Optimization for Cable Tensioning

### Current Configuration Strategy
The motor current settings have been optimized specifically for cable-driven robot tensioning to prevent step loss during G8 operations.

### Optimal Current Settings (XYZI Axes)
```
Running Current:        800mA  # Normal operation power
Homing Current:         500mA  # Lower for better stall detection  
Homing Hold Current:    800mA  # HIGH to resist pulling during tensioning
Regular Hold Current:   560mA  # 70% of run (HOLD_MULTIPLIER 0.7)
Individual Hold:        640mA  # 80% per-axis override (X/Y/Z/I_HOLD_MULTIPLIER 0.8)
```

### Current Strategy Rationale
1. **Lower Homing Current (500mA)**: Easier stallguard detection, won't overpower holding motors
2. **High Homing Hold Current (800mA)**: Prevents step loss when other motors pull
3. **Dual Holding System**: 
   - Regular holding: 560mA (energy efficient during idle)
   - Tensioning holding: 640mA (high torque during G8 operations)

### Configuration Locations
- **Global Settings**: `Configuration_adv.h` line 3038 (`HOLD_MULTIPLIER 0.7`)
- **XYZI Current Settings**: `Configuration_adv.h` lines 3063-3150
- **Individual Hold Multipliers**: Each axis has `X_HOLD_MULTIPLIER 0.8` etc.

### EEPROM Settings
- **Enabled**: `Configuration.h` line 2465 (`#define EEPROM_SETTINGS`)
- **Required for G8**: EEPROM must be enabled for cable length persistence
- **Commands**: `M500` (save), `M501` (load), `M502` (reset to defaults)

### Hardware Configuration  
- **Motherboard**: BOARD_BTT_SKR_MINI_E3_V3_0
- **Axes**: X, Y, Z, I (fourth axis instead of standard extruder)
- **Serial**: 250000 baud, USB serial port (-1)
- **Stepper Drivers**: TMC2209 with stallguard sensorless homing
- **Stallguard Sensitivity**: YZI=21 (like G28), X=custom tuned

## Troubleshooting & Development Notes

### Common Issues and Solutions

#### G8 EEPROM Issues
- **Error**: "Warning:EEPROM disabled" → **Solution**: Enable `EEPROM_SETTINGS` in `Configuration.h`
- **Error**: "DEFAULT_NOMINAL_FILAMENT_DIA not declared" → **Solution**: Uncomment line in `Configuration.h`

#### Motor Step Loss During Tensioning
- **Problem**: Holding motors lose steps when tensioning motor pulls
- **Solution**: Implemented dual holding current system with higher torque during tensioning
- **Verification**: Check that `X/Y/Z/I_HOLD_MULTIPLIER` values are enabled and set to 0.8

#### Stallguard Detection Issues  
- **Problem**: G8 doesn't detect cable tension properly
- **Solutions**:
  - Verify stallguard sensitivity settings (YZI=21 recommended)
  - Check that `X/Y/Z/I_SENSORLESS` are enabled
  - Ensure proper home direction configuration
  - Use slower feedrates for better detection (15mm/min recommended)

### Recent Development Progress (2025-08-29)

#### Completed Features
1. **G8 Round-Robin Cable Tensioning System**
   - Implemented complete round-robin algorithm
   - Added EEPROM persistence for cable lengths
   - Integrated stallguard detection for all XYZI axes
   - Added comprehensive error checking and safety limits

2. **EEPROM Integration**
   - Added `cable_lengths[4]` to EEPROM structure (`settings.cpp:254`)
   - Implemented save/load/reset functionality
   - Added proper initialization and default values
   - Enabled EEPROM settings in configuration

3. **Motor Current Optimization**
   - Optimized current settings for cable tensioning
   - Implemented dual holding current system
   - Reduced homing current for better stall detection
   - Added individual axis hold multiplier overrides

#### Build Status
- **Last Successful Build**: 2025-08-29 
- **RAM Usage**: 8.7% (12,864 / 147,456 bytes)
- **Flash Usage**: 26.1% (136,596 / 524,288 bytes)
- **Target Board**: STM32G0B1RE_btt (BTT SKR Mini E3 V3.0)

#### Next Development Steps
1. **Testing & Validation**
   - Test G8 with optimized current settings
   - Validate EEPROM persistence across power cycles
   - Fine-tune stallguard sensitivity if needed
   
2. **Potential Enhancements**
   - Add cable tension monitoring/reporting
   - Implement automatic re-tensioning on power-up
   - Add G8 status/diagnostic commands

### Build Tools and Scripts
- **mftest**: Primary build script in `buildroot/bin/mftest`
- **Configuration Scripts**: Various tools in `buildroot/bin/` for config management (opt_enable, opt_disable, opt_set, etc.)
- **Testing Framework**: Comprehensive test suite with both build tests and unit tests