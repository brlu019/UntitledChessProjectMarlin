/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"
#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/endstops.h"
#include "../../module/settings.h"

#if ENABLED(SENSORLESS_HOMING)
  #include "../../feature/tmc_util.h"
  #include "../../module/stepper/trinamic.h"
#endif

/**
 * G8: Round-Robin Incremental Cable Tensioning
 * 
 * Tensions cables by finding the longest one and tensioning it by one increment,
 * then repeating until all cables are tensioned. This ensures even tensioning.
 * 
 * Parameters:
 *   S<steps>  - Steps per increment (default: 20)
 *   F<rate>   - Feedrate for tensioning moves (default: 60 mm/min)
 *   M<mm>     - Maximum delta allowed per cable (default: 20mm)
 *   R         - Reset cable lengths to current position
 * 
 * Example: G8 S10 F30 M15
 */
void GcodeSuite::G8() {
  
  SERIAL_ECHOLNPGM("G8: Round-Robin Cable Tensioning");
  
  // Parse parameters
  const uint8_t steps_per_increment = parser.byteval('S', 20);
  const feedRate_t tensioning_feedrate = parser.linearval('F', 60.0f);
  const float max_delta_mm = parser.floatval('M', 20.0f);
  const bool reset_lengths = parser.boolval('R');
  
  // Safety check
  if (steps_per_increment < 1 || steps_per_increment > 50) {
    SERIAL_ERROR_MSG("G8: Steps per increment must be 1-50");
    return;
  }
  
  if (max_delta_mm < 5.0f || max_delta_mm > 200.0f) {
    SERIAL_ERROR_MSG("G8: Max delta must be 5-200mm");
    return;
  }

  // Reset cable lengths if requested
  if (reset_lengths) {
    SERIAL_ECHOLNPGM("G8: Resetting stored cable lengths to zero");
    for (uint8_t i = 0; i < 4; i++) cable_lengths[i] = 0.0f;
    MarlinSettings::save();  // Save to EEPROM
    SERIAL_ECHOLNPGM("Cable lengths reset and saved to EEPROM");
    SERIAL_ECHOLNPGM("Current position: X:", current_position.x, " Y:", current_position.y, " Z:", current_position.z, " I:", current_position.i);
    return;
  }
  
  SERIAL_ECHOLNPGM("G8: Starting round-robin incremental cable tensioning");
  SERIAL_ECHOLNPGM("Steps per increment: ", steps_per_increment);
  SERIAL_ECHOLNPGM("Tensioning feedrate: ", tensioning_feedrate);
  SERIAL_ECHOLNPGM("Max delta limit: ", max_delta_mm, "mm");
  
  // Setup for incremental round-robin tensioning
  const AxisEnum axes_to_tension[] = {X_AXIS, Y_AXIS, Z_AXIS, I_AXIS};
  const char axis_chars[] = {'X', 'Y', 'Z', 'I'};
  const uint8_t num_axes = 4;
  
  SERIAL_ECHOLNPGM("Starting round-robin incremental tensioning on XYZI axes...");
  
  // Use optimal parameters for stallguard detection:
  // - Slower speed gives stallguard time to detect resistance  
  // - Larger moves build sustained motor load
  const float step_size_mm = 5.0f;  // Larger steps for sustained load
  const feedRate_t slow_feedrate = 15.0f;  // Much slower for better stallguard detection
  const feedRate_t move_feedrate = MMM_TO_MMS(slow_feedrate);
  
  // Load stored cable lengths from EEPROM as starting point
  SERIAL_ECHOLNPGM("Loading stored cable lengths from EEPROM:");
  for (uint8_t i = 0; i < 4; i++) {
    SERIAL_ECHOPGM("  ", axis_chars[i], ": ");
    SERIAL_ECHO(cable_lengths[i]);
    SERIAL_ECHOLNPGM("mm (stored)");
  }
  
  // Track state for each axis
  bool axis_tensioned[4] = {false, false, false, false};  // Whether each axis is done
  uint8_t total_tensioned = 0;  // How many axes are finished
  
  // Set sensitivity to 21 (like G28) for YZI, keep X as is
  #if ENABLED(SENSORLESS_HOMING)
    #if Y_SENSORLESS
      stepperY.homing_threshold(21);
      SERIAL_ECHOLNPGM("Set Y stallguard sensitivity to 21 (like G28)");
    #endif
    #if Z_SENSORLESS
      stepperZ.homing_threshold(21);
      SERIAL_ECHOLNPGM("Set Z stallguard sensitivity to 21 (like G28)");
    #endif
    #if I_SENSORLESS
      stepperI.homing_threshold(21);
      SERIAL_ECHOLNPGM("Set I stallguard sensitivity to 21 (like G28)");
    #endif
  #endif
  
  SERIAL_ECHOLNPGM("Step size: ", step_size_mm, "mm per increment");
  SERIAL_ECHOLNPGM("Stallguard feedrate: ", slow_feedrate, "mm/min");
  SERIAL_ECHOLNPGM("Max movement limit: ", max_delta_mm, "mm per axis");
  SERIAL_ECHOLNPGM("");
  
  // Main tensioning loop - round robin until all axes are tensioned
  uint16_t global_iteration = 0;
  const uint16_t max_global_iterations = 200; // Safety limit for entire process
  
  while (total_tensioned < num_axes && global_iteration < max_global_iterations) {
    global_iteration++;
    
    SERIAL_ECHOLNPGM("=== Global Iteration ", global_iteration, " ===");
    SERIAL_ECHOLNPGM("Tensioned axes: ", total_tensioned, "/", num_axes);
    
    // Show current cable lengths
    SERIAL_ECHOPGM("Cable lengths: ");
    for (uint8_t i = 0; i < num_axes; i++) {
      SERIAL_ECHOPGM(axis_chars[i], ":");
      SERIAL_ECHO(cable_lengths[i]);
      SERIAL_ECHOPGM("mm ");
    }
    SERIAL_EOL();
    
    // Find the axis with the longest cable that isn't tensioned yet
    float max_length = -1.0f;
    uint8_t longest_axis_idx = 255; // Invalid index
    
    for (uint8_t i = 0; i < num_axes; i++) {
      if (!axis_tensioned[i] && cable_lengths[i] > max_length) {
        max_length = cable_lengths[i];
        longest_axis_idx = i;
      }
    }
    
    // Safety check
    if (longest_axis_idx >= num_axes) {
      SERIAL_ECHOLNPGM("ERROR: Could not find axis to tension!");
      break;
    }
    
    const AxisEnum current_axis = axes_to_tension[longest_axis_idx];
    const char axis_char = axis_chars[longest_axis_idx];
    
    SERIAL_ECHOLNPGM("");
    SERIAL_ECHOPGM("Tensioning longest cable: ", axis_char, "-axis (");
    SERIAL_ECHO(cable_lengths[longest_axis_idx]);
    SERIAL_ECHOLNPGM("mm)");
    
    // Check if this axis has reached movement limit
    if (cable_lengths[longest_axis_idx] >= max_delta_mm) {
      SERIAL_ECHOLNPGM("LIMIT REACHED: ", axis_char, "-axis at max movement (", max_delta_mm, "mm)");
      axis_tensioned[longest_axis_idx] = true;
      total_tensioned++;
      continue;
    }
    
    // Get home direction for this axis
    const int home_dir_value = home_dir(current_axis);
    const float move_distance = step_size_mm * home_dir_value;
    
    SERIAL_ECHOLNPGM("Moving ", axis_char, " by ", move_distance, "mm (home dir: ", home_dir_value, ")");
    
    // Clear endstop state for polled stallguard endstops
    TERN_(SPI_ENDSTOPS, endstops.clear_endstop_state());
    
    // Enable endstops for stallguard detection
    endstops.enable(true);
    
    // Enable stallguard for this axis
    #if ENABLED(SENSORLESS_HOMING)
      sensorless_t stealth_states = start_sensorless_homing_per_axis(current_axis);
      #if SENSORLESS_STALLGUARD_DELAY
        safe_delay(SENSORLESS_STALLGUARD_DELAY);
      #endif
    #endif
    
    // Perform the tensioning move
    abce_pos_t target = planner.get_axis_positions_mm();
    target[current_axis] = 0; // Zero this axis for relative move
    planner.set_machine_position_mm(target);
    
    target[current_axis] = move_distance;
    
    #if HAS_DIST_MM_ARG
      const xyze_float_t cart_dist_mm{0};
    #endif
    
    planner.buffer_segment(target OPTARG(HAS_DIST_MM_ARG, cart_dist_mm), move_feedrate, active_extruder);
    planner.synchronize();
    
    // Check for stallguard detection
    bool stall_detected = endstops.trigger_state();
    
    SERIAL_ECHOLNPGM("  Endstop trigger state: ", stall_detected);
    
    if (stall_detected) {
      // Mark endstop as hit on purpose
      endstops.hit_on_purpose();
      SERIAL_ECHOLNPGM("  *** STALLGUARD DETECTED! ", axis_char, "-axis tensioned! ***");
      axis_tensioned[longest_axis_idx] = true;
      total_tensioned++;
    } else {
      // Update cable length and continue
      cable_lengths[longest_axis_idx] += step_size_mm;
      SERIAL_ECHOPGM("  No stall - ", axis_char, " cable now ");
      SERIAL_ECHO(cable_lengths[longest_axis_idx]);
      SERIAL_ECHOLNPGM("mm");
    }
    
    // Disable stallguard for this axis
    #if ENABLED(SENSORLESS_HOMING)
      end_sensorless_homing_per_axis(current_axis, stealth_states);
      #if SENSORLESS_STALLGUARD_DELAY
        safe_delay(SENSORLESS_STALLGUARD_DELAY);
      #endif
    #endif
    
    // Disable endstops
    endstops.not_homing();
    
    // Small delay between moves
    safe_delay(200);
  }
  
  // Clear endstop state
  TERN_(SPI_ENDSTOPS, endstops.clear_endstop_state());
  
  // Report final results
  SERIAL_ECHOLNPGM("");
  SERIAL_ECHOLNPGM("=== G8 Round-Robin Tensioning Complete ===");
  SERIAL_ECHOLNPGM("Global iterations: ", global_iteration);
  SERIAL_ECHOLNPGM("Tensioned axes: ", total_tensioned, "/", num_axes);
  
  SERIAL_ECHOLNPGM("Final cable lengths:");
  for (uint8_t i = 0; i < num_axes; i++) {
    SERIAL_ECHOPGM("  ", axis_chars[i], ": ");
    SERIAL_ECHO(cable_lengths[i]);
    SERIAL_ECHOPGM("mm");
    if (axis_tensioned[i]) {
      SERIAL_ECHOPGM(" (TENSIONED)");
    } else {
      SERIAL_ECHOPGM(" (FAILED)");
    }
    SERIAL_EOL();
  }
  
  if (total_tensioned == num_axes) {
    SERIAL_ECHOLNPGM("SUCCESS: All XYZI axes tensioned!");
  } else {
    SERIAL_ECHOLNPGM("Some axes failed to tension - check sensitivity settings or cable resistance");
  }
  
  // Save updated cable lengths to EEPROM
  SERIAL_ECHOLNPGM("");
  SERIAL_ECHOLNPGM("Saving cable lengths to EEPROM...");
  if (MarlinSettings::save()) {
    SERIAL_ECHOLNPGM("Cable lengths saved successfully");
  } else {
    SERIAL_ECHOLNPGM("WARNING: Failed to save cable lengths to EEPROM");
  }
}