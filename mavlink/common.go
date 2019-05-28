package mavlink

import (
	"encoding/binary"
	"encoding/json"
	"fmt"
	"math"
)

//////////////////////////////////////////////////
//
// NOTE: do not edit,
// this file created automatically by mavgen.go
//
//////////////////////////////////////////////////

// MavAutopilot: Micro air vehicle / autopilot classes. This identifies the individual model.
const (
	MAV_AUTOPILOT_GENERIC                                      = 0  // Generic autopilot, full support for everything
	MAV_AUTOPILOT_RESERVED                                     = 1  // Reserved for future use.
	MAV_AUTOPILOT_SLUGS                                        = 2  // SLUGS autopilot, http://slugsuav.soe.ucsc.edu
	MAV_AUTOPILOT_ARDUPILOTMEGA                                = 3  // ArduPilot - Plane/Copter/Rover/Sub/Tracker, http://ardupilot.org
	MAV_AUTOPILOT_OPENPILOT                                    = 4  // OpenPilot, http://openpilot.org
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       = 5  // Generic autopilot only supporting simple waypoints
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6  // Generic autopilot supporting waypoints and other simple navigation commands
	MAV_AUTOPILOT_GENERIC_MISSION_FULL                         = 7  // Generic autopilot supporting the full mission command set
	MAV_AUTOPILOT_INVALID                                      = 8  // No valid autopilot, e.g. a GCS or other MAVLink component
	MAV_AUTOPILOT_PPZ                                          = 9  // PPZ UAV - http://nongnu.org/paparazzi
	MAV_AUTOPILOT_UDB                                          = 10 // UAV Dev Board
	MAV_AUTOPILOT_FP                                           = 11 // FlexiPilot
	MAV_AUTOPILOT_PX4                                          = 12 // PX4 Autopilot - http://px4.io/
	MAV_AUTOPILOT_SMACCMPILOT                                  = 13 // SMACCMPilot - http://smaccmpilot.org
	MAV_AUTOPILOT_AUTOQUAD                                     = 14 // AutoQuad -- http://autoquad.org
	MAV_AUTOPILOT_ARMAZILA                                     = 15 // Armazila -- http://armazila.com
	MAV_AUTOPILOT_AEROB                                        = 16 // Aerob -- http://aerob.ru
	MAV_AUTOPILOT_ASLUAV                                       = 17 // ASLUAV autopilot -- http://www.asl.ethz.ch
	MAV_AUTOPILOT_SMARTAP                                      = 18 // SmartAP Autopilot - http://sky-drones.com
	MAV_AUTOPILOT_AIRRAILS                                     = 19 // AirRails - http://uaventure.com
)

// MavType: MAVLINK system type. All components in a system should report this type in their HEARTBEAT.
const (
	MAV_TYPE_GENERIC            = 0  // Generic micro air vehicle.
	MAV_TYPE_FIXED_WING         = 1  // Fixed wing aircraft.
	MAV_TYPE_QUADROTOR          = 2  // Quadrotor
	MAV_TYPE_COAXIAL            = 3  // Coaxial helicopter
	MAV_TYPE_HELICOPTER         = 4  // Normal helicopter with tail rotor.
	MAV_TYPE_ANTENNA_TRACKER    = 5  // Ground installation
	MAV_TYPE_GCS                = 6  // Operator control unit / ground control station
	MAV_TYPE_AIRSHIP            = 7  // Airship, controlled
	MAV_TYPE_FREE_BALLOON       = 8  // Free balloon, uncontrolled
	MAV_TYPE_ROCKET             = 9  // Rocket
	MAV_TYPE_GROUND_ROVER       = 10 // Ground rover
	MAV_TYPE_SURFACE_BOAT       = 11 // Surface vessel, boat, ship
	MAV_TYPE_SUBMARINE          = 12 // Submarine
	MAV_TYPE_HEXAROTOR          = 13 // Hexarotor
	MAV_TYPE_OCTOROTOR          = 14 // Octorotor
	MAV_TYPE_TRICOPTER          = 15 // Tricopter
	MAV_TYPE_FLAPPING_WING      = 16 // Flapping wing
	MAV_TYPE_KITE               = 17 // Kite
	MAV_TYPE_ONBOARD_CONTROLLER = 18 // Onboard companion controller
	MAV_TYPE_VTOL_DUOROTOR      = 19 // Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
	MAV_TYPE_VTOL_QUADROTOR     = 20 // Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
	MAV_TYPE_VTOL_TILTROTOR     = 21 // Tiltrotor VTOL
	MAV_TYPE_VTOL_RESERVED2     = 22 // VTOL reserved 2
	MAV_TYPE_VTOL_RESERVED3     = 23 // VTOL reserved 3
	MAV_TYPE_VTOL_RESERVED4     = 24 // VTOL reserved 4
	MAV_TYPE_VTOL_RESERVED5     = 25 // VTOL reserved 5
	MAV_TYPE_GIMBAL             = 26 // Gimbal (standalone)
	MAV_TYPE_ADSB               = 27 // ADSB system (standalone)
	MAV_TYPE_PARAFOIL           = 28 // Steerable, nonrigid airfoil
	MAV_TYPE_DODECAROTOR        = 29 // Dodecarotor
	MAV_TYPE_CAMERA             = 30 // Camera (standalone)
	MAV_TYPE_CHARGING_STATION   = 31 // Charging station
	MAV_TYPE_FLARM              = 32 // FLARM collision avoidance system (standalone)
)

// FirmwareVersionType: These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.
const (
	FIRMWARE_VERSION_TYPE_DEV      = 0   // development release
	FIRMWARE_VERSION_TYPE_ALPHA    = 64  // alpha release
	FIRMWARE_VERSION_TYPE_BETA     = 128 // beta release
	FIRMWARE_VERSION_TYPE_RC       = 192 // release candidate
	FIRMWARE_VERSION_TYPE_OFFICIAL = 255 // official stable release
)

// HlFailureFlag: Flags to report failure cases over the high latency telemtry.
const (
	HL_FAILURE_FLAG_GPS                   = 1    // GPS failure.
	HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE = 2    // Differential pressure sensor failure.
	HL_FAILURE_FLAG_ABSOLUTE_PRESSURE     = 4    // Absolute pressure sensor failure.
	HL_FAILURE_FLAG_3D_ACCEL              = 8    // Accelerometer sensor failure.
	HL_FAILURE_FLAG_3D_GYRO               = 16   // Gyroscope sensor failure.
	HL_FAILURE_FLAG_3D_MAG                = 32   // Magnetometer sensor failure.
	HL_FAILURE_FLAG_TERRAIN               = 64   // Terrain subsystem failure.
	HL_FAILURE_FLAG_BATTERY               = 128  // Battery failure/critical low battery.
	HL_FAILURE_FLAG_RC_RECEIVER           = 256  // RC receiver failure/no rc connection.
	HL_FAILURE_FLAG_OFFBOARD_LINK         = 512  // Offboard link failure.
	HL_FAILURE_FLAG_ENGINE                = 1024 // Engine failure.
	HL_FAILURE_FLAG_GEOFENCE              = 2048 // Geofence violation.
	HL_FAILURE_FLAG_ESTIMATOR             = 4096 // Estimator failure, for example measurement rejection or large variances.
	HL_FAILURE_FLAG_MISSION               = 8192 // Mission failure.
)

// MavModeFlag: These flags encode the MAV mode.
const (
	MAV_MODE_FLAG_SAFETY_ARMED         = 128 // 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
	MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64  // 0b01000000 remote control input is enabled.
	MAV_MODE_FLAG_HIL_ENABLED          = 32  // 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
	MAV_MODE_FLAG_STABILIZE_ENABLED    = 16  // 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
	MAV_MODE_FLAG_GUIDED_ENABLED       = 8   // 0b00001000 guided mode enabled, system flies waypoints / mission items.
	MAV_MODE_FLAG_AUTO_ENABLED         = 4   // 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
	MAV_MODE_FLAG_TEST_ENABLED         = 2   // 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
	MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  = 1   // 0b00000001 Reserved for future use.
)

// MavModeFlagDecodePosition: These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
const (
	MAV_MODE_FLAG_DECODE_POSITION_SAFETY      = 128 // First bit:  10000000
	MAV_MODE_FLAG_DECODE_POSITION_MANUAL      = 64  // Second bit: 01000000
	MAV_MODE_FLAG_DECODE_POSITION_HIL         = 32  // Third bit:  00100000
	MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   = 16  // Fourth bit: 00010000
	MAV_MODE_FLAG_DECODE_POSITION_GUIDED      = 8   // Fifth bit:  00001000
	MAV_MODE_FLAG_DECODE_POSITION_AUTO        = 4   // Sixt bit:   00000100
	MAV_MODE_FLAG_DECODE_POSITION_TEST        = 2   // Seventh bit: 00000010
	MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1   // Eighth bit: 00000001
)

// MavGoto: Override command, pauses current mission execution and moves immediately to a position
const (
	MAV_GOTO_DO_HOLD                    = 0 // Hold at the current position.
	MAV_GOTO_DO_CONTINUE                = 1 // Continue with the next item in mission execution.
	MAV_GOTO_HOLD_AT_CURRENT_POSITION   = 2 // Hold at the current position of the system
	MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3 // Hold at the position specified in the parameters of the DO_HOLD action
)

// MavMode: These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
const (
	MAV_MODE_PREFLIGHT          = 0   // System is not ready to fly, booting, calibrating, etc. No flag is set.
	MAV_MODE_STABILIZE_DISARMED = 80  // System is allowed to be active, under assisted RC control.
	MAV_MODE_STABILIZE_ARMED    = 208 // System is allowed to be active, under assisted RC control.
	MAV_MODE_MANUAL_DISARMED    = 64  // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_MANUAL_ARMED       = 192 // System is allowed to be active, under manual (RC) control, no stabilization
	MAV_MODE_GUIDED_DISARMED    = 88  // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_GUIDED_ARMED       = 216 // System is allowed to be active, under autonomous control, manual setpoint
	MAV_MODE_AUTO_DISARMED      = 92  // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
	MAV_MODE_AUTO_ARMED         = 220 // System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
	MAV_MODE_TEST_DISARMED      = 66  // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
	MAV_MODE_TEST_ARMED         = 194 // UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
)

// MavState:
const (
	MAV_STATE_UNINIT             = 0 // Uninitialized system, state is unknown.
	MAV_STATE_BOOT               = 1 // System is booting up.
	MAV_STATE_CALIBRATING        = 2 // System is calibrating and not flight-ready.
	MAV_STATE_STANDBY            = 3 // System is grounded and on standby. It can be launched any time.
	MAV_STATE_ACTIVE             = 4 // System is active and might be already airborne. Motors are engaged.
	MAV_STATE_CRITICAL           = 5 // System is in a non-normal flight mode. It can however still navigate.
	MAV_STATE_EMERGENCY          = 6 // System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
	MAV_STATE_POWEROFF           = 7 // System just initialized its power-down sequence, will shut down now.
	MAV_STATE_FLIGHT_TERMINATION = 8 // System is terminating itself.
)

// MavComponent: Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).       Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.       When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded.
const (
	MAV_COMP_ID_ALL                      = 0   // Used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces.
	MAV_COMP_ID_AUTOPILOT1               = 1   // System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.
	MAV_COMP_ID_CAMERA                   = 100 // Camera #1.
	MAV_COMP_ID_CAMERA2                  = 101 // Camera #2.
	MAV_COMP_ID_CAMERA3                  = 102 // Camera #3.
	MAV_COMP_ID_CAMERA4                  = 103 // Camera #4.
	MAV_COMP_ID_CAMERA5                  = 104 // Camera #5.
	MAV_COMP_ID_CAMERA6                  = 105 // Camera #6.
	MAV_COMP_ID_SERVO1                   = 140 // Servo #1.
	MAV_COMP_ID_SERVO2                   = 141 // Servo #2.
	MAV_COMP_ID_SERVO3                   = 142 // Servo #3.
	MAV_COMP_ID_SERVO4                   = 143 // Servo #4.
	MAV_COMP_ID_SERVO5                   = 144 // Servo #5.
	MAV_COMP_ID_SERVO6                   = 145 // Servo #6.
	MAV_COMP_ID_SERVO7                   = 146 // Servo #7.
	MAV_COMP_ID_SERVO8                   = 147 // Servo #8.
	MAV_COMP_ID_SERVO9                   = 148 // Servo #9.
	MAV_COMP_ID_SERVO10                  = 149 // Servo #10.
	MAV_COMP_ID_SERVO11                  = 150 // Servo #11.
	MAV_COMP_ID_SERVO12                  = 151 // Servo #12.
	MAV_COMP_ID_SERVO13                  = 152 // Servo #13.
	MAV_COMP_ID_SERVO14                  = 153 // Servo #14.
	MAV_COMP_ID_GIMBAL                   = 154 // Gimbal component.
	MAV_COMP_ID_LOG                      = 155 // Logging component.
	MAV_COMP_ID_ADSB                     = 156 // Automatic Dependent Surveillance-Broadcast (ADS-B) component.
	MAV_COMP_ID_OSD                      = 157 // On Screen Display (OSD) devices for video links.
	MAV_COMP_ID_PERIPHERAL               = 158 // Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.
	MAV_COMP_ID_QX1_GIMBAL               = 159 // Gimbal ID for QX1.
	MAV_COMP_ID_FLARM                    = 160 // FLARM collision alert component.
	MAV_COMP_ID_MISSIONPLANNER           = 190 // Component that can generate/supply a mission flight plan (e.g. GCS or developer API).
	MAV_COMP_ID_PATHPLANNER              = 195 // Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).
	MAV_COMP_ID_OBSTACLE_AVOIDANCE       = 196 // Component that plans a collision free path between two points.
	MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197 // Component that provides position estimates using VIO techniques.
	MAV_COMP_ID_IMU                      = 200 // Inertial Measurement Unit (IMU) #1.
	MAV_COMP_ID_IMU_2                    = 201 // Inertial Measurement Unit (IMU) #2.
	MAV_COMP_ID_IMU_3                    = 202 // Inertial Measurement Unit (IMU) #3.
	MAV_COMP_ID_GPS                      = 220 // GPS #1.
	MAV_COMP_ID_GPS2                     = 221 // GPS #2.
	MAV_COMP_ID_UDP_BRIDGE               = 240 // Component to bridge MAVLink to UDP (i.e. from a UART).
	MAV_COMP_ID_UART_BRIDGE              = 241 // Component to bridge to UART (i.e. from UDP).
	MAV_COMP_ID_SYSTEM_CONTROL           = 250 // Component for handling system messages (e.g. to ARM, takeoff, etc.).
)

// MavSysStatusSensor: These encode the sensors whose status is sent as part of the SYS_STATUS message.
const (
	MAV_SYS_STATUS_SENSOR_3D_GYRO                = 1         // 0x01 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL               = 2         // 0x02 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG                 = 4         // 0x04 3D magnetometer
	MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE      = 8         // 0x08 absolute pressure
	MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE  = 16        // 0x10 differential pressure
	MAV_SYS_STATUS_SENSOR_GPS                    = 32        // 0x20 GPS
	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW           = 64        // 0x40 optical flow
	MAV_SYS_STATUS_SENSOR_VISION_POSITION        = 128       // 0x80 computer vision position
	MAV_SYS_STATUS_SENSOR_LASER_POSITION         = 256       // 0x100 laser based position
	MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH  = 512       // 0x200 external ground truth (Vicon or Leica)
	MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL   = 1024      // 0x400 3D angular rate control
	MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048      // 0x800 attitude stabilization
	MAV_SYS_STATUS_SENSOR_YAW_POSITION           = 4096      // 0x1000 yaw position
	MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL     = 8192      // 0x2000 z/altitude control
	MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL    = 16384     // 0x4000 x/y position control
	MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS          = 32768     // 0x8000 motor outputs / control
	MAV_SYS_STATUS_SENSOR_RC_RECEIVER            = 65536     // 0x10000 rc receiver
	MAV_SYS_STATUS_SENSOR_3D_GYRO2               = 131072    // 0x20000 2nd 3D gyro
	MAV_SYS_STATUS_SENSOR_3D_ACCEL2              = 262144    // 0x40000 2nd 3D accelerometer
	MAV_SYS_STATUS_SENSOR_3D_MAG2                = 524288    // 0x80000 2nd 3D magnetometer
	MAV_SYS_STATUS_GEOFENCE                      = 1048576   // 0x100000 geofence
	MAV_SYS_STATUS_AHRS                          = 2097152   // 0x200000 AHRS subsystem health
	MAV_SYS_STATUS_TERRAIN                       = 4194304   // 0x400000 Terrain subsystem health
	MAV_SYS_STATUS_REVERSE_MOTOR                 = 8388608   // 0x800000 Motors are reversed
	MAV_SYS_STATUS_LOGGING                       = 16777216  // 0x1000000 Logging
	MAV_SYS_STATUS_SENSOR_BATTERY                = 33554432  // 0x2000000 Battery
	MAV_SYS_STATUS_SENSOR_PROXIMITY              = 67108864  // 0x4000000 Proximity
	MAV_SYS_STATUS_SENSOR_SATCOM                 = 134217728 // 0x8000000 Satellite Communication
)

// MavFrame:
const (
	MAV_FRAME_GLOBAL                  = 0  // Global (WGS84) coordinate frame + MSL altitude. First value / x: latitude, second value / y: longitude, third value / z: positive altitude over mean sea level (MSL).
	MAV_FRAME_LOCAL_NED               = 1  // Local coordinate frame, Z-down (x: north, y: east, z: down).
	MAV_FRAME_MISSION                 = 2  // NOT a coordinate frame, indicates a mission command.
	MAV_FRAME_GLOBAL_RELATIVE_ALT     = 3  // Global (WGS84) coordinate frame + altitude relative to the home position. First value / x: latitude, second value / y: longitude, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_ENU               = 4  // Local coordinate frame, Z-up (x: east, y: north, z: up).
	MAV_FRAME_GLOBAL_INT              = 5  // Global (WGS84) coordinate frame (scaled) + MSL altitude. First value / x: latitude in degrees*1.0e-7, second value / y: longitude in degrees*1.0e-7, third value / z: positive altitude over mean sea level (MSL).
	MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6  // Global (WGS84) coordinate frame (scaled) + altitude relative to the home position. First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude with 0 being at the altitude of the home location.
	MAV_FRAME_LOCAL_OFFSET_NED        = 7  // Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.
	MAV_FRAME_BODY_NED                = 8  // Setpoint in body NED frame. This makes sense if all position control is externalized - e.g. useful to command 2 m/s^2 acceleration to the right.
	MAV_FRAME_BODY_OFFSET_NED         = 9  // Offset in body NED frame. This makes sense if adding setpoints to the current flight path, to avoid an obstacle - e.g. useful to command 2 m/s^2 acceleration to the east.
	MAV_FRAME_GLOBAL_TERRAIN_ALT      = 10 // Global (WGS84) coordinate frame with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees, second value / y: longitude in degrees, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
	MAV_FRAME_GLOBAL_TERRAIN_ALT_INT  = 11 // Global (WGS84) coordinate frame (scaled) with AGL altitude (at the waypoint coordinate). First value / x: latitude in degrees*10e-7, second value / y: longitude in degrees*10e-7, third value / z: positive altitude in meters with 0 being at ground level in terrain model.
	MAV_FRAME_BODY_FRD                = 12 // Body fixed frame of reference, Z-down (x: forward, y: right, z: down).
	MAV_FRAME_BODY_FLU                = 13 // Body fixed frame of reference, Z-up (x: forward, y: left, z: up).
	MAV_FRAME_MOCAP_NED               = 14 // Odometry local coordinate frame of data given by a motion capture system, Z-down (x: north, y: east, z: down).
	MAV_FRAME_MOCAP_ENU               = 15 // Odometry local coordinate frame of data given by a motion capture system, Z-up (x: east, y: north, z: up).
	MAV_FRAME_VISION_NED              = 16 // Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: north, y: east, z: down).
	MAV_FRAME_VISION_ENU              = 17 // Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: east, y: north, z: up).
	MAV_FRAME_ESTIM_NED               = 18 // Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: north, y: east, z: down).
	MAV_FRAME_ESTIM_ENU               = 19 // Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: east, y: noth, z: up).
)

// MavlinkDataStreamType:
const (
	MAVLINK_DATA_STREAM_IMG_JPEG   = 0 //
	MAVLINK_DATA_STREAM_IMG_BMP    = 1 //
	MAVLINK_DATA_STREAM_IMG_RAW8U  = 2 //
	MAVLINK_DATA_STREAM_IMG_RAW32U = 3 //
	MAVLINK_DATA_STREAM_IMG_PGM    = 4 //
	MAVLINK_DATA_STREAM_IMG_PNG    = 5 //
)

// FenceAction:
const (
	FENCE_ACTION_NONE            = 0 // Disable fenced mode
	FENCE_ACTION_GUIDED          = 1 // Switched to guided mode to return point (fence point 0)
	FENCE_ACTION_REPORT          = 2 // Report fence breach, but don't take action
	FENCE_ACTION_GUIDED_THR_PASS = 3 // Switched to guided mode to return point (fence point 0) with manual throttle control
	FENCE_ACTION_RTL             = 4 // Switch to RTL (return to launch) mode and head for the return point.
)

// FenceBreach:
const (
	FENCE_BREACH_NONE     = 0 // No last fence breach
	FENCE_BREACH_MINALT   = 1 // Breached minimum altitude
	FENCE_BREACH_MAXALT   = 2 // Breached maximum altitude
	FENCE_BREACH_BOUNDARY = 3 // Breached fence boundary
)

// MavMountMode: Enumeration of possible mount operation modes
const (
	MAV_MOUNT_MODE_RETRACT           = 0 // Load and keep safe position (Roll,Pitch,Yaw) from permant memory and stop stabilization
	MAV_MOUNT_MODE_NEUTRAL           = 1 // Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
	MAV_MOUNT_MODE_MAVLINK_TARGETING = 2 // Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_RC_TARGETING      = 3 // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
	MAV_MOUNT_MODE_GPS_POINT         = 4 // Load neutral position and start to point to Lat,Lon,Alt
)

// UavcanNodeHealth: Generalized UAVCAN node health
const (
	UAVCAN_NODE_HEALTH_OK       = 0 // The node is functioning properly.
	UAVCAN_NODE_HEALTH_WARNING  = 1 // A critical parameter went out of range or the node has encountered a minor failure.
	UAVCAN_NODE_HEALTH_ERROR    = 2 // The node has encountered a major failure.
	UAVCAN_NODE_HEALTH_CRITICAL = 3 // The node has suffered a fatal malfunction.
)

// UavcanNodeMode: Generalized UAVCAN node mode
const (
	UAVCAN_NODE_MODE_OPERATIONAL     = 0 // The node is performing its primary functions.
	UAVCAN_NODE_MODE_INITIALIZATION  = 1 // The node is initializing; this mode is entered immediately after startup.
	UAVCAN_NODE_MODE_MAINTENANCE     = 2 // The node is under maintenance.
	UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3 // The node is in the process of updating its software.
	UAVCAN_NODE_MODE_OFFLINE         = 7 // The node is no longer available online.
)

// MavCmd: Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data.
const (
	MAV_CMD_NAV_WAYPOINT                       = 16    // Navigate to waypoint.
	MAV_CMD_NAV_LOITER_UNLIM                   = 17    // Loiter around this waypoint an unlimited amount of time
	MAV_CMD_NAV_LOITER_TURNS                   = 18    // Loiter around this waypoint for X turns
	MAV_CMD_NAV_LOITER_TIME                    = 19    // Loiter around this waypoint for X seconds
	MAV_CMD_NAV_RETURN_TO_LAUNCH               = 20    // Return to launch location
	MAV_CMD_NAV_LAND                           = 21    // Land at location.
	MAV_CMD_NAV_TAKEOFF                        = 22    // Takeoff from ground / hand
	MAV_CMD_NAV_LAND_LOCAL                     = 23    // Land at local position (local frame only)
	MAV_CMD_NAV_TAKEOFF_LOCAL                  = 24    // Takeoff from local position (local frame only)
	MAV_CMD_NAV_FOLLOW                         = 25    // Vehicle following, i.e. this waypoint represents the position of a moving vehicle
	MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT        = 30    // Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
	MAV_CMD_NAV_LOITER_TO_ALT                  = 31    // Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached.  Additionally, if the Heading Required parameter is non-zero the  aircraft will not leave the loiter until heading toward the next waypoint.
	MAV_CMD_DO_FOLLOW                          = 32    // Being following a target
	MAV_CMD_DO_FOLLOW_REPOSITION               = 33    // Reposition the MAV after a follow target command has been sent
	MAV_CMD_DO_ORBIT                           = 34    // Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults.
	MAV_CMD_NAV_ROI                            = 80    // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_NAV_PATHPLANNING                   = 81    // Control autonomous path planning on the MAV.
	MAV_CMD_NAV_SPLINE_WAYPOINT                = 82    // Navigate to waypoint using a spline path.
	MAV_CMD_NAV_VTOL_TAKEOFF                   = 84    // Takeoff from ground using VTOL mode, and transition to forward flight with specified heading.
	MAV_CMD_NAV_VTOL_LAND                      = 85    // Land using VTOL mode
	MAV_CMD_NAV_GUIDED_ENABLE                  = 92    // hand control over to an external controller
	MAV_CMD_NAV_DELAY                          = 93    // Delay the next navigation command a number of seconds or until a specified time
	MAV_CMD_NAV_PAYLOAD_PLACE                  = 94    // Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.
	MAV_CMD_NAV_LAST                           = 95    // NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
	MAV_CMD_CONDITION_DELAY                    = 112   // Delay mission state machine.
	MAV_CMD_CONDITION_CHANGE_ALT               = 113   // Ascend/descend at rate.  Delay mission state machine until desired altitude reached.
	MAV_CMD_CONDITION_DISTANCE                 = 114   // Delay mission state machine until within desired distance of next NAV point.
	MAV_CMD_CONDITION_YAW                      = 115   // Reach a certain target angle.
	MAV_CMD_CONDITION_LAST                     = 159   // NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
	MAV_CMD_DO_SET_MODE                        = 176   // Set system mode.
	MAV_CMD_DO_JUMP                            = 177   // Jump to the desired command in the mission list.  Repeat this action only the specified number of times
	MAV_CMD_DO_CHANGE_SPEED                    = 178   // Change speed and/or throttle set points.
	MAV_CMD_DO_SET_HOME                        = 179   // Changes the home location either to the current location or a specified location.
	MAV_CMD_DO_SET_PARAMETER                   = 180   // Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
	MAV_CMD_DO_SET_RELAY                       = 181   // Set a relay to a condition.
	MAV_CMD_DO_REPEAT_RELAY                    = 182   // Cycle a relay on and off for a desired number of cycles with a desired period.
	MAV_CMD_DO_SET_SERVO                       = 183   // Set a servo to a desired PWM value.
	MAV_CMD_DO_REPEAT_SERVO                    = 184   // Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
	MAV_CMD_DO_FLIGHTTERMINATION               = 185   // Terminate flight immediately
	MAV_CMD_DO_CHANGE_ALTITUDE                 = 186   // Change altitude set point.
	MAV_CMD_DO_LAND_START                      = 189   // Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence.
	MAV_CMD_DO_RALLY_LAND                      = 190   // Mission command to perform a landing from a rally point.
	MAV_CMD_DO_GO_AROUND                       = 191   // Mission command to safely abort an autonomous landing.
	MAV_CMD_DO_REPOSITION                      = 192   // Reposition the vehicle to a specific WGS84 global position.
	MAV_CMD_DO_PAUSE_CONTINUE                  = 193   // If in a GPS controlled position mode, hold the current position or continue.
	MAV_CMD_DO_SET_REVERSE                     = 194   // Set moving direction to forward or reverse.
	MAV_CMD_DO_SET_ROI_LOCATION                = 195   // Sets the region of interest (ROI) to a location. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET           = 196   // Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_SET_ROI_NONE                    = 197   // Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_CONTROL_VIDEO                   = 200   // Control onboard camera system.
	MAV_CMD_DO_SET_ROI                         = 201   // Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras.
	MAV_CMD_DO_DIGICAM_CONFIGURE               = 202   // Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
	MAV_CMD_DO_DIGICAM_CONTROL                 = 203   // Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
	MAV_CMD_DO_MOUNT_CONFIGURE                 = 204   // Mission command to configure a camera or antenna mount
	MAV_CMD_DO_MOUNT_CONTROL                   = 205   // Mission command to control a camera or antenna mount
	MAV_CMD_DO_SET_CAM_TRIGG_DIST              = 206   // Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
	MAV_CMD_DO_FENCE_ENABLE                    = 207   // Mission command to enable the geofence
	MAV_CMD_DO_PARACHUTE                       = 208   // Mission command to trigger a parachute
	MAV_CMD_DO_MOTOR_TEST                      = 209   // Mission command to perform motor test
	MAV_CMD_DO_INVERTED_FLIGHT                 = 210   // Change to/from inverted flight
	MAV_CMD_NAV_SET_YAW_SPEED                  = 213   // Sets a desired vehicle turn angle and speed change
	MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL          = 214   // Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
	MAV_CMD_DO_MOUNT_CONTROL_QUAT              = 220   // Mission command to control a camera or antenna mount, using a quaternion as reference.
	MAV_CMD_DO_GUIDED_MASTER                   = 221   // set id of master controller
	MAV_CMD_DO_GUIDED_LIMITS                   = 222   // Set limits for external control
	MAV_CMD_DO_ENGINE_CONTROL                  = 223   // Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
	MAV_CMD_DO_SET_MISSION_CURRENT             = 224   // Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
	MAV_CMD_DO_LAST                            = 240   // NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
	MAV_CMD_PREFLIGHT_CALIBRATION              = 241   // Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS       = 242   // Set sensor offsets. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_UAVCAN                   = 243   // Trigger UAVCAN config. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_STORAGE                  = 245   // Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN          = 246   // Request the reboot or shutdown of system components.
	MAV_CMD_OVERRIDE_GOTO                      = 252   // Hold / continue the current action
	MAV_CMD_MISSION_START                      = 300   // start running a mission
	MAV_CMD_COMPONENT_ARM_DISARM               = 400   // Arms / Disarms a component
	MAV_CMD_GET_HOME_POSITION                  = 410   // Request the home position from the vehicle.
	MAV_CMD_START_RX_PAIR                      = 500   // Starts receiver pairing
	MAV_CMD_GET_MESSAGE_INTERVAL               = 510   // Request the interval between messages for a particular MAVLink message ID
	MAV_CMD_SET_MESSAGE_INTERVAL               = 511   // Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM
	MAV_CMD_REQUEST_MESSAGE                    = 512   // Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
	MAV_CMD_REQUEST_PROTOCOL_VERSION           = 519   // Request MAVLink protocol version compatibility
	MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES     = 520   // Request autopilot capabilities
	MAV_CMD_REQUEST_CAMERA_INFORMATION         = 521   // Request camera information (CAMERA_INFORMATION).
	MAV_CMD_REQUEST_CAMERA_SETTINGS            = 522   // Request camera settings (CAMERA_SETTINGS).
	MAV_CMD_REQUEST_STORAGE_INFORMATION        = 525   // Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
	MAV_CMD_STORAGE_FORMAT                     = 526   // Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
	MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS      = 527   // Request camera capture status (CAMERA_CAPTURE_STATUS)
	MAV_CMD_REQUEST_FLIGHT_INFORMATION         = 528   // Request flight information (FLIGHT_INFORMATION)
	MAV_CMD_RESET_CAMERA_SETTINGS              = 529   // Reset all camera settings to Factory Default
	MAV_CMD_SET_CAMERA_MODE                    = 530   // Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.
	MAV_CMD_SET_CAMERA_ZOOM                    = 531   // Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved values.
	MAV_CMD_SET_CAMERA_FOCUS                   = 532   // Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). Use NaN for reserved values.
	MAV_CMD_JUMP_TAG                           = 600   // Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
	MAV_CMD_DO_JUMP_TAG                        = 601   // Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.
	MAV_CMD_IMAGE_START_CAPTURE                = 2000  // Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values.
	MAV_CMD_IMAGE_STOP_CAPTURE                 = 2001  // Stop image capture sequence Use NaN for reserved values.
	MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE       = 2002  // Re-request a CAMERA_IMAGE_CAPTURE message. Use NaN for reserved values.
	MAV_CMD_DO_TRIGGER_CONTROL                 = 2003  // Enable or disable on-board camera triggering system.
	MAV_CMD_VIDEO_START_CAPTURE                = 2500  // Starts video capture (recording). Use NaN for reserved values.
	MAV_CMD_VIDEO_STOP_CAPTURE                 = 2501  // Stop the current video capture (recording). Use NaN for reserved values.
	MAV_CMD_VIDEO_START_STREAMING              = 2502  // Start video streaming
	MAV_CMD_VIDEO_STOP_STREAMING               = 2503  // Stop the given video stream
	MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION   = 2504  // Request video stream information (VIDEO_STREAM_INFORMATION)
	MAV_CMD_REQUEST_VIDEO_STREAM_STATUS        = 2505  // Request video stream status (VIDEO_STREAM_STATUS)
	MAV_CMD_LOGGING_START                      = 2510  // Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
	MAV_CMD_LOGGING_STOP                       = 2511  // Request to stop streaming log data over MAVLink
	MAV_CMD_AIRFRAME_CONFIGURATION             = 2520  //
	MAV_CMD_CONTROL_HIGH_LATENCY               = 2600  // Request to start/stop transmitting over the high latency telemetry
	MAV_CMD_PANORAMA_CREATE                    = 2800  // Create a panorama at the current position
	MAV_CMD_DO_VTOL_TRANSITION                 = 3000  // Request VTOL transition
	MAV_CMD_ARM_AUTHORIZATION_REQUEST          = 3001  // Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
	MAV_CMD_SET_GUIDED_SUBMODE_STANDARD        = 4000  // This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
	MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE          = 4001  // This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
	MAV_CMD_CONDITION_GATE                     = 4501  // Delay mission state machine until gate has been reached.
	MAV_CMD_NAV_FENCE_RETURN_POINT             = 5000  // Fence return point. There can only be one fence return point.
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001  // Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
	MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002  // Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
	MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION         = 5003  // Circular fence area. The vehicle must stay inside this area.
	MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION         = 5004  // Circular fence area. The vehicle must stay outside this area.
	MAV_CMD_NAV_RALLY_POINT                    = 5100  // Rally point. You can have multiple rally points defined.
	MAV_CMD_UAVCAN_GET_NODE_INFO               = 5200  // Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
	MAV_CMD_PAYLOAD_PREPARE_DEPLOY             = 30001 // Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
	MAV_CMD_PAYLOAD_CONTROL_DEPLOY             = 30002 // Control the payload deployment.
	MAV_CMD_WAYPOINT_USER_1                    = 31000 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_2                    = 31001 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_3                    = 31002 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_4                    = 31003 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_WAYPOINT_USER_5                    = 31004 // User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
	MAV_CMD_SPATIAL_USER_1                     = 31005 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_2                     = 31006 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_3                     = 31007 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_4                     = 31008 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_SPATIAL_USER_5                     = 31009 // User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
	MAV_CMD_USER_1                             = 31010 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_2                             = 31011 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_3                             = 31012 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_4                             = 31013 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
	MAV_CMD_USER_5                             = 31014 // User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
)

// MavDataStream: A data stream is not a fixed set of messages, but rather a      recommendation to the autopilot software. Individual autopilots may or may not obey      the recommended messages.
const (
	MAV_DATA_STREAM_ALL             = 0  // Enable all data streams
	MAV_DATA_STREAM_RAW_SENSORS     = 1  // Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
	MAV_DATA_STREAM_EXTENDED_STATUS = 2  // Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
	MAV_DATA_STREAM_RC_CHANNELS     = 3  // Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
	MAV_DATA_STREAM_RAW_CONTROLLER  = 4  // Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
	MAV_DATA_STREAM_POSITION        = 6  // Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
	MAV_DATA_STREAM_EXTRA1          = 10 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA2          = 11 // Dependent on the autopilot
	MAV_DATA_STREAM_EXTRA3          = 12 // Dependent on the autopilot
)

// MavRoi: The ROI (region of interest) for the vehicle. This can be                 be used by the vehicle for camera/vehicle attitude alignment (see                 MAV_CMD_NAV_ROI).
const (
	MAV_ROI_NONE     = 0 // No region of interest.
	MAV_ROI_WPNEXT   = 1 // Point toward next waypoint, with optional pitch/roll/yaw offset.
	MAV_ROI_WPINDEX  = 2 // Point toward given waypoint.
	MAV_ROI_LOCATION = 3 // Point toward fixed location.
	MAV_ROI_TARGET   = 4 // Point toward of given id.
)

// MavCmdAck: ACK / NACK / ERROR values as a result of MAV_CMDs and for mission item transmission.
const (
	MAV_CMD_ACK_OK                                 = 0 // Command / mission item is ok.
	MAV_CMD_ACK_ERR_FAIL                           = 1 // Generic error message if none of the other reasons fails or if no detailed error reporting is implemented.
	MAV_CMD_ACK_ERR_ACCESS_DENIED                  = 2 // The system is refusing to accept this command from this source / communication partner.
	MAV_CMD_ACK_ERR_NOT_SUPPORTED                  = 3 // Command or mission item is not supported, other commands would be accepted.
	MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED = 4 // The coordinate frame of this command / mission item is not supported.
	MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE       = 5 // The coordinate frame of this command is ok, but he coordinate values exceed the safety limits of this system. This is a generic error, please use the more specific error messages below if possible.
	MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE             = 6 // The X or latitude value is out of range.
	MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE             = 7 // The Y or longitude value is out of range.
	MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE             = 8 // The Z or altitude value is out of range.
)

// MavParamType: Specifies the datatype of a MAVLink parameter.
const (
	MAV_PARAM_TYPE_UINT8  = 1  // 8-bit unsigned integer
	MAV_PARAM_TYPE_INT8   = 2  // 8-bit signed integer
	MAV_PARAM_TYPE_UINT16 = 3  // 16-bit unsigned integer
	MAV_PARAM_TYPE_INT16  = 4  // 16-bit signed integer
	MAV_PARAM_TYPE_UINT32 = 5  // 32-bit unsigned integer
	MAV_PARAM_TYPE_INT32  = 6  // 32-bit signed integer
	MAV_PARAM_TYPE_UINT64 = 7  // 64-bit unsigned integer
	MAV_PARAM_TYPE_INT64  = 8  // 64-bit signed integer
	MAV_PARAM_TYPE_REAL32 = 9  // 32-bit floating-point
	MAV_PARAM_TYPE_REAL64 = 10 // 64-bit floating-point
)

// MavParamExtType: Specifies the datatype of a MAVLink extended parameter.
const (
	MAV_PARAM_EXT_TYPE_UINT8  = 1  // 8-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT8   = 2  // 8-bit signed integer
	MAV_PARAM_EXT_TYPE_UINT16 = 3  // 16-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT16  = 4  // 16-bit signed integer
	MAV_PARAM_EXT_TYPE_UINT32 = 5  // 32-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT32  = 6  // 32-bit signed integer
	MAV_PARAM_EXT_TYPE_UINT64 = 7  // 64-bit unsigned integer
	MAV_PARAM_EXT_TYPE_INT64  = 8  // 64-bit signed integer
	MAV_PARAM_EXT_TYPE_REAL32 = 9  // 32-bit floating-point
	MAV_PARAM_EXT_TYPE_REAL64 = 10 // 64-bit floating-point
	MAV_PARAM_EXT_TYPE_CUSTOM = 11 // Custom Type
)

// MavResult: result from a mavlink command
const (
	MAV_RESULT_ACCEPTED             = 0 // Command ACCEPTED and EXECUTED
	MAV_RESULT_TEMPORARILY_REJECTED = 1 // Command TEMPORARY REJECTED/DENIED
	MAV_RESULT_DENIED               = 2 // Command PERMANENTLY DENIED
	MAV_RESULT_UNSUPPORTED          = 3 // Command UNKNOWN/UNSUPPORTED
	MAV_RESULT_FAILED               = 4 // Command executed, but failed
	MAV_RESULT_IN_PROGRESS          = 5 // WIP: Command being executed
)

// MavMissionResult: Result of mission operation (in a MISSION_ACK message).
const (
	MAV_MISSION_ACCEPTED            = 0  // mission accepted OK
	MAV_MISSION_ERROR               = 1  // Generic error / not accepting mission commands at all right now.
	MAV_MISSION_UNSUPPORTED_FRAME   = 2  // Coordinate frame is not supported.
	MAV_MISSION_UNSUPPORTED         = 3  // Command is not supported.
	MAV_MISSION_NO_SPACE            = 4  // Mission item exceeds storage space.
	MAV_MISSION_INVALID             = 5  // One of the parameters has an invalid value.
	MAV_MISSION_INVALID_PARAM1      = 6  // param1 has an invalid value.
	MAV_MISSION_INVALID_PARAM2      = 7  // param2 has an invalid value.
	MAV_MISSION_INVALID_PARAM3      = 8  // param3 has an invalid value.
	MAV_MISSION_INVALID_PARAM4      = 9  // param4 has an invalid value.
	MAV_MISSION_INVALID_PARAM5_X    = 10 // x / param5 has an invalid value.
	MAV_MISSION_INVALID_PARAM6_Y    = 11 // y / param6 has an invalid value.
	MAV_MISSION_INVALID_PARAM7      = 12 // z / param7 has an invalid value.
	MAV_MISSION_INVALID_SEQUENCE    = 13 // Mission item received out of sequence
	MAV_MISSION_DENIED              = 14 // Not accepting any mission commands from this communication partner.
	MAV_MISSION_OPERATION_CANCELLED = 15 // Current mission operation cancelled (e.g. mission upload, mission download).
)

// MavSeverity: Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
const (
	MAV_SEVERITY_EMERGENCY = 0 // System is unusable. This is a "panic" condition.
	MAV_SEVERITY_ALERT     = 1 // Action should be taken immediately. Indicates error in non-critical systems.
	MAV_SEVERITY_CRITICAL  = 2 // Action must be taken immediately. Indicates failure in a primary system.
	MAV_SEVERITY_ERROR     = 3 // Indicates an error in secondary/redundant systems.
	MAV_SEVERITY_WARNING   = 4 // Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
	MAV_SEVERITY_NOTICE    = 5 // An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
	MAV_SEVERITY_INFO      = 6 // Normal operational messages. Useful for logging. No action is required for these messages.
	MAV_SEVERITY_DEBUG     = 7 // Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
)

// MavPowerStatus: Power supply status flags (bitmask)
const (
	MAV_POWER_STATUS_BRICK_VALID                = 1  // main brick power supply valid
	MAV_POWER_STATUS_SERVO_VALID                = 2  // main servo power supply valid for FMU
	MAV_POWER_STATUS_USB_CONNECTED              = 4  // USB power is connected
	MAV_POWER_STATUS_PERIPH_OVERCURRENT         = 8  // peripheral supply is in over-current state
	MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16 // hi-power peripheral supply is in over-current state
	MAV_POWER_STATUS_CHANGED                    = 32 // Power status has changed since boot
)

// SerialControlDev: SERIAL_CONTROL device types
const (
	SERIAL_CONTROL_DEV_TELEM1 = 0  // First telemetry port
	SERIAL_CONTROL_DEV_TELEM2 = 1  // Second telemetry port
	SERIAL_CONTROL_DEV_GPS1   = 2  // First GPS port
	SERIAL_CONTROL_DEV_GPS2   = 3  // Second GPS port
	SERIAL_CONTROL_DEV_SHELL  = 10 // system shell
)

// SerialControlFlag: SERIAL_CONTROL flags (bitmask)
const (
	SERIAL_CONTROL_FLAG_REPLY     = 1  // Set if this is a reply
	SERIAL_CONTROL_FLAG_RESPOND   = 2  // Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
	SERIAL_CONTROL_FLAG_EXCLUSIVE = 4  // Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
	SERIAL_CONTROL_FLAG_BLOCKING  = 8  // Block on writes to the serial port
	SERIAL_CONTROL_FLAG_MULTI     = 16 // Send multiple replies until port is drained
)

// MavDistanceSensor: Enumeration of distance sensor types
const (
	MAV_DISTANCE_SENSOR_LASER      = 0 // Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
	MAV_DISTANCE_SENSOR_ULTRASOUND = 1 // Ultrasound rangefinder, e.g. MaxBotix units
	MAV_DISTANCE_SENSOR_INFRARED   = 2 // Infrared rangefinder, e.g. Sharp units
	MAV_DISTANCE_SENSOR_RADAR      = 3 // Radar type, e.g. uLanding units
	MAV_DISTANCE_SENSOR_UNKNOWN    = 4 // Broken or unknown type, e.g. analog units
)

// MavSensorOrientation: Enumeration of sensor orientation, according to its rotations
const (
	MAV_SENSOR_ROTATION_NONE                     = 0   // Roll: 0, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_YAW_45                   = 1   // Roll: 0, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_YAW_90                   = 2   // Roll: 0, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_YAW_135                  = 3   // Roll: 0, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_YAW_180                  = 4   // Roll: 0, Pitch: 0, Yaw: 180
	MAV_SENSOR_ROTATION_YAW_225                  = 5   // Roll: 0, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_YAW_270                  = 6   // Roll: 0, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_YAW_315                  = 7   // Roll: 0, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_180                 = 8   // Roll: 180, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_45          = 9   // Roll: 180, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_180_YAW_90          = 10  // Roll: 180, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_180_YAW_135         = 11  // Roll: 180, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_180                = 12  // Roll: 0, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_YAW_225         = 13  // Roll: 180, Pitch: 0, Yaw: 225
	MAV_SENSOR_ROTATION_ROLL_180_YAW_270         = 14  // Roll: 180, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_180_YAW_315         = 15  // Roll: 180, Pitch: 0, Yaw: 315
	MAV_SENSOR_ROTATION_ROLL_90                  = 16  // Roll: 90, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_YAW_45           = 17  // Roll: 90, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_90_YAW_90           = 18  // Roll: 90, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_135          = 19  // Roll: 90, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_ROLL_270                 = 20  // Roll: 270, Pitch: 0, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_YAW_45          = 21  // Roll: 270, Pitch: 0, Yaw: 45
	MAV_SENSOR_ROTATION_ROLL_270_YAW_90          = 22  // Roll: 270, Pitch: 0, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_270_YAW_135         = 23  // Roll: 270, Pitch: 0, Yaw: 135
	MAV_SENSOR_ROTATION_PITCH_90                 = 24  // Roll: 0, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_270                = 25  // Roll: 0, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_PITCH_180_YAW_90         = 26  // Roll: 0, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_PITCH_180_YAW_270        = 27  // Roll: 0, Pitch: 180, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_90         = 28  // Roll: 90, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_90        = 29  // Roll: 180, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_90        = 30  // Roll: 270, Pitch: 90, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180        = 31  // Roll: 90, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_180       = 32  // Roll: 270, Pitch: 180, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_270        = 33  // Roll: 90, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_180_PITCH_270       = 34  // Roll: 180, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_270_PITCH_270       = 35  // Roll: 270, Pitch: 270, Yaw: 0
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36  // Roll: 90, Pitch: 180, Yaw: 90
	MAV_SENSOR_ROTATION_ROLL_90_YAW_270          = 37  // Roll: 90, Pitch: 0, Yaw: 270
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 = 38  // Roll: 90, Pitch: 68, Yaw: 293
	MAV_SENSOR_ROTATION_PITCH_315                = 39  // Pitch: 315
	MAV_SENSOR_ROTATION_ROLL_90_PITCH_315        = 40  // Roll: 90, Pitch: 315
	MAV_SENSOR_ROTATION_CUSTOM                   = 100 // Custom orientation
)

// MavProtocolCapability: Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability.
const (
	MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT                  = 1     // Autopilot supports MISSION float message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT                    = 2     // Autopilot supports the new param float message type.
	MAV_PROTOCOL_CAPABILITY_MISSION_INT                    = 4     // Autopilot supports MISSION_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_COMMAND_INT                    = 8     // Autopilot supports COMMAND_INT scaled integer message type.
	MAV_PROTOCOL_CAPABILITY_PARAM_UNION                    = 16    // Autopilot supports the new param union message type.
	MAV_PROTOCOL_CAPABILITY_FTP                            = 32    // Autopilot supports the new FILE_TRANSFER_PROTOCOL message type.
	MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET            = 64    // Autopilot supports commanding attitude offboard.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED  = 128   // Autopilot supports commanding position and velocity targets in local NED frame.
	MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256   // Autopilot supports commanding position and velocity targets in global scaled integers.
	MAV_PROTOCOL_CAPABILITY_TERRAIN                        = 512   // Autopilot supports terrain protocol / data handling.
	MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET            = 1024  // Autopilot supports direct actuator control.
	MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION             = 2048  // Autopilot supports the flight termination command.
	MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION            = 4096  // Autopilot supports onboard compass calibration.
	MAV_PROTOCOL_CAPABILITY_MAVLINK2                       = 8192  // Autopilot supports MAVLink version 2.
	MAV_PROTOCOL_CAPABILITY_MISSION_FENCE                  = 16384 // Autopilot supports mission fence protocol.
	MAV_PROTOCOL_CAPABILITY_MISSION_RALLY                  = 32768 // Autopilot supports mission rally point protocol.
	MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION             = 65536 // Autopilot supports the flight information protocol.
)

// MavMissionType: Type of mission items being requested/sent in mission protocol.
const (
	MAV_MISSION_TYPE_MISSION = 0   // Items are mission commands for main mission.
	MAV_MISSION_TYPE_FENCE   = 1   // Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
	MAV_MISSION_TYPE_RALLY   = 2   // Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.
	MAV_MISSION_TYPE_ALL     = 255 // Only used in MISSION_CLEAR_ALL to clear all mission types.
)

// MavEstimatorType: Enumeration of estimator types
const (
	MAV_ESTIMATOR_TYPE_NAIVE   = 1 // This is a naive estimator without any real covariance feedback.
	MAV_ESTIMATOR_TYPE_VISION  = 2 // Computer vision based estimate. Might be up to scale.
	MAV_ESTIMATOR_TYPE_VIO     = 3 // Visual-inertial estimate.
	MAV_ESTIMATOR_TYPE_GPS     = 4 // Plain GPS estimate.
	MAV_ESTIMATOR_TYPE_GPS_INS = 5 // Estimator integrating GPS and inertial sensing.
)

// MavBatteryType: Enumeration of battery types
const (
	MAV_BATTERY_TYPE_UNKNOWN = 0 // Not specified.
	MAV_BATTERY_TYPE_LIPO    = 1 // Lithium polymer battery
	MAV_BATTERY_TYPE_LIFE    = 2 // Lithium-iron-phosphate battery
	MAV_BATTERY_TYPE_LION    = 3 // Lithium-ION battery
	MAV_BATTERY_TYPE_NIMH    = 4 // Nickel metal hydride battery
)

// MavBatteryFunction: Enumeration of battery functions
const (
	MAV_BATTERY_FUNCTION_UNKNOWN    = 0 // Battery function is unknown
	MAV_BATTERY_FUNCTION_ALL        = 1 // Battery supports all flight systems
	MAV_BATTERY_FUNCTION_PROPULSION = 2 // Battery for the propulsion system
	MAV_BATTERY_FUNCTION_AVIONICS   = 3 // Avionics battery
	MAV_BATTERY_TYPE_PAYLOAD        = 4 // Payload battery
)

// MavBatteryChargeState: Enumeration for battery charge states.
const (
	MAV_BATTERY_CHARGE_STATE_UNDEFINED = 0 // Low battery state is not provided
	MAV_BATTERY_CHARGE_STATE_OK        = 1 // Battery is not in low state. Normal operation.
	MAV_BATTERY_CHARGE_STATE_LOW       = 2 // Battery state is low, warn and monitor close.
	MAV_BATTERY_CHARGE_STATE_CRITICAL  = 3 // Battery state is critical, return or abort immediately.
	MAV_BATTERY_CHARGE_STATE_EMERGENCY = 4 // Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage.
	MAV_BATTERY_CHARGE_STATE_FAILED    = 5 // Battery failed, damage unavoidable.
	MAV_BATTERY_CHARGE_STATE_UNHEALTHY = 6 // Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited.
	MAV_BATTERY_CHARGE_STATE_CHARGING  = 7 // Battery is charging.
)

// MavSmartBatteryFault: Smart battery supply status/fault flags (bitmask) for health indication.
const (
	MAV_SMART_BATTERY_FAULT_DEEP_DISCHARGE    = 1  // Battery has deep discharged.
	MAV_SMART_BATTERY_FAULT_SPIKES            = 2  // Voltage spikes.
	MAV_SMART_BATTERY_FAULT_SINGLE_CELL_FAIL  = 4  // Single cell has failed.
	MAV_SMART_BATTERY_FAULT_OVER_CURRENT      = 8  // Over-current fault.
	MAV_SMART_BATTERY_FAULT_OVER_TEMPERATURE  = 16 // Over-temperature fault.
	MAV_SMART_BATTERY_FAULT_UNDER_TEMPERATURE = 32 // Under-temperature fault.
)

// MavVtolState: Enumeration of VTOL states
const (
	MAV_VTOL_STATE_UNDEFINED        = 0 // MAV is not configured as VTOL
	MAV_VTOL_STATE_TRANSITION_TO_FW = 1 // VTOL is in transition from multicopter to fixed-wing
	MAV_VTOL_STATE_TRANSITION_TO_MC = 2 // VTOL is in transition from fixed-wing to multicopter
	MAV_VTOL_STATE_MC               = 3 // VTOL is in multicopter state
	MAV_VTOL_STATE_FW               = 4 // VTOL is in fixed-wing state
)

// MavLandedState: Enumeration of landed detector states
const (
	MAV_LANDED_STATE_UNDEFINED = 0 // MAV landed state is unknown
	MAV_LANDED_STATE_ON_GROUND = 1 // MAV is landed (on ground)
	MAV_LANDED_STATE_IN_AIR    = 2 // MAV is in air
	MAV_LANDED_STATE_TAKEOFF   = 3 // MAV currently taking off
	MAV_LANDED_STATE_LANDING   = 4 // MAV currently landing
)

// AdsbAltitudeType: Enumeration of the ADSB altimeter types
const (
	ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0 // Altitude reported from a Baro source using QNH reference
	ADSB_ALTITUDE_TYPE_GEOMETRIC    = 1 // Altitude reported from a GNSS source
)

// AdsbEmitterType: ADSB classification for the type of vehicle emitting the transponder signal
const (
	ADSB_EMITTER_TYPE_NO_INFO           = 0  //
	ADSB_EMITTER_TYPE_LIGHT             = 1  //
	ADSB_EMITTER_TYPE_SMALL             = 2  //
	ADSB_EMITTER_TYPE_LARGE             = 3  //
	ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4  //
	ADSB_EMITTER_TYPE_HEAVY             = 5  //
	ADSB_EMITTER_TYPE_HIGHLY_MANUV      = 6  //
	ADSB_EMITTER_TYPE_ROTOCRAFT         = 7  //
	ADSB_EMITTER_TYPE_UNASSIGNED        = 8  //
	ADSB_EMITTER_TYPE_GLIDER            = 9  //
	ADSB_EMITTER_TYPE_LIGHTER_AIR       = 10 //
	ADSB_EMITTER_TYPE_PARACHUTE         = 11 //
	ADSB_EMITTER_TYPE_ULTRA_LIGHT       = 12 //
	ADSB_EMITTER_TYPE_UNASSIGNED2       = 13 //
	ADSB_EMITTER_TYPE_UAV               = 14 //
	ADSB_EMITTER_TYPE_SPACE             = 15 //
	ADSB_EMITTER_TYPE_UNASSGINED3       = 16 //
	ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17 //
	ADSB_EMITTER_TYPE_SERVICE_SURFACE   = 18 //
	ADSB_EMITTER_TYPE_POINT_OBSTACLE    = 19 //
)

// AdsbFlags: These flags indicate status such as data validity of each data source. Set = data valid
const (
	ADSB_FLAGS_VALID_COORDS   = 1  //
	ADSB_FLAGS_VALID_ALTITUDE = 2  //
	ADSB_FLAGS_VALID_HEADING  = 4  //
	ADSB_FLAGS_VALID_VELOCITY = 8  //
	ADSB_FLAGS_VALID_CALLSIGN = 16 //
	ADSB_FLAGS_VALID_SQUAWK   = 32 //
	ADSB_FLAGS_SIMULATED      = 64 //
)

// MavDoRepositionFlags: Bitmap of options for the MAV_CMD_DO_REPOSITION
const (
	MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1 // The aircraft should immediately transition into guided. This should not be set for follow me applications
)

// EstimatorStatusFlags: Flags in EKF_STATUS message
const (
	ESTIMATOR_ATTITUDE           = 1    // True if the attitude estimate is good
	ESTIMATOR_VELOCITY_HORIZ     = 2    // True if the horizontal velocity estimate is good
	ESTIMATOR_VELOCITY_VERT      = 4    // True if the  vertical velocity estimate is good
	ESTIMATOR_POS_HORIZ_REL      = 8    // True if the horizontal position (relative) estimate is good
	ESTIMATOR_POS_HORIZ_ABS      = 16   // True if the horizontal position (absolute) estimate is good
	ESTIMATOR_POS_VERT_ABS       = 32   // True if the vertical position (absolute) estimate is good
	ESTIMATOR_POS_VERT_AGL       = 64   // True if the vertical position (above ground) estimate is good
	ESTIMATOR_CONST_POS_MODE     = 128  // True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
	ESTIMATOR_PRED_POS_HORIZ_REL = 256  // True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
	ESTIMATOR_PRED_POS_HORIZ_ABS = 512  // True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
	ESTIMATOR_GPS_GLITCH         = 1024 // True if the EKF has detected a GPS glitch
	ESTIMATOR_ACCEL_ERROR        = 2048 // True if the EKF has detected bad accelerometer data
)

// MotorTestOrder:
const (
	MOTOR_TEST_ORDER_DEFAULT  = 0 // default autopilot motor test method
	MOTOR_TEST_ORDER_SEQUENCE = 1 // motor numbers are specified as their index in a predefined vehicle-specific sequence
	MOTOR_TEST_ORDER_BOARD    = 2 // motor numbers are specified as the output as labeled on the board
)

// MotorTestThrottleType:
const (
	MOTOR_TEST_THROTTLE_PERCENT = 0 // throttle as a percentage from 0 ~ 100
	MOTOR_TEST_THROTTLE_PWM     = 1 // throttle as an absolute PWM value (normally in range of 1000~2000)
	MOTOR_TEST_THROTTLE_PILOT   = 2 // throttle pass-through from pilot's transmitter
	MOTOR_TEST_COMPASS_CAL      = 3 // per-motor compass calibration test
)

// GpsInputIgnoreFlags:
const (
	GPS_INPUT_IGNORE_FLAG_ALT                 = 1   // ignore altitude field
	GPS_INPUT_IGNORE_FLAG_HDOP                = 2   // ignore hdop field
	GPS_INPUT_IGNORE_FLAG_VDOP                = 4   // ignore vdop field
	GPS_INPUT_IGNORE_FLAG_VEL_HORIZ           = 8   // ignore horizontal velocity field (vn and ve)
	GPS_INPUT_IGNORE_FLAG_VEL_VERT            = 16  // ignore vertical velocity field (vd)
	GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY      = 32  // ignore speed accuracy field
	GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64  // ignore horizontal accuracy field
	GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY   = 128 // ignore vertical accuracy field
)

// MavCollisionAction: Possible actions an aircraft can take to avoid a collision.
const (
	MAV_COLLISION_ACTION_NONE               = 0 // Ignore any potential collisions
	MAV_COLLISION_ACTION_REPORT             = 1 // Report potential collision
	MAV_COLLISION_ACTION_ASCEND_OR_DESCEND  = 2 // Ascend or Descend to avoid threat
	MAV_COLLISION_ACTION_MOVE_HORIZONTALLY  = 3 // Move horizontally to avoid threat
	MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4 // Aircraft to move perpendicular to the collision's velocity vector
	MAV_COLLISION_ACTION_RTL                = 5 // Aircraft to fly directly back to its launch point
	MAV_COLLISION_ACTION_HOVER              = 6 // Aircraft to stop in place
)

// MavCollisionThreatLevel: Aircraft-rated danger from this threat.
const (
	MAV_COLLISION_THREAT_LEVEL_NONE = 0 // Not a threat
	MAV_COLLISION_THREAT_LEVEL_LOW  = 1 // Craft is mildly concerned about this threat
	MAV_COLLISION_THREAT_LEVEL_HIGH = 2 // Craft is panicking, and may take actions to avoid threat
)

// MavCollisionSrc: Source of information about this collision.
const (
	MAV_COLLISION_SRC_ADSB                   = 0 // ID field references ADSB_VEHICLE packets
	MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1 // ID field references MAVLink SRC ID
)

// GpsFixType: Type of GPS fix
const (
	GPS_FIX_TYPE_NO_GPS    = 0 // No GPS connected
	GPS_FIX_TYPE_NO_FIX    = 1 // No position information, GPS is connected
	GPS_FIX_TYPE_2D_FIX    = 2 // 2D position
	GPS_FIX_TYPE_3D_FIX    = 3 // 3D position
	GPS_FIX_TYPE_DGPS      = 4 // DGPS/SBAS aided 3D position
	GPS_FIX_TYPE_RTK_FLOAT = 5 // RTK float, 3D position
	GPS_FIX_TYPE_RTK_FIXED = 6 // RTK Fixed, 3D position
	GPS_FIX_TYPE_STATIC    = 7 // Static fixed, typically used for base stations
	GPS_FIX_TYPE_PPP       = 8 // PPP, 3D position.
)

// RtkBaselineCoordinateSystem: RTK GPS baseline coordinate system, used for RTK corrections
const (
	RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0 // Earth-centered, Earth-fixed
	RTK_BASELINE_COORDINATE_SYSTEM_NED  = 1 // North, East, Down
)

// LandingTargetType: Type of landing target
const (
	LANDING_TARGET_TYPE_LIGHT_BEACON    = 0 // Landing target signaled by light beacon (ex: IR-LOCK)
	LANDING_TARGET_TYPE_RADIO_BEACON    = 1 // Landing target signaled by radio beacon (ex: ILS, NDB)
	LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2 // Landing target represented by a fiducial marker (ex: ARTag)
	LANDING_TARGET_TYPE_VISION_OTHER    = 3 // Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
)

// VtolTransitionHeading: Direction of VTOL transition
const (
	VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT = 0 // Respect the heading configuration of the vehicle.
	VTOL_TRANSITION_HEADING_NEXT_WAYPOINT   = 1 // Use the heading pointing towards the next waypoint.
	VTOL_TRANSITION_HEADING_TAKEOFF         = 2 // Use the heading on takeoff (while sitting on the ground).
	VTOL_TRANSITION_HEADING_SPECIFIED       = 3 // Use the specified heading in parameter 4.
	VTOL_TRANSITION_HEADING_ANY             = 4 // Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).
)

// CameraCapFlags: Camera capability flags (Bitmap)
const (
	CAMERA_CAP_FLAGS_CAPTURE_VIDEO                   = 1   // Camera is able to record video
	CAMERA_CAP_FLAGS_CAPTURE_IMAGE                   = 2   // Camera is able to capture images
	CAMERA_CAP_FLAGS_HAS_MODES                       = 4   // Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
	CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8   // Camera can capture images while in video mode
	CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16  // Camera can capture videos while in Photo/Image mode
	CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE           = 32  // Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
	CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM                  = 64  // Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
	CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS                 = 128 // Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
	CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM                = 256 // Camera has video streaming capabilities (use MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION for video streaming info)
)

// VideoStreamStatusFlags: Stream status flags (Bitmap)
const (
	VIDEO_STREAM_STATUS_FLAGS_RUNNING = 1 // Stream is active (running)
	VIDEO_STREAM_STATUS_FLAGS_THERMAL = 2 // Stream is thermal imaging
)

// VideoStreamType: Video stream types
const (
	VIDEO_STREAM_TYPE_RTSP         = 0 // Stream is RTSP
	VIDEO_STREAM_TYPE_RTPUDP       = 1 // Stream is RTP UDP (URI gives the port number)
	VIDEO_STREAM_TYPE_TCP_MPEG     = 2 // Stream is MPEG on TCP
	VIDEO_STREAM_TYPE_MPEG_TS_H264 = 3 // Stream is h.264 on MPEG TS (URI gives the port number)
)

// CameraZoomType: Zoom types for MAV_CMD_SET_CAMERA_ZOOM
const (
	ZOOM_TYPE_STEP       = 0 // Zoom one step increment (-1 for wide, 1 for tele)
	ZOOM_TYPE_CONTINUOUS = 1 // Continuous zoom up/down until stopped (-1 for wide, 1 for tele, 0 to stop zooming)
	ZOOM_TYPE_RANGE      = 2 // Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
)

// SetFocusType: Focus types for MAV_CMD_SET_CAMERA_FOCUS
const (
	FOCUS_TYPE_STEP       = 0 // Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
	FOCUS_TYPE_CONTINUOUS = 1 // Continuous focus up/down until stopped (-1 for focusing in, 1 for focusing out towards infinity, 0 to stop focusing)
	FOCUS_TYPE_RANGE      = 2 // Zoom value as proportion of full camera range (a value between 0.0 and 100.0)
)

// ParamAck: Result from a PARAM_EXT_SET message.
const (
	PARAM_ACK_ACCEPTED          = 0 // Parameter value ACCEPTED and SET
	PARAM_ACK_VALUE_UNSUPPORTED = 1 // Parameter value UNKNOWN/UNSUPPORTED
	PARAM_ACK_FAILED            = 2 // Parameter failed to set
	PARAM_ACK_IN_PROGRESS       = 3 // Parameter value received but not yet validated or set. A subsequent PARAM_EXT_ACK will follow once operation is completed with the actual result. These are for parameters that may take longer to set. Instead of waiting for an ACK and potentially timing out, you will immediately receive this response to let you know it was received.
)

// CameraMode: Camera Modes.
const (
	CAMERA_MODE_IMAGE        = 0 // Camera is in image/photo capture mode.
	CAMERA_MODE_VIDEO        = 1 // Camera is in video capture mode.
	CAMERA_MODE_IMAGE_SURVEY = 2 // Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.
)

// MavArmAuthDeniedReason:
const (
	MAV_ARM_AUTH_DENIED_REASON_GENERIC          = 0 // Not a specific reason
	MAV_ARM_AUTH_DENIED_REASON_NONE             = 1 // Authorizer will send the error as string to GCS
	MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2 // At least one waypoint have a invalid value
	MAV_ARM_AUTH_DENIED_REASON_TIMEOUT          = 3 // Timeout in the authorizer process(in case it depends on network)
	MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE  = 4 // Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.
	MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER      = 5 // Weather is not good to fly
)

// RcType: RC type
const (
	RC_TYPE_SPEKTRUM_DSM2 = 0 // Spektrum DSM2
	RC_TYPE_SPEKTRUM_DSMX = 1 // Spektrum DSMX
)

// PositionTargetTypemask: Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration.
const (
	POSITION_TARGET_TYPEMASK_X_IGNORE        = 1    // Ignore position x
	POSITION_TARGET_TYPEMASK_Y_IGNORE        = 2    // Ignore position y
	POSITION_TARGET_TYPEMASK_Z_IGNORE        = 4    // Ignore position z
	POSITION_TARGET_TYPEMASK_VX_IGNORE       = 8    // Ignore velocity x
	POSITION_TARGET_TYPEMASK_VY_IGNORE       = 16   // Ignore velocity y
	POSITION_TARGET_TYPEMASK_VZ_IGNORE       = 32   // Ignore velocity z
	POSITION_TARGET_TYPEMASK_AX_IGNORE       = 64   // Ignore acceleration x
	POSITION_TARGET_TYPEMASK_AY_IGNORE       = 128  // Ignore acceleration y
	POSITION_TARGET_TYPEMASK_AZ_IGNORE       = 256  // Ignore acceleration z
	POSITION_TARGET_TYPEMASK_FORCE_SET       = 512  // Use force instead of acceleration
	POSITION_TARGET_TYPEMASK_YAW_IGNORE      = 1024 // Ignore yaw
	POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048 // Ignore yaw rate
)

// UtmFlightState: Airborne status of UAS.
const (
	UTM_FLIGHT_STATE_UNKNOWN   = 1  // The flight state can't be determined.
	UTM_FLIGHT_STATE_GROUND    = 2  // UAS on ground.
	UTM_FLIGHT_STATE_AIRBORNE  = 3  // UAS airborne.
	UTM_FLIGHT_STATE_EMERGENCY = 16 // UAS is in an emergency flight state.
	UTM_FLIGHT_STATE_NOCTRL    = 32 // UAS has no active controls.
)

// UtmDataAvailFlags: Flags for the global position report.
const (
	UTM_DATA_AVAIL_FLAGS_TIME_VALID                  = 1   // The field time contains valid data.
	UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE            = 2   // The field uas_id contains valid data.
	UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE          = 4   // The fields lat, lon and h_acc contain valid data.
	UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE          = 8   // The fields alt and v_acc contain valid data.
	UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE = 16  // The field relative_alt contains valid data.
	UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE   = 32  // The fields vx and vy contain valid data.
	UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE     = 64  // The field vz contains valid data.
	UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE     = 128 // The fields next_lat, next_lon and next_alt contain valid data.
)

// CellularNetworkRadioType: Cellular network radio type
const (
	CELLULAR_NETWORK_RADIO_TYPE_NONE  = 0 //
	CELLULAR_NETWORK_RADIO_TYPE_GSM   = 1 //
	CELLULAR_NETWORK_RADIO_TYPE_CDMA  = 2 //
	CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3 //
	CELLULAR_NETWORK_RADIO_TYPE_LTE   = 4 //
)

// CellularNetworkStatusFlag: These flags encode the cellular network status
const (
	CELLULAR_NETWORK_STATUS_FLAG_ROAMING = 1 // Roaming is active
)

// PrecisionLandMode: Precision land modes (used in MAV_CMD_NAV_LAND).
const (
	PRECISION_LAND_MODE_DISABLED      = 0 // Normal (non-precision) landing.
	PRECISION_LAND_MODE_OPPORTUNISTIC = 1 // Use precision landing if beacon detected when land command accepted, otherwise land normally.
	PRECISION_LAND_MODE_REQUIRED      = 2 // Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found).
)

// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot).
type Heartbeat struct {
	CustomMode     uint32 `json:"customMode"`     // A bitfield for use for autopilot-specific flags
	Type           uint8  `json:"type"`           // Type of the system (quadrotor, helicopter, etc.). Components use the same type as their associated system.
	Autopilot      uint8  `json:"autopilot"`      // Autopilot type / class.
	BaseMode       uint8  `json:"baseMode"`       // System mode bitmap.
	SystemStatus   uint8  `json:"systemStatus"`   // System status flag.
	MavlinkVersion uint8  `json:"mavlinkVersion"` // MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
}

func (self *Heartbeat) MsgID() MessageID {
	return MSG_ID_HEARTBEAT
}

func (self *Heartbeat) MsgName() string {
	return "Heartbeat"
}

func (self *Heartbeat) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.CustomMode))
	payload[4] = byte(self.Type)
	payload[5] = byte(self.Autopilot)
	payload[6] = byte(self.BaseMode)
	payload[7] = byte(self.SystemStatus)
	payload[8] = byte(self.MavlinkVersion)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Heartbeat) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	self.CustomMode = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Type = uint8(p.Payload[4])
	self.Autopilot = uint8(p.Payload[5])
	self.BaseMode = uint8(p.Payload[6])
	self.SystemStatus = uint8(p.Payload[7])
	self.MavlinkVersion = uint8(p.Payload[8])
	return nil
}

func (self *Heartbeat) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HeartbeatFromJSON(data []byte) (*Heartbeat, error) {
	p := &Heartbeat{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
type SysStatus struct {
	OnboardControlSensorsPresent uint32 `json:"onboardControlSensorsPresent"` // Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
	OnboardControlSensorsEnabled uint32 `json:"onboardControlSensorsEnabled"` // Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
	OnboardControlSensorsHealth  uint32 `json:"onboardControlSensorsHealth"`  // Bitmap showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled.
	Load                         uint16 `json:"load"`                         // Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
	VoltageBattery               uint16 `json:"voltageBattery"`               // Battery voltage
	CurrentBattery               int16  `json:"currentBattery"`               // Battery current, -1: autopilot does not measure the current
	DropRateComm                 uint16 `json:"dropRateComm"`                 // Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsComm                   uint16 `json:"errorsComm"`                   // Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	ErrorsCount1                 uint16 `json:"errorsCount1"`                 // Autopilot-specific errors
	ErrorsCount2                 uint16 `json:"errorsCount2"`                 // Autopilot-specific errors
	ErrorsCount3                 uint16 `json:"errorsCount3"`                 // Autopilot-specific errors
	ErrorsCount4                 uint16 `json:"errorsCount4"`                 // Autopilot-specific errors
	BatteryRemaining             int8   `json:"batteryRemaining"`             // Remaining battery energy, -1: autopilot estimate the remaining battery
}

func (self *SysStatus) MsgID() MessageID {
	return MSG_ID_SYS_STATUS
}

func (self *SysStatus) MsgName() string {
	return "SysStatus"
}

func (self *SysStatus) Pack(p *Packet) error {
	payload := make([]byte, 31)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.OnboardControlSensorsPresent))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.OnboardControlSensorsEnabled))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.OnboardControlSensorsHealth))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Load))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.VoltageBattery))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.CurrentBattery))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.DropRateComm))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.ErrorsComm))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.ErrorsCount1))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.ErrorsCount2))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.ErrorsCount3))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.ErrorsCount4))
	payload[30] = byte(self.BatteryRemaining)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SysStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 31 {
		return fmt.Errorf("payload too small")
	}
	self.OnboardControlSensorsPresent = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.OnboardControlSensorsEnabled = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.OnboardControlSensorsHealth = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Load = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.VoltageBattery = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.DropRateComm = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.ErrorsComm = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.ErrorsCount1 = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.ErrorsCount2 = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.ErrorsCount3 = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.ErrorsCount4 = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.BatteryRemaining = int8(p.Payload[30])
	return nil
}

func (self *SysStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SysStatusFromJSON(data []byte) (*SysStatus, error) {
	p := &SysStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The system time is the time of the master clock, typically the computer clock of the main onboard computer.
type SystemTime struct {
	TimeUnixUsec uint64 `json:"timeUnixUsec"` // Timestamp (UNIX epoch time).
	TimeBootMs   uint32 `json:"timeBootMs"`   // Timestamp (time since system boot).
}

func (self *SystemTime) MsgID() MessageID {
	return MSG_ID_SYSTEM_TIME
}

func (self *SystemTime) MsgName() string {
	return "SystemTime"
}

func (self *SystemTime) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUnixUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.TimeBootMs))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SystemTime) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUnixUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

func (self *SystemTime) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SystemTimeFromJSON(data []byte) (*SystemTime, error) {
	p := &SystemTime{}
	err := json.Unmarshal(data, p)
	return p, err
}

// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections.
type Ping struct {
	TimeUsec        uint64 `json:"timeUsec"`        // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Seq             uint32 `json:"seq"`             // PING sequence
	TargetSystem    uint8  `json:"targetSystem"`    // 0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system
	TargetComponent uint8  `json:"targetComponent"` // 0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.
}

func (self *Ping) MsgID() MessageID {
	return MSG_ID_PING
}

func (self *Ping) MsgName() string {
	return "Ping"
}

func (self *Ping) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Seq))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ping) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Seq = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	return nil
}

func (self *Ping) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func PingFromJSON(data []byte) (*Ping, error) {
	p := &Ping{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request to control this MAV
type ChangeOperatorControl struct {
	TargetSystem   uint8    `json:"targetSystem"`   // System the GCS requests control for
	ControlRequest uint8    `json:"controlRequest"` // 0: request control of this MAV, 1: Release control of this MAV
	Version        uint8    `json:"version"`        // 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
	Passkey        [25]byte `json:"passkey"`        // Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
}

func (self *ChangeOperatorControl) MsgID() MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL
}

func (self *ChangeOperatorControl) MsgName() string {
	return "ChangeOperatorControl"
}

func (self *ChangeOperatorControl) Pack(p *Packet) error {
	payload := make([]byte, 28)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.ControlRequest)
	payload[2] = byte(self.Version)
	copy(payload[3:], self.Passkey[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ChangeOperatorControl) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.ControlRequest = uint8(p.Payload[1])
	self.Version = uint8(p.Payload[2])
	copy(self.Passkey[:], p.Payload[3:28])
	return nil
}

func (self *ChangeOperatorControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ChangeOperatorControlFromJSON(data []byte) (*ChangeOperatorControl, error) {
	p := &ChangeOperatorControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Accept / deny control of this MAV
type ChangeOperatorControlAck struct {
	GcsSystemId    uint8 `json:"gcsSystemId"`    // ID of the GCS this message
	ControlRequest uint8 `json:"controlRequest"` // 0: request control of this MAV, 1: Release control of this MAV
	Ack            uint8 `json:"ack"`            // 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
}

func (self *ChangeOperatorControlAck) MsgID() MessageID {
	return MSG_ID_CHANGE_OPERATOR_CONTROL_ACK
}

func (self *ChangeOperatorControlAck) MsgName() string {
	return "ChangeOperatorControlAck"
}

func (self *ChangeOperatorControlAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.GcsSystemId)
	payload[1] = byte(self.ControlRequest)
	payload[2] = byte(self.Ack)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ChangeOperatorControlAck) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.GcsSystemId = uint8(p.Payload[0])
	self.ControlRequest = uint8(p.Payload[1])
	self.Ack = uint8(p.Payload[2])
	return nil
}

func (self *ChangeOperatorControlAck) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ChangeOperatorControlAckFromJSON(data []byte) (*ChangeOperatorControlAck, error) {
	p := &ChangeOperatorControlAck{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
type AuthKey struct {
	Key [32]byte `json:"key"` // key
}

func (self *AuthKey) MsgID() MessageID {
	return MSG_ID_AUTH_KEY
}

func (self *AuthKey) MsgName() string {
	return "AuthKey"
}

func (self *AuthKey) Pack(p *Packet) error {
	payload := make([]byte, 32)
	copy(payload[0:], self.Key[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AuthKey) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	copy(self.Key[:], p.Payload[0:32])
	return nil
}

func (self *AuthKey) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AuthKeyFromJSON(data []byte) (*AuthKey, error) {
	p := &AuthKey{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
type SetMode struct {
	CustomMode   uint32 `json:"customMode"`   // The new autopilot-specific mode. This field can be ignored by an autopilot.
	TargetSystem uint8  `json:"targetSystem"` // The system setting the mode
	BaseMode     uint8  `json:"baseMode"`     // The new base mode.
}

func (self *SetMode) MsgID() MessageID {
	return MSG_ID_SET_MODE
}

func (self *SetMode) MsgName() string {
	return "SetMode"
}

func (self *SetMode) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.CustomMode))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.BaseMode)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetMode) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.CustomMode = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.BaseMode = uint8(p.Payload[5])
	return nil
}

func (self *SetMode) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetModeFromJSON(data []byte) (*SetMode, error) {
	p := &SetMode{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code.
type ParamRequestRead struct {
	ParamIndex      int16    `json:"paramIndex"`      // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
	TargetSystem    uint8    `json:"targetSystem"`    // System ID
	TargetComponent uint8    `json:"targetComponent"` // Component ID
	ParamId         [16]byte `json:"paramId"`         // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
}

func (self *ParamRequestRead) MsgID() MessageID {
	return MSG_ID_PARAM_REQUEST_READ
}

func (self *ParamRequestRead) MsgName() string {
	return "ParamRequestRead"
}

func (self *ParamRequestRead) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.ParamIndex))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)
	copy(payload[4:], self.ParamId[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ParamRequestRead) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.ParamIndex = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	copy(self.ParamId[:], p.Payload[4:20])
	return nil
}

func (self *ParamRequestRead) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ParamRequestReadFromJSON(data []byte) (*ParamRequestRead, error) {
	p := &ParamRequestRead{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request all parameters of this component. After this request, all parameters are emitted.
type ParamRequestList struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *ParamRequestList) MsgID() MessageID {
	return MSG_ID_PARAM_REQUEST_LIST
}

func (self *ParamRequestList) MsgName() string {
	return "ParamRequestList"
}

func (self *ParamRequestList) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ParamRequestList) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

func (self *ParamRequestList) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ParamRequestListFromJSON(data []byte) (*ParamRequestList, error) {
	p := &ParamRequestList{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout.
type ParamValue struct {
	ParamValue float32  `json:"paramValue"` // Onboard parameter value
	ParamCount uint16   `json:"paramCount"` // Total number of onboard parameters
	ParamIndex uint16   `json:"paramIndex"` // Index of this onboard parameter
	ParamId    [16]byte `json:"paramId"`    // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType  uint8    `json:"paramType"`  // Onboard parameter type.
}

func (self *ParamValue) MsgID() MessageID {
	return MSG_ID_PARAM_VALUE
}

func (self *ParamValue) MsgName() string {
	return "ParamValue"
}

func (self *ParamValue) Pack(p *Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ParamValue))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.ParamCount))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.ParamIndex))
	copy(payload[8:], self.ParamId[:])
	payload[24] = byte(self.ParamType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ParamValue) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	self.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.ParamCount = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.ParamIndex = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	copy(self.ParamId[:], p.Payload[8:24])
	self.ParamType = uint8(p.Payload[24])
	return nil
}

func (self *ParamValue) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ParamValueFromJSON(data []byte) (*ParamValue, error) {
	p := &ParamValue{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set a parameter value (write new value to permanent storage). IMPORTANT: The receiving component should acknowledge the new parameter value by sending a PARAM_VALUE message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message.
type ParamSet struct {
	ParamValue      float32  `json:"paramValue"`      // Onboard parameter value
	TargetSystem    uint8    `json:"targetSystem"`    // System ID
	TargetComponent uint8    `json:"targetComponent"` // Component ID
	ParamId         [16]byte `json:"paramId"`         // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParamType       uint8    `json:"paramType"`       // Onboard parameter type.
}

func (self *ParamSet) MsgID() MessageID {
	return MSG_ID_PARAM_SET
}

func (self *ParamSet) MsgName() string {
	return "ParamSet"
}

func (self *ParamSet) Pack(p *Packet) error {
	payload := make([]byte, 23)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ParamValue))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)
	copy(payload[6:], self.ParamId[:])
	payload[22] = byte(self.ParamType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ParamSet) Unpack(p *Packet) error {
	if len(p.Payload) < 23 {
		return fmt.Errorf("payload too small")
	}
	self.ParamValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	copy(self.ParamId[:], p.Payload[6:22])
	self.ParamType = uint8(p.Payload[22])
	return nil
}

func (self *ParamSet) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ParamSetFromJSON(data []byte) (*ParamSet, error) {
	p := &ParamSet{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type GpsRawInt struct {
	TimeUsec          uint64 `json:"timeUsec"`          // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Lat               int32  `json:"lat"`               // Latitude (WGS84, EGM96 ellipsoid)
	Lon               int32  `json:"lon"`               // Longitude (WGS84, EGM96 ellipsoid)
	Alt               int32  `json:"alt"`               // Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
	Eph               uint16 `json:"eph"`               // GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
	Epv               uint16 `json:"epv"`               // GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
	Vel               uint16 `json:"vel"`               // GPS ground speed. If unknown, set to: UINT16_MAX
	Cog               uint16 `json:"cog"`               // Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           uint8  `json:"fixType"`           // GPS fix type.
	SatellitesVisible uint8  `json:"satellitesVisible"` // Number of satellites visible. If unknown, set to 255
}

func (self *GpsRawInt) MsgID() MessageID {
	return MSG_ID_GPS_RAW_INT
}

func (self *GpsRawInt) MsgName() string {
	return "GpsRawInt"
}

func (self *GpsRawInt) Pack(p *Packet) error {
	payload := make([]byte, 30)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Alt))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Eph))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Epv))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Vel))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Cog))
	payload[28] = byte(self.FixType)
	payload[29] = byte(self.SatellitesVisible)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsRawInt) Unpack(p *Packet) error {
	if len(p.Payload) < 30 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Eph = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Epv = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Vel = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Cog = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.FixType = uint8(p.Payload[28])
	self.SatellitesVisible = uint8(p.Payload[29])
	return nil
}

func (self *GpsRawInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsRawIntFromJSON(data []byte) (*GpsRawInt, error) {
	p := &GpsRawInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites.
type GpsStatus struct {
	SatellitesVisible  uint8     `json:"satellitesVisible"`  // Number of satellites visible
	SatellitePrn       [20]uint8 `json:"satellitePrn"`       // Global satellite ID
	SatelliteUsed      [20]uint8 `json:"satelliteUsed"`      // 0: Satellite not used, 1: used for localization
	SatelliteElevation [20]uint8 `json:"satelliteElevation"` // Elevation (0: right on top of receiver, 90: on the horizon) of satellite
	SatelliteAzimuth   [20]uint8 `json:"satelliteAzimuth"`   // Direction of satellite, 0: 0 deg, 255: 360 deg.
	SatelliteSnr       [20]uint8 `json:"satelliteSnr"`       // Signal to noise ratio of satellite
}

func (self *GpsStatus) MsgID() MessageID {
	return MSG_ID_GPS_STATUS
}

func (self *GpsStatus) MsgName() string {
	return "GpsStatus"
}

func (self *GpsStatus) Pack(p *Packet) error {
	payload := make([]byte, 101)
	payload[0] = byte(self.SatellitesVisible)
	copy(payload[1:], self.SatellitePrn[:])
	copy(payload[21:], self.SatelliteUsed[:])
	copy(payload[41:], self.SatelliteElevation[:])
	copy(payload[61:], self.SatelliteAzimuth[:])
	copy(payload[81:], self.SatelliteSnr[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 101 {
		return fmt.Errorf("payload too small")
	}
	self.SatellitesVisible = uint8(p.Payload[0])
	copy(self.SatellitePrn[:], p.Payload[1:21])
	copy(self.SatelliteUsed[:], p.Payload[21:41])
	copy(self.SatelliteElevation[:], p.Payload[41:61])
	copy(self.SatelliteAzimuth[:], p.Payload[61:81])
	copy(self.SatelliteSnr[:], p.Payload[81:101])
	return nil
}

func (self *GpsStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsStatusFromJSON(data []byte) (*GpsStatus, error) {
	p := &GpsStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu struct {
	TimeBootMs uint32 `json:"timeBootMs"` // Timestamp (time since system boot).
	Xacc       int16  `json:"xacc"`       // X acceleration
	Yacc       int16  `json:"yacc"`       // Y acceleration
	Zacc       int16  `json:"zacc"`       // Z acceleration
	Xgyro      int16  `json:"xgyro"`      // Angular speed around X axis
	Ygyro      int16  `json:"ygyro"`      // Angular speed around Y axis
	Zgyro      int16  `json:"zgyro"`      // Angular speed around Z axis
	Xmag       int16  `json:"xmag"`       // X Magnetic field
	Ymag       int16  `json:"ymag"`       // Y Magnetic field
	Zmag       int16  `json:"zmag"`       // Z Magnetic field
}

func (self *ScaledImu) MsgID() MessageID {
	return MSG_ID_SCALED_IMU
}

func (self *ScaledImu) MsgName() string {
	return "ScaledImu"
}

func (self *ScaledImu) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Xacc))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Yacc))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Zacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Xgyro))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Ygyro))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Zgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Xmag))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Ymag))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Zmag))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ScaledImu) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

func (self *ScaledImu) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ScaledImuFromJSON(data []byte) (*ScaledImu, error) {
	p := &ScaledImu{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging.
type RawImu struct {
	TimeUsec uint64 `json:"timeUsec"` // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Xacc     int16  `json:"xacc"`     // X acceleration (raw)
	Yacc     int16  `json:"yacc"`     // Y acceleration (raw)
	Zacc     int16  `json:"zacc"`     // Z acceleration (raw)
	Xgyro    int16  `json:"xgyro"`    // Angular speed around X axis (raw)
	Ygyro    int16  `json:"ygyro"`    // Angular speed around Y axis (raw)
	Zgyro    int16  `json:"zgyro"`    // Angular speed around Z axis (raw)
	Xmag     int16  `json:"xmag"`     // X Magnetic field (raw)
	Ymag     int16  `json:"ymag"`     // Y Magnetic field (raw)
	Zmag     int16  `json:"zmag"`     // Z Magnetic field (raw)
}

func (self *RawImu) MsgID() MessageID {
	return MSG_ID_RAW_IMU
}

func (self *RawImu) MsgName() string {
	return "RawImu"
}

func (self *RawImu) Pack(p *Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Xacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Yacc))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Zacc))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Xgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Ygyro))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Zgyro))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Xmag))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Ymag))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Zmag))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RawImu) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[24:]))
	return nil
}

func (self *RawImu) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RawImuFromJSON(data []byte) (*RawImu, error) {
	p := &RawImu{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
type RawPressure struct {
	TimeUsec    uint64 `json:"timeUsec"`    // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	PressAbs    int16  `json:"pressAbs"`    // Absolute pressure (raw)
	PressDiff1  int16  `json:"pressDiff1"`  // Differential pressure 1 (raw, 0 if nonexistent)
	PressDiff2  int16  `json:"pressDiff2"`  // Differential pressure 2 (raw, 0 if nonexistent)
	Temperature int16  `json:"temperature"` // Raw Temperature measurement (raw)
}

func (self *RawPressure) MsgID() MessageID {
	return MSG_ID_RAW_PRESSURE
}

func (self *RawPressure) MsgName() string {
	return "RawPressure"
}

func (self *RawPressure) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.PressAbs))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.PressDiff1))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.PressDiff2))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Temperature))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RawPressure) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.PressAbs = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.PressDiff1 = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.PressDiff2 = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	return nil
}

func (self *RawPressure) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RawPressureFromJSON(data []byte) (*RawPressure, error) {
	p := &RawPressure{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
type ScaledPressure struct {
	TimeBootMs  uint32  `json:"timeBootMs"`  // Timestamp (time since system boot).
	PressAbs    float32 `json:"pressAbs"`    // Absolute pressure
	PressDiff   float32 `json:"pressDiff"`   // Differential pressure 1
	Temperature int16   `json:"temperature"` // Temperature
}

func (self *ScaledPressure) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE
}

func (self *ScaledPressure) MsgName() string {
	return "ScaledPressure"
}

func (self *ScaledPressure) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.PressAbs))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.PressDiff))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Temperature))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ScaledPressure) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

func (self *ScaledPressure) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ScaledPressureFromJSON(data []byte) (*ScaledPressure, error) {
	p := &ScaledPressure{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right).
type Attitude struct {
	TimeBootMs uint32  `json:"timeBootMs"` // Timestamp (time since system boot).
	Roll       float32 `json:"roll"`       // Roll angle (-pi..+pi)
	Pitch      float32 `json:"pitch"`      // Pitch angle (-pi..+pi)
	Yaw        float32 `json:"yaw"`        // Yaw angle (-pi..+pi)
	Rollspeed  float32 `json:"rollspeed"`  // Roll angular speed
	Pitchspeed float32 `json:"pitchspeed"` // Pitch angular speed
	Yawspeed   float32 `json:"yawspeed"`   // Yaw angular speed
}

func (self *Attitude) MsgID() MessageID {
	return MSG_ID_ATTITUDE
}

func (self *Attitude) MsgName() string {
	return "Attitude"
}

func (self *Attitude) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Rollspeed))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Yawspeed))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Attitude) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

func (self *Attitude) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AttitudeFromJSON(data []byte) (*Attitude, error) {
	p := &Attitude{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternion struct {
	TimeBootMs uint32  `json:"timeBootMs"` // Timestamp (time since system boot).
	Q1         float32 `json:"q1"`         // Quaternion component 1, w (1 in null-rotation)
	Q2         float32 `json:"q2"`         // Quaternion component 2, x (0 in null-rotation)
	Q3         float32 `json:"q3"`         // Quaternion component 3, y (0 in null-rotation)
	Q4         float32 `json:"q4"`         // Quaternion component 4, z (0 in null-rotation)
	Rollspeed  float32 `json:"rollspeed"`  // Roll angular speed
	Pitchspeed float32 `json:"pitchspeed"` // Pitch angular speed
	Yawspeed   float32 `json:"yawspeed"`   // Yaw angular speed
}

func (self *AttitudeQuaternion) MsgID() MessageID {
	return MSG_ID_ATTITUDE_QUATERNION
}

func (self *AttitudeQuaternion) MsgName() string {
	return "AttitudeQuaternion"
}

func (self *AttitudeQuaternion) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Q1))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Q2))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Q3))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Q4))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Rollspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Yawspeed))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AttitudeQuaternion) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Q1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Q2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Q3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Q4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

func (self *AttitudeQuaternion) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AttitudeQuaternionFromJSON(data []byte) (*AttitudeQuaternion, error) {
	p := &AttitudeQuaternion{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNed struct {
	TimeBootMs uint32  `json:"timeBootMs"` // Timestamp (time since system boot).
	X          float32 `json:"x"`          // X Position
	Y          float32 `json:"y"`          // Y Position
	Z          float32 `json:"z"`          // Z Position
	Vx         float32 `json:"vx"`         // X Speed
	Vy         float32 `json:"vy"`         // Y Speed
	Vz         float32 `json:"vz"`         // Z Speed
}

func (self *LocalPositionNed) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED
}

func (self *LocalPositionNed) MsgName() string {
	return "LocalPositionNed"
}

func (self *LocalPositionNed) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vz))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LocalPositionNed) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

func (self *LocalPositionNed) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LocalPositionNedFromJSON(data []byte) (*LocalPositionNed, error) {
	p := &LocalPositionNed{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
//                is designed as scaled integer message since the resolution of float is not sufficient.
type GlobalPositionInt struct {
	TimeBootMs  uint32 `json:"timeBootMs"`  // Timestamp (time since system boot).
	Lat         int32  `json:"lat"`         // Latitude, expressed
	Lon         int32  `json:"lon"`         // Longitude, expressed
	Alt         int32  `json:"alt"`         // Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
	RelativeAlt int32  `json:"relativeAlt"` // Altitude above ground
	Vx          int16  `json:"vx"`          // Ground X Speed (Latitude, positive north)
	Vy          int16  `json:"vy"`          // Ground Y Speed (Longitude, positive east)
	Vz          int16  `json:"vz"`          // Ground Z Speed (Altitude, positive down)
	Hdg         uint16 `json:"hdg"`         // Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
}

func (self *GlobalPositionInt) MsgID() MessageID {
	return MSG_ID_GLOBAL_POSITION_INT
}

func (self *GlobalPositionInt) MsgName() string {
	return "GlobalPositionInt"
}

func (self *GlobalPositionInt) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Alt))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.RelativeAlt))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Vx))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Vy))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Vz))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Hdg))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GlobalPositionInt) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.RelativeAlt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vx = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Vy = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Vz = int16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Hdg = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	return nil
}

func (self *GlobalPositionInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GlobalPositionIntFromJSON(data []byte) (*GlobalPositionInt, error) {
	p := &GlobalPositionInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX.
type RcChannelsScaled struct {
	TimeBootMs  uint32 `json:"timeBootMs"`  // Timestamp (time since system boot).
	Chan1Scaled int16  `json:"chan1Scaled"` // RC channel 1 value scaled.
	Chan2Scaled int16  `json:"chan2Scaled"` // RC channel 2 value scaled.
	Chan3Scaled int16  `json:"chan3Scaled"` // RC channel 3 value scaled.
	Chan4Scaled int16  `json:"chan4Scaled"` // RC channel 4 value scaled.
	Chan5Scaled int16  `json:"chan5Scaled"` // RC channel 5 value scaled.
	Chan6Scaled int16  `json:"chan6Scaled"` // RC channel 6 value scaled.
	Chan7Scaled int16  `json:"chan7Scaled"` // RC channel 7 value scaled.
	Chan8Scaled int16  `json:"chan8Scaled"` // RC channel 8 value scaled.
	Port        uint8  `json:"port"`        // Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
	Rssi        uint8  `json:"rssi"`        // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

func (self *RcChannelsScaled) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_SCALED
}

func (self *RcChannelsScaled) MsgName() string {
	return "RcChannelsScaled"
}

func (self *RcChannelsScaled) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Chan1Scaled))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Chan2Scaled))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Chan3Scaled))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Chan4Scaled))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Chan5Scaled))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Chan6Scaled))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Chan7Scaled))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Chan8Scaled))
	payload[20] = byte(self.Port)
	payload[21] = byte(self.Rssi)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RcChannelsScaled) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Chan1Scaled = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Chan2Scaled = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Chan3Scaled = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Chan4Scaled = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Chan5Scaled = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Chan6Scaled = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Chan7Scaled = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Chan8Scaled = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Port = uint8(p.Payload[20])
	self.Rssi = uint8(p.Payload[21])
	return nil
}

func (self *RcChannelsScaled) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RcChannelsScaledFromJSON(data []byte) (*RcChannelsScaled, error) {
	p := &RcChannelsScaled{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
type RcChannelsRaw struct {
	TimeBootMs uint32 `json:"timeBootMs"` // Timestamp (time since system boot).
	Chan1Raw   uint16 `json:"chan1Raw"`   // RC channel 1 value.
	Chan2Raw   uint16 `json:"chan2Raw"`   // RC channel 2 value.
	Chan3Raw   uint16 `json:"chan3Raw"`   // RC channel 3 value.
	Chan4Raw   uint16 `json:"chan4Raw"`   // RC channel 4 value.
	Chan5Raw   uint16 `json:"chan5Raw"`   // RC channel 5 value.
	Chan6Raw   uint16 `json:"chan6Raw"`   // RC channel 6 value.
	Chan7Raw   uint16 `json:"chan7Raw"`   // RC channel 7 value.
	Chan8Raw   uint16 `json:"chan8Raw"`   // RC channel 8 value.
	Port       uint8  `json:"port"`       // Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
	Rssi       uint8  `json:"rssi"`       // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

func (self *RcChannelsRaw) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_RAW
}

func (self *RcChannelsRaw) MsgName() string {
	return "RcChannelsRaw"
}

func (self *RcChannelsRaw) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Chan8Raw))
	payload[20] = byte(self.Port)
	payload[21] = byte(self.Rssi)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RcChannelsRaw) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Port = uint8(p.Payload[20])
	self.Rssi = uint8(p.Payload[21])
	return nil
}

func (self *RcChannelsRaw) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RcChannelsRawFromJSON(data []byte) (*RcChannelsRaw, error) {
	p := &RcChannelsRaw{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
type ServoOutputRaw struct {
	TimeUsec  uint32 `json:"timeUsec"`  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Servo1Raw uint16 `json:"servo1Raw"` // Servo output 1 value
	Servo2Raw uint16 `json:"servo2Raw"` // Servo output 2 value
	Servo3Raw uint16 `json:"servo3Raw"` // Servo output 3 value
	Servo4Raw uint16 `json:"servo4Raw"` // Servo output 4 value
	Servo5Raw uint16 `json:"servo5Raw"` // Servo output 5 value
	Servo6Raw uint16 `json:"servo6Raw"` // Servo output 6 value
	Servo7Raw uint16 `json:"servo7Raw"` // Servo output 7 value
	Servo8Raw uint16 `json:"servo8Raw"` // Servo output 8 value
	Port      uint8  `json:"port"`      // Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
}

func (self *ServoOutputRaw) MsgID() MessageID {
	return MSG_ID_SERVO_OUTPUT_RAW
}

func (self *ServoOutputRaw) MsgName() string {
	return "ServoOutputRaw"
}

func (self *ServoOutputRaw) Pack(p *Packet) error {
	payload := make([]byte, 21)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeUsec))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Servo1Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Servo2Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Servo3Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Servo4Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Servo5Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Servo6Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Servo7Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Servo8Raw))
	payload[20] = byte(self.Port)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ServoOutputRaw) Unpack(p *Packet) error {
	if len(p.Payload) < 21 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Servo1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Servo2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Servo3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Servo4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Servo5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Servo6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Servo7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Servo8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Port = uint8(p.Payload[20])
	return nil
}

func (self *ServoOutputRaw) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ServoOutputRawFromJSON(data []byte) (*ServoOutputRaw, error) {
	p := &ServoOutputRaw{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one waypoint.
type MissionRequestPartialList struct {
	StartIndex      int16 `json:"startIndex"`      // Start index
	EndIndex        int16 `json:"endIndex"`        // End index, -1 by default (-1: send list to end). Else a valid index of the list
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *MissionRequestPartialList) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_PARTIAL_LIST
}

func (self *MissionRequestPartialList) MsgName() string {
	return "MissionRequestPartialList"
}

func (self *MissionRequestPartialList) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.StartIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.EndIndex))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionRequestPartialList) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.StartIndex = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.EndIndex = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

func (self *MissionRequestPartialList) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionRequestPartialListFromJSON(data []byte) (*MissionRequestPartialList, error) {
	p := &MissionRequestPartialList{}
	err := json.Unmarshal(data, p)
	return p, err
}

// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
type MissionWritePartialList struct {
	StartIndex      int16 `json:"startIndex"`      // Start index. Must be smaller / equal to the largest index of the current onboard list.
	EndIndex        int16 `json:"endIndex"`        // End index, equal or greater than start index.
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *MissionWritePartialList) MsgID() MessageID {
	return MSG_ID_MISSION_WRITE_PARTIAL_LIST
}

func (self *MissionWritePartialList) MsgName() string {
	return "MissionWritePartialList"
}

func (self *MissionWritePartialList) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.StartIndex))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.EndIndex))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionWritePartialList) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.StartIndex = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.EndIndex = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

func (self *MissionWritePartialList) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionWritePartialListFromJSON(data []byte) (*MissionWritePartialList, error) {
	p := &MissionWritePartialList{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/services/mission.html.
type MissionItem struct {
	Param1          float32 `json:"param1"`          // PARAM1, see MAV_CMD enum
	Param2          float32 `json:"param2"`          // PARAM2, see MAV_CMD enum
	Param3          float32 `json:"param3"`          // PARAM3, see MAV_CMD enum
	Param4          float32 `json:"param4"`          // PARAM4, see MAV_CMD enum
	X               float32 `json:"x"`               // PARAM5 / local: X coordinate, global: latitude
	Y               float32 `json:"y"`               // PARAM6 / local: Y coordinate, global: longitude
	Z               float32 `json:"z"`               // PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
	Seq             uint16  `json:"seq"`             // Sequence
	Command         uint16  `json:"command"`         // The scheduled action for the waypoint.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID
	TargetComponent uint8   `json:"targetComponent"` // Component ID
	Frame           uint8   `json:"frame"`           // The coordinate system of the waypoint.
	Current         uint8   `json:"current"`         // false:0, true:1
	Autocontinue    uint8   `json:"autocontinue"`    // Autocontinue to next waypoint
}

func (self *MissionItem) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM
}

func (self *MissionItem) MsgName() string {
	return "MissionItem"
}

func (self *MissionItem) Pack(p *Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Param4))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Seq))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Command))
	payload[32] = byte(self.TargetSystem)
	payload[33] = byte(self.TargetComponent)
	payload[34] = byte(self.Frame)
	payload[35] = byte(self.Current)
	payload[36] = byte(self.Autocontinue)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionItem) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	self.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Command = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.TargetSystem = uint8(p.Payload[32])
	self.TargetComponent = uint8(p.Payload[33])
	self.Frame = uint8(p.Payload[34])
	self.Current = uint8(p.Payload[35])
	self.Autocontinue = uint8(p.Payload[36])
	return nil
}

func (self *MissionItem) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionItemFromJSON(data []byte) (*MissionItem, error) {
	p := &MissionItem{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
type MissionRequest struct {
	Seq             uint16 `json:"seq"`             // Sequence
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *MissionRequest) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST
}

func (self *MissionRequest) MsgName() string {
	return "MissionRequest"
}

func (self *MissionRequest) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Seq))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	return nil
}

func (self *MissionRequest) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionRequestFromJSON(data []byte) (*MissionRequest, error) {
	p := &MissionRequest{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
type MissionSetCurrent struct {
	Seq             uint16 `json:"seq"`             // Sequence
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *MissionSetCurrent) MsgID() MessageID {
	return MSG_ID_MISSION_SET_CURRENT
}

func (self *MissionSetCurrent) MsgName() string {
	return "MissionSetCurrent"
}

func (self *MissionSetCurrent) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Seq))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionSetCurrent) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	return nil
}

func (self *MissionSetCurrent) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionSetCurrentFromJSON(data []byte) (*MissionSetCurrent, error) {
	p := &MissionSetCurrent{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
type MissionCurrent struct {
	Seq uint16 `json:"seq"` // Sequence
}

func (self *MissionCurrent) MsgID() MessageID {
	return MSG_ID_MISSION_CURRENT
}

func (self *MissionCurrent) MsgName() string {
	return "MissionCurrent"
}

func (self *MissionCurrent) Pack(p *Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Seq))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionCurrent) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	return nil
}

func (self *MissionCurrent) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionCurrentFromJSON(data []byte) (*MissionCurrent, error) {
	p := &MissionCurrent{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request the overall list of mission items from the system/component.
type MissionRequestList struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *MissionRequestList) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_LIST
}

func (self *MissionRequestList) MsgName() string {
	return "MissionRequestList"
}

func (self *MissionRequestList) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionRequestList) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

func (self *MissionRequestList) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionRequestListFromJSON(data []byte) (*MissionRequestList, error) {
	p := &MissionRequestList{}
	err := json.Unmarshal(data, p)
	return p, err
}

// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
type MissionCount struct {
	Count           uint16 `json:"count"`           // Number of mission items in the sequence
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *MissionCount) MsgID() MessageID {
	return MSG_ID_MISSION_COUNT
}

func (self *MissionCount) MsgName() string {
	return "MissionCount"
}

func (self *MissionCount) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Count))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionCount) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Count = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	return nil
}

func (self *MissionCount) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionCountFromJSON(data []byte) (*MissionCount, error) {
	p := &MissionCount{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Delete all mission items at once.
type MissionClearAll struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *MissionClearAll) MsgID() MessageID {
	return MSG_ID_MISSION_CLEAR_ALL
}

func (self *MissionClearAll) MsgName() string {
	return "MissionClearAll"
}

func (self *MissionClearAll) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionClearAll) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

func (self *MissionClearAll) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionClearAllFromJSON(data []byte) (*MissionClearAll, error) {
	p := &MissionClearAll{}
	err := json.Unmarshal(data, p)
	return p, err
}

// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
type MissionItemReached struct {
	Seq uint16 `json:"seq"` // Sequence
}

func (self *MissionItemReached) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM_REACHED
}

func (self *MissionItemReached) MsgName() string {
	return "MissionItemReached"
}

func (self *MissionItemReached) Pack(p *Packet) error {
	payload := make([]byte, 2)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Seq))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionItemReached) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	return nil
}

func (self *MissionItemReached) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionItemReachedFromJSON(data []byte) (*MissionItemReached, error) {
	p := &MissionItemReached{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
type MissionAck struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
	Type            uint8 `json:"type"`            // Mission result.
}

func (self *MissionAck) MsgID() MessageID {
	return MSG_ID_MISSION_ACK
}

func (self *MissionAck) MsgName() string {
	return "MissionAck"
}

func (self *MissionAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Type)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionAck) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Type = uint8(p.Payload[2])
	return nil
}

func (self *MissionAck) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionAckFromJSON(data []byte) (*MissionAck, error) {
	p := &MissionAck{}
	err := json.Unmarshal(data, p)
	return p, err
}

// As local waypoints exist, the global waypoint reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor.
type SetGpsGlobalOrigin struct {
	Latitude     int32 `json:"latitude"`     // Latitude (WGS84)
	Longitude    int32 `json:"longitude"`    // Longitude (WGS84)
	Altitude     int32 `json:"altitude"`     // Altitude (MSL). Positive for up.
	TargetSystem uint8 `json:"targetSystem"` // System ID
}

func (self *SetGpsGlobalOrigin) MsgID() MessageID {
	return MSG_ID_SET_GPS_GLOBAL_ORIGIN
}

func (self *SetGpsGlobalOrigin) MsgName() string {
	return "SetGpsGlobalOrigin"
}

func (self *SetGpsGlobalOrigin) Pack(p *Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Altitude))
	payload[12] = byte(self.TargetSystem)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetGpsGlobalOrigin) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	return nil
}

func (self *SetGpsGlobalOrigin) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetGpsGlobalOriginFromJSON(data []byte) (*SetGpsGlobalOrigin, error) {
	p := &SetGpsGlobalOrigin{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position
type GpsGlobalOrigin struct {
	Latitude  int32 `json:"latitude"`  // Latitude (WGS84)
	Longitude int32 `json:"longitude"` // Longitude (WGS84)
	Altitude  int32 `json:"altitude"`  // Altitude (MSL). Positive for up.
}

func (self *GpsGlobalOrigin) MsgID() MessageID {
	return MSG_ID_GPS_GLOBAL_ORIGIN
}

func (self *GpsGlobalOrigin) MsgName() string {
	return "GpsGlobalOrigin"
}

func (self *GpsGlobalOrigin) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Altitude))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsGlobalOrigin) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

func (self *GpsGlobalOrigin) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsGlobalOriginFromJSON(data []byte) (*GpsGlobalOrigin, error) {
	p := &GpsGlobalOrigin{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
type ParamMapRc struct {
	ParamValue0             float32  `json:"paramValue0"`             // Initial parameter value
	Scale                   float32  `json:"scale"`                   // Scale, maps the RC range [-1, 1] to a parameter value
	ParamValueMin           float32  `json:"paramValueMin"`           // Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
	ParamValueMax           float32  `json:"paramValueMax"`           // Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
	ParamIndex              int16    `json:"paramIndex"`              // Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
	TargetSystem            uint8    `json:"targetSystem"`            // System ID
	TargetComponent         uint8    `json:"targetComponent"`         // Component ID
	ParamId                 [16]byte `json:"paramId"`                 // Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
	ParameterRcChannelIndex uint8    `json:"parameterRcChannelIndex"` // Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a potentiometer-knob on the RC.
}

func (self *ParamMapRc) MsgID() MessageID {
	return MSG_ID_PARAM_MAP_RC
}

func (self *ParamMapRc) MsgName() string {
	return "ParamMapRc"
}

func (self *ParamMapRc) Pack(p *Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ParamValue0))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Scale))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.ParamValueMin))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.ParamValueMax))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.ParamIndex))
	payload[18] = byte(self.TargetSystem)
	payload[19] = byte(self.TargetComponent)
	copy(payload[20:], self.ParamId[:])
	payload[36] = byte(self.ParameterRcChannelIndex)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ParamMapRc) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	self.ParamValue0 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Scale = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.ParamValueMin = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.ParamValueMax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.ParamIndex = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.TargetSystem = uint8(p.Payload[18])
	self.TargetComponent = uint8(p.Payload[19])
	copy(self.ParamId[:], p.Payload[20:36])
	self.ParameterRcChannelIndex = uint8(p.Payload[36])
	return nil
}

func (self *ParamMapRc) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ParamMapRcFromJSON(data []byte) (*ParamMapRc, error) {
	p := &ParamMapRc{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
type MissionRequestInt struct {
	Seq             uint16 `json:"seq"`             // Sequence
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *MissionRequestInt) MsgID() MessageID {
	return MSG_ID_MISSION_REQUEST_INT
}

func (self *MissionRequestInt) MsgName() string {
	return "MissionRequestInt"
}

func (self *MissionRequestInt) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Seq))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionRequestInt) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	return nil
}

func (self *MissionRequestInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionRequestIntFromJSON(data []byte) (*MissionRequestInt, error) {
	p := &MissionRequestInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
type SafetySetAllowedArea struct {
	P1x             float32 `json:"p1x"`             // x position 1 / Latitude 1
	P1y             float32 `json:"p1y"`             // y position 1 / Longitude 1
	P1z             float32 `json:"p1z"`             // z position 1 / Altitude 1
	P2x             float32 `json:"p2x"`             // x position 2 / Latitude 2
	P2y             float32 `json:"p2y"`             // y position 2 / Longitude 2
	P2z             float32 `json:"p2z"`             // z position 2 / Altitude 2
	TargetSystem    uint8   `json:"targetSystem"`    // System ID
	TargetComponent uint8   `json:"targetComponent"` // Component ID
	Frame           uint8   `json:"frame"`           // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

func (self *SafetySetAllowedArea) MsgID() MessageID {
	return MSG_ID_SAFETY_SET_ALLOWED_AREA
}

func (self *SafetySetAllowedArea) MsgName() string {
	return "SafetySetAllowedArea"
}

func (self *SafetySetAllowedArea) Pack(p *Packet) error {
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.P1x))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.P1y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.P1z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P2x))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.P2y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.P2z))
	payload[24] = byte(self.TargetSystem)
	payload[25] = byte(self.TargetComponent)
	payload[26] = byte(self.Frame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SafetySetAllowedArea) Unpack(p *Packet) error {
	if len(p.Payload) < 27 {
		return fmt.Errorf("payload too small")
	}
	self.P1x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.P1y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.P1z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P2x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.P2y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.P2z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.TargetSystem = uint8(p.Payload[24])
	self.TargetComponent = uint8(p.Payload[25])
	self.Frame = uint8(p.Payload[26])
	return nil
}

func (self *SafetySetAllowedArea) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SafetySetAllowedAreaFromJSON(data []byte) (*SafetySetAllowedArea, error) {
	p := &SafetySetAllowedArea{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Read out the safety zone the MAV currently assumes.
type SafetyAllowedArea struct {
	P1x   float32 `json:"p1x"`   // x position 1 / Latitude 1
	P1y   float32 `json:"p1y"`   // y position 1 / Longitude 1
	P1z   float32 `json:"p1z"`   // z position 1 / Altitude 1
	P2x   float32 `json:"p2x"`   // x position 2 / Latitude 2
	P2y   float32 `json:"p2y"`   // y position 2 / Longitude 2
	P2z   float32 `json:"p2z"`   // z position 2 / Altitude 2
	Frame uint8   `json:"frame"` // Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
}

func (self *SafetyAllowedArea) MsgID() MessageID {
	return MSG_ID_SAFETY_ALLOWED_AREA
}

func (self *SafetyAllowedArea) MsgName() string {
	return "SafetyAllowedArea"
}

func (self *SafetyAllowedArea) Pack(p *Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.P1x))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.P1y))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.P1z))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P2x))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.P2y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.P2z))
	payload[24] = byte(self.Frame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SafetyAllowedArea) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	self.P1x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.P1y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.P1z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P2x = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.P2y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.P2z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Frame = uint8(p.Payload[24])
	return nil
}

func (self *SafetyAllowedArea) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SafetyAllowedAreaFromJSON(data []byte) (*SafetyAllowedArea, error) {
	p := &SafetyAllowedArea{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
type AttitudeQuaternionCov struct {
	TimeUsec   uint64     `json:"timeUsec"`   // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Q          [4]float32 `json:"q"`          // Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
	Rollspeed  float32    `json:"rollspeed"`  // Roll angular speed
	Pitchspeed float32    `json:"pitchspeed"` // Pitch angular speed
	Yawspeed   float32    `json:"yawspeed"`   // Yaw angular speed
	Covariance [9]float32 `json:"covariance"` // Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
}

func (self *AttitudeQuaternionCov) MsgID() MessageID {
	return MSG_ID_ATTITUDE_QUATERNION_COV
}

func (self *AttitudeQuaternionCov) MsgName() string {
	return "AttitudeQuaternionCov"
}

func (self *AttitudeQuaternionCov) Pack(p *Packet) error {
	payload := make([]byte, 72)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Rollspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Yawspeed))
	for i, v := range self.Covariance {
		binary.LittleEndian.PutUint32(payload[36+i*4:], math.Float32bits(v))
	}

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AttitudeQuaternionCov) Unpack(p *Packet) error {
	if len(p.Payload) < 72 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	self.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	for i := 0; i < len(self.Covariance); i++ {
		self.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36+i*4:]))
	}
	return nil
}

func (self *AttitudeQuaternionCov) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AttitudeQuaternionCovFromJSON(data []byte) (*AttitudeQuaternionCov, error) {
	p := &AttitudeQuaternionCov{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The state of the fixed wing navigation and position controller.
type NavControllerOutput struct {
	NavRoll       float32 `json:"navRoll"`       // Current desired roll
	NavPitch      float32 `json:"navPitch"`      // Current desired pitch
	AltError      float32 `json:"altError"`      // Current altitude error
	AspdError     float32 `json:"aspdError"`     // Current airspeed error
	XtrackError   float32 `json:"xtrackError"`   // Current crosstrack error on x-y plane
	NavBearing    int16   `json:"navBearing"`    // Current desired heading
	TargetBearing int16   `json:"targetBearing"` // Bearing to current waypoint/target
	WpDist        uint16  `json:"wpDist"`        // Distance to active waypoint
}

func (self *NavControllerOutput) MsgID() MessageID {
	return MSG_ID_NAV_CONTROLLER_OUTPUT
}

func (self *NavControllerOutput) MsgName() string {
	return "NavControllerOutput"
}

func (self *NavControllerOutput) Pack(p *Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.NavRoll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.NavPitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.AltError))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.AspdError))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.XtrackError))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.NavBearing))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.TargetBearing))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.WpDist))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *NavControllerOutput) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	self.NavRoll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.NavPitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.AltError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AspdError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.XtrackError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.NavBearing = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.TargetBearing = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.WpDist = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	return nil
}

func (self *NavControllerOutput) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func NavControllerOutputFromJSON(data []byte) (*NavControllerOutput, error) {
	p := &NavControllerOutput{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
type GlobalPositionIntCov struct {
	TimeUsec      uint64      `json:"timeUsec"`      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Lat           int32       `json:"lat"`           // Latitude
	Lon           int32       `json:"lon"`           // Longitude
	Alt           int32       `json:"alt"`           // Altitude in meters above MSL
	RelativeAlt   int32       `json:"relativeAlt"`   // Altitude above ground
	Vx            float32     `json:"vx"`            // Ground X Speed (Latitude)
	Vy            float32     `json:"vy"`            // Ground Y Speed (Longitude)
	Vz            float32     `json:"vz"`            // Ground Z Speed (Altitude)
	Covariance    [36]float32 `json:"covariance"`    // Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
	EstimatorType uint8       `json:"estimatorType"` // Class id of the estimator this estimate originated from.
}

func (self *GlobalPositionIntCov) MsgID() MessageID {
	return MSG_ID_GLOBAL_POSITION_INT_COV
}

func (self *GlobalPositionIntCov) MsgName() string {
	return "GlobalPositionIntCov"
}

func (self *GlobalPositionIntCov) Pack(p *Packet) error {
	payload := make([]byte, 181)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Alt))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.RelativeAlt))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Vz))
	for i, v := range self.Covariance {
		binary.LittleEndian.PutUint32(payload[36+i*4:], math.Float32bits(v))
	}
	payload[180] = byte(self.EstimatorType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GlobalPositionIntCov) Unpack(p *Packet) error {
	if len(p.Payload) < 181 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.RelativeAlt = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	for i := 0; i < len(self.Covariance); i++ {
		self.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36+i*4:]))
	}
	self.EstimatorType = uint8(p.Payload[180])
	return nil
}

func (self *GlobalPositionIntCov) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GlobalPositionIntCovFromJSON(data []byte) (*GlobalPositionIntCov, error) {
	p := &GlobalPositionIntCov{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedCov struct {
	TimeUsec      uint64      `json:"timeUsec"`      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	X             float32     `json:"x"`             // X Position
	Y             float32     `json:"y"`             // Y Position
	Z             float32     `json:"z"`             // Z Position
	Vx            float32     `json:"vx"`            // X Speed
	Vy            float32     `json:"vy"`            // Y Speed
	Vz            float32     `json:"vz"`            // Z Speed
	Ax            float32     `json:"ax"`            // X Acceleration
	Ay            float32     `json:"ay"`            // Y Acceleration
	Az            float32     `json:"az"`            // Z Acceleration
	Covariance    [45]float32 `json:"covariance"`    // Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
	EstimatorType uint8       `json:"estimatorType"` // Class id of the estimator this estimate originated from.
}

func (self *LocalPositionNedCov) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED_COV
}

func (self *LocalPositionNedCov) MsgName() string {
	return "LocalPositionNedCov"
}

func (self *LocalPositionNedCov) Pack(p *Packet) error {
	payload := make([]byte, 225)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Ax))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Ay))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Az))
	for i, v := range self.Covariance {
		binary.LittleEndian.PutUint32(payload[44+i*4:], math.Float32bits(v))
	}
	payload[224] = byte(self.EstimatorType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LocalPositionNedCov) Unpack(p *Packet) error {
	if len(p.Payload) < 225 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Ax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Ay = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Az = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	for i := 0; i < len(self.Covariance); i++ {
		self.Covariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44+i*4:]))
	}
	self.EstimatorType = uint8(p.Payload[224])
	return nil
}

func (self *LocalPositionNedCov) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LocalPositionNedCovFromJSON(data []byte) (*LocalPositionNedCov, error) {
	p := &LocalPositionNedCov{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
type RcChannels struct {
	TimeBootMs uint32 `json:"timeBootMs"` // Timestamp (time since system boot).
	Chan1Raw   uint16 `json:"chan1Raw"`   // RC channel 1 value.
	Chan2Raw   uint16 `json:"chan2Raw"`   // RC channel 2 value.
	Chan3Raw   uint16 `json:"chan3Raw"`   // RC channel 3 value.
	Chan4Raw   uint16 `json:"chan4Raw"`   // RC channel 4 value.
	Chan5Raw   uint16 `json:"chan5Raw"`   // RC channel 5 value.
	Chan6Raw   uint16 `json:"chan6Raw"`   // RC channel 6 value.
	Chan7Raw   uint16 `json:"chan7Raw"`   // RC channel 7 value.
	Chan8Raw   uint16 `json:"chan8Raw"`   // RC channel 8 value.
	Chan9Raw   uint16 `json:"chan9Raw"`   // RC channel 9 value.
	Chan10Raw  uint16 `json:"chan10Raw"`  // RC channel 10 value.
	Chan11Raw  uint16 `json:"chan11Raw"`  // RC channel 11 value.
	Chan12Raw  uint16 `json:"chan12Raw"`  // RC channel 12 value.
	Chan13Raw  uint16 `json:"chan13Raw"`  // RC channel 13 value.
	Chan14Raw  uint16 `json:"chan14Raw"`  // RC channel 14 value.
	Chan15Raw  uint16 `json:"chan15Raw"`  // RC channel 15 value.
	Chan16Raw  uint16 `json:"chan16Raw"`  // RC channel 16 value.
	Chan17Raw  uint16 `json:"chan17Raw"`  // RC channel 17 value.
	Chan18Raw  uint16 `json:"chan18Raw"`  // RC channel 18 value.
	Chancount  uint8  `json:"chancount"`  // Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
	Rssi       uint8  `json:"rssi"`       // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

func (self *RcChannels) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS
}

func (self *RcChannels) MsgName() string {
	return "RcChannels"
}

func (self *RcChannels) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Chan8Raw))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Chan9Raw))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Chan10Raw))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Chan11Raw))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Chan12Raw))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Chan13Raw))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Chan14Raw))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.Chan15Raw))
	binary.LittleEndian.PutUint16(payload[34:], uint16(self.Chan16Raw))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.Chan17Raw))
	binary.LittleEndian.PutUint16(payload[38:], uint16(self.Chan18Raw))
	payload[40] = byte(self.Chancount)
	payload[41] = byte(self.Rssi)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RcChannels) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Chan9Raw = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Chan10Raw = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Chan11Raw = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Chan12Raw = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.Chan13Raw = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Chan14Raw = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.Chan15Raw = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.Chan16Raw = uint16(binary.LittleEndian.Uint16(p.Payload[34:]))
	self.Chan17Raw = uint16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.Chan18Raw = uint16(binary.LittleEndian.Uint16(p.Payload[38:]))
	self.Chancount = uint8(p.Payload[40])
	self.Rssi = uint8(p.Payload[41])
	return nil
}

func (self *RcChannels) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RcChannelsFromJSON(data []byte) (*RcChannels, error) {
	p := &RcChannels{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a data stream.
type RequestDataStream struct {
	ReqMessageRate  uint16 `json:"reqMessageRate"`  // The requested message rate
	TargetSystem    uint8  `json:"targetSystem"`    // The target requested to send the message stream.
	TargetComponent uint8  `json:"targetComponent"` // The target requested to send the message stream.
	ReqStreamId     uint8  `json:"reqStreamId"`     // The ID of the requested data stream
	StartStop       uint8  `json:"startStop"`       // 1 to start sending, 0 to stop sending.
}

func (self *RequestDataStream) MsgID() MessageID {
	return MSG_ID_REQUEST_DATA_STREAM
}

func (self *RequestDataStream) MsgName() string {
	return "RequestDataStream"
}

func (self *RequestDataStream) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.ReqMessageRate))
	payload[2] = byte(self.TargetSystem)
	payload[3] = byte(self.TargetComponent)
	payload[4] = byte(self.ReqStreamId)
	payload[5] = byte(self.StartStop)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RequestDataStream) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.ReqMessageRate = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[2])
	self.TargetComponent = uint8(p.Payload[3])
	self.ReqStreamId = uint8(p.Payload[4])
	self.StartStop = uint8(p.Payload[5])
	return nil
}

func (self *RequestDataStream) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RequestDataStreamFromJSON(data []byte) (*RequestDataStream, error) {
	p := &RequestDataStream{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data stream status information.
type DataStream struct {
	MessageRate uint16 `json:"messageRate"` // The message rate
	StreamId    uint8  `json:"streamId"`    // The ID of the requested data stream
	OnOff       uint8  `json:"onOff"`       // 1 stream is enabled, 0 stream is stopped.
}

func (self *DataStream) MsgID() MessageID {
	return MSG_ID_DATA_STREAM
}

func (self *DataStream) MsgName() string {
	return "DataStream"
}

func (self *DataStream) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.MessageRate))
	payload[2] = byte(self.StreamId)
	payload[3] = byte(self.OnOff)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DataStream) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.MessageRate = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.StreamId = uint8(p.Payload[2])
	self.OnOff = uint8(p.Payload[3])
	return nil
}

func (self *DataStream) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DataStreamFromJSON(data []byte) (*DataStream, error) {
	p := &DataStream{}
	err := json.Unmarshal(data, p)
	return p, err
}

// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their
type ManualControl struct {
	X       int16  `json:"x"`       // X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	Y       int16  `json:"y"`       // Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	Z       int16  `json:"z"`       // Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
	R       int16  `json:"r"`       // R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	Buttons uint16 `json:"buttons"` // A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	Target  uint8  `json:"target"`  // The system to be controlled.
}

func (self *ManualControl) MsgID() MessageID {
	return MSG_ID_MANUAL_CONTROL
}

func (self *ManualControl) MsgName() string {
	return "ManualControl"
}

func (self *ManualControl) Pack(p *Packet) error {
	payload := make([]byte, 11)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.X))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Y))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Z))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.R))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Buttons))
	payload[10] = byte(self.Target)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ManualControl) Unpack(p *Packet) error {
	if len(p.Payload) < 11 {
		return fmt.Errorf("payload too small")
	}
	self.X = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Y = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Z = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.R = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Buttons = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Target = uint8(p.Payload[10])
	return nil
}

func (self *ManualControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ManualControlFromJSON(data []byte) (*ManualControl, error) {
	p := &ManualControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type RcChannelsOverride struct {
	Chan1Raw        uint16 `json:"chan1Raw"`        // RC channel 1 value. A value of UINT16_MAX means to ignore this field.
	Chan2Raw        uint16 `json:"chan2Raw"`        // RC channel 2 value. A value of UINT16_MAX means to ignore this field.
	Chan3Raw        uint16 `json:"chan3Raw"`        // RC channel 3 value. A value of UINT16_MAX means to ignore this field.
	Chan4Raw        uint16 `json:"chan4Raw"`        // RC channel 4 value. A value of UINT16_MAX means to ignore this field.
	Chan5Raw        uint16 `json:"chan5Raw"`        // RC channel 5 value. A value of UINT16_MAX means to ignore this field.
	Chan6Raw        uint16 `json:"chan6Raw"`        // RC channel 6 value. A value of UINT16_MAX means to ignore this field.
	Chan7Raw        uint16 `json:"chan7Raw"`        // RC channel 7 value. A value of UINT16_MAX means to ignore this field.
	Chan8Raw        uint16 `json:"chan8Raw"`        // RC channel 8 value. A value of UINT16_MAX means to ignore this field.
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *RcChannelsOverride) MsgID() MessageID {
	return MSG_ID_RC_CHANNELS_OVERRIDE
}

func (self *RcChannelsOverride) MsgName() string {
	return "RcChannelsOverride"
}

func (self *RcChannelsOverride) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Chan8Raw))
	payload[16] = byte(self.TargetSystem)
	payload[17] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RcChannelsOverride) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.TargetSystem = uint8(p.Payload[16])
	self.TargetComponent = uint8(p.Payload[17])
	return nil
}

func (self *RcChannelsOverride) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RcChannelsOverrideFromJSON(data []byte) (*RcChannelsOverride, error) {
	p := &RcChannelsOverride{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message encoding a mission item. This message is emitted to announce
//                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/services/mission.html.
type MissionItemInt struct {
	Param1          float32 `json:"param1"`          // PARAM1, see MAV_CMD enum
	Param2          float32 `json:"param2"`          // PARAM2, see MAV_CMD enum
	Param3          float32 `json:"param3"`          // PARAM3, see MAV_CMD enum
	Param4          float32 `json:"param4"`          // PARAM4, see MAV_CMD enum
	X               int32   `json:"x"`               // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   `json:"y"`               // PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
	Z               float32 `json:"z"`               // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
	Seq             uint16  `json:"seq"`             // Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
	Command         uint16  `json:"command"`         // The scheduled action for the waypoint.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID
	TargetComponent uint8   `json:"targetComponent"` // Component ID
	Frame           uint8   `json:"frame"`           // The coordinate system of the waypoint.
	Current         uint8   `json:"current"`         // false:0, true:1
	Autocontinue    uint8   `json:"autocontinue"`    // Autocontinue to next waypoint
}

func (self *MissionItemInt) MsgID() MessageID {
	return MSG_ID_MISSION_ITEM_INT
}

func (self *MissionItemInt) MsgName() string {
	return "MissionItemInt"
}

func (self *MissionItemInt) Pack(p *Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Param4))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.X))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Y))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Seq))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Command))
	payload[32] = byte(self.TargetSystem)
	payload[33] = byte(self.TargetComponent)
	payload[34] = byte(self.Frame)
	payload[35] = byte(self.Current)
	payload[36] = byte(self.Autocontinue)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MissionItemInt) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	self.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.X = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Y = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Seq = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Command = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.TargetSystem = uint8(p.Payload[32])
	self.TargetComponent = uint8(p.Payload[33])
	self.Frame = uint8(p.Payload[34])
	self.Current = uint8(p.Payload[35])
	self.Autocontinue = uint8(p.Payload[36])
	return nil
}

func (self *MissionItemInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MissionItemIntFromJSON(data []byte) (*MissionItemInt, error) {
	p := &MissionItemInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Metrics typically displayed on a HUD for fixed wing aircraft.
type VfrHud struct {
	Airspeed    float32 `json:"airspeed"`    // Current indicated airspeed (IAS).
	Groundspeed float32 `json:"groundspeed"` // Current ground speed.
	Alt         float32 `json:"alt"`         // Current altitude (MSL).
	Climb       float32 `json:"climb"`       // Current climb rate.
	Heading     int16   `json:"heading"`     // Current heading in compass units (0-360, 0=north).
	Throttle    uint16  `json:"throttle"`    // Current throttle setting (0 to 100).
}

func (self *VfrHud) MsgID() MessageID {
	return MSG_ID_VFR_HUD
}

func (self *VfrHud) MsgName() string {
	return "VfrHud"
}

func (self *VfrHud) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Airspeed))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Groundspeed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Climb))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Throttle))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *VfrHud) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Groundspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Climb = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Heading = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Throttle = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	return nil
}

func (self *VfrHud) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func VfrHudFromJSON(data []byte) (*VfrHud, error) {
	p := &VfrHud{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value.
type CommandInt struct {
	Param1          float32 `json:"param1"`          // PARAM1, see MAV_CMD enum
	Param2          float32 `json:"param2"`          // PARAM2, see MAV_CMD enum
	Param3          float32 `json:"param3"`          // PARAM3, see MAV_CMD enum
	Param4          float32 `json:"param4"`          // PARAM4, see MAV_CMD enum
	X               int32   `json:"x"`               // PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
	Y               int32   `json:"y"`               // PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
	Z               float32 `json:"z"`               // PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
	Command         uint16  `json:"command"`         // The scheduled action for the mission item.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID
	TargetComponent uint8   `json:"targetComponent"` // Component ID
	Frame           uint8   `json:"frame"`           // The coordinate system of the COMMAND.
	Current         uint8   `json:"current"`         // false:0, true:1
	Autocontinue    uint8   `json:"autocontinue"`    // autocontinue to next wp
}

func (self *CommandInt) MsgID() MessageID {
	return MSG_ID_COMMAND_INT
}

func (self *CommandInt) MsgName() string {
	return "CommandInt"
}

func (self *CommandInt) Pack(p *Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Param4))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.X))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Y))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Command))
	payload[30] = byte(self.TargetSystem)
	payload[31] = byte(self.TargetComponent)
	payload[32] = byte(self.Frame)
	payload[33] = byte(self.Current)
	payload[34] = byte(self.Autocontinue)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CommandInt) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	self.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.X = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Y = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Command = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.TargetSystem = uint8(p.Payload[30])
	self.TargetComponent = uint8(p.Payload[31])
	self.Frame = uint8(p.Payload[32])
	self.Current = uint8(p.Payload[33])
	self.Autocontinue = uint8(p.Payload[34])
	return nil
}

func (self *CommandInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CommandIntFromJSON(data []byte) (*CommandInt, error) {
	p := &CommandInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send a command with up to seven parameters to the MAV
type CommandLong struct {
	Param1          float32 `json:"param1"`          // Parameter 1 (for the specific command).
	Param2          float32 `json:"param2"`          // Parameter 2 (for the specific command).
	Param3          float32 `json:"param3"`          // Parameter 3 (for the specific command).
	Param4          float32 `json:"param4"`          // Parameter 4 (for the specific command).
	Param5          float32 `json:"param5"`          // Parameter 5 (for the specific command).
	Param6          float32 `json:"param6"`          // Parameter 6 (for the specific command).
	Param7          float32 `json:"param7"`          // Parameter 7 (for the specific command).
	Command         uint16  `json:"command"`         // Command ID (of command to send).
	TargetSystem    uint8   `json:"targetSystem"`    // System which should execute the command
	TargetComponent uint8   `json:"targetComponent"` // Component which should execute the command, 0 for all components
	Confirmation    uint8   `json:"confirmation"`    // 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
}

func (self *CommandLong) MsgID() MessageID {
	return MSG_ID_COMMAND_LONG
}

func (self *CommandLong) MsgName() string {
	return "CommandLong"
}

func (self *CommandLong) Pack(p *Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Param1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Param2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Param3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Param4))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Param5))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Param6))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Param7))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Command))
	payload[30] = byte(self.TargetSystem)
	payload[31] = byte(self.TargetComponent)
	payload[32] = byte(self.Confirmation)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CommandLong) Unpack(p *Packet) error {
	if len(p.Payload) < 33 {
		return fmt.Errorf("payload too small")
	}
	self.Param1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Param2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Param3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Param4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Param5 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Param6 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Param7 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Command = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.TargetSystem = uint8(p.Payload[30])
	self.TargetComponent = uint8(p.Payload[31])
	self.Confirmation = uint8(p.Payload[32])
	return nil
}

func (self *CommandLong) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CommandLongFromJSON(data []byte) (*CommandLong, error) {
	p := &CommandLong{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Report status of a command. Includes feedback whether the command was executed.
type CommandAck struct {
	Command uint16 `json:"command"` // Command ID (of acknowledged command).
	Result  uint8  `json:"result"`  // Result of command.
}

func (self *CommandAck) MsgID() MessageID {
	return MSG_ID_COMMAND_ACK
}

func (self *CommandAck) MsgName() string {
	return "CommandAck"
}

func (self *CommandAck) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Command))
	payload[2] = byte(self.Result)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CommandAck) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.Command = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Result = uint8(p.Payload[2])
	return nil
}

func (self *CommandAck) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CommandAckFromJSON(data []byte) (*CommandAck, error) {
	p := &CommandAck{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Setpoint in roll, pitch, yaw and thrust from the operator
type ManualSetpoint struct {
	TimeBootMs           uint32  `json:"timeBootMs"`           // Timestamp (time since system boot).
	Roll                 float32 `json:"roll"`                 // Desired roll rate
	Pitch                float32 `json:"pitch"`                // Desired pitch rate
	Yaw                  float32 `json:"yaw"`                  // Desired yaw rate
	Thrust               float32 `json:"thrust"`               // Collective thrust, normalized to 0 .. 1
	ModeSwitch           uint8   `json:"modeSwitch"`           // Flight mode switch position, 0.. 255
	ManualOverrideSwitch uint8   `json:"manualOverrideSwitch"` // Override mode switch position, 0.. 255
}

func (self *ManualSetpoint) MsgID() MessageID {
	return MSG_ID_MANUAL_SETPOINT
}

func (self *ManualSetpoint) MsgName() string {
	return "ManualSetpoint"
}

func (self *ManualSetpoint) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Thrust))
	payload[20] = byte(self.ModeSwitch)
	payload[21] = byte(self.ManualOverrideSwitch)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ManualSetpoint) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.ModeSwitch = uint8(p.Payload[20])
	self.ManualOverrideSwitch = uint8(p.Payload[21])
	return nil
}

func (self *ManualSetpoint) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ManualSetpointFromJSON(data []byte) (*ManualSetpoint, error) {
	p := &ManualSetpoint{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
type SetAttitudeTarget struct {
	TimeBootMs      uint32     `json:"timeBootMs"`      // Timestamp (time since system boot).
	Q               [4]float32 `json:"q"`               // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate    float32    `json:"bodyRollRate"`    // Body roll rate
	BodyPitchRate   float32    `json:"bodyPitchRate"`   // Body pitch rate
	BodyYawRate     float32    `json:"bodyYawRate"`     // Body yaw rate
	Thrust          float32    `json:"thrust"`          // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TargetSystem    uint8      `json:"targetSystem"`    // System ID
	TargetComponent uint8      `json:"targetComponent"` // Component ID
	TypeMask        uint8      `json:"typeMask"`        // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
}

func (self *SetAttitudeTarget) MsgID() MessageID {
	return MSG_ID_SET_ATTITUDE_TARGET
}

func (self *SetAttitudeTarget) MsgName() string {
	return "SetAttitudeTarget"
}

func (self *SetAttitudeTarget) Pack(p *Packet) error {
	payload := make([]byte, 39)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[4+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.BodyRollRate))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.BodyPitchRate))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.BodyYawRate))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Thrust))
	payload[36] = byte(self.TargetSystem)
	payload[37] = byte(self.TargetComponent)
	payload[38] = byte(self.TypeMask)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetAttitudeTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 39 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4+i*4:]))
	}
	self.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.TargetSystem = uint8(p.Payload[36])
	self.TargetComponent = uint8(p.Payload[37])
	self.TypeMask = uint8(p.Payload[38])
	return nil
}

func (self *SetAttitudeTarget) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetAttitudeTargetFromJSON(data []byte) (*SetAttitudeTarget, error) {
	p := &SetAttitudeTarget{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
type AttitudeTarget struct {
	TimeBootMs    uint32     `json:"timeBootMs"`    // Timestamp (time since system boot).
	Q             [4]float32 `json:"q"`             // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	BodyRollRate  float32    `json:"bodyRollRate"`  // Body roll rate
	BodyPitchRate float32    `json:"bodyPitchRate"` // Body pitch rate
	BodyYawRate   float32    `json:"bodyYawRate"`   // Body yaw rate
	Thrust        float32    `json:"thrust"`        // Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
	TypeMask      uint8      `json:"typeMask"`      // Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude
}

func (self *AttitudeTarget) MsgID() MessageID {
	return MSG_ID_ATTITUDE_TARGET
}

func (self *AttitudeTarget) MsgName() string {
	return "AttitudeTarget"
}

func (self *AttitudeTarget) Pack(p *Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[4+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.BodyRollRate))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.BodyPitchRate))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.BodyYawRate))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Thrust))
	payload[36] = byte(self.TypeMask)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AttitudeTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4+i*4:]))
	}
	self.BodyRollRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.BodyPitchRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.BodyYawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Thrust = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.TypeMask = uint8(p.Payload[36])
	return nil
}

func (self *AttitudeTarget) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AttitudeTargetFromJSON(data []byte) (*AttitudeTarget, error) {
	p := &AttitudeTarget{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetLocalNed struct {
	TimeBootMs      uint32  `json:"timeBootMs"`      // Timestamp (time since system boot).
	X               float32 `json:"x"`               // X Position in NED frame
	Y               float32 `json:"y"`               // Y Position in NED frame
	Z               float32 `json:"z"`               // Z Position in NED frame (note, altitude is negative in NED)
	Vx              float32 `json:"vx"`              // X velocity in NED frame
	Vy              float32 `json:"vy"`              // Y velocity in NED frame
	Vz              float32 `json:"vz"`              // Z velocity in NED frame
	Afx             float32 `json:"afx"`             // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 `json:"afy"`             // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 `json:"afz"`             // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 `json:"yaw"`             // yaw setpoint
	YawRate         float32 `json:"yawRate"`         // yaw rate setpoint
	TypeMask        uint16  `json:"typeMask"`        // Bitmap to indicate which dimensions should be ignored by the vehicle.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID
	TargetComponent uint8   `json:"targetComponent"` // Component ID
	CoordinateFrame uint8   `json:"coordinateFrame"` // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

func (self *SetPositionTargetLocalNed) MsgID() MessageID {
	return MSG_ID_SET_POSITION_TARGET_LOCAL_NED
}

func (self *SetPositionTargetLocalNed) MsgName() string {
	return "SetPositionTargetLocalNed"
}

func (self *SetPositionTargetLocalNed) Pack(p *Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.TypeMask))
	payload[50] = byte(self.TargetSystem)
	payload[51] = byte(self.TargetComponent)
	payload[52] = byte(self.CoordinateFrame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetPositionTargetLocalNed) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.TargetSystem = uint8(p.Payload[50])
	self.TargetComponent = uint8(p.Payload[51])
	self.CoordinateFrame = uint8(p.Payload[52])
	return nil
}

func (self *SetPositionTargetLocalNed) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetPositionTargetLocalNedFromJSON(data []byte) (*SetPositionTargetLocalNed, error) {
	p := &SetPositionTargetLocalNed{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
type PositionTargetLocalNed struct {
	TimeBootMs      uint32  `json:"timeBootMs"`      // Timestamp (time since system boot).
	X               float32 `json:"x"`               // X Position in NED frame
	Y               float32 `json:"y"`               // Y Position in NED frame
	Z               float32 `json:"z"`               // Z Position in NED frame (note, altitude is negative in NED)
	Vx              float32 `json:"vx"`              // X velocity in NED frame
	Vy              float32 `json:"vy"`              // Y velocity in NED frame
	Vz              float32 `json:"vz"`              // Z velocity in NED frame
	Afx             float32 `json:"afx"`             // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 `json:"afy"`             // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 `json:"afz"`             // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 `json:"yaw"`             // yaw setpoint
	YawRate         float32 `json:"yawRate"`         // yaw rate setpoint
	TypeMask        uint16  `json:"typeMask"`        // Bitmap to indicate which dimensions should be ignored by the vehicle.
	CoordinateFrame uint8   `json:"coordinateFrame"` // Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
}

func (self *PositionTargetLocalNed) MsgID() MessageID {
	return MSG_ID_POSITION_TARGET_LOCAL_NED
}

func (self *PositionTargetLocalNed) MsgName() string {
	return "PositionTargetLocalNed"
}

func (self *PositionTargetLocalNed) Pack(p *Packet) error {
	payload := make([]byte, 51)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.TypeMask))
	payload[50] = byte(self.CoordinateFrame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PositionTargetLocalNed) Unpack(p *Packet) error {
	if len(p.Payload) < 51 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.CoordinateFrame = uint8(p.Payload[50])
	return nil
}

func (self *PositionTargetLocalNed) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func PositionTargetLocalNedFromJSON(data []byte) (*PositionTargetLocalNed, error) {
	p := &PositionTargetLocalNed{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
type SetPositionTargetGlobalInt struct {
	TimeBootMs      uint32  `json:"timeBootMs"`      // Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32   `json:"latInt"`          // X Position in WGS84 frame
	LonInt          int32   `json:"lonInt"`          // Y Position in WGS84 frame
	Alt             float32 `json:"alt"`             // Altitude (MSL, Relative to home, or AGL - depending on frame)
	Vx              float32 `json:"vx"`              // X velocity in NED frame
	Vy              float32 `json:"vy"`              // Y velocity in NED frame
	Vz              float32 `json:"vz"`              // Z velocity in NED frame
	Afx             float32 `json:"afx"`             // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 `json:"afy"`             // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 `json:"afz"`             // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 `json:"yaw"`             // yaw setpoint
	YawRate         float32 `json:"yawRate"`         // yaw rate setpoint
	TypeMask        uint16  `json:"typeMask"`        // Bitmap to indicate which dimensions should be ignored by the vehicle.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID
	TargetComponent uint8   `json:"targetComponent"` // Component ID
	CoordinateFrame uint8   `json:"coordinateFrame"` // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

func (self *SetPositionTargetGlobalInt) MsgID() MessageID {
	return MSG_ID_SET_POSITION_TARGET_GLOBAL_INT
}

func (self *SetPositionTargetGlobalInt) MsgName() string {
	return "SetPositionTargetGlobalInt"
}

func (self *SetPositionTargetGlobalInt) Pack(p *Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.LatInt))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.LonInt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.TypeMask))
	payload[50] = byte(self.TargetSystem)
	payload[51] = byte(self.TargetComponent)
	payload[52] = byte(self.CoordinateFrame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetPositionTargetGlobalInt) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LatInt = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.LonInt = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.TargetSystem = uint8(p.Payload[50])
	self.TargetComponent = uint8(p.Payload[51])
	self.CoordinateFrame = uint8(p.Payload[52])
	return nil
}

func (self *SetPositionTargetGlobalInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetPositionTargetGlobalIntFromJSON(data []byte) (*SetPositionTargetGlobalInt, error) {
	p := &SetPositionTargetGlobalInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
type PositionTargetGlobalInt struct {
	TimeBootMs      uint32  `json:"timeBootMs"`      // Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
	LatInt          int32   `json:"latInt"`          // X Position in WGS84 frame
	LonInt          int32   `json:"lonInt"`          // Y Position in WGS84 frame
	Alt             float32 `json:"alt"`             // Altitude (MSL, AGL or relative to home altitude, depending on frame)
	Vx              float32 `json:"vx"`              // X velocity in NED frame
	Vy              float32 `json:"vy"`              // Y velocity in NED frame
	Vz              float32 `json:"vz"`              // Z velocity in NED frame
	Afx             float32 `json:"afx"`             // X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afy             float32 `json:"afy"`             // Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Afz             float32 `json:"afz"`             // Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
	Yaw             float32 `json:"yaw"`             // yaw setpoint
	YawRate         float32 `json:"yawRate"`         // yaw rate setpoint
	TypeMask        uint16  `json:"typeMask"`        // Bitmap to indicate which dimensions should be ignored by the vehicle.
	CoordinateFrame uint8   `json:"coordinateFrame"` // Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11
}

func (self *PositionTargetGlobalInt) MsgID() MessageID {
	return MSG_ID_POSITION_TARGET_GLOBAL_INT
}

func (self *PositionTargetGlobalInt) MsgName() string {
	return "PositionTargetGlobalInt"
}

func (self *PositionTargetGlobalInt) Pack(p *Packet) error {
	payload := make([]byte, 51)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.LatInt))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.LonInt))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Afx))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Afy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Afz))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.YawRate))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.TypeMask))
	payload[50] = byte(self.CoordinateFrame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PositionTargetGlobalInt) Unpack(p *Packet) error {
	if len(p.Payload) < 51 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LatInt = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.LonInt = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Afx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Afy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Afz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.TypeMask = uint16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.CoordinateFrame = uint8(p.Payload[50])
	return nil
}

func (self *PositionTargetGlobalInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func PositionTargetGlobalIntFromJSON(data []byte) (*PositionTargetGlobalInt, error) {
	p := &PositionTargetGlobalInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
type LocalPositionNedSystemGlobalOffset struct {
	TimeBootMs uint32  `json:"timeBootMs"` // Timestamp (time since system boot).
	X          float32 `json:"x"`          // X Position
	Y          float32 `json:"y"`          // Y Position
	Z          float32 `json:"z"`          // Z Position
	Roll       float32 `json:"roll"`       // Roll
	Pitch      float32 `json:"pitch"`      // Pitch
	Yaw        float32 `json:"yaw"`        // Yaw
}

func (self *LocalPositionNedSystemGlobalOffset) MsgID() MessageID {
	return MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET
}

func (self *LocalPositionNedSystemGlobalOffset) MsgName() string {
	return "LocalPositionNedSystemGlobalOffset"
}

func (self *LocalPositionNedSystemGlobalOffset) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Yaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LocalPositionNedSystemGlobalOffset) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

func (self *LocalPositionNedSystemGlobalOffset) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LocalPositionNedSystemGlobalOffsetFromJSON(data []byte) (*LocalPositionNedSystemGlobalOffset, error) {
	p := &LocalPositionNedSystemGlobalOffset{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilState struct {
	TimeUsec   uint64  `json:"timeUsec"`   // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Roll       float32 `json:"roll"`       // Roll angle
	Pitch      float32 `json:"pitch"`      // Pitch angle
	Yaw        float32 `json:"yaw"`        // Yaw angle
	Rollspeed  float32 `json:"rollspeed"`  // Body frame roll / phi angular speed
	Pitchspeed float32 `json:"pitchspeed"` // Body frame pitch / theta angular speed
	Yawspeed   float32 `json:"yawspeed"`   // Body frame yaw / psi angular speed
	Lat        int32   `json:"lat"`        // Latitude
	Lon        int32   `json:"lon"`        // Longitude
	Alt        int32   `json:"alt"`        // Altitude
	Vx         int16   `json:"vx"`         // Ground X Speed (Latitude)
	Vy         int16   `json:"vy"`         // Ground Y Speed (Longitude)
	Vz         int16   `json:"vz"`         // Ground Z Speed (Altitude)
	Xacc       int16   `json:"xacc"`       // X acceleration
	Yacc       int16   `json:"yacc"`       // Y acceleration
	Zacc       int16   `json:"zacc"`       // Z acceleration
}

func (self *HilState) MsgID() MessageID {
	return MSG_ID_HIL_STATE
}

func (self *HilState) MsgName() string {
	return "HilState"
}

func (self *HilState) Pack(p *Packet) error {
	payload := make([]byte, 56)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Rollspeed))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Yawspeed))
	binary.LittleEndian.PutUint32(payload[32:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[40:], uint32(self.Alt))
	binary.LittleEndian.PutUint16(payload[44:], uint16(self.Vx))
	binary.LittleEndian.PutUint16(payload[46:], uint16(self.Vy))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.Vz))
	binary.LittleEndian.PutUint16(payload[50:], uint16(self.Xacc))
	binary.LittleEndian.PutUint16(payload[52:], uint16(self.Yacc))
	binary.LittleEndian.PutUint16(payload[54:], uint16(self.Zacc))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilState) Unpack(p *Packet) error {
	if len(p.Payload) < 56 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Vx = int16(binary.LittleEndian.Uint16(p.Payload[44:]))
	self.Vy = int16(binary.LittleEndian.Uint16(p.Payload[46:]))
	self.Vz = int16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[50:]))
	self.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[52:]))
	self.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[54:]))
	return nil
}

func (self *HilState) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilStateFromJSON(data []byte) (*HilState, error) {
	p := &HilState{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sent from autopilot to simulation. Hardware in the loop control outputs
type HilControls struct {
	TimeUsec      uint64  `json:"timeUsec"`      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	RollAilerons  float32 `json:"rollAilerons"`  // Control output -1 .. 1
	PitchElevator float32 `json:"pitchElevator"` // Control output -1 .. 1
	YawRudder     float32 `json:"yawRudder"`     // Control output -1 .. 1
	Throttle      float32 `json:"throttle"`      // Throttle 0 .. 1
	Aux1          float32 `json:"aux1"`          // Aux 1, -1 .. 1
	Aux2          float32 `json:"aux2"`          // Aux 2, -1 .. 1
	Aux3          float32 `json:"aux3"`          // Aux 3, -1 .. 1
	Aux4          float32 `json:"aux4"`          // Aux 4, -1 .. 1
	Mode          uint8   `json:"mode"`          // System mode.
	NavMode       uint8   `json:"navMode"`       // Navigation mode (MAV_NAV_MODE)
}

func (self *HilControls) MsgID() MessageID {
	return MSG_ID_HIL_CONTROLS
}

func (self *HilControls) MsgName() string {
	return "HilControls"
}

func (self *HilControls) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.RollAilerons))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.PitchElevator))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.YawRudder))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Throttle))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Aux1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Aux2))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Aux3))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Aux4))
	payload[40] = byte(self.Mode)
	payload[41] = byte(self.NavMode)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilControls) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.RollAilerons = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.PitchElevator = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.YawRudder = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Throttle = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Aux1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Aux2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Aux3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Aux4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Mode = uint8(p.Payload[40])
	self.NavMode = uint8(p.Payload[41])
	return nil
}

func (self *HilControls) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilControlsFromJSON(data []byte) (*HilControls, error) {
	p := &HilControls{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
type HilRcInputsRaw struct {
	TimeUsec  uint64 `json:"timeUsec"`  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Chan1Raw  uint16 `json:"chan1Raw"`  // RC channel 1 value
	Chan2Raw  uint16 `json:"chan2Raw"`  // RC channel 2 value
	Chan3Raw  uint16 `json:"chan3Raw"`  // RC channel 3 value
	Chan4Raw  uint16 `json:"chan4Raw"`  // RC channel 4 value
	Chan5Raw  uint16 `json:"chan5Raw"`  // RC channel 5 value
	Chan6Raw  uint16 `json:"chan6Raw"`  // RC channel 6 value
	Chan7Raw  uint16 `json:"chan7Raw"`  // RC channel 7 value
	Chan8Raw  uint16 `json:"chan8Raw"`  // RC channel 8 value
	Chan9Raw  uint16 `json:"chan9Raw"`  // RC channel 9 value
	Chan10Raw uint16 `json:"chan10Raw"` // RC channel 10 value
	Chan11Raw uint16 `json:"chan11Raw"` // RC channel 11 value
	Chan12Raw uint16 `json:"chan12Raw"` // RC channel 12 value
	Rssi      uint8  `json:"rssi"`      // Receive signal strength indicator in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
}

func (self *HilRcInputsRaw) MsgID() MessageID {
	return MSG_ID_HIL_RC_INPUTS_RAW
}

func (self *HilRcInputsRaw) MsgName() string {
	return "HilRcInputsRaw"
}

func (self *HilRcInputsRaw) Pack(p *Packet) error {
	payload := make([]byte, 33)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Chan1Raw))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Chan2Raw))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Chan3Raw))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Chan4Raw))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Chan5Raw))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Chan6Raw))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Chan7Raw))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Chan8Raw))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Chan9Raw))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Chan10Raw))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Chan11Raw))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Chan12Raw))
	payload[32] = byte(self.Rssi)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilRcInputsRaw) Unpack(p *Packet) error {
	if len(p.Payload) < 33 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Chan1Raw = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Chan2Raw = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Chan3Raw = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Chan4Raw = uint16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Chan5Raw = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Chan6Raw = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Chan7Raw = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Chan8Raw = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Chan9Raw = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Chan10Raw = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.Chan11Raw = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Chan12Raw = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.Rssi = uint8(p.Payload[32])
	return nil
}

func (self *HilRcInputsRaw) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilRcInputsRawFromJSON(data []byte) (*HilRcInputsRaw, error) {
	p := &HilRcInputsRaw{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)
type HilActuatorControls struct {
	TimeUsec uint64      `json:"timeUsec"` // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Flags    uint64      `json:"flags"`    // Flags as bitfield, reserved for future use.
	Controls [16]float32 `json:"controls"` // Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
	Mode     uint8       `json:"mode"`     // System mode. Includes arming state.
}

func (self *HilActuatorControls) MsgID() MessageID {
	return MSG_ID_HIL_ACTUATOR_CONTROLS
}

func (self *HilActuatorControls) MsgName() string {
	return "HilActuatorControls"
}

func (self *HilActuatorControls) Pack(p *Packet) error {
	payload := make([]byte, 81)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint64(payload[8:], uint64(self.Flags))
	for i, v := range self.Controls {
		binary.LittleEndian.PutUint32(payload[16+i*4:], math.Float32bits(v))
	}
	payload[80] = byte(self.Mode)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilActuatorControls) Unpack(p *Packet) error {
	if len(p.Payload) < 81 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Flags = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	for i := 0; i < len(self.Controls); i++ {
		self.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16+i*4:]))
	}
	self.Mode = uint8(p.Payload[80])
	return nil
}

func (self *HilActuatorControls) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilActuatorControlsFromJSON(data []byte) (*HilActuatorControls, error) {
	p := &HilActuatorControls{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Optical flow from a flow sensor (e.g. optical mouse sensor)
type OpticalFlow struct {
	TimeUsec       uint64  `json:"timeUsec"`       // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	FlowCompMX     float32 `json:"flowCompMX"`     // Flow in x-sensor direction, angular-speed compensated
	FlowCompMY     float32 `json:"flowCompMY"`     // Flow in y-sensor direction, angular-speed compensated
	GroundDistance float32 `json:"groundDistance"` // Ground distance. Positive value: distance known. Negative value: Unknown distance
	FlowX          int16   `json:"flowX"`          // Flow in x-sensor direction
	FlowY          int16   `json:"flowY"`          // Flow in y-sensor direction
	SensorId       uint8   `json:"sensorId"`       // Sensor ID
	Quality        uint8   `json:"quality"`        // Optical flow quality / confidence. 0: bad, 255: maximum quality
}

func (self *OpticalFlow) MsgID() MessageID {
	return MSG_ID_OPTICAL_FLOW
}

func (self *OpticalFlow) MsgName() string {
	return "OpticalFlow"
}

func (self *OpticalFlow) Pack(p *Packet) error {
	payload := make([]byte, 26)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.FlowCompMX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.FlowCompMY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.GroundDistance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.FlowX))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.FlowY))
	payload[24] = byte(self.SensorId)
	payload[25] = byte(self.Quality)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *OpticalFlow) Unpack(p *Packet) error {
	if len(p.Payload) < 26 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.FlowCompMX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.FlowCompMY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.GroundDistance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.FlowX = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.FlowY = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.SensorId = uint8(p.Payload[24])
	self.Quality = uint8(p.Payload[25])
	return nil
}

func (self *OpticalFlow) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func OpticalFlowFromJSON(data []byte) (*OpticalFlow, error) {
	p := &OpticalFlow{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Global position/attitude estimate from a vision source.
type GlobalVisionPositionEstimate struct {
	Usec  uint64  `json:"usec"`  // Timestamp (UNIX time or since system boot)
	X     float32 `json:"x"`     // Global X position
	Y     float32 `json:"y"`     // Global Y position
	Z     float32 `json:"z"`     // Global Z position
	Roll  float32 `json:"roll"`  // Roll angle
	Pitch float32 `json:"pitch"` // Pitch angle
	Yaw   float32 `json:"yaw"`   // Yaw angle
}

func (self *GlobalVisionPositionEstimate) MsgID() MessageID {
	return MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE
}

func (self *GlobalVisionPositionEstimate) MsgName() string {
	return "GlobalVisionPositionEstimate"
}

func (self *GlobalVisionPositionEstimate) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Yaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GlobalVisionPositionEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

func (self *GlobalVisionPositionEstimate) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GlobalVisionPositionEstimateFromJSON(data []byte) (*GlobalVisionPositionEstimate, error) {
	p := &GlobalVisionPositionEstimate{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Global position/attitude estimate from a vision source.
type VisionPositionEstimate struct {
	Usec  uint64  `json:"usec"`  // Timestamp (UNIX time or time since system boot)
	X     float32 `json:"x"`     // Global X position
	Y     float32 `json:"y"`     // Global Y position
	Z     float32 `json:"z"`     // Global Z position
	Roll  float32 `json:"roll"`  // Roll angle
	Pitch float32 `json:"pitch"` // Pitch angle
	Yaw   float32 `json:"yaw"`   // Yaw angle
}

func (self *VisionPositionEstimate) MsgID() MessageID {
	return MSG_ID_VISION_POSITION_ESTIMATE
}

func (self *VisionPositionEstimate) MsgName() string {
	return "VisionPositionEstimate"
}

func (self *VisionPositionEstimate) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Yaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *VisionPositionEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

func (self *VisionPositionEstimate) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func VisionPositionEstimateFromJSON(data []byte) (*VisionPositionEstimate, error) {
	p := &VisionPositionEstimate{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Speed estimate from a vision source.
type VisionSpeedEstimate struct {
	Usec uint64  `json:"usec"` // Timestamp (UNIX time or time since system boot)
	X    float32 `json:"x"`    // Global X speed
	Y    float32 `json:"y"`    // Global Y speed
	Z    float32 `json:"z"`    // Global Z speed
}

func (self *VisionSpeedEstimate) MsgID() MessageID {
	return MSG_ID_VISION_SPEED_ESTIMATE
}

func (self *VisionSpeedEstimate) MsgName() string {
	return "VisionSpeedEstimate"
}

func (self *VisionSpeedEstimate) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Z))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *VisionSpeedEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	return nil
}

func (self *VisionSpeedEstimate) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func VisionSpeedEstimateFromJSON(data []byte) (*VisionSpeedEstimate, error) {
	p := &VisionSpeedEstimate{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Global position estimate from a Vicon motion system source.
type ViconPositionEstimate struct {
	Usec  uint64  `json:"usec"`  // Timestamp (UNIX time or time since system boot)
	X     float32 `json:"x"`     // Global X position
	Y     float32 `json:"y"`     // Global Y position
	Z     float32 `json:"z"`     // Global Z position
	Roll  float32 `json:"roll"`  // Roll angle
	Pitch float32 `json:"pitch"` // Pitch angle
	Yaw   float32 `json:"yaw"`   // Yaw angle
}

func (self *ViconPositionEstimate) MsgID() MessageID {
	return MSG_ID_VICON_POSITION_ESTIMATE
}

func (self *ViconPositionEstimate) MsgName() string {
	return "ViconPositionEstimate"
}

func (self *ViconPositionEstimate) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Usec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Z))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Yaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ViconPositionEstimate) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.Usec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

func (self *ViconPositionEstimate) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ViconPositionEstimateFromJSON(data []byte) (*ViconPositionEstimate, error) {
	p := &ViconPositionEstimate{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The IMU readings in SI units in NED body frame
type HighresImu struct {
	TimeUsec      uint64  `json:"timeUsec"`      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Xacc          float32 `json:"xacc"`          // X acceleration
	Yacc          float32 `json:"yacc"`          // Y acceleration
	Zacc          float32 `json:"zacc"`          // Z acceleration
	Xgyro         float32 `json:"xgyro"`         // Angular speed around X axis
	Ygyro         float32 `json:"ygyro"`         // Angular speed around Y axis
	Zgyro         float32 `json:"zgyro"`         // Angular speed around Z axis
	Xmag          float32 `json:"xmag"`          // X Magnetic field
	Ymag          float32 `json:"ymag"`          // Y Magnetic field
	Zmag          float32 `json:"zmag"`          // Z Magnetic field
	AbsPressure   float32 `json:"absPressure"`   // Absolute pressure
	DiffPressure  float32 `json:"diffPressure"`  // Differential pressure
	PressureAlt   float32 `json:"pressureAlt"`   // Altitude calculated from pressure
	Temperature   float32 `json:"temperature"`   // Temperature
	FieldsUpdated uint16  `json:"fieldsUpdated"` // Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature
}

func (self *HighresImu) MsgID() MessageID {
	return MSG_ID_HIGHRES_IMU
}

func (self *HighresImu) MsgName() string {
	return "HighresImu"
}

func (self *HighresImu) Pack(p *Packet) error {
	payload := make([]byte, 62)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Xacc))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Yacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Zacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Xgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Ygyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Zgyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Xmag))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Ymag))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Zmag))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.AbsPressure))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.DiffPressure))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.PressureAlt))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(self.Temperature))
	binary.LittleEndian.PutUint16(payload[60:], uint16(self.FieldsUpdated))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HighresImu) Unpack(p *Packet) error {
	if len(p.Payload) < 62 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Xmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Ymag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Zmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.AbsPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.PressureAlt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	self.FieldsUpdated = uint16(binary.LittleEndian.Uint16(p.Payload[60:]))
	return nil
}

func (self *HighresImu) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HighresImuFromJSON(data []byte) (*HighresImu, error) {
	p := &HighresImu{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
type OpticalFlowRad struct {
	TimeUsec            uint64  `json:"timeUsec"`            // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	IntegrationTimeUs   uint32  `json:"integrationTimeUs"`   // Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 `json:"integratedX"`         // Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 `json:"integratedY"`         // Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 `json:"integratedXgyro"`     // RH rotation around X axis
	IntegratedYgyro     float32 `json:"integratedYgyro"`     // RH rotation around Y axis
	IntegratedZgyro     float32 `json:"integratedZgyro"`     // RH rotation around Z axis
	TimeDeltaDistanceUs uint32  `json:"timeDeltaDistanceUs"` // Time since the distance was sampled.
	Distance            float32 `json:"distance"`            // Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
	Temperature         int16   `json:"temperature"`         // Temperature
	SensorId            uint8   `json:"sensorId"`            // Sensor ID
	Quality             uint8   `json:"quality"`             // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}

func (self *OpticalFlowRad) MsgID() MessageID {
	return MSG_ID_OPTICAL_FLOW_RAD
}

func (self *OpticalFlowRad) MsgName() string {
	return "OpticalFlowRad"
}

func (self *OpticalFlowRad) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.IntegrationTimeUs))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.IntegratedX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.IntegratedY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.IntegratedXgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.IntegratedYgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.IntegratedZgyro))
	binary.LittleEndian.PutUint32(payload[32:], uint32(self.TimeDeltaDistanceUs))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Distance))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.Temperature))
	payload[42] = byte(self.SensorId)
	payload[43] = byte(self.Quality)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *OpticalFlowRad) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.IntegrationTimeUs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.IntegratedX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.IntegratedY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.IntegratedXgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.IntegratedYgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.IntegratedZgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.TimeDeltaDistanceUs = uint32(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.SensorId = uint8(p.Payload[42])
	self.Quality = uint8(p.Payload[43])
	return nil
}

func (self *OpticalFlowRad) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func OpticalFlowRadFromJSON(data []byte) (*OpticalFlowRad, error) {
	p := &OpticalFlowRad{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The IMU readings in SI units in NED body frame
type HilSensor struct {
	TimeUsec      uint64  `json:"timeUsec"`      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Xacc          float32 `json:"xacc"`          // X acceleration
	Yacc          float32 `json:"yacc"`          // Y acceleration
	Zacc          float32 `json:"zacc"`          // Z acceleration
	Xgyro         float32 `json:"xgyro"`         // Angular speed around X axis in body frame
	Ygyro         float32 `json:"ygyro"`         // Angular speed around Y axis in body frame
	Zgyro         float32 `json:"zgyro"`         // Angular speed around Z axis in body frame
	Xmag          float32 `json:"xmag"`          // X Magnetic field
	Ymag          float32 `json:"ymag"`          // Y Magnetic field
	Zmag          float32 `json:"zmag"`          // Z Magnetic field
	AbsPressure   float32 `json:"absPressure"`   // Absolute pressure
	DiffPressure  float32 `json:"diffPressure"`  // Differential pressure (airspeed)
	PressureAlt   float32 `json:"pressureAlt"`   // Altitude calculated from pressure
	Temperature   float32 `json:"temperature"`   // Temperature
	FieldsUpdated uint32  `json:"fieldsUpdated"` // Bitmap for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim.
}

func (self *HilSensor) MsgID() MessageID {
	return MSG_ID_HIL_SENSOR
}

func (self *HilSensor) MsgName() string {
	return "HilSensor"
}

func (self *HilSensor) Pack(p *Packet) error {
	payload := make([]byte, 64)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Xacc))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Yacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Zacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Xgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Ygyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Zgyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Xmag))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Ymag))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Zmag))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.AbsPressure))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.DiffPressure))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.PressureAlt))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(self.Temperature))
	binary.LittleEndian.PutUint32(payload[60:], uint32(self.FieldsUpdated))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilSensor) Unpack(p *Packet) error {
	if len(p.Payload) < 64 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Xmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Ymag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Zmag = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.AbsPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.PressureAlt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.Temperature = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	self.FieldsUpdated = uint32(binary.LittleEndian.Uint32(p.Payload[60:]))
	return nil
}

func (self *HilSensor) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilSensorFromJSON(data []byte) (*HilSensor, error) {
	p := &HilSensor{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of simulation environment, if used
type SimState struct {
	Q1         float32 `json:"q1"`         // True attitude quaternion component 1, w (1 in null-rotation)
	Q2         float32 `json:"q2"`         // True attitude quaternion component 2, x (0 in null-rotation)
	Q3         float32 `json:"q3"`         // True attitude quaternion component 3, y (0 in null-rotation)
	Q4         float32 `json:"q4"`         // True attitude quaternion component 4, z (0 in null-rotation)
	Roll       float32 `json:"roll"`       // Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
	Pitch      float32 `json:"pitch"`      // Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
	Yaw        float32 `json:"yaw"`        // Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
	Xacc       float32 `json:"xacc"`       // X acceleration
	Yacc       float32 `json:"yacc"`       // Y acceleration
	Zacc       float32 `json:"zacc"`       // Z acceleration
	Xgyro      float32 `json:"xgyro"`      // Angular speed around X axis
	Ygyro      float32 `json:"ygyro"`      // Angular speed around Y axis
	Zgyro      float32 `json:"zgyro"`      // Angular speed around Z axis
	Lat        float32 `json:"lat"`        // Latitude
	Lon        float32 `json:"lon"`        // Longitude
	Alt        float32 `json:"alt"`        // Altitude
	StdDevHorz float32 `json:"stdDevHorz"` // Horizontal position standard deviation
	StdDevVert float32 `json:"stdDevVert"` // Vertical position standard deviation
	Vn         float32 `json:"vn"`         // True velocity in NORTH direction in earth-fixed NED frame
	Ve         float32 `json:"ve"`         // True velocity in EAST direction in earth-fixed NED frame
	Vd         float32 `json:"vd"`         // True velocity in DOWN direction in earth-fixed NED frame
}

func (self *SimState) MsgID() MessageID {
	return MSG_ID_SIM_STATE
}

func (self *SimState) MsgName() string {
	return "SimState"
}

func (self *SimState) Pack(p *Packet) error {
	payload := make([]byte, 84)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Q1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Q2))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Q3))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Q4))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Xacc))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Yacc))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Zacc))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Xgyro))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Ygyro))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.Zgyro))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.Lat))
	binary.LittleEndian.PutUint32(payload[56:], math.Float32bits(self.Lon))
	binary.LittleEndian.PutUint32(payload[60:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[64:], math.Float32bits(self.StdDevHorz))
	binary.LittleEndian.PutUint32(payload[68:], math.Float32bits(self.StdDevVert))
	binary.LittleEndian.PutUint32(payload[72:], math.Float32bits(self.Vn))
	binary.LittleEndian.PutUint32(payload[76:], math.Float32bits(self.Ve))
	binary.LittleEndian.PutUint32(payload[80:], math.Float32bits(self.Vd))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SimState) Unpack(p *Packet) error {
	if len(p.Payload) < 84 {
		return fmt.Errorf("payload too small")
	}
	self.Q1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Q2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Q3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Q4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.Lon = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[56:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60:]))
	self.StdDevHorz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[64:]))
	self.StdDevVert = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68:]))
	self.Vn = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72:]))
	self.Ve = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[76:]))
	self.Vd = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80:]))
	return nil
}

func (self *SimState) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SimStateFromJSON(data []byte) (*SimState, error) {
	p := &SimState{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status generated by radio and injected into MAVLink stream.
type RadioStatus struct {
	Rxerrors uint16 `json:"rxerrors"` // Count of radio packet receive errors (since boot).
	Fixed    uint16 `json:"fixed"`    // Count of error corrected radio packets (since boot).
	Rssi     uint8  `json:"rssi"`     // Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
	Remrssi  uint8  `json:"remrssi"`  // Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], 255: invalid/unknown.
	Txbuf    uint8  `json:"txbuf"`    // Remaining free transmitter buffer space.
	Noise    uint8  `json:"noise"`    // Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
	Remnoise uint8  `json:"remnoise"` // Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], 255: invalid/unknown.
}

func (self *RadioStatus) MsgID() MessageID {
	return MSG_ID_RADIO_STATUS
}

func (self *RadioStatus) MsgName() string {
	return "RadioStatus"
}

func (self *RadioStatus) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Rxerrors))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Fixed))
	payload[4] = byte(self.Rssi)
	payload[5] = byte(self.Remrssi)
	payload[6] = byte(self.Txbuf)
	payload[7] = byte(self.Noise)
	payload[8] = byte(self.Remnoise)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RadioStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	self.Rxerrors = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Fixed = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Rssi = uint8(p.Payload[4])
	self.Remrssi = uint8(p.Payload[5])
	self.Txbuf = uint8(p.Payload[6])
	self.Noise = uint8(p.Payload[7])
	self.Remnoise = uint8(p.Payload[8])
	return nil
}

func (self *RadioStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RadioStatusFromJSON(data []byte) (*RadioStatus, error) {
	p := &RadioStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// File transfer message
type FileTransferProtocol struct {
	TargetNetwork   uint8      `json:"targetNetwork"`   // Network ID (0 for broadcast)
	TargetSystem    uint8      `json:"targetSystem"`    // System ID (0 for broadcast)
	TargetComponent uint8      `json:"targetComponent"` // Component ID (0 for broadcast)
	Payload         [251]uint8 `json:"payload"`         // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

func (self *FileTransferProtocol) MsgID() MessageID {
	return MSG_ID_FILE_TRANSFER_PROTOCOL
}

func (self *FileTransferProtocol) MsgName() string {
	return "FileTransferProtocol"
}

func (self *FileTransferProtocol) Pack(p *Packet) error {
	payload := make([]byte, 254)
	payload[0] = byte(self.TargetNetwork)
	payload[1] = byte(self.TargetSystem)
	payload[2] = byte(self.TargetComponent)
	copy(payload[3:], self.Payload[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FileTransferProtocol) Unpack(p *Packet) error {
	if len(p.Payload) < 254 {
		return fmt.Errorf("payload too small")
	}
	self.TargetNetwork = uint8(p.Payload[0])
	self.TargetSystem = uint8(p.Payload[1])
	self.TargetComponent = uint8(p.Payload[2])
	copy(self.Payload[:], p.Payload[3:254])
	return nil
}

func (self *FileTransferProtocol) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func FileTransferProtocolFromJSON(data []byte) (*FileTransferProtocol, error) {
	p := &FileTransferProtocol{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Time synchronization message.
type Timesync struct {
	Tc1 int64 `json:"tc1"` // Time sync timestamp 1
	Ts1 int64 `json:"ts1"` // Time sync timestamp 2
}

func (self *Timesync) MsgID() MessageID {
	return MSG_ID_TIMESYNC
}

func (self *Timesync) MsgName() string {
	return "Timesync"
}

func (self *Timesync) Pack(p *Packet) error {
	payload := make([]byte, 16)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Tc1))
	binary.LittleEndian.PutUint64(payload[8:], uint64(self.Ts1))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Timesync) Unpack(p *Packet) error {
	if len(p.Payload) < 16 {
		return fmt.Errorf("payload too small")
	}
	self.Tc1 = int64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Ts1 = int64(binary.LittleEndian.Uint64(p.Payload[8:]))
	return nil
}

func (self *Timesync) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func TimesyncFromJSON(data []byte) (*Timesync, error) {
	p := &Timesync{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Camera-IMU triggering and synchronisation message.
type CameraTrigger struct {
	TimeUsec uint64 `json:"timeUsec"` // Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Seq      uint32 `json:"seq"`      // Image frame sequence
}

func (self *CameraTrigger) MsgID() MessageID {
	return MSG_ID_CAMERA_TRIGGER
}

func (self *CameraTrigger) MsgName() string {
	return "CameraTrigger"
}

func (self *CameraTrigger) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Seq))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraTrigger) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Seq = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

func (self *CameraTrigger) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CameraTriggerFromJSON(data []byte) (*CameraTrigger, error) {
	p := &CameraTrigger{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The global position, as returned by the Global Positioning System (GPS). This is
//                  NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate.
type HilGps struct {
	TimeUsec          uint64 `json:"timeUsec"`          // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Lat               int32  `json:"lat"`               // Latitude (WGS84)
	Lon               int32  `json:"lon"`               // Longitude (WGS84)
	Alt               int32  `json:"alt"`               // Altitude (MSL). Positive for up.
	Eph               uint16 `json:"eph"`               // GPS HDOP horizontal dilution of position. If unknown, set to: 65535
	Epv               uint16 `json:"epv"`               // GPS VDOP vertical dilution of position. If unknown, set to: 65535
	Vel               uint16 `json:"vel"`               // GPS ground speed. If unknown, set to: 65535
	Vn                int16  `json:"vn"`                // GPS velocity in NORTH direction in earth-fixed NED frame
	Ve                int16  `json:"ve"`                // GPS velocity in EAST direction in earth-fixed NED frame
	Vd                int16  `json:"vd"`                // GPS velocity in DOWN direction in earth-fixed NED frame
	Cog               uint16 `json:"cog"`               // Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535
	FixType           uint8  `json:"fixType"`           // 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	SatellitesVisible uint8  `json:"satellitesVisible"` // Number of satellites visible. If unknown, set to 255
}

func (self *HilGps) MsgID() MessageID {
	return MSG_ID_HIL_GPS
}

func (self *HilGps) MsgName() string {
	return "HilGps"
}

func (self *HilGps) Pack(p *Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Alt))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Eph))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Epv))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Vel))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Vn))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Ve))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Vd))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.Cog))
	payload[34] = byte(self.FixType)
	payload[35] = byte(self.SatellitesVisible)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilGps) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Eph = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Epv = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Vel = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Vn = int16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.Ve = int16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Vd = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.Cog = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.FixType = uint8(p.Payload[34])
	self.SatellitesVisible = uint8(p.Payload[35])
	return nil
}

func (self *HilGps) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilGpsFromJSON(data []byte) (*HilGps, error) {
	p := &HilGps{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
type HilOpticalFlow struct {
	TimeUsec            uint64  `json:"timeUsec"`            // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	IntegrationTimeUs   uint32  `json:"integrationTimeUs"`   // Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
	IntegratedX         float32 `json:"integratedX"`         // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
	IntegratedY         float32 `json:"integratedY"`         // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
	IntegratedXgyro     float32 `json:"integratedXgyro"`     // RH rotation around X axis
	IntegratedYgyro     float32 `json:"integratedYgyro"`     // RH rotation around Y axis
	IntegratedZgyro     float32 `json:"integratedZgyro"`     // RH rotation around Z axis
	TimeDeltaDistanceUs uint32  `json:"timeDeltaDistanceUs"` // Time since the distance was sampled.
	Distance            float32 `json:"distance"`            // Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
	Temperature         int16   `json:"temperature"`         // Temperature
	SensorId            uint8   `json:"sensorId"`            // Sensor ID
	Quality             uint8   `json:"quality"`             // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
}

func (self *HilOpticalFlow) MsgID() MessageID {
	return MSG_ID_HIL_OPTICAL_FLOW
}

func (self *HilOpticalFlow) MsgName() string {
	return "HilOpticalFlow"
}

func (self *HilOpticalFlow) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.IntegrationTimeUs))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.IntegratedX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.IntegratedY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.IntegratedXgyro))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.IntegratedYgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.IntegratedZgyro))
	binary.LittleEndian.PutUint32(payload[32:], uint32(self.TimeDeltaDistanceUs))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Distance))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.Temperature))
	payload[42] = byte(self.SensorId)
	payload[43] = byte(self.Quality)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilOpticalFlow) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.IntegrationTimeUs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.IntegratedX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.IntegratedY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.IntegratedXgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.IntegratedYgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.IntegratedZgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.TimeDeltaDistanceUs = uint32(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.SensorId = uint8(p.Payload[42])
	self.Quality = uint8(p.Payload[43])
	return nil
}

func (self *HilOpticalFlow) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilOpticalFlowFromJSON(data []byte) (*HilOpticalFlow, error) {
	p := &HilOpticalFlow{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
type HilStateQuaternion struct {
	TimeUsec           uint64     `json:"timeUsec"`           // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	AttitudeQuaternion [4]float32 `json:"attitudeQuaternion"` // Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
	Rollspeed          float32    `json:"rollspeed"`          // Body frame roll / phi angular speed
	Pitchspeed         float32    `json:"pitchspeed"`         // Body frame pitch / theta angular speed
	Yawspeed           float32    `json:"yawspeed"`           // Body frame yaw / psi angular speed
	Lat                int32      `json:"lat"`                // Latitude
	Lon                int32      `json:"lon"`                // Longitude
	Alt                int32      `json:"alt"`                // Altitude
	Vx                 int16      `json:"vx"`                 // Ground X Speed (Latitude)
	Vy                 int16      `json:"vy"`                 // Ground Y Speed (Longitude)
	Vz                 int16      `json:"vz"`                 // Ground Z Speed (Altitude)
	IndAirspeed        uint16     `json:"indAirspeed"`        // Indicated airspeed
	TrueAirspeed       uint16     `json:"trueAirspeed"`       // True airspeed
	Xacc               int16      `json:"xacc"`               // X acceleration
	Yacc               int16      `json:"yacc"`               // Y acceleration
	Zacc               int16      `json:"zacc"`               // Z acceleration
}

func (self *HilStateQuaternion) MsgID() MessageID {
	return MSG_ID_HIL_STATE_QUATERNION
}

func (self *HilStateQuaternion) MsgName() string {
	return "HilStateQuaternion"
}

func (self *HilStateQuaternion) Pack(p *Packet) error {
	payload := make([]byte, 64)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	for i, v := range self.AttitudeQuaternion {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Rollspeed))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Pitchspeed))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Yawspeed))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[40:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[44:], uint32(self.Alt))
	binary.LittleEndian.PutUint16(payload[48:], uint16(self.Vx))
	binary.LittleEndian.PutUint16(payload[50:], uint16(self.Vy))
	binary.LittleEndian.PutUint16(payload[52:], uint16(self.Vz))
	binary.LittleEndian.PutUint16(payload[54:], uint16(self.IndAirspeed))
	binary.LittleEndian.PutUint16(payload[56:], uint16(self.TrueAirspeed))
	binary.LittleEndian.PutUint16(payload[58:], uint16(self.Xacc))
	binary.LittleEndian.PutUint16(payload[60:], uint16(self.Yacc))
	binary.LittleEndian.PutUint16(payload[62:], uint16(self.Zacc))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HilStateQuaternion) Unpack(p *Packet) error {
	if len(p.Payload) < 64 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(self.AttitudeQuaternion); i++ {
		self.AttitudeQuaternion[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	self.Rollspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Pitchspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Yawspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.Vx = int16(binary.LittleEndian.Uint16(p.Payload[48:]))
	self.Vy = int16(binary.LittleEndian.Uint16(p.Payload[50:]))
	self.Vz = int16(binary.LittleEndian.Uint16(p.Payload[52:]))
	self.IndAirspeed = uint16(binary.LittleEndian.Uint16(p.Payload[54:]))
	self.TrueAirspeed = uint16(binary.LittleEndian.Uint16(p.Payload[56:]))
	self.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[58:]))
	self.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[60:]))
	self.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[62:]))
	return nil
}

func (self *HilStateQuaternion) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HilStateQuaternionFromJSON(data []byte) (*HilStateQuaternion, error) {
	p := &HilStateQuaternion{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu2 struct {
	TimeBootMs uint32 `json:"timeBootMs"` // Timestamp (time since system boot).
	Xacc       int16  `json:"xacc"`       // X acceleration
	Yacc       int16  `json:"yacc"`       // Y acceleration
	Zacc       int16  `json:"zacc"`       // Z acceleration
	Xgyro      int16  `json:"xgyro"`      // Angular speed around X axis
	Ygyro      int16  `json:"ygyro"`      // Angular speed around Y axis
	Zgyro      int16  `json:"zgyro"`      // Angular speed around Z axis
	Xmag       int16  `json:"xmag"`       // X Magnetic field
	Ymag       int16  `json:"ymag"`       // Y Magnetic field
	Zmag       int16  `json:"zmag"`       // Z Magnetic field
}

func (self *ScaledImu2) MsgID() MessageID {
	return MSG_ID_SCALED_IMU2
}

func (self *ScaledImu2) MsgName() string {
	return "ScaledImu2"
}

func (self *ScaledImu2) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Xacc))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Yacc))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Zacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Xgyro))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Ygyro))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Zgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Xmag))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Ymag))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Zmag))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ScaledImu2) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

func (self *ScaledImu2) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ScaledImu2FromJSON(data []byte) (*ScaledImu2, error) {
	p := &ScaledImu2{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called.
type LogRequestList struct {
	Start           uint16 `json:"start"`           // First log id (0 for first available)
	End             uint16 `json:"end"`             // Last log id (0xffff for last available)
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *LogRequestList) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_LIST
}

func (self *LogRequestList) MsgName() string {
	return "LogRequestList"
}

func (self *LogRequestList) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Start))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.End))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LogRequestList) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.Start = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.End = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	return nil
}

func (self *LogRequestList) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LogRequestListFromJSON(data []byte) (*LogRequestList, error) {
	p := &LogRequestList{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reply to LOG_REQUEST_LIST
type LogEntry struct {
	TimeUtc    uint32 `json:"timeUtc"`    // UTC timestamp of log since 1970, or 0 if not available
	Size       uint32 `json:"size"`       // Size of the log (may be approximate)
	Id         uint16 `json:"id"`         // Log id
	NumLogs    uint16 `json:"numLogs"`    // Total number of logs
	LastLogNum uint16 `json:"lastLogNum"` // High log number
}

func (self *LogEntry) MsgID() MessageID {
	return MSG_ID_LOG_ENTRY
}

func (self *LogEntry) MsgName() string {
	return "LogEntry"
}

func (self *LogEntry) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeUtc))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Size))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Id))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.NumLogs))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.LastLogNum))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LogEntry) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUtc = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Size = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Id = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.NumLogs = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.LastLogNum = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

func (self *LogEntry) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LogEntryFromJSON(data []byte) (*LogEntry, error) {
	p := &LogEntry{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a chunk of a log
type LogRequestData struct {
	Ofs             uint32 `json:"ofs"`             // Offset into the log
	Count           uint32 `json:"count"`           // Number of bytes
	Id              uint16 `json:"id"`              // Log id (from LOG_ENTRY reply)
	TargetSystem    uint8  `json:"targetSystem"`    // System ID
	TargetComponent uint8  `json:"targetComponent"` // Component ID
}

func (self *LogRequestData) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_DATA
}

func (self *LogRequestData) MsgName() string {
	return "LogRequestData"
}

func (self *LogRequestData) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Ofs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Count))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Id))
	payload[10] = byte(self.TargetSystem)
	payload[11] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LogRequestData) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Ofs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Count = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Id = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[10])
	self.TargetComponent = uint8(p.Payload[11])
	return nil
}

func (self *LogRequestData) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LogRequestDataFromJSON(data []byte) (*LogRequestData, error) {
	p := &LogRequestData{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reply to LOG_REQUEST_DATA
type LogData struct {
	Ofs   uint32    `json:"ofs"`   // Offset into the log
	Id    uint16    `json:"id"`    // Log id (from LOG_ENTRY reply)
	Count uint8     `json:"count"` // Number of bytes (zero for end of log)
	Data  [90]uint8 `json:"data"`  // log data
}

func (self *LogData) MsgID() MessageID {
	return MSG_ID_LOG_DATA
}

func (self *LogData) MsgName() string {
	return "LogData"
}

func (self *LogData) Pack(p *Packet) error {
	payload := make([]byte, 97)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Ofs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Id))
	payload[6] = byte(self.Count)
	copy(payload[7:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LogData) Unpack(p *Packet) error {
	if len(p.Payload) < 97 {
		return fmt.Errorf("payload too small")
	}
	self.Ofs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Id = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Count = uint8(p.Payload[6])
	copy(self.Data[:], p.Payload[7:97])
	return nil
}

func (self *LogData) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LogDataFromJSON(data []byte) (*LogData, error) {
	p := &LogData{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Erase all logs
type LogErase struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *LogErase) MsgID() MessageID {
	return MSG_ID_LOG_ERASE
}

func (self *LogErase) MsgName() string {
	return "LogErase"
}

func (self *LogErase) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LogErase) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

func (self *LogErase) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LogEraseFromJSON(data []byte) (*LogErase, error) {
	p := &LogErase{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Stop log transfer and resume normal logging
type LogRequestEnd struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID
	TargetComponent uint8 `json:"targetComponent"` // Component ID
}

func (self *LogRequestEnd) MsgID() MessageID {
	return MSG_ID_LOG_REQUEST_END
}

func (self *LogRequestEnd) MsgName() string {
	return "LogRequestEnd"
}

func (self *LogRequestEnd) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LogRequestEnd) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

func (self *LogRequestEnd) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LogRequestEndFromJSON(data []byte) (*LogRequestEnd, error) {
	p := &LogRequestEnd{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data for injecting into the onboard GPS (used for DGPS)
type GpsInjectData struct {
	TargetSystem    uint8      `json:"targetSystem"`    // System ID
	TargetComponent uint8      `json:"targetComponent"` // Component ID
	Len             uint8      `json:"len"`             // Data length
	Data            [110]uint8 `json:"data"`            // Raw data (110 is enough for 12 satellites of RTCMv2)
}

func (self *GpsInjectData) MsgID() MessageID {
	return MSG_ID_GPS_INJECT_DATA
}

func (self *GpsInjectData) MsgName() string {
	return "GpsInjectData"
}

func (self *GpsInjectData) Pack(p *Packet) error {
	payload := make([]byte, 113)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Len)
	copy(payload[3:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsInjectData) Unpack(p *Packet) error {
	if len(p.Payload) < 113 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Len = uint8(p.Payload[2])
	copy(self.Data[:], p.Payload[3:113])
	return nil
}

func (self *GpsInjectData) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsInjectDataFromJSON(data []byte) (*GpsInjectData, error) {
	p := &GpsInjectData{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Second GPS data.
type Gps2Raw struct {
	TimeUsec          uint64 `json:"timeUsec"`          // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Lat               int32  `json:"lat"`               // Latitude (WGS84)
	Lon               int32  `json:"lon"`               // Longitude (WGS84)
	Alt               int32  `json:"alt"`               // Altitude (MSL). Positive for up.
	DgpsAge           uint32 `json:"dgpsAge"`           // Age of DGPS info
	Eph               uint16 `json:"eph"`               // GPS HDOP horizontal dilution of position. If unknown, set to: UINT16_MAX
	Epv               uint16 `json:"epv"`               // GPS VDOP vertical dilution of position. If unknown, set to: UINT16_MAX
	Vel               uint16 `json:"vel"`               // GPS ground speed. If unknown, set to: UINT16_MAX
	Cog               uint16 `json:"cog"`               // Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	FixType           uint8  `json:"fixType"`           // GPS fix type.
	SatellitesVisible uint8  `json:"satellitesVisible"` // Number of satellites visible. If unknown, set to 255
	DgpsNumch         uint8  `json:"dgpsNumch"`         // Number of DGPS satellites
}

func (self *Gps2Raw) MsgID() MessageID {
	return MSG_ID_GPS2_RAW
}

func (self *Gps2Raw) MsgName() string {
	return "Gps2Raw"
}

func (self *Gps2Raw) Pack(p *Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Alt))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.DgpsAge))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Eph))
	binary.LittleEndian.PutUint16(payload[26:], uint16(self.Epv))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Vel))
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.Cog))
	payload[32] = byte(self.FixType)
	payload[33] = byte(self.SatellitesVisible)
	payload[34] = byte(self.DgpsNumch)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Gps2Raw) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Alt = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.DgpsAge = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Eph = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.Epv = uint16(binary.LittleEndian.Uint16(p.Payload[26:]))
	self.Vel = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.Cog = uint16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.FixType = uint8(p.Payload[32])
	self.SatellitesVisible = uint8(p.Payload[33])
	self.DgpsNumch = uint8(p.Payload[34])
	return nil
}

func (self *Gps2Raw) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Gps2RawFromJSON(data []byte) (*Gps2Raw, error) {
	p := &Gps2Raw{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Power supply status
type PowerStatus struct {
	Vcc    uint16 `json:"vcc"`    // 5V rail voltage.
	Vservo uint16 `json:"vservo"` // Servo rail voltage.
	Flags  uint16 `json:"flags"`  // Bitmap of power supply status flags.
}

func (self *PowerStatus) MsgID() MessageID {
	return MSG_ID_POWER_STATUS
}

func (self *PowerStatus) MsgName() string {
	return "PowerStatus"
}

func (self *PowerStatus) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Vcc))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Vservo))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Flags))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PowerStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.Vcc = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Vservo = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	return nil
}

func (self *PowerStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func PowerStatusFromJSON(data []byte) (*PowerStatus, error) {
	p := &PowerStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
type SerialControl struct {
	Baudrate uint32    `json:"baudrate"` // Baudrate of transfer. Zero means no change.
	Timeout  uint16    `json:"timeout"`  // Timeout for reply data
	Device   uint8     `json:"device"`   // Serial control device type.
	Flags    uint8     `json:"flags"`    // Bitmap of serial control flags.
	Count    uint8     `json:"count"`    // how many bytes in this transfer
	Data     [70]uint8 `json:"data"`     // serial data
}

func (self *SerialControl) MsgID() MessageID {
	return MSG_ID_SERIAL_CONTROL
}

func (self *SerialControl) MsgName() string {
	return "SerialControl"
}

func (self *SerialControl) Pack(p *Packet) error {
	payload := make([]byte, 79)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Baudrate))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Timeout))
	payload[6] = byte(self.Device)
	payload[7] = byte(self.Flags)
	payload[8] = byte(self.Count)
	copy(payload[9:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SerialControl) Unpack(p *Packet) error {
	if len(p.Payload) < 79 {
		return fmt.Errorf("payload too small")
	}
	self.Baudrate = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Timeout = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Device = uint8(p.Payload[6])
	self.Flags = uint8(p.Payload[7])
	self.Count = uint8(p.Payload[8])
	copy(self.Data[:], p.Payload[9:79])
	return nil
}

func (self *SerialControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SerialControlFromJSON(data []byte) (*SerialControl, error) {
	p := &SerialControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type GpsRtk struct {
	TimeLastBaselineMs uint32 `json:"timeLastBaselineMs"` // Time since boot of last baseline message received.
	Tow                uint32 `json:"tow"`                // GPS Time of Week of last baseline
	BaselineAMm        int32  `json:"baselineAMm"`        // Current baseline in ECEF x or NED north component.
	BaselineBMm        int32  `json:"baselineBMm"`        // Current baseline in ECEF y or NED east component.
	BaselineCMm        int32  `json:"baselineCMm"`        // Current baseline in ECEF z or NED down component.
	Accuracy           uint32 `json:"accuracy"`           // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  `json:"iarNumHypotheses"`   // Current number of integer ambiguity hypotheses.
	Wn                 uint16 `json:"wn"`                 // GPS Week Number of last baseline
	RtkReceiverId      uint8  `json:"rtkReceiverId"`      // Identification of connected RTK receiver.
	RtkHealth          uint8  `json:"rtkHealth"`          // GPS-specific health report for RTK data.
	RtkRate            uint8  `json:"rtkRate"`            // Rate of baseline messages being received by GPS
	Nsats              uint8  `json:"nsats"`              // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  `json:"baselineCoordsType"` // Coordinate system of baseline
}

func (self *GpsRtk) MsgID() MessageID {
	return MSG_ID_GPS_RTK
}

func (self *GpsRtk) MsgName() string {
	return "GpsRtk"
}

func (self *GpsRtk) Pack(p *Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeLastBaselineMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Tow))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.BaselineAMm))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.BaselineBMm))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.BaselineCMm))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Accuracy))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.IarNumHypotheses))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Wn))
	payload[30] = byte(self.RtkReceiverId)
	payload[31] = byte(self.RtkHealth)
	payload[32] = byte(self.RtkRate)
	payload[33] = byte(self.Nsats)
	payload[34] = byte(self.BaselineCoordsType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsRtk) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	self.TimeLastBaselineMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Tow = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.BaselineAMm = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.BaselineBMm = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.BaselineCMm = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Accuracy = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.IarNumHypotheses = int32(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Wn = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.RtkReceiverId = uint8(p.Payload[30])
	self.RtkHealth = uint8(p.Payload[31])
	self.RtkRate = uint8(p.Payload[32])
	self.Nsats = uint8(p.Payload[33])
	self.BaselineCoordsType = uint8(p.Payload[34])
	return nil
}

func (self *GpsRtk) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsRtkFromJSON(data []byte) (*GpsRtk, error) {
	p := &GpsRtk{}
	err := json.Unmarshal(data, p)
	return p, err
}

// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
type Gps2Rtk struct {
	TimeLastBaselineMs uint32 `json:"timeLastBaselineMs"` // Time since boot of last baseline message received.
	Tow                uint32 `json:"tow"`                // GPS Time of Week of last baseline
	BaselineAMm        int32  `json:"baselineAMm"`        // Current baseline in ECEF x or NED north component.
	BaselineBMm        int32  `json:"baselineBMm"`        // Current baseline in ECEF y or NED east component.
	BaselineCMm        int32  `json:"baselineCMm"`        // Current baseline in ECEF z or NED down component.
	Accuracy           uint32 `json:"accuracy"`           // Current estimate of baseline accuracy.
	IarNumHypotheses   int32  `json:"iarNumHypotheses"`   // Current number of integer ambiguity hypotheses.
	Wn                 uint16 `json:"wn"`                 // GPS Week Number of last baseline
	RtkReceiverId      uint8  `json:"rtkReceiverId"`      // Identification of connected RTK receiver.
	RtkHealth          uint8  `json:"rtkHealth"`          // GPS-specific health report for RTK data.
	RtkRate            uint8  `json:"rtkRate"`            // Rate of baseline messages being received by GPS
	Nsats              uint8  `json:"nsats"`              // Current number of sats used for RTK calculation.
	BaselineCoordsType uint8  `json:"baselineCoordsType"` // Coordinate system of baseline
}

func (self *Gps2Rtk) MsgID() MessageID {
	return MSG_ID_GPS2_RTK
}

func (self *Gps2Rtk) MsgName() string {
	return "Gps2Rtk"
}

func (self *Gps2Rtk) Pack(p *Packet) error {
	payload := make([]byte, 35)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeLastBaselineMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Tow))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.BaselineAMm))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.BaselineBMm))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.BaselineCMm))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Accuracy))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.IarNumHypotheses))
	binary.LittleEndian.PutUint16(payload[28:], uint16(self.Wn))
	payload[30] = byte(self.RtkReceiverId)
	payload[31] = byte(self.RtkHealth)
	payload[32] = byte(self.RtkRate)
	payload[33] = byte(self.Nsats)
	payload[34] = byte(self.BaselineCoordsType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Gps2Rtk) Unpack(p *Packet) error {
	if len(p.Payload) < 35 {
		return fmt.Errorf("payload too small")
	}
	self.TimeLastBaselineMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Tow = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.BaselineAMm = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.BaselineBMm = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.BaselineCMm = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Accuracy = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.IarNumHypotheses = int32(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Wn = uint16(binary.LittleEndian.Uint16(p.Payload[28:]))
	self.RtkReceiverId = uint8(p.Payload[30])
	self.RtkHealth = uint8(p.Payload[31])
	self.RtkRate = uint8(p.Payload[32])
	self.Nsats = uint8(p.Payload[33])
	self.BaselineCoordsType = uint8(p.Payload[34])
	return nil
}

func (self *Gps2Rtk) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Gps2RtkFromJSON(data []byte) (*Gps2Rtk, error) {
	p := &Gps2Rtk{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
type ScaledImu3 struct {
	TimeBootMs uint32 `json:"timeBootMs"` // Timestamp (time since system boot).
	Xacc       int16  `json:"xacc"`       // X acceleration
	Yacc       int16  `json:"yacc"`       // Y acceleration
	Zacc       int16  `json:"zacc"`       // Z acceleration
	Xgyro      int16  `json:"xgyro"`      // Angular speed around X axis
	Ygyro      int16  `json:"ygyro"`      // Angular speed around Y axis
	Zgyro      int16  `json:"zgyro"`      // Angular speed around Z axis
	Xmag       int16  `json:"xmag"`       // X Magnetic field
	Ymag       int16  `json:"ymag"`       // Y Magnetic field
	Zmag       int16  `json:"zmag"`       // Z Magnetic field
}

func (self *ScaledImu3) MsgID() MessageID {
	return MSG_ID_SCALED_IMU3
}

func (self *ScaledImu3) MsgName() string {
	return "ScaledImu3"
}

func (self *ScaledImu3) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Xacc))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Yacc))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Zacc))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Xgyro))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Ygyro))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Zgyro))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Xmag))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Ymag))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Zmag))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ScaledImu3) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Xacc = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Yacc = int16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Zacc = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Xgyro = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.Ygyro = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Zgyro = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Xmag = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Ymag = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Zmag = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

func (self *ScaledImu3) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ScaledImu3FromJSON(data []byte) (*ScaledImu3, error) {
	p := &ScaledImu3{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
type DataTransmissionHandshake struct {
	Size       uint32 `json:"size"`       // total data size (set on ACK only).
	Width      uint16 `json:"width"`      // Width of a matrix or image.
	Height     uint16 `json:"height"`     // Height of a matrix or image.
	Packets    uint16 `json:"packets"`    // Number of packets being sent (set on ACK only).
	Type       uint8  `json:"type"`       // Type of requested/acknowledged data.
	Payload    uint8  `json:"payload"`    // Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only).
	JpgQuality uint8  `json:"jpgQuality"` // JPEG quality. Values: [1-100].
}

func (self *DataTransmissionHandshake) MsgID() MessageID {
	return MSG_ID_DATA_TRANSMISSION_HANDSHAKE
}

func (self *DataTransmissionHandshake) MsgName() string {
	return "DataTransmissionHandshake"
}

func (self *DataTransmissionHandshake) Pack(p *Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Size))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Width))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Height))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Packets))
	payload[10] = byte(self.Type)
	payload[11] = byte(self.Payload)
	payload[12] = byte(self.JpgQuality)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DataTransmissionHandshake) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	self.Size = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Width = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Height = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Packets = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Type = uint8(p.Payload[10])
	self.Payload = uint8(p.Payload[11])
	self.JpgQuality = uint8(p.Payload[12])
	return nil
}

func (self *DataTransmissionHandshake) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DataTransmissionHandshakeFromJSON(data []byte) (*DataTransmissionHandshake, error) {
	p := &DataTransmissionHandshake{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
type EncapsulatedData struct {
	Seqnr uint16     `json:"seqnr"` // sequence number (starting with 0 on every transmission)
	Data  [253]uint8 `json:"data"`  // image data bytes
}

func (self *EncapsulatedData) MsgID() MessageID {
	return MSG_ID_ENCAPSULATED_DATA
}

func (self *EncapsulatedData) MsgName() string {
	return "EncapsulatedData"
}

func (self *EncapsulatedData) Pack(p *Packet) error {
	payload := make([]byte, 255)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Seqnr))
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *EncapsulatedData) Unpack(p *Packet) error {
	if len(p.Payload) < 255 {
		return fmt.Errorf("payload too small")
	}
	self.Seqnr = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	copy(self.Data[:], p.Payload[2:255])
	return nil
}

func (self *EncapsulatedData) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func EncapsulatedDataFromJSON(data []byte) (*EncapsulatedData, error) {
	p := &EncapsulatedData{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Distance sensor information for an onboard rangefinder.
type DistanceSensor struct {
	TimeBootMs      uint32 `json:"timeBootMs"`      // Timestamp (time since system boot).
	MinDistance     uint16 `json:"minDistance"`     // Minimum distance the sensor can measure
	MaxDistance     uint16 `json:"maxDistance"`     // Maximum distance the sensor can measure
	CurrentDistance uint16 `json:"currentDistance"` // Current distance reading
	Type            uint8  `json:"type"`            // Type of distance sensor.
	Id              uint8  `json:"id"`              // Onboard ID of the sensor
	Orientation     uint8  `json:"orientation"`     // Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
	Covariance      uint8  `json:"covariance"`      // Measurement variance. Max standard deviation is 6cm. 256 if unknown.
}

func (self *DistanceSensor) MsgID() MessageID {
	return MSG_ID_DISTANCE_SENSOR
}

func (self *DistanceSensor) MsgName() string {
	return "DistanceSensor"
}

func (self *DistanceSensor) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.MinDistance))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.MaxDistance))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.CurrentDistance))
	payload[10] = byte(self.Type)
	payload[11] = byte(self.Id)
	payload[12] = byte(self.Orientation)
	payload[13] = byte(self.Covariance)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DistanceSensor) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.MinDistance = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.MaxDistance = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.CurrentDistance = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Type = uint8(p.Payload[10])
	self.Id = uint8(p.Payload[11])
	self.Orientation = uint8(p.Payload[12])
	self.Covariance = uint8(p.Payload[13])
	return nil
}

func (self *DistanceSensor) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DistanceSensorFromJSON(data []byte) (*DistanceSensor, error) {
	p := &DistanceSensor{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request for terrain data and terrain status
type TerrainRequest struct {
	Mask        uint64 `json:"mask"`        // Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
	Lat         int32  `json:"lat"`         // Latitude of SW corner of first grid
	Lon         int32  `json:"lon"`         // Longitude of SW corner of first grid
	GridSpacing uint16 `json:"gridSpacing"` // Grid spacing
}

func (self *TerrainRequest) MsgID() MessageID {
	return MSG_ID_TERRAIN_REQUEST
}

func (self *TerrainRequest) MsgName() string {
	return "TerrainRequest"
}

func (self *TerrainRequest) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Mask))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lon))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.GridSpacing))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *TerrainRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.Mask = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.GridSpacing = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	return nil
}

func (self *TerrainRequest) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func TerrainRequestFromJSON(data []byte) (*TerrainRequest, error) {
	p := &TerrainRequest{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST
type TerrainData struct {
	Lat         int32     `json:"lat"`         // Latitude of SW corner of first grid
	Lon         int32     `json:"lon"`         // Longitude of SW corner of first grid
	GridSpacing uint16    `json:"gridSpacing"` // Grid spacing
	Data        [16]int16 `json:"data"`        // Terrain data MSL
	Gridbit     uint8     `json:"gridbit"`     // bit within the terrain request mask
}

func (self *TerrainData) MsgID() MessageID {
	return MSG_ID_TERRAIN_DATA
}

func (self *TerrainData) MsgName() string {
	return "TerrainData"
}

func (self *TerrainData) Pack(p *Packet) error {
	payload := make([]byte, 43)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lon))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.GridSpacing))
	for i, v := range self.Data {
		binary.LittleEndian.PutUint16(payload[10+i*2:], uint16(v))
	}
	payload[42] = byte(self.Gridbit)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *TerrainData) Unpack(p *Packet) error {
	if len(p.Payload) < 43 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.GridSpacing = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	for i := 0; i < len(self.Data); i++ {
		self.Data[i] = int16(binary.LittleEndian.Uint16(p.Payload[10+i*2:]))
	}
	self.Gridbit = uint8(p.Payload[42])
	return nil
}

func (self *TerrainData) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func TerrainDataFromJSON(data []byte) (*TerrainData, error) {
	p := &TerrainData{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission.
type TerrainCheck struct {
	Lat int32 `json:"lat"` // Latitude
	Lon int32 `json:"lon"` // Longitude
}

func (self *TerrainCheck) MsgID() MessageID {
	return MSG_ID_TERRAIN_CHECK
}

func (self *TerrainCheck) MsgName() string {
	return "TerrainCheck"
}

func (self *TerrainCheck) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lon))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *TerrainCheck) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

func (self *TerrainCheck) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func TerrainCheckFromJSON(data []byte) (*TerrainCheck, error) {
	p := &TerrainCheck{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Response from a TERRAIN_CHECK request
type TerrainReport struct {
	Lat           int32   `json:"lat"`           // Latitude
	Lon           int32   `json:"lon"`           // Longitude
	TerrainHeight float32 `json:"terrainHeight"` // Terrain height MSL
	CurrentHeight float32 `json:"currentHeight"` // Current vehicle height above lat/lon terrain height
	Spacing       uint16  `json:"spacing"`       // grid spacing (zero if terrain at this location unavailable)
	Pending       uint16  `json:"pending"`       // Number of 4x4 terrain blocks waiting to be received or read from disk
	Loaded        uint16  `json:"loaded"`        // Number of 4x4 terrain blocks in memory
}

func (self *TerrainReport) MsgID() MessageID {
	return MSG_ID_TERRAIN_REPORT
}

func (self *TerrainReport) MsgName() string {
	return "TerrainReport"
}

func (self *TerrainReport) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.TerrainHeight))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.CurrentHeight))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Spacing))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Pending))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Loaded))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *TerrainReport) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.TerrainHeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.CurrentHeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Spacing = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Pending = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.Loaded = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

func (self *TerrainReport) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func TerrainReportFromJSON(data []byte) (*TerrainReport, error) {
	p := &TerrainReport{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Barometer readings for 2nd barometer
type ScaledPressure2 struct {
	TimeBootMs  uint32  `json:"timeBootMs"`  // Timestamp (time since system boot).
	PressAbs    float32 `json:"pressAbs"`    // Absolute pressure
	PressDiff   float32 `json:"pressDiff"`   // Differential pressure
	Temperature int16   `json:"temperature"` // Temperature measurement
}

func (self *ScaledPressure2) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE2
}

func (self *ScaledPressure2) MsgName() string {
	return "ScaledPressure2"
}

func (self *ScaledPressure2) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.PressAbs))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.PressDiff))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Temperature))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ScaledPressure2) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

func (self *ScaledPressure2) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ScaledPressure2FromJSON(data []byte) (*ScaledPressure2, error) {
	p := &ScaledPressure2{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Motion capture attitude and position
type AttPosMocap struct {
	TimeUsec uint64     `json:"timeUsec"` // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Q        [4]float32 `json:"q"`        // Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	X        float32    `json:"x"`        // X position (NED)
	Y        float32    `json:"y"`        // Y position (NED)
	Z        float32    `json:"z"`        // Z position (NED)
}

func (self *AttPosMocap) MsgID() MessageID {
	return MSG_ID_ATT_POS_MOCAP
}

func (self *AttPosMocap) MsgName() string {
	return "AttPosMocap"
}

func (self *AttPosMocap) Pack(p *Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Z))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AttPosMocap) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	return nil
}

func (self *AttPosMocap) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AttPosMocapFromJSON(data []byte) (*AttPosMocap, error) {
	p := &AttPosMocap{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set the vehicle attitude and body angular rates.
type SetActuatorControlTarget struct {
	TimeUsec        uint64     `json:"timeUsec"`        // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Controls        [8]float32 `json:"controls"`        // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx        uint8      `json:"groupMlx"`        // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
	TargetSystem    uint8      `json:"targetSystem"`    // System ID
	TargetComponent uint8      `json:"targetComponent"` // Component ID
}

func (self *SetActuatorControlTarget) MsgID() MessageID {
	return MSG_ID_SET_ACTUATOR_CONTROL_TARGET
}

func (self *SetActuatorControlTarget) MsgName() string {
	return "SetActuatorControlTarget"
}

func (self *SetActuatorControlTarget) Pack(p *Packet) error {
	payload := make([]byte, 43)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	for i, v := range self.Controls {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	payload[40] = byte(self.GroupMlx)
	payload[41] = byte(self.TargetSystem)
	payload[42] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetActuatorControlTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 43 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(self.Controls); i++ {
		self.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	self.GroupMlx = uint8(p.Payload[40])
	self.TargetSystem = uint8(p.Payload[41])
	self.TargetComponent = uint8(p.Payload[42])
	return nil
}

func (self *SetActuatorControlTarget) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetActuatorControlTargetFromJSON(data []byte) (*SetActuatorControlTarget, error) {
	p := &SetActuatorControlTarget{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set the vehicle attitude and body angular rates.
type ActuatorControlTarget struct {
	TimeUsec uint64     `json:"timeUsec"` // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	Controls [8]float32 `json:"controls"` // Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
	GroupMlx uint8      `json:"groupMlx"` // Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
}

func (self *ActuatorControlTarget) MsgID() MessageID {
	return MSG_ID_ACTUATOR_CONTROL_TARGET
}

func (self *ActuatorControlTarget) MsgName() string {
	return "ActuatorControlTarget"
}

func (self *ActuatorControlTarget) Pack(p *Packet) error {
	payload := make([]byte, 41)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	for i, v := range self.Controls {
		binary.LittleEndian.PutUint32(payload[8+i*4:], math.Float32bits(v))
	}
	payload[40] = byte(self.GroupMlx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ActuatorControlTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 41 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	for i := 0; i < len(self.Controls); i++ {
		self.Controls[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8+i*4:]))
	}
	self.GroupMlx = uint8(p.Payload[40])
	return nil
}

func (self *ActuatorControlTarget) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ActuatorControlTargetFromJSON(data []byte) (*ActuatorControlTarget, error) {
	p := &ActuatorControlTarget{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The current system altitude.
type Altitude struct {
	TimeUsec          uint64  `json:"timeUsec"`          // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	AltitudeMonotonic float32 `json:"altitudeMonotonic"` // This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
	AltitudeAmsl      float32 `json:"altitudeAmsl"`      // This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output MSL by default and not the WGS84 altitude.
	AltitudeLocal     float32 `json:"altitudeLocal"`     // This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
	AltitudeRelative  float32 `json:"altitudeRelative"`  // This is the altitude above the home position. It resets on each change of the current home position.
	AltitudeTerrain   float32 `json:"altitudeTerrain"`   // This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
	BottomClearance   float32 `json:"bottomClearance"`   // This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
}

func (self *Altitude) MsgID() MessageID {
	return MSG_ID_ALTITUDE
}

func (self *Altitude) MsgName() string {
	return "Altitude"
}

func (self *Altitude) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.AltitudeMonotonic))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.AltitudeAmsl))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.AltitudeLocal))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.AltitudeRelative))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.AltitudeTerrain))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.BottomClearance))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Altitude) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.AltitudeMonotonic = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AltitudeAmsl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.AltitudeLocal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.AltitudeRelative = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.AltitudeTerrain = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.BottomClearance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

func (self *Altitude) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AltitudeFromJSON(data []byte) (*Altitude, error) {
	p := &Altitude{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The autopilot is requesting a resource (file, binary, other type of data)
type ResourceRequest struct {
	RequestId    uint8      `json:"requestId"`    // Request ID. This ID should be re-used when sending back URI contents
	UriType      uint8      `json:"uriType"`      // The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
	Uri          [120]uint8 `json:"uri"`          // The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
	TransferType uint8      `json:"transferType"` // The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
	Storage      [120]uint8 `json:"storage"`      // The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
}

func (self *ResourceRequest) MsgID() MessageID {
	return MSG_ID_RESOURCE_REQUEST
}

func (self *ResourceRequest) MsgName() string {
	return "ResourceRequest"
}

func (self *ResourceRequest) Pack(p *Packet) error {
	payload := make([]byte, 243)
	payload[0] = byte(self.RequestId)
	payload[1] = byte(self.UriType)
	copy(payload[2:], self.Uri[:])
	payload[122] = byte(self.TransferType)
	copy(payload[123:], self.Storage[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ResourceRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 243 {
		return fmt.Errorf("payload too small")
	}
	self.RequestId = uint8(p.Payload[0])
	self.UriType = uint8(p.Payload[1])
	copy(self.Uri[:], p.Payload[2:122])
	self.TransferType = uint8(p.Payload[122])
	copy(self.Storage[:], p.Payload[123:243])
	return nil
}

func (self *ResourceRequest) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ResourceRequestFromJSON(data []byte) (*ResourceRequest, error) {
	p := &ResourceRequest{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Barometer readings for 3rd barometer
type ScaledPressure3 struct {
	TimeBootMs  uint32  `json:"timeBootMs"`  // Timestamp (time since system boot).
	PressAbs    float32 `json:"pressAbs"`    // Absolute pressure
	PressDiff   float32 `json:"pressDiff"`   // Differential pressure
	Temperature int16   `json:"temperature"` // Temperature measurement
}

func (self *ScaledPressure3) MsgID() MessageID {
	return MSG_ID_SCALED_PRESSURE3
}

func (self *ScaledPressure3) MsgName() string {
	return "ScaledPressure3"
}

func (self *ScaledPressure3) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.PressAbs))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.PressDiff))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Temperature))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ScaledPressure3) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PressAbs = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PressDiff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	return nil
}

func (self *ScaledPressure3) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ScaledPressure3FromJSON(data []byte) (*ScaledPressure3, error) {
	p := &ScaledPressure3{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Current motion information from a designated system
type FollowTarget struct {
	Timestamp       uint64     `json:"timestamp"`       // Timestamp (time since system boot).
	CustomState     uint64     `json:"customState"`     // button states or switches of a tracker device
	Lat             int32      `json:"lat"`             // Latitude (WGS84)
	Lon             int32      `json:"lon"`             // Longitude (WGS84)
	Alt             float32    `json:"alt"`             // Altitude (MSL)
	Vel             [3]float32 `json:"vel"`             // target velocity (0,0,0) for unknown
	Acc             [3]float32 `json:"acc"`             // linear target acceleration (0,0,0) for unknown
	AttitudeQ       [4]float32 `json:"attitudeQ"`       // (1 0 0 0 for unknown)
	Rates           [3]float32 `json:"rates"`           // (0 0 0 for unknown)
	PositionCov     [3]float32 `json:"positionCov"`     // eph epv
	EstCapabilities uint8      `json:"estCapabilities"` // bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
}

func (self *FollowTarget) MsgID() MessageID {
	return MSG_ID_FOLLOW_TARGET
}

func (self *FollowTarget) MsgName() string {
	return "FollowTarget"
}

func (self *FollowTarget) Pack(p *Packet) error {
	payload := make([]byte, 93)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Timestamp))
	binary.LittleEndian.PutUint64(payload[8:], uint64(self.CustomState))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Alt))
	for i, v := range self.Vel {
		binary.LittleEndian.PutUint32(payload[28+i*4:], math.Float32bits(v))
	}
	for i, v := range self.Acc {
		binary.LittleEndian.PutUint32(payload[40+i*4:], math.Float32bits(v))
	}
	for i, v := range self.AttitudeQ {
		binary.LittleEndian.PutUint32(payload[52+i*4:], math.Float32bits(v))
	}
	for i, v := range self.Rates {
		binary.LittleEndian.PutUint32(payload[68+i*4:], math.Float32bits(v))
	}
	for i, v := range self.PositionCov {
		binary.LittleEndian.PutUint32(payload[80+i*4:], math.Float32bits(v))
	}
	payload[92] = byte(self.EstCapabilities)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FollowTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 93 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.CustomState = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	for i := 0; i < len(self.Vel); i++ {
		self.Vel[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28+i*4:]))
	}
	for i := 0; i < len(self.Acc); i++ {
		self.Acc[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40+i*4:]))
	}
	for i := 0; i < len(self.AttitudeQ); i++ {
		self.AttitudeQ[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52+i*4:]))
	}
	for i := 0; i < len(self.Rates); i++ {
		self.Rates[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[68+i*4:]))
	}
	for i := 0; i < len(self.PositionCov); i++ {
		self.PositionCov[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[80+i*4:]))
	}
	self.EstCapabilities = uint8(p.Payload[92])
	return nil
}

func (self *FollowTarget) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func FollowTargetFromJSON(data []byte) (*FollowTarget, error) {
	p := &FollowTarget{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The smoothed, monotonic system state used to feed the control loops of the system.
type ControlSystemState struct {
	TimeUsec    uint64     `json:"timeUsec"`    // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	XAcc        float32    `json:"xAcc"`        // X acceleration in body frame
	YAcc        float32    `json:"yAcc"`        // Y acceleration in body frame
	ZAcc        float32    `json:"zAcc"`        // Z acceleration in body frame
	XVel        float32    `json:"xVel"`        // X velocity in body frame
	YVel        float32    `json:"yVel"`        // Y velocity in body frame
	ZVel        float32    `json:"zVel"`        // Z velocity in body frame
	XPos        float32    `json:"xPos"`        // X position in local frame
	YPos        float32    `json:"yPos"`        // Y position in local frame
	ZPos        float32    `json:"zPos"`        // Z position in local frame
	Airspeed    float32    `json:"airspeed"`    // Airspeed, set to -1 if unknown
	VelVariance [3]float32 `json:"velVariance"` // Variance of body velocity estimate
	PosVariance [3]float32 `json:"posVariance"` // Variance in local position
	Q           [4]float32 `json:"q"`           // The attitude, represented as Quaternion
	RollRate    float32    `json:"rollRate"`    // Angular rate in roll axis
	PitchRate   float32    `json:"pitchRate"`   // Angular rate in pitch axis
	YawRate     float32    `json:"yawRate"`     // Angular rate in yaw axis
}

func (self *ControlSystemState) MsgID() MessageID {
	return MSG_ID_CONTROL_SYSTEM_STATE
}

func (self *ControlSystemState) MsgName() string {
	return "ControlSystemState"
}

func (self *ControlSystemState) Pack(p *Packet) error {
	payload := make([]byte, 100)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.XAcc))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.YAcc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.ZAcc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.XVel))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.YVel))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.ZVel))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.XPos))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.YPos))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.ZPos))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Airspeed))
	for i, v := range self.VelVariance {
		binary.LittleEndian.PutUint32(payload[48+i*4:], math.Float32bits(v))
	}
	for i, v := range self.PosVariance {
		binary.LittleEndian.PutUint32(payload[60+i*4:], math.Float32bits(v))
	}
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[72+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[88:], math.Float32bits(self.RollRate))
	binary.LittleEndian.PutUint32(payload[92:], math.Float32bits(self.PitchRate))
	binary.LittleEndian.PutUint32(payload[96:], math.Float32bits(self.YawRate))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ControlSystemState) Unpack(p *Packet) error {
	if len(p.Payload) < 100 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.XAcc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.YAcc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.ZAcc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.XVel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.YVel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.ZVel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.XPos = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.YPos = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.ZPos = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Airspeed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	for i := 0; i < len(self.VelVariance); i++ {
		self.VelVariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48+i*4:]))
	}
	for i := 0; i < len(self.PosVariance); i++ {
		self.PosVariance[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[60+i*4:]))
	}
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[72+i*4:]))
	}
	self.RollRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[88:]))
	self.PitchRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[92:]))
	self.YawRate = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[96:]))
	return nil
}

func (self *ControlSystemState) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ControlSystemStateFromJSON(data []byte) (*ControlSystemState, error) {
	p := &ControlSystemState{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Battery information. Updates GCS with flight controller battery status. Use SMART_BATTERY_* messages instead for smart batteries.
type BatteryStatus struct {
	CurrentConsumed  int32      `json:"currentConsumed"`  // Consumed charge, -1: autopilot does not provide consumption estimate
	EnergyConsumed   int32      `json:"energyConsumed"`   // Consumed energy, -1: autopilot does not provide energy consumption estimate
	Temperature      int16      `json:"temperature"`      // Temperature of the battery. INT16_MAX for unknown temperature.
	Voltages         [10]uint16 `json:"voltages"`         // Battery voltage of cells. Cells above the valid cell count for this battery should have the UINT16_MAX value.
	CurrentBattery   int16      `json:"currentBattery"`   // Battery current, -1: autopilot does not measure the current
	Id               uint8      `json:"id"`               // Battery ID
	BatteryFunction  uint8      `json:"batteryFunction"`  // Function of the battery
	Type             uint8      `json:"type"`             // Type (chemistry) of the battery
	BatteryRemaining int8       `json:"batteryRemaining"` // Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
}

func (self *BatteryStatus) MsgID() MessageID {
	return MSG_ID_BATTERY_STATUS
}

func (self *BatteryStatus) MsgName() string {
	return "BatteryStatus"
}

func (self *BatteryStatus) Pack(p *Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.CurrentConsumed))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.EnergyConsumed))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Temperature))
	for i, v := range self.Voltages {
		binary.LittleEndian.PutUint16(payload[10+i*2:], uint16(v))
	}
	binary.LittleEndian.PutUint16(payload[30:], uint16(self.CurrentBattery))
	payload[32] = byte(self.Id)
	payload[33] = byte(self.BatteryFunction)
	payload[34] = byte(self.Type)
	payload[35] = byte(self.BatteryRemaining)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *BatteryStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	self.CurrentConsumed = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.EnergyConsumed = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Temperature = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	for i := 0; i < len(self.Voltages); i++ {
		self.Voltages[i] = uint16(binary.LittleEndian.Uint16(p.Payload[10+i*2:]))
	}
	self.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[30:]))
	self.Id = uint8(p.Payload[32])
	self.BatteryFunction = uint8(p.Payload[33])
	self.Type = uint8(p.Payload[34])
	self.BatteryRemaining = int8(p.Payload[35])
	return nil
}

func (self *BatteryStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func BatteryStatusFromJSON(data []byte) (*BatteryStatus, error) {
	p := &BatteryStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Version and capability of autopilot software
type AutopilotVersion struct {
	Capabilities            uint64   `json:"capabilities"`            // Bitmap of capabilities
	Uid                     uint64   `json:"uid"`                     // UID if provided by hardware (see uid2)
	FlightSwVersion         uint32   `json:"flightSwVersion"`         // Firmware version number
	MiddlewareSwVersion     uint32   `json:"middlewareSwVersion"`     // Middleware version number
	OsSwVersion             uint32   `json:"osSwVersion"`             // Operating system version number
	BoardVersion            uint32   `json:"boardVersion"`            // HW / board version (last 8 bytes should be silicon ID, if any)
	VendorId                uint16   `json:"vendorId"`                // ID of the board vendor
	ProductId               uint16   `json:"productId"`               // ID of the product
	FlightCustomVersion     [8]uint8 `json:"flightCustomVersion"`     // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	MiddlewareCustomVersion [8]uint8 `json:"middlewareCustomVersion"` // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
	OsCustomVersion         [8]uint8 `json:"osCustomVersion"`         // Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
}

func (self *AutopilotVersion) MsgID() MessageID {
	return MSG_ID_AUTOPILOT_VERSION
}

func (self *AutopilotVersion) MsgName() string {
	return "AutopilotVersion"
}

func (self *AutopilotVersion) Pack(p *Packet) error {
	payload := make([]byte, 60)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.Capabilities))
	binary.LittleEndian.PutUint64(payload[8:], uint64(self.Uid))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.FlightSwVersion))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.MiddlewareSwVersion))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.OsSwVersion))
	binary.LittleEndian.PutUint32(payload[28:], uint32(self.BoardVersion))
	binary.LittleEndian.PutUint16(payload[32:], uint16(self.VendorId))
	binary.LittleEndian.PutUint16(payload[34:], uint16(self.ProductId))
	copy(payload[36:], self.FlightCustomVersion[:])
	copy(payload[44:], self.MiddlewareCustomVersion[:])
	copy(payload[52:], self.OsCustomVersion[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AutopilotVersion) Unpack(p *Packet) error {
	if len(p.Payload) < 60 {
		return fmt.Errorf("payload too small")
	}
	self.Capabilities = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Uid = uint64(binary.LittleEndian.Uint64(p.Payload[8:]))
	self.FlightSwVersion = uint32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.MiddlewareSwVersion = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.OsSwVersion = uint32(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.BoardVersion = uint32(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.VendorId = uint16(binary.LittleEndian.Uint16(p.Payload[32:]))
	self.ProductId = uint16(binary.LittleEndian.Uint16(p.Payload[34:]))
	copy(self.FlightCustomVersion[:], p.Payload[36:44])
	copy(self.MiddlewareCustomVersion[:], p.Payload[44:52])
	copy(self.OsCustomVersion[:], p.Payload[52:60])
	return nil
}

func (self *AutopilotVersion) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AutopilotVersionFromJSON(data []byte) (*AutopilotVersion, error) {
	p := &AutopilotVersion{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
type LandingTarget struct {
	TimeUsec  uint64  `json:"timeUsec"`  // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	AngleX    float32 `json:"angleX"`    // X-axis angular offset of the target from the center of the image
	AngleY    float32 `json:"angleY"`    // Y-axis angular offset of the target from the center of the image
	Distance  float32 `json:"distance"`  // Distance to the target from the vehicle
	SizeX     float32 `json:"sizeX"`     // Size of target along x-axis
	SizeY     float32 `json:"sizeY"`     // Size of target along y-axis
	TargetNum uint8   `json:"targetNum"` // The ID of the target if multiple targets are present
	Frame     uint8   `json:"frame"`     // Coordinate frame used for following fields.
}

func (self *LandingTarget) MsgID() MessageID {
	return MSG_ID_LANDING_TARGET
}

func (self *LandingTarget) MsgName() string {
	return "LandingTarget"
}

func (self *LandingTarget) Pack(p *Packet) error {
	payload := make([]byte, 30)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.AngleX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.AngleY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Distance))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.SizeX))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.SizeY))
	payload[28] = byte(self.TargetNum)
	payload[29] = byte(self.Frame)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LandingTarget) Unpack(p *Packet) error {
	if len(p.Payload) < 30 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.AngleX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AngleY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.SizeX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.SizeY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.TargetNum = uint8(p.Payload[28])
	self.Frame = uint8(p.Payload[29])
	return nil
}

func (self *LandingTarget) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LandingTargetFromJSON(data []byte) (*LandingTarget, error) {
	p := &LandingTarget{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
type EstimatorStatus struct {
	TimeUsec         uint64  `json:"timeUsec"`         // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	VelRatio         float32 `json:"velRatio"`         // Velocity innovation test ratio
	PosHorizRatio    float32 `json:"posHorizRatio"`    // Horizontal position innovation test ratio
	PosVertRatio     float32 `json:"posVertRatio"`     // Vertical position innovation test ratio
	MagRatio         float32 `json:"magRatio"`         // Magnetometer innovation test ratio
	HaglRatio        float32 `json:"haglRatio"`        // Height above terrain innovation test ratio
	TasRatio         float32 `json:"tasRatio"`         // True airspeed innovation test ratio
	PosHorizAccuracy float32 `json:"posHorizAccuracy"` // Horizontal position 1-STD accuracy relative to the EKF local origin
	PosVertAccuracy  float32 `json:"posVertAccuracy"`  // Vertical position 1-STD accuracy relative to the EKF local origin
	Flags            uint16  `json:"flags"`            // Bitmap indicating which EKF outputs are valid.
}

func (self *EstimatorStatus) MsgID() MessageID {
	return MSG_ID_ESTIMATOR_STATUS
}

func (self *EstimatorStatus) MsgName() string {
	return "EstimatorStatus"
}

func (self *EstimatorStatus) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.VelRatio))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.PosHorizRatio))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.PosVertRatio))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.MagRatio))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.HaglRatio))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.TasRatio))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.PosHorizAccuracy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.PosVertAccuracy))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.Flags))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *EstimatorStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.VelRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.PosHorizRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.PosVertRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.MagRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.HaglRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.TasRatio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.PosHorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.PosVertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	return nil
}

func (self *EstimatorStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func EstimatorStatusFromJSON(data []byte) (*EstimatorStatus, error) {
	p := &EstimatorStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Wind covariance estimate from vehicle.
type WindCov struct {
	TimeUsec      uint64  `json:"timeUsec"`      // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	WindX         float32 `json:"windX"`         // Wind in X (NED) direction
	WindY         float32 `json:"windY"`         // Wind in Y (NED) direction
	WindZ         float32 `json:"windZ"`         // Wind in Z (NED) direction
	VarHoriz      float32 `json:"varHoriz"`      // Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate.
	VarVert       float32 `json:"varVert"`       // Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate.
	WindAlt       float32 `json:"windAlt"`       // Altitude (MSL) that this measurement was taken at
	HorizAccuracy float32 `json:"horizAccuracy"` // Horizontal speed 1-STD accuracy
	VertAccuracy  float32 `json:"vertAccuracy"`  // Vertical speed 1-STD accuracy
}

func (self *WindCov) MsgID() MessageID {
	return MSG_ID_WIND_COV
}

func (self *WindCov) MsgName() string {
	return "WindCov"
}

func (self *WindCov) Pack(p *Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.WindX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.WindY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.WindZ))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.VarHoriz))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.VarVert))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.WindAlt))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.HorizAccuracy))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.VertAccuracy))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *WindCov) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.WindX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.WindY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.WindZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.VarHoriz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.VarVert = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.WindAlt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.HorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.VertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	return nil
}

func (self *WindCov) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func WindCovFromJSON(data []byte) (*WindCov, error) {
	p := &WindCov{}
	err := json.Unmarshal(data, p)
	return p, err
}

// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.
type GpsInput struct {
	TimeUsec          uint64  `json:"timeUsec"`          // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	TimeWeekMs        uint32  `json:"timeWeekMs"`        // GPS time (from start of GPS week)
	Lat               int32   `json:"lat"`               // Latitude (WGS84)
	Lon               int32   `json:"lon"`               // Longitude (WGS84)
	Alt               float32 `json:"alt"`               // Altitude (MSL). Positive for up.
	Hdop              float32 `json:"hdop"`              // GPS HDOP horizontal dilution of position
	Vdop              float32 `json:"vdop"`              // GPS VDOP vertical dilution of position
	Vn                float32 `json:"vn"`                // GPS velocity in NORTH direction in earth-fixed NED frame
	Ve                float32 `json:"ve"`                // GPS velocity in EAST direction in earth-fixed NED frame
	Vd                float32 `json:"vd"`                // GPS velocity in DOWN direction in earth-fixed NED frame
	SpeedAccuracy     float32 `json:"speedAccuracy"`     // GPS speed accuracy
	HorizAccuracy     float32 `json:"horizAccuracy"`     // GPS horizontal accuracy
	VertAccuracy      float32 `json:"vertAccuracy"`      // GPS vertical accuracy
	IgnoreFlags       uint16  `json:"ignoreFlags"`       // Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
	TimeWeek          uint16  `json:"timeWeek"`          // GPS week number
	GpsId             uint8   `json:"gpsId"`             // ID of the GPS for multiple GPS inputs
	FixType           uint8   `json:"fixType"`           // 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
	SatellitesVisible uint8   `json:"satellitesVisible"` // Number of satellites visible.
}

func (self *GpsInput) MsgID() MessageID {
	return MSG_ID_GPS_INPUT
}

func (self *GpsInput) MsgName() string {
	return "GpsInput"
}

func (self *GpsInput) Pack(p *Packet) error {
	payload := make([]byte, 63)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.TimeWeekMs))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Alt))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Hdop))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Vdop))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Vn))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Ve))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Vd))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.SpeedAccuracy))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.HorizAccuracy))
	binary.LittleEndian.PutUint32(payload[52:], math.Float32bits(self.VertAccuracy))
	binary.LittleEndian.PutUint16(payload[56:], uint16(self.IgnoreFlags))
	binary.LittleEndian.PutUint16(payload[58:], uint16(self.TimeWeek))
	payload[60] = byte(self.GpsId)
	payload[61] = byte(self.FixType)
	payload[62] = byte(self.SatellitesVisible)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsInput) Unpack(p *Packet) error {
	if len(p.Payload) < 63 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.TimeWeekMs = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Alt = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Hdop = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Vdop = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Vn = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Ve = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Vd = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.SpeedAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.HorizAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.VertAccuracy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[52:]))
	self.IgnoreFlags = uint16(binary.LittleEndian.Uint16(p.Payload[56:]))
	self.TimeWeek = uint16(binary.LittleEndian.Uint16(p.Payload[58:]))
	self.GpsId = uint8(p.Payload[60])
	self.FixType = uint8(p.Payload[61])
	self.SatellitesVisible = uint8(p.Payload[62])
	return nil
}

func (self *GpsInput) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsInputFromJSON(data []byte) (*GpsInput, error) {
	p := &GpsInput{}
	err := json.Unmarshal(data, p)
	return p, err
}

// RTCM message for injecting into the onboard GPS (used for DGPS)
type GpsRtcmData struct {
	Flags uint8      `json:"flags"` // LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
	Len   uint8      `json:"len"`   // data length
	Data  [180]uint8 `json:"data"`  // RTCM message (may be fragmented)
}

func (self *GpsRtcmData) MsgID() MessageID {
	return MSG_ID_GPS_RTCM_DATA
}

func (self *GpsRtcmData) MsgName() string {
	return "GpsRtcmData"
}

func (self *GpsRtcmData) Pack(p *Packet) error {
	payload := make([]byte, 182)
	payload[0] = byte(self.Flags)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GpsRtcmData) Unpack(p *Packet) error {
	if len(p.Payload) < 182 {
		return fmt.Errorf("payload too small")
	}
	self.Flags = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:182])
	return nil
}

func (self *GpsRtcmData) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GpsRtcmDataFromJSON(data []byte) (*GpsRtcmData, error) {
	p := &GpsRtcmData{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message appropriate for high latency connections like Iridium
type HighLatency struct {
	CustomMode       uint32 `json:"customMode"`       // A bitfield for use for autopilot-specific flags.
	Latitude         int32  `json:"latitude"`         // Latitude
	Longitude        int32  `json:"longitude"`        // Longitude
	Roll             int16  `json:"roll"`             // roll
	Pitch            int16  `json:"pitch"`            // pitch
	Heading          uint16 `json:"heading"`          // heading
	HeadingSp        int16  `json:"headingSp"`        // heading setpoint
	AltitudeAmsl     int16  `json:"altitudeAmsl"`     // Altitude above mean sea level
	AltitudeSp       int16  `json:"altitudeSp"`       // Altitude setpoint relative to the home position
	WpDistance       uint16 `json:"wpDistance"`       // distance to target
	BaseMode         uint8  `json:"baseMode"`         // Bitmap of enabled system modes.
	LandedState      uint8  `json:"landedState"`      // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
	Throttle         int8   `json:"throttle"`         // throttle (percentage)
	Airspeed         uint8  `json:"airspeed"`         // airspeed
	AirspeedSp       uint8  `json:"airspeedSp"`       // airspeed setpoint
	Groundspeed      uint8  `json:"groundspeed"`      // groundspeed
	ClimbRate        int8   `json:"climbRate"`        // climb rate
	GpsNsat          uint8  `json:"gpsNsat"`          // Number of satellites visible. If unknown, set to 255
	GpsFixType       uint8  `json:"gpsFixType"`       // GPS Fix type.
	BatteryRemaining uint8  `json:"batteryRemaining"` // Remaining battery (percentage)
	Temperature      int8   `json:"temperature"`      // Autopilot temperature (degrees C)
	TemperatureAir   int8   `json:"temperatureAir"`   // Air temperature (degrees C) from airspeed sensor
	Failsafe         uint8  `json:"failsafe"`         // failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
	WpNum            uint8  `json:"wpNum"`            // current waypoint number
}

func (self *HighLatency) MsgID() MessageID {
	return MSG_ID_HIGH_LATENCY
}

func (self *HighLatency) MsgName() string {
	return "HighLatency"
}

func (self *HighLatency) Pack(p *Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.CustomMode))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Longitude))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.Roll))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Pitch))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.HeadingSp))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.AltitudeAmsl))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.AltitudeSp))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.WpDistance))
	payload[26] = byte(self.BaseMode)
	payload[27] = byte(self.LandedState)
	payload[28] = byte(self.Throttle)
	payload[29] = byte(self.Airspeed)
	payload[30] = byte(self.AirspeedSp)
	payload[31] = byte(self.Groundspeed)
	payload[32] = byte(self.ClimbRate)
	payload[33] = byte(self.GpsNsat)
	payload[34] = byte(self.GpsFixType)
	payload[35] = byte(self.BatteryRemaining)
	payload[36] = byte(self.Temperature)
	payload[37] = byte(self.TemperatureAir)
	payload[38] = byte(self.Failsafe)
	payload[39] = byte(self.WpNum)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HighLatency) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	self.CustomMode = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Roll = int16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Pitch = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.Heading = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.HeadingSp = int16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.AltitudeAmsl = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.AltitudeSp = int16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.WpDistance = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.BaseMode = uint8(p.Payload[26])
	self.LandedState = uint8(p.Payload[27])
	self.Throttle = int8(p.Payload[28])
	self.Airspeed = uint8(p.Payload[29])
	self.AirspeedSp = uint8(p.Payload[30])
	self.Groundspeed = uint8(p.Payload[31])
	self.ClimbRate = int8(p.Payload[32])
	self.GpsNsat = uint8(p.Payload[33])
	self.GpsFixType = uint8(p.Payload[34])
	self.BatteryRemaining = uint8(p.Payload[35])
	self.Temperature = int8(p.Payload[36])
	self.TemperatureAir = int8(p.Payload[37])
	self.Failsafe = uint8(p.Payload[38])
	self.WpNum = uint8(p.Payload[39])
	return nil
}

func (self *HighLatency) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HighLatencyFromJSON(data []byte) (*HighLatency, error) {
	p := &HighLatency{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message appropriate for high latency connections like Iridium (version 2)
type HighLatency2 struct {
	Timestamp      uint32 `json:"timestamp"`      // Timestamp (milliseconds since boot or Unix epoch)
	Latitude       int32  `json:"latitude"`       // Latitude
	Longitude      int32  `json:"longitude"`      // Longitude
	CustomMode     uint16 `json:"customMode"`     // A bitfield for use for autopilot-specific flags (2 byte version).
	Altitude       int16  `json:"altitude"`       // Altitude above mean sea level
	TargetAltitude int16  `json:"targetAltitude"` // Altitude setpoint
	TargetDistance uint16 `json:"targetDistance"` // Distance to target waypoint or position
	WpNum          uint16 `json:"wpNum"`          // Current waypoint number
	FailureFlags   uint16 `json:"failureFlags"`   // Bitmap of failure flags.
	Type           uint8  `json:"type"`           // Type of the MAV (quadrotor, helicopter, etc.)
	Autopilot      uint8  `json:"autopilot"`      // Autopilot type / class.
	Heading        uint8  `json:"heading"`        // Heading
	TargetHeading  uint8  `json:"targetHeading"`  // Heading setpoint
	Throttle       uint8  `json:"throttle"`       // Throttle
	Airspeed       uint8  `json:"airspeed"`       // Airspeed
	AirspeedSp     uint8  `json:"airspeedSp"`     // Airspeed setpoint
	Groundspeed    uint8  `json:"groundspeed"`    // Groundspeed
	Windspeed      uint8  `json:"windspeed"`      // Windspeed
	WindHeading    uint8  `json:"windHeading"`    // Wind heading
	Eph            uint8  `json:"eph"`            // Maximum error horizontal position since last message
	Epv            uint8  `json:"epv"`            // Maximum error vertical position since last message
	TemperatureAir int8   `json:"temperatureAir"` // Air temperature from airspeed sensor
	ClimbRate      int8   `json:"climbRate"`      // Maximum climb rate magnitude since last message
	Battery        int8   `json:"battery"`        // Battery (percentage, -1 for DNU)
	Custom0        int8   `json:"custom0"`        // Field for custom payload.
	Custom1        int8   `json:"custom1"`        // Field for custom payload.
	Custom2        int8   `json:"custom2"`        // Field for custom payload.
}

func (self *HighLatency2) MsgID() MessageID {
	return MSG_ID_HIGH_LATENCY2
}

func (self *HighLatency2) MsgName() string {
	return "HighLatency2"
}

func (self *HighLatency2) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Timestamp))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Longitude))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.CustomMode))
	binary.LittleEndian.PutUint16(payload[14:], uint16(self.Altitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.TargetAltitude))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.TargetDistance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.WpNum))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.FailureFlags))
	payload[24] = byte(self.Type)
	payload[25] = byte(self.Autopilot)
	payload[26] = byte(self.Heading)
	payload[27] = byte(self.TargetHeading)
	payload[28] = byte(self.Throttle)
	payload[29] = byte(self.Airspeed)
	payload[30] = byte(self.AirspeedSp)
	payload[31] = byte(self.Groundspeed)
	payload[32] = byte(self.Windspeed)
	payload[33] = byte(self.WindHeading)
	payload[34] = byte(self.Eph)
	payload[35] = byte(self.Epv)
	payload[36] = byte(self.TemperatureAir)
	payload[37] = byte(self.ClimbRate)
	payload[38] = byte(self.Battery)
	payload[39] = byte(self.Custom0)
	payload[40] = byte(self.Custom1)
	payload[41] = byte(self.Custom2)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HighLatency2) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.Timestamp = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.CustomMode = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.Altitude = int16(binary.LittleEndian.Uint16(p.Payload[14:]))
	self.TargetAltitude = int16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.TargetDistance = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.WpNum = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.FailureFlags = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Type = uint8(p.Payload[24])
	self.Autopilot = uint8(p.Payload[25])
	self.Heading = uint8(p.Payload[26])
	self.TargetHeading = uint8(p.Payload[27])
	self.Throttle = uint8(p.Payload[28])
	self.Airspeed = uint8(p.Payload[29])
	self.AirspeedSp = uint8(p.Payload[30])
	self.Groundspeed = uint8(p.Payload[31])
	self.Windspeed = uint8(p.Payload[32])
	self.WindHeading = uint8(p.Payload[33])
	self.Eph = uint8(p.Payload[34])
	self.Epv = uint8(p.Payload[35])
	self.TemperatureAir = int8(p.Payload[36])
	self.ClimbRate = int8(p.Payload[37])
	self.Battery = int8(p.Payload[38])
	self.Custom0 = int8(p.Payload[39])
	self.Custom1 = int8(p.Payload[40])
	self.Custom2 = int8(p.Payload[41])
	return nil
}

func (self *HighLatency2) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HighLatency2FromJSON(data []byte) (*HighLatency2, error) {
	p := &HighLatency2{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Vibration levels and accelerometer clipping
type Vibration struct {
	TimeUsec   uint64  `json:"timeUsec"`   // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	VibrationX float32 `json:"vibrationX"` // Vibration levels on X-axis
	VibrationY float32 `json:"vibrationY"` // Vibration levels on Y-axis
	VibrationZ float32 `json:"vibrationZ"` // Vibration levels on Z-axis
	Clipping0  uint32  `json:"clipping0"`  // first accelerometer clipping count
	Clipping1  uint32  `json:"clipping1"`  // second accelerometer clipping count
	Clipping2  uint32  `json:"clipping2"`  // third accelerometer clipping count
}

func (self *Vibration) MsgID() MessageID {
	return MSG_ID_VIBRATION
}

func (self *Vibration) MsgName() string {
	return "Vibration"
}

func (self *Vibration) Pack(p *Packet) error {
	payload := make([]byte, 32)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.VibrationX))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.VibrationY))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.VibrationZ))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Clipping0))
	binary.LittleEndian.PutUint32(payload[24:], uint32(self.Clipping1))
	binary.LittleEndian.PutUint32(payload[28:], uint32(self.Clipping2))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Vibration) Unpack(p *Packet) error {
	if len(p.Payload) < 32 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.VibrationX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.VibrationY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.VibrationZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Clipping0 = uint32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Clipping1 = uint32(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Clipping2 = uint32(binary.LittleEndian.Uint32(p.Payload[28:]))
	return nil
}

func (self *Vibration) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func VibrationFromJSON(data []byte) (*Vibration, error) {
	p := &Vibration{}
	err := json.Unmarshal(data, p)
	return p, err
}

// This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type HomePosition struct {
	Latitude  int32      `json:"latitude"`  // Latitude (WGS84)
	Longitude int32      `json:"longitude"` // Longitude (WGS84)
	Altitude  int32      `json:"altitude"`  // Altitude (MSL). Positive for up.
	X         float32    `json:"x"`         // Local X position of this position in the local coordinate frame
	Y         float32    `json:"y"`         // Local Y position of this position in the local coordinate frame
	Z         float32    `json:"z"`         // Local Z position of this position in the local coordinate frame
	Q         [4]float32 `json:"q"`         // World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
	ApproachX float32    `json:"approachX"` // Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachY float32    `json:"approachY"` // Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachZ float32    `json:"approachZ"` // Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
}

func (self *HomePosition) MsgID() MessageID {
	return MSG_ID_HOME_POSITION
}

func (self *HomePosition) MsgName() string {
	return "HomePosition"
}

func (self *HomePosition) Pack(p *Packet) error {
	payload := make([]byte, 52)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Altitude))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Z))
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[24+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.ApproachX))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.ApproachY))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.ApproachZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *HomePosition) Unpack(p *Packet) error {
	if len(p.Payload) < 52 {
		return fmt.Errorf("payload too small")
	}
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24+i*4:]))
	}
	self.ApproachX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.ApproachY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.ApproachZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	return nil
}

func (self *HomePosition) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HomePositionFromJSON(data []byte) (*HomePosition, error) {
	p := &HomePosition{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitly set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
type SetHomePosition struct {
	Latitude     int32      `json:"latitude"`     // Latitude (WGS84)
	Longitude    int32      `json:"longitude"`    // Longitude (WGS84)
	Altitude     int32      `json:"altitude"`     // Altitude (MSL). Positive for up.
	X            float32    `json:"x"`            // Local X position of this position in the local coordinate frame
	Y            float32    `json:"y"`            // Local Y position of this position in the local coordinate frame
	Z            float32    `json:"z"`            // Local Z position of this position in the local coordinate frame
	Q            [4]float32 `json:"q"`            // World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
	ApproachX    float32    `json:"approachX"`    // Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachY    float32    `json:"approachY"`    // Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	ApproachZ    float32    `json:"approachZ"`    // Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
	TargetSystem uint8      `json:"targetSystem"` // System ID.
}

func (self *SetHomePosition) MsgID() MessageID {
	return MSG_ID_SET_HOME_POSITION
}

func (self *SetHomePosition) MsgName() string {
	return "SetHomePosition"
}

func (self *SetHomePosition) Pack(p *Packet) error {
	payload := make([]byte, 53)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Latitude))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Longitude))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Altitude))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Z))
	for i, v := range self.Q {
		binary.LittleEndian.PutUint32(payload[24+i*4:], math.Float32bits(v))
	}
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.ApproachX))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.ApproachY))
	binary.LittleEndian.PutUint32(payload[48:], math.Float32bits(self.ApproachZ))
	payload[52] = byte(self.TargetSystem)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetHomePosition) Unpack(p *Packet) error {
	if len(p.Payload) < 53 {
		return fmt.Errorf("payload too small")
	}
	self.Latitude = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Longitude = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	for i := 0; i < len(self.Q); i++ {
		self.Q[i] = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24+i*4:]))
	}
	self.ApproachX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.ApproachY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	self.ApproachZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[48:]))
	self.TargetSystem = uint8(p.Payload[52])
	return nil
}

func (self *SetHomePosition) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetHomePositionFromJSON(data []byte) (*SetHomePosition, error) {
	p := &SetHomePosition{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The interval between messages for a particular MAVLink message ID. This interface replaces DATA_STREAM
type MessageInterval struct {
	IntervalUs int32  `json:"intervalUs"` // The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, &gt; 0 indicates the interval at which it is sent.
	MessageId  uint16 `json:"messageId"`  // The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
}

func (self *MessageInterval) MsgID() MessageID {
	return MSG_ID_MESSAGE_INTERVAL
}

func (self *MessageInterval) MsgName() string {
	return "MessageInterval"
}

func (self *MessageInterval) Pack(p *Packet) error {
	payload := make([]byte, 6)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.IntervalUs))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.MessageId))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MessageInterval) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.IntervalUs = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.MessageId = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	return nil
}

func (self *MessageInterval) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MessageIntervalFromJSON(data []byte) (*MessageInterval, error) {
	p := &MessageInterval{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Provides state for additional features
type ExtendedSysState struct {
	VtolState   uint8 `json:"vtolState"`   // The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
	LandedState uint8 `json:"landedState"` // The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
}

func (self *ExtendedSysState) MsgID() MessageID {
	return MSG_ID_EXTENDED_SYS_STATE
}

func (self *ExtendedSysState) MsgName() string {
	return "ExtendedSysState"
}

func (self *ExtendedSysState) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.VtolState)
	payload[1] = byte(self.LandedState)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ExtendedSysState) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.VtolState = uint8(p.Payload[0])
	self.LandedState = uint8(p.Payload[1])
	return nil
}

func (self *ExtendedSysState) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ExtendedSysStateFromJSON(data []byte) (*ExtendedSysState, error) {
	p := &ExtendedSysState{}
	err := json.Unmarshal(data, p)
	return p, err
}

// The location and information of an ADSB vehicle
type AdsbVehicle struct {
	IcaoAddress  uint32  `json:"icaoAddress"`  // ICAO address
	Lat          int32   `json:"lat"`          // Latitude
	Lon          int32   `json:"lon"`          // Longitude
	Altitude     int32   `json:"altitude"`     // Altitude(ASL)
	Heading      uint16  `json:"heading"`      // Course over ground
	HorVelocity  uint16  `json:"horVelocity"`  // The horizontal velocity
	VerVelocity  int16   `json:"verVelocity"`  // The vertical velocity. Positive is up
	Flags        uint16  `json:"flags"`        // Bitmap to indicate various statuses including valid data fields
	Squawk       uint16  `json:"squawk"`       // Squawk code
	AltitudeType uint8   `json:"altitudeType"` // ADSB altitude type.
	Callsign     [9]byte `json:"callsign"`     // The callsign, 8+null
	EmitterType  uint8   `json:"emitterType"`  // ADSB emitter type.
	Tslc         uint8   `json:"tslc"`         // Time since last communication in seconds
}

func (self *AdsbVehicle) MsgID() MessageID {
	return MSG_ID_ADSB_VEHICLE
}

func (self *AdsbVehicle) MsgName() string {
	return "AdsbVehicle"
}

func (self *AdsbVehicle) Pack(p *Packet) error {
	payload := make([]byte, 38)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.IcaoAddress))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lon))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Altitude))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Heading))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.HorVelocity))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.VerVelocity))
	binary.LittleEndian.PutUint16(payload[22:], uint16(self.Flags))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.Squawk))
	payload[26] = byte(self.AltitudeType)
	copy(payload[27:], self.Callsign[:])
	payload[36] = byte(self.EmitterType)
	payload[37] = byte(self.Tslc)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AdsbVehicle) Unpack(p *Packet) error {
	if len(p.Payload) < 38 {
		return fmt.Errorf("payload too small")
	}
	self.IcaoAddress = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Lon = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Heading = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.HorVelocity = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	self.VerVelocity = int16(binary.LittleEndian.Uint16(p.Payload[20:]))
	self.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[22:]))
	self.Squawk = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.AltitudeType = uint8(p.Payload[26])
	copy(self.Callsign[:], p.Payload[27:36])
	self.EmitterType = uint8(p.Payload[36])
	self.Tslc = uint8(p.Payload[37])
	return nil
}

func (self *AdsbVehicle) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AdsbVehicleFromJSON(data []byte) (*AdsbVehicle, error) {
	p := &AdsbVehicle{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Information about a potential collision
type Collision struct {
	Id                     uint32  `json:"id"`                     // Unique identifier, domain based on src field
	TimeToMinimumDelta     float32 `json:"timeToMinimumDelta"`     // Estimated time until collision occurs
	AltitudeMinimumDelta   float32 `json:"altitudeMinimumDelta"`   // Closest vertical distance between vehicle and object
	HorizontalMinimumDelta float32 `json:"horizontalMinimumDelta"` // Closest horizontal distance between vehicle and object
	Src                    uint8   `json:"src"`                    // Collision data source
	Action                 uint8   `json:"action"`                 // Action that is being taken to avoid this collision
	ThreatLevel            uint8   `json:"threatLevel"`            // How concerned the aircraft is about this collision
}

func (self *Collision) MsgID() MessageID {
	return MSG_ID_COLLISION
}

func (self *Collision) MsgName() string {
	return "Collision"
}

func (self *Collision) Pack(p *Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Id))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.TimeToMinimumDelta))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.AltitudeMinimumDelta))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.HorizontalMinimumDelta))
	payload[16] = byte(self.Src)
	payload[17] = byte(self.Action)
	payload[18] = byte(self.ThreatLevel)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Collision) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	self.Id = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TimeToMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.AltitudeMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.HorizontalMinimumDelta = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Src = uint8(p.Payload[16])
	self.Action = uint8(p.Payload[17])
	self.ThreatLevel = uint8(p.Payload[18])
	return nil
}

func (self *Collision) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CollisionFromJSON(data []byte) (*Collision, error) {
	p := &Collision{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
type V2Extension struct {
	MessageType     uint16     `json:"messageType"`     // A code that identifies the software component that understands this message (analogous to USB device classes or mime type strings).  If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
	TargetNetwork   uint8      `json:"targetNetwork"`   // Network ID (0 for broadcast)
	TargetSystem    uint8      `json:"targetSystem"`    // System ID (0 for broadcast)
	TargetComponent uint8      `json:"targetComponent"` // Component ID (0 for broadcast)
	Payload         [249]uint8 `json:"payload"`         // Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification.
}

func (self *V2Extension) MsgID() MessageID {
	return MSG_ID_V2_EXTENSION
}

func (self *V2Extension) MsgName() string {
	return "V2Extension"
}

func (self *V2Extension) Pack(p *Packet) error {
	payload := make([]byte, 254)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.MessageType))
	payload[2] = byte(self.TargetNetwork)
	payload[3] = byte(self.TargetSystem)
	payload[4] = byte(self.TargetComponent)
	copy(payload[5:], self.Payload[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *V2Extension) Unpack(p *Packet) error {
	if len(p.Payload) < 254 {
		return fmt.Errorf("payload too small")
	}
	self.MessageType = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.TargetNetwork = uint8(p.Payload[2])
	self.TargetSystem = uint8(p.Payload[3])
	self.TargetComponent = uint8(p.Payload[4])
	copy(self.Payload[:], p.Payload[5:254])
	return nil
}

func (self *V2Extension) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func V2ExtensionFromJSON(data []byte) (*V2Extension, error) {
	p := &V2Extension{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type MemoryVect struct {
	Address uint16   `json:"address"` // Starting address of the debug variables
	Ver     uint8    `json:"ver"`     // Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
	Type    uint8    `json:"type"`    // Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
	Value   [32]int8 `json:"value"`   // Memory contents at specified address
}

func (self *MemoryVect) MsgID() MessageID {
	return MSG_ID_MEMORY_VECT
}

func (self *MemoryVect) MsgName() string {
	return "MemoryVect"
}

func (self *MemoryVect) Pack(p *Packet) error {
	payload := make([]byte, 36)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Address))
	payload[2] = byte(self.Ver)
	payload[3] = byte(self.Type)
	for i, v := range self.Value {
		payload[4+i*1] = byte(v)
	}

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MemoryVect) Unpack(p *Packet) error {
	if len(p.Payload) < 36 {
		return fmt.Errorf("payload too small")
	}
	self.Address = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Ver = uint8(p.Payload[2])
	self.Type = uint8(p.Payload[3])
	for i := 0; i < len(self.Value); i++ {
		self.Value[i] = int8(p.Payload[4+i*1])
	}
	return nil
}

func (self *MemoryVect) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MemoryVectFromJSON(data []byte) (*MemoryVect, error) {
	p := &MemoryVect{}
	err := json.Unmarshal(data, p)
	return p, err
}

// To debug something using a named 3D vector.
type DebugVect struct {
	TimeUsec uint64   `json:"timeUsec"` // Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude the number.
	X        float32  `json:"x"`        // x
	Y        float32  `json:"y"`        // y
	Z        float32  `json:"z"`        // z
	Name     [10]byte `json:"name"`     // Name
}

func (self *DebugVect) MsgID() MessageID {
	return MSG_ID_DEBUG_VECT
}

func (self *DebugVect) MsgName() string {
	return "DebugVect"
}

func (self *DebugVect) Pack(p *Packet) error {
	payload := make([]byte, 30)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.X))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Y))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Z))
	copy(payload[20:], self.Name[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DebugVect) Unpack(p *Packet) error {
	if len(p.Payload) < 30 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.X = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Y = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Z = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	copy(self.Name[:], p.Payload[20:30])
	return nil
}

func (self *DebugVect) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DebugVectFromJSON(data []byte) (*DebugVect, error) {
	p := &DebugVect{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueFloat struct {
	TimeBootMs uint32   `json:"timeBootMs"` // Timestamp (time since system boot).
	Value      float32  `json:"value"`      // Floating point value
	Name       [10]byte `json:"name"`       // Name of the debug variable
}

func (self *NamedValueFloat) MsgID() MessageID {
	return MSG_ID_NAMED_VALUE_FLOAT
}

func (self *NamedValueFloat) MsgName() string {
	return "NamedValueFloat"
}

func (self *NamedValueFloat) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Value))
	copy(payload[8:], self.Name[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *NamedValueFloat) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Value = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	copy(self.Name[:], p.Payload[8:18])
	return nil
}

func (self *NamedValueFloat) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func NamedValueFloatFromJSON(data []byte) (*NamedValueFloat, error) {
	p := &NamedValueFloat{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
type NamedValueInt struct {
	TimeBootMs uint32   `json:"timeBootMs"` // Timestamp (time since system boot).
	Value      int32    `json:"value"`      // Signed integer value
	Name       [10]byte `json:"name"`       // Name of the debug variable
}

func (self *NamedValueInt) MsgID() MessageID {
	return MSG_ID_NAMED_VALUE_INT
}

func (self *NamedValueInt) MsgName() string {
	return "NamedValueInt"
}

func (self *NamedValueInt) Pack(p *Packet) error {
	payload := make([]byte, 18)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Value))
	copy(payload[8:], self.Name[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *NamedValueInt) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Value = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	copy(self.Name[:], p.Payload[8:18])
	return nil
}

func (self *NamedValueInt) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func NamedValueIntFromJSON(data []byte) (*NamedValueInt, error) {
	p := &NamedValueInt{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
type Statustext struct {
	Severity uint8    `json:"severity"` // Severity of status. Relies on the definitions within RFC-5424.
	Text     [50]byte `json:"text"`     // Status text message, without null termination character
}

func (self *Statustext) MsgID() MessageID {
	return MSG_ID_STATUSTEXT
}

func (self *Statustext) MsgName() string {
	return "Statustext"
}

func (self *Statustext) Pack(p *Packet) error {
	payload := make([]byte, 51)
	payload[0] = byte(self.Severity)
	copy(payload[1:], self.Text[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Statustext) Unpack(p *Packet) error {
	if len(p.Payload) < 51 {
		return fmt.Errorf("payload too small")
	}
	self.Severity = uint8(p.Payload[0])
	copy(self.Text[:], p.Payload[1:51])
	return nil
}

func (self *Statustext) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func StatustextFromJSON(data []byte) (*Statustext, error) {
	p := &Statustext{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
type Debug struct {
	TimeBootMs uint32  `json:"timeBootMs"` // Timestamp (time since system boot).
	Value      float32 `json:"value"`      // DEBUG value
	Ind        uint8   `json:"ind"`        // index of debug variable
}

func (self *Debug) MsgID() MessageID {
	return MSG_ID_DEBUG
}

func (self *Debug) MsgName() string {
	return "Debug"
}

func (self *Debug) Pack(p *Packet) error {
	payload := make([]byte, 9)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.TimeBootMs))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Value))
	payload[8] = byte(self.Ind)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Debug) Unpack(p *Packet) error {
	if len(p.Payload) < 9 {
		return fmt.Errorf("payload too small")
	}
	self.TimeBootMs = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Value = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Ind = uint8(p.Payload[8])
	return nil
}

func (self *Debug) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DebugFromJSON(data []byte) (*Debug, error) {
	p := &Debug{}
	err := json.Unmarshal(data, p)
	return p, err
}

func init() {
	Messages["heartbeat"] = &Heartbeat{}
	MessageIDs[0] = &Heartbeat{}
	Messages["sysstatus"] = &SysStatus{}
	MessageIDs[1] = &SysStatus{}
	Messages["systemtime"] = &SystemTime{}
	MessageIDs[2] = &SystemTime{}
	Messages["ping"] = &Ping{}
	MessageIDs[4] = &Ping{}
	Messages["changeoperatorcontrol"] = &ChangeOperatorControl{}
	MessageIDs[5] = &ChangeOperatorControl{}
	Messages["changeoperatorcontrolack"] = &ChangeOperatorControlAck{}
	MessageIDs[6] = &ChangeOperatorControlAck{}
	Messages["authkey"] = &AuthKey{}
	MessageIDs[7] = &AuthKey{}
	Messages["setmode"] = &SetMode{}
	MessageIDs[11] = &SetMode{}
	Messages["paramrequestread"] = &ParamRequestRead{}
	MessageIDs[20] = &ParamRequestRead{}
	Messages["paramrequestlist"] = &ParamRequestList{}
	MessageIDs[21] = &ParamRequestList{}
	Messages["paramvalue"] = &ParamValue{}
	MessageIDs[22] = &ParamValue{}
	Messages["paramset"] = &ParamSet{}
	MessageIDs[23] = &ParamSet{}
	Messages["gpsrawint"] = &GpsRawInt{}
	MessageIDs[24] = &GpsRawInt{}
	Messages["gpsstatus"] = &GpsStatus{}
	MessageIDs[25] = &GpsStatus{}
	Messages["scaledimu"] = &ScaledImu{}
	MessageIDs[26] = &ScaledImu{}
	Messages["rawimu"] = &RawImu{}
	MessageIDs[27] = &RawImu{}
	Messages["rawpressure"] = &RawPressure{}
	MessageIDs[28] = &RawPressure{}
	Messages["scaledpressure"] = &ScaledPressure{}
	MessageIDs[29] = &ScaledPressure{}
	Messages["attitude"] = &Attitude{}
	MessageIDs[30] = &Attitude{}
	Messages["attitudequaternion"] = &AttitudeQuaternion{}
	MessageIDs[31] = &AttitudeQuaternion{}
	Messages["localpositionned"] = &LocalPositionNed{}
	MessageIDs[32] = &LocalPositionNed{}
	Messages["globalpositionint"] = &GlobalPositionInt{}
	MessageIDs[33] = &GlobalPositionInt{}
	Messages["rcchannelsscaled"] = &RcChannelsScaled{}
	MessageIDs[34] = &RcChannelsScaled{}
	Messages["rcchannelsraw"] = &RcChannelsRaw{}
	MessageIDs[35] = &RcChannelsRaw{}
	Messages["servooutputraw"] = &ServoOutputRaw{}
	MessageIDs[36] = &ServoOutputRaw{}
	Messages["missionrequestpartiallist"] = &MissionRequestPartialList{}
	MessageIDs[37] = &MissionRequestPartialList{}
	Messages["missionwritepartiallist"] = &MissionWritePartialList{}
	MessageIDs[38] = &MissionWritePartialList{}
	Messages["missionitem"] = &MissionItem{}
	MessageIDs[39] = &MissionItem{}
	Messages["missionrequest"] = &MissionRequest{}
	MessageIDs[40] = &MissionRequest{}
	Messages["missionsetcurrent"] = &MissionSetCurrent{}
	MessageIDs[41] = &MissionSetCurrent{}
	Messages["missioncurrent"] = &MissionCurrent{}
	MessageIDs[42] = &MissionCurrent{}
	Messages["missionrequestlist"] = &MissionRequestList{}
	MessageIDs[43] = &MissionRequestList{}
	Messages["missioncount"] = &MissionCount{}
	MessageIDs[44] = &MissionCount{}
	Messages["missionclearall"] = &MissionClearAll{}
	MessageIDs[45] = &MissionClearAll{}
	Messages["missionitemreached"] = &MissionItemReached{}
	MessageIDs[46] = &MissionItemReached{}
	Messages["missionack"] = &MissionAck{}
	MessageIDs[47] = &MissionAck{}
	Messages["setgpsglobalorigin"] = &SetGpsGlobalOrigin{}
	MessageIDs[48] = &SetGpsGlobalOrigin{}
	Messages["gpsglobalorigin"] = &GpsGlobalOrigin{}
	MessageIDs[49] = &GpsGlobalOrigin{}
	Messages["parammaprc"] = &ParamMapRc{}
	MessageIDs[50] = &ParamMapRc{}
	Messages["missionrequestint"] = &MissionRequestInt{}
	MessageIDs[51] = &MissionRequestInt{}
	Messages["safetysetallowedarea"] = &SafetySetAllowedArea{}
	MessageIDs[54] = &SafetySetAllowedArea{}
	Messages["safetyallowedarea"] = &SafetyAllowedArea{}
	MessageIDs[55] = &SafetyAllowedArea{}
	Messages["attitudequaternioncov"] = &AttitudeQuaternionCov{}
	MessageIDs[61] = &AttitudeQuaternionCov{}
	Messages["navcontrolleroutput"] = &NavControllerOutput{}
	MessageIDs[62] = &NavControllerOutput{}
	Messages["globalpositionintcov"] = &GlobalPositionIntCov{}
	MessageIDs[63] = &GlobalPositionIntCov{}
	Messages["localpositionnedcov"] = &LocalPositionNedCov{}
	MessageIDs[64] = &LocalPositionNedCov{}
	Messages["rcchannels"] = &RcChannels{}
	MessageIDs[65] = &RcChannels{}
	Messages["requestdatastream"] = &RequestDataStream{}
	MessageIDs[66] = &RequestDataStream{}
	Messages["datastream"] = &DataStream{}
	MessageIDs[67] = &DataStream{}
	Messages["manualcontrol"] = &ManualControl{}
	MessageIDs[69] = &ManualControl{}
	Messages["rcchannelsoverride"] = &RcChannelsOverride{}
	MessageIDs[70] = &RcChannelsOverride{}
	Messages["missionitemint"] = &MissionItemInt{}
	MessageIDs[73] = &MissionItemInt{}
	Messages["vfrhud"] = &VfrHud{}
	MessageIDs[74] = &VfrHud{}
	Messages["commandint"] = &CommandInt{}
	MessageIDs[75] = &CommandInt{}
	Messages["commandlong"] = &CommandLong{}
	MessageIDs[76] = &CommandLong{}
	Messages["commandack"] = &CommandAck{}
	MessageIDs[77] = &CommandAck{}
	Messages["manualsetpoint"] = &ManualSetpoint{}
	MessageIDs[81] = &ManualSetpoint{}
	Messages["setattitudetarget"] = &SetAttitudeTarget{}
	MessageIDs[82] = &SetAttitudeTarget{}
	Messages["attitudetarget"] = &AttitudeTarget{}
	MessageIDs[83] = &AttitudeTarget{}
	Messages["setpositiontargetlocalned"] = &SetPositionTargetLocalNed{}
	MessageIDs[84] = &SetPositionTargetLocalNed{}
	Messages["positiontargetlocalned"] = &PositionTargetLocalNed{}
	MessageIDs[85] = &PositionTargetLocalNed{}
	Messages["setpositiontargetglobalint"] = &SetPositionTargetGlobalInt{}
	MessageIDs[86] = &SetPositionTargetGlobalInt{}
	Messages["positiontargetglobalint"] = &PositionTargetGlobalInt{}
	MessageIDs[87] = &PositionTargetGlobalInt{}
	Messages["localpositionnedsystemglobaloffset"] = &LocalPositionNedSystemGlobalOffset{}
	MessageIDs[89] = &LocalPositionNedSystemGlobalOffset{}
	Messages["hilstate"] = &HilState{}
	MessageIDs[90] = &HilState{}
	Messages["hilcontrols"] = &HilControls{}
	MessageIDs[91] = &HilControls{}
	Messages["hilrcinputsraw"] = &HilRcInputsRaw{}
	MessageIDs[92] = &HilRcInputsRaw{}
	Messages["hilactuatorcontrols"] = &HilActuatorControls{}
	MessageIDs[93] = &HilActuatorControls{}
	Messages["opticalflow"] = &OpticalFlow{}
	MessageIDs[100] = &OpticalFlow{}
	Messages["globalvisionpositionestimate"] = &GlobalVisionPositionEstimate{}
	MessageIDs[101] = &GlobalVisionPositionEstimate{}
	Messages["visionpositionestimate"] = &VisionPositionEstimate{}
	MessageIDs[102] = &VisionPositionEstimate{}
	Messages["visionspeedestimate"] = &VisionSpeedEstimate{}
	MessageIDs[103] = &VisionSpeedEstimate{}
	Messages["viconpositionestimate"] = &ViconPositionEstimate{}
	MessageIDs[104] = &ViconPositionEstimate{}
	Messages["highresimu"] = &HighresImu{}
	MessageIDs[105] = &HighresImu{}
	Messages["opticalflowrad"] = &OpticalFlowRad{}
	MessageIDs[106] = &OpticalFlowRad{}
	Messages["hilsensor"] = &HilSensor{}
	MessageIDs[107] = &HilSensor{}
	Messages["simstate"] = &SimState{}
	MessageIDs[108] = &SimState{}
	Messages["radiostatus"] = &RadioStatus{}
	MessageIDs[109] = &RadioStatus{}
	Messages["filetransferprotocol"] = &FileTransferProtocol{}
	MessageIDs[110] = &FileTransferProtocol{}
	Messages["timesync"] = &Timesync{}
	MessageIDs[111] = &Timesync{}
	Messages["cameratrigger"] = &CameraTrigger{}
	MessageIDs[112] = &CameraTrigger{}
	Messages["hilgps"] = &HilGps{}
	MessageIDs[113] = &HilGps{}
	Messages["hilopticalflow"] = &HilOpticalFlow{}
	MessageIDs[114] = &HilOpticalFlow{}
	Messages["hilstatequaternion"] = &HilStateQuaternion{}
	MessageIDs[115] = &HilStateQuaternion{}
	Messages["scaledimu2"] = &ScaledImu2{}
	MessageIDs[116] = &ScaledImu2{}
	Messages["logrequestlist"] = &LogRequestList{}
	MessageIDs[117] = &LogRequestList{}
	Messages["logentry"] = &LogEntry{}
	MessageIDs[118] = &LogEntry{}
	Messages["logrequestdata"] = &LogRequestData{}
	MessageIDs[119] = &LogRequestData{}
	Messages["logdata"] = &LogData{}
	MessageIDs[120] = &LogData{}
	Messages["logerase"] = &LogErase{}
	MessageIDs[121] = &LogErase{}
	Messages["logrequestend"] = &LogRequestEnd{}
	MessageIDs[122] = &LogRequestEnd{}
	Messages["gpsinjectdata"] = &GpsInjectData{}
	MessageIDs[123] = &GpsInjectData{}
	Messages["gps2raw"] = &Gps2Raw{}
	MessageIDs[124] = &Gps2Raw{}
	Messages["powerstatus"] = &PowerStatus{}
	MessageIDs[125] = &PowerStatus{}
	Messages["serialcontrol"] = &SerialControl{}
	MessageIDs[126] = &SerialControl{}
	Messages["gpsrtk"] = &GpsRtk{}
	MessageIDs[127] = &GpsRtk{}
	Messages["gps2rtk"] = &Gps2Rtk{}
	MessageIDs[128] = &Gps2Rtk{}
	Messages["scaledimu3"] = &ScaledImu3{}
	MessageIDs[129] = &ScaledImu3{}
	Messages["datatransmissionhandshake"] = &DataTransmissionHandshake{}
	MessageIDs[130] = &DataTransmissionHandshake{}
	Messages["encapsulateddata"] = &EncapsulatedData{}
	MessageIDs[131] = &EncapsulatedData{}
	Messages["distancesensor"] = &DistanceSensor{}
	MessageIDs[132] = &DistanceSensor{}
	Messages["terrainrequest"] = &TerrainRequest{}
	MessageIDs[133] = &TerrainRequest{}
	Messages["terraindata"] = &TerrainData{}
	MessageIDs[134] = &TerrainData{}
	Messages["terraincheck"] = &TerrainCheck{}
	MessageIDs[135] = &TerrainCheck{}
	Messages["terrainreport"] = &TerrainReport{}
	MessageIDs[136] = &TerrainReport{}
	Messages["scaledpressure2"] = &ScaledPressure2{}
	MessageIDs[137] = &ScaledPressure2{}
	Messages["attposmocap"] = &AttPosMocap{}
	MessageIDs[138] = &AttPosMocap{}
	Messages["setactuatorcontroltarget"] = &SetActuatorControlTarget{}
	MessageIDs[139] = &SetActuatorControlTarget{}
	Messages["actuatorcontroltarget"] = &ActuatorControlTarget{}
	MessageIDs[140] = &ActuatorControlTarget{}
	Messages["altitude"] = &Altitude{}
	MessageIDs[141] = &Altitude{}
	Messages["resourcerequest"] = &ResourceRequest{}
	MessageIDs[142] = &ResourceRequest{}
	Messages["scaledpressure3"] = &ScaledPressure3{}
	MessageIDs[143] = &ScaledPressure3{}
	Messages["followtarget"] = &FollowTarget{}
	MessageIDs[144] = &FollowTarget{}
	Messages["controlsystemstate"] = &ControlSystemState{}
	MessageIDs[146] = &ControlSystemState{}
	Messages["batterystatus"] = &BatteryStatus{}
	MessageIDs[147] = &BatteryStatus{}
	Messages["autopilotversion"] = &AutopilotVersion{}
	MessageIDs[148] = &AutopilotVersion{}
	Messages["landingtarget"] = &LandingTarget{}
	MessageIDs[149] = &LandingTarget{}
	Messages["estimatorstatus"] = &EstimatorStatus{}
	MessageIDs[230] = &EstimatorStatus{}
	Messages["windcov"] = &WindCov{}
	MessageIDs[231] = &WindCov{}
	Messages["gpsinput"] = &GpsInput{}
	MessageIDs[232] = &GpsInput{}
	Messages["gpsrtcmdata"] = &GpsRtcmData{}
	MessageIDs[233] = &GpsRtcmData{}
	Messages["highlatency"] = &HighLatency{}
	MessageIDs[234] = &HighLatency{}
	Messages["highlatency2"] = &HighLatency2{}
	MessageIDs[235] = &HighLatency2{}
	Messages["vibration"] = &Vibration{}
	MessageIDs[241] = &Vibration{}
	Messages["homeposition"] = &HomePosition{}
	MessageIDs[242] = &HomePosition{}
	Messages["sethomeposition"] = &SetHomePosition{}
	MessageIDs[243] = &SetHomePosition{}
	Messages["messageinterval"] = &MessageInterval{}
	MessageIDs[244] = &MessageInterval{}
	Messages["extendedsysstate"] = &ExtendedSysState{}
	MessageIDs[245] = &ExtendedSysState{}
	Messages["adsbvehicle"] = &AdsbVehicle{}
	MessageIDs[246] = &AdsbVehicle{}
	Messages["collision"] = &Collision{}
	MessageIDs[247] = &Collision{}
	Messages["v2extension"] = &V2Extension{}
	MessageIDs[248] = &V2Extension{}
	Messages["memoryvect"] = &MemoryVect{}
	MessageIDs[249] = &MemoryVect{}
	Messages["debugvect"] = &DebugVect{}
	MessageIDs[250] = &DebugVect{}
	Messages["namedvaluefloat"] = &NamedValueFloat{}
	MessageIDs[251] = &NamedValueFloat{}
	Messages["namedvalueint"] = &NamedValueInt{}
	MessageIDs[252] = &NamedValueInt{}
	Messages["statustext"] = &Statustext{}
	MessageIDs[253] = &Statustext{}
	Messages["debug"] = &Debug{}
	MessageIDs[254] = &Debug{}

}

// Message IDs
const (
	MSG_ID_HEARTBEAT                               MessageID = 0
	MSG_ID_SYS_STATUS                              MessageID = 1
	MSG_ID_SYSTEM_TIME                             MessageID = 2
	MSG_ID_PING                                    MessageID = 4
	MSG_ID_CHANGE_OPERATOR_CONTROL                 MessageID = 5
	MSG_ID_CHANGE_OPERATOR_CONTROL_ACK             MessageID = 6
	MSG_ID_AUTH_KEY                                MessageID = 7
	MSG_ID_SET_MODE                                MessageID = 11
	MSG_ID_PARAM_REQUEST_READ                      MessageID = 20
	MSG_ID_PARAM_REQUEST_LIST                      MessageID = 21
	MSG_ID_PARAM_VALUE                             MessageID = 22
	MSG_ID_PARAM_SET                               MessageID = 23
	MSG_ID_GPS_RAW_INT                             MessageID = 24
	MSG_ID_GPS_STATUS                              MessageID = 25
	MSG_ID_SCALED_IMU                              MessageID = 26
	MSG_ID_RAW_IMU                                 MessageID = 27
	MSG_ID_RAW_PRESSURE                            MessageID = 28
	MSG_ID_SCALED_PRESSURE                         MessageID = 29
	MSG_ID_ATTITUDE                                MessageID = 30
	MSG_ID_ATTITUDE_QUATERNION                     MessageID = 31
	MSG_ID_LOCAL_POSITION_NED                      MessageID = 32
	MSG_ID_GLOBAL_POSITION_INT                     MessageID = 33
	MSG_ID_RC_CHANNELS_SCALED                      MessageID = 34
	MSG_ID_RC_CHANNELS_RAW                         MessageID = 35
	MSG_ID_SERVO_OUTPUT_RAW                        MessageID = 36
	MSG_ID_MISSION_REQUEST_PARTIAL_LIST            MessageID = 37
	MSG_ID_MISSION_WRITE_PARTIAL_LIST              MessageID = 38
	MSG_ID_MISSION_ITEM                            MessageID = 39
	MSG_ID_MISSION_REQUEST                         MessageID = 40
	MSG_ID_MISSION_SET_CURRENT                     MessageID = 41
	MSG_ID_MISSION_CURRENT                         MessageID = 42
	MSG_ID_MISSION_REQUEST_LIST                    MessageID = 43
	MSG_ID_MISSION_COUNT                           MessageID = 44
	MSG_ID_MISSION_CLEAR_ALL                       MessageID = 45
	MSG_ID_MISSION_ITEM_REACHED                    MessageID = 46
	MSG_ID_MISSION_ACK                             MessageID = 47
	MSG_ID_SET_GPS_GLOBAL_ORIGIN                   MessageID = 48
	MSG_ID_GPS_GLOBAL_ORIGIN                       MessageID = 49
	MSG_ID_PARAM_MAP_RC                            MessageID = 50
	MSG_ID_MISSION_REQUEST_INT                     MessageID = 51
	MSG_ID_SAFETY_SET_ALLOWED_AREA                 MessageID = 54
	MSG_ID_SAFETY_ALLOWED_AREA                     MessageID = 55
	MSG_ID_ATTITUDE_QUATERNION_COV                 MessageID = 61
	MSG_ID_NAV_CONTROLLER_OUTPUT                   MessageID = 62
	MSG_ID_GLOBAL_POSITION_INT_COV                 MessageID = 63
	MSG_ID_LOCAL_POSITION_NED_COV                  MessageID = 64
	MSG_ID_RC_CHANNELS                             MessageID = 65
	MSG_ID_REQUEST_DATA_STREAM                     MessageID = 66
	MSG_ID_DATA_STREAM                             MessageID = 67
	MSG_ID_MANUAL_CONTROL                          MessageID = 69
	MSG_ID_RC_CHANNELS_OVERRIDE                    MessageID = 70
	MSG_ID_MISSION_ITEM_INT                        MessageID = 73
	MSG_ID_VFR_HUD                                 MessageID = 74
	MSG_ID_COMMAND_INT                             MessageID = 75
	MSG_ID_COMMAND_LONG                            MessageID = 76
	MSG_ID_COMMAND_ACK                             MessageID = 77
	MSG_ID_MANUAL_SETPOINT                         MessageID = 81
	MSG_ID_SET_ATTITUDE_TARGET                     MessageID = 82
	MSG_ID_ATTITUDE_TARGET                         MessageID = 83
	MSG_ID_SET_POSITION_TARGET_LOCAL_NED           MessageID = 84
	MSG_ID_POSITION_TARGET_LOCAL_NED               MessageID = 85
	MSG_ID_SET_POSITION_TARGET_GLOBAL_INT          MessageID = 86
	MSG_ID_POSITION_TARGET_GLOBAL_INT              MessageID = 87
	MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET MessageID = 89
	MSG_ID_HIL_STATE                               MessageID = 90
	MSG_ID_HIL_CONTROLS                            MessageID = 91
	MSG_ID_HIL_RC_INPUTS_RAW                       MessageID = 92
	MSG_ID_HIL_ACTUATOR_CONTROLS                   MessageID = 93
	MSG_ID_OPTICAL_FLOW                            MessageID = 100
	MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE         MessageID = 101
	MSG_ID_VISION_POSITION_ESTIMATE                MessageID = 102
	MSG_ID_VISION_SPEED_ESTIMATE                   MessageID = 103
	MSG_ID_VICON_POSITION_ESTIMATE                 MessageID = 104
	MSG_ID_HIGHRES_IMU                             MessageID = 105
	MSG_ID_OPTICAL_FLOW_RAD                        MessageID = 106
	MSG_ID_HIL_SENSOR                              MessageID = 107
	MSG_ID_SIM_STATE                               MessageID = 108
	MSG_ID_RADIO_STATUS                            MessageID = 109
	MSG_ID_FILE_TRANSFER_PROTOCOL                  MessageID = 110
	MSG_ID_TIMESYNC                                MessageID = 111
	MSG_ID_CAMERA_TRIGGER                          MessageID = 112
	MSG_ID_HIL_GPS                                 MessageID = 113
	MSG_ID_HIL_OPTICAL_FLOW                        MessageID = 114
	MSG_ID_HIL_STATE_QUATERNION                    MessageID = 115
	MSG_ID_SCALED_IMU2                             MessageID = 116
	MSG_ID_LOG_REQUEST_LIST                        MessageID = 117
	MSG_ID_LOG_ENTRY                               MessageID = 118
	MSG_ID_LOG_REQUEST_DATA                        MessageID = 119
	MSG_ID_LOG_DATA                                MessageID = 120
	MSG_ID_LOG_ERASE                               MessageID = 121
	MSG_ID_LOG_REQUEST_END                         MessageID = 122
	MSG_ID_GPS_INJECT_DATA                         MessageID = 123
	MSG_ID_GPS2_RAW                                MessageID = 124
	MSG_ID_POWER_STATUS                            MessageID = 125
	MSG_ID_SERIAL_CONTROL                          MessageID = 126
	MSG_ID_GPS_RTK                                 MessageID = 127
	MSG_ID_GPS2_RTK                                MessageID = 128
	MSG_ID_SCALED_IMU3                             MessageID = 129
	MSG_ID_DATA_TRANSMISSION_HANDSHAKE             MessageID = 130
	MSG_ID_ENCAPSULATED_DATA                       MessageID = 131
	MSG_ID_DISTANCE_SENSOR                         MessageID = 132
	MSG_ID_TERRAIN_REQUEST                         MessageID = 133
	MSG_ID_TERRAIN_DATA                            MessageID = 134
	MSG_ID_TERRAIN_CHECK                           MessageID = 135
	MSG_ID_TERRAIN_REPORT                          MessageID = 136
	MSG_ID_SCALED_PRESSURE2                        MessageID = 137
	MSG_ID_ATT_POS_MOCAP                           MessageID = 138
	MSG_ID_SET_ACTUATOR_CONTROL_TARGET             MessageID = 139
	MSG_ID_ACTUATOR_CONTROL_TARGET                 MessageID = 140
	MSG_ID_ALTITUDE                                MessageID = 141
	MSG_ID_RESOURCE_REQUEST                        MessageID = 142
	MSG_ID_SCALED_PRESSURE3                        MessageID = 143
	MSG_ID_FOLLOW_TARGET                           MessageID = 144
	MSG_ID_CONTROL_SYSTEM_STATE                    MessageID = 146
	MSG_ID_BATTERY_STATUS                          MessageID = 147
	MSG_ID_AUTOPILOT_VERSION                       MessageID = 148
	MSG_ID_LANDING_TARGET                          MessageID = 149
	MSG_ID_ESTIMATOR_STATUS                        MessageID = 230
	MSG_ID_WIND_COV                                MessageID = 231
	MSG_ID_GPS_INPUT                               MessageID = 232
	MSG_ID_GPS_RTCM_DATA                           MessageID = 233
	MSG_ID_HIGH_LATENCY                            MessageID = 234
	MSG_ID_HIGH_LATENCY2                           MessageID = 235
	MSG_ID_VIBRATION                               MessageID = 241
	MSG_ID_HOME_POSITION                           MessageID = 242
	MSG_ID_SET_HOME_POSITION                       MessageID = 243
	MSG_ID_MESSAGE_INTERVAL                        MessageID = 244
	MSG_ID_EXTENDED_SYS_STATE                      MessageID = 245
	MSG_ID_ADSB_VEHICLE                            MessageID = 246
	MSG_ID_COLLISION                               MessageID = 247
	MSG_ID_V2_EXTENSION                            MessageID = 248
	MSG_ID_MEMORY_VECT                             MessageID = 249
	MSG_ID_DEBUG_VECT                              MessageID = 250
	MSG_ID_NAMED_VALUE_FLOAT                       MessageID = 251
	MSG_ID_NAMED_VALUE_INT                         MessageID = 252
	MSG_ID_STATUSTEXT                              MessageID = 253
	MSG_ID_DEBUG                                   MessageID = 254
)

// DialectCommon is the dialect represented by common.xml
var DialectCommon *Dialect = &Dialect{
	Name: "common",
	crcExtras: map[MessageID]uint8{
		MSG_ID_HEARTBEAT:                               50,
		MSG_ID_SYS_STATUS:                              124,
		MSG_ID_SYSTEM_TIME:                             137,
		MSG_ID_PING:                                    237,
		MSG_ID_CHANGE_OPERATOR_CONTROL:                 217,
		MSG_ID_CHANGE_OPERATOR_CONTROL_ACK:             104,
		MSG_ID_AUTH_KEY:                                119,
		MSG_ID_SET_MODE:                                89,
		MSG_ID_PARAM_REQUEST_READ:                      214,
		MSG_ID_PARAM_REQUEST_LIST:                      159,
		MSG_ID_PARAM_VALUE:                             220,
		MSG_ID_PARAM_SET:                               168,
		MSG_ID_GPS_RAW_INT:                             24,
		MSG_ID_GPS_STATUS:                              23,
		MSG_ID_SCALED_IMU:                              170,
		MSG_ID_RAW_IMU:                                 144,
		MSG_ID_RAW_PRESSURE:                            67,
		MSG_ID_SCALED_PRESSURE:                         115,
		MSG_ID_ATTITUDE:                                39,
		MSG_ID_ATTITUDE_QUATERNION:                     246,
		MSG_ID_LOCAL_POSITION_NED:                      185,
		MSG_ID_GLOBAL_POSITION_INT:                     104,
		MSG_ID_RC_CHANNELS_SCALED:                      237,
		MSG_ID_RC_CHANNELS_RAW:                         244,
		MSG_ID_SERVO_OUTPUT_RAW:                        222,
		MSG_ID_MISSION_REQUEST_PARTIAL_LIST:            212,
		MSG_ID_MISSION_WRITE_PARTIAL_LIST:              9,
		MSG_ID_MISSION_ITEM:                            254,
		MSG_ID_MISSION_REQUEST:                         230,
		MSG_ID_MISSION_SET_CURRENT:                     28,
		MSG_ID_MISSION_CURRENT:                         28,
		MSG_ID_MISSION_REQUEST_LIST:                    132,
		MSG_ID_MISSION_COUNT:                           221,
		MSG_ID_MISSION_CLEAR_ALL:                       232,
		MSG_ID_MISSION_ITEM_REACHED:                    11,
		MSG_ID_MISSION_ACK:                             153,
		MSG_ID_SET_GPS_GLOBAL_ORIGIN:                   41,
		MSG_ID_GPS_GLOBAL_ORIGIN:                       39,
		MSG_ID_PARAM_MAP_RC:                            78,
		MSG_ID_MISSION_REQUEST_INT:                     196,
		MSG_ID_SAFETY_SET_ALLOWED_AREA:                 15,
		MSG_ID_SAFETY_ALLOWED_AREA:                     3,
		MSG_ID_ATTITUDE_QUATERNION_COV:                 167,
		MSG_ID_NAV_CONTROLLER_OUTPUT:                   183,
		MSG_ID_GLOBAL_POSITION_INT_COV:                 119,
		MSG_ID_LOCAL_POSITION_NED_COV:                  191,
		MSG_ID_RC_CHANNELS:                             118,
		MSG_ID_REQUEST_DATA_STREAM:                     148,
		MSG_ID_DATA_STREAM:                             21,
		MSG_ID_MANUAL_CONTROL:                          243,
		MSG_ID_RC_CHANNELS_OVERRIDE:                    124,
		MSG_ID_MISSION_ITEM_INT:                        38,
		MSG_ID_VFR_HUD:                                 20,
		MSG_ID_COMMAND_INT:                             158,
		MSG_ID_COMMAND_LONG:                            152,
		MSG_ID_COMMAND_ACK:                             143,
		MSG_ID_MANUAL_SETPOINT:                         106,
		MSG_ID_SET_ATTITUDE_TARGET:                     49,
		MSG_ID_ATTITUDE_TARGET:                         22,
		MSG_ID_SET_POSITION_TARGET_LOCAL_NED:           143,
		MSG_ID_POSITION_TARGET_LOCAL_NED:               140,
		MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:          5,
		MSG_ID_POSITION_TARGET_GLOBAL_INT:              150,
		MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: 231,
		MSG_ID_HIL_STATE:                               183,
		MSG_ID_HIL_CONTROLS:                            63,
		MSG_ID_HIL_RC_INPUTS_RAW:                       54,
		MSG_ID_HIL_ACTUATOR_CONTROLS:                   47,
		MSG_ID_OPTICAL_FLOW:                            175,
		MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:         102,
		MSG_ID_VISION_POSITION_ESTIMATE:                158,
		MSG_ID_VISION_SPEED_ESTIMATE:                   208,
		MSG_ID_VICON_POSITION_ESTIMATE:                 56,
		MSG_ID_HIGHRES_IMU:                             93,
		MSG_ID_OPTICAL_FLOW_RAD:                        138,
		MSG_ID_HIL_SENSOR:                              108,
		MSG_ID_SIM_STATE:                               32,
		MSG_ID_RADIO_STATUS:                            185,
		MSG_ID_FILE_TRANSFER_PROTOCOL:                  84,
		MSG_ID_TIMESYNC:                                34,
		MSG_ID_CAMERA_TRIGGER:                          174,
		MSG_ID_HIL_GPS:                                 124,
		MSG_ID_HIL_OPTICAL_FLOW:                        237,
		MSG_ID_HIL_STATE_QUATERNION:                    4,
		MSG_ID_SCALED_IMU2:                             76,
		MSG_ID_LOG_REQUEST_LIST:                        128,
		MSG_ID_LOG_ENTRY:                               56,
		MSG_ID_LOG_REQUEST_DATA:                        116,
		MSG_ID_LOG_DATA:                                134,
		MSG_ID_LOG_ERASE:                               237,
		MSG_ID_LOG_REQUEST_END:                         203,
		MSG_ID_GPS_INJECT_DATA:                         250,
		MSG_ID_GPS2_RAW:                                87,
		MSG_ID_POWER_STATUS:                            203,
		MSG_ID_SERIAL_CONTROL:                          220,
		MSG_ID_GPS_RTK:                                 25,
		MSG_ID_GPS2_RTK:                                226,
		MSG_ID_SCALED_IMU3:                             46,
		MSG_ID_DATA_TRANSMISSION_HANDSHAKE:             29,
		MSG_ID_ENCAPSULATED_DATA:                       223,
		MSG_ID_DISTANCE_SENSOR:                         85,
		MSG_ID_TERRAIN_REQUEST:                         6,
		MSG_ID_TERRAIN_DATA:                            229,
		MSG_ID_TERRAIN_CHECK:                           203,
		MSG_ID_TERRAIN_REPORT:                          1,
		MSG_ID_SCALED_PRESSURE2:                        195,
		MSG_ID_ATT_POS_MOCAP:                           109,
		MSG_ID_SET_ACTUATOR_CONTROL_TARGET:             168,
		MSG_ID_ACTUATOR_CONTROL_TARGET:                 181,
		MSG_ID_ALTITUDE:                                47,
		MSG_ID_RESOURCE_REQUEST:                        72,
		MSG_ID_SCALED_PRESSURE3:                        131,
		MSG_ID_FOLLOW_TARGET:                           127,
		MSG_ID_CONTROL_SYSTEM_STATE:                    103,
		MSG_ID_BATTERY_STATUS:                          154,
		MSG_ID_AUTOPILOT_VERSION:                       178,
		MSG_ID_LANDING_TARGET:                          200,
		MSG_ID_ESTIMATOR_STATUS:                        163,
		MSG_ID_WIND_COV:                                105,
		MSG_ID_GPS_INPUT:                               151,
		MSG_ID_GPS_RTCM_DATA:                           35,
		MSG_ID_HIGH_LATENCY:                            150,
		MSG_ID_HIGH_LATENCY2:                           179,
		MSG_ID_VIBRATION:                               90,
		MSG_ID_HOME_POSITION:                           104,
		MSG_ID_SET_HOME_POSITION:                       85,
		MSG_ID_MESSAGE_INTERVAL:                        95,
		MSG_ID_EXTENDED_SYS_STATE:                      130,
		MSG_ID_ADSB_VEHICLE:                            184,
		MSG_ID_COLLISION:                               81,
		MSG_ID_V2_EXTENSION:                            8,
		MSG_ID_MEMORY_VECT:                             204,
		MSG_ID_DEBUG_VECT:                              49,
		MSG_ID_NAMED_VALUE_FLOAT:                       170,
		MSG_ID_NAMED_VALUE_INT:                         44,
		MSG_ID_STATUSTEXT:                              83,
		MSG_ID_DEBUG:                                   46,
	},
	messageConstructorByMsgId: map[MessageID]func(*Packet) Message{
		MSG_ID_HEARTBEAT: func(pkt *Packet) Message {
			msg := new(Heartbeat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SYS_STATUS: func(pkt *Packet) Message {
			msg := new(SysStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SYSTEM_TIME: func(pkt *Packet) Message {
			msg := new(SystemTime)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PING: func(pkt *Packet) Message {
			msg := new(Ping)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CHANGE_OPERATOR_CONTROL: func(pkt *Packet) Message {
			msg := new(ChangeOperatorControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CHANGE_OPERATOR_CONTROL_ACK: func(pkt *Packet) Message {
			msg := new(ChangeOperatorControlAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTH_KEY: func(pkt *Packet) Message {
			msg := new(AuthKey)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_MODE: func(pkt *Packet) Message {
			msg := new(SetMode)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_REQUEST_READ: func(pkt *Packet) Message {
			msg := new(ParamRequestRead)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(ParamRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_VALUE: func(pkt *Packet) Message {
			msg := new(ParamValue)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_SET: func(pkt *Packet) Message {
			msg := new(ParamSet)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RAW_INT: func(pkt *Packet) Message {
			msg := new(GpsRawInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_STATUS: func(pkt *Packet) Message {
			msg := new(GpsStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU: func(pkt *Packet) Message {
			msg := new(ScaledImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RAW_IMU: func(pkt *Packet) Message {
			msg := new(RawImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RAW_PRESSURE: func(pkt *Packet) Message {
			msg := new(RawPressure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE: func(pkt *Packet) Message {
			msg := new(ScaledPressure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE: func(pkt *Packet) Message {
			msg := new(Attitude)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_QUATERNION: func(pkt *Packet) Message {
			msg := new(AttitudeQuaternion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED: func(pkt *Packet) Message {
			msg := new(LocalPositionNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_POSITION_INT: func(pkt *Packet) Message {
			msg := new(GlobalPositionInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_SCALED: func(pkt *Packet) Message {
			msg := new(RcChannelsScaled)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_RAW: func(pkt *Packet) Message {
			msg := new(RcChannelsRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERVO_OUTPUT_RAW: func(pkt *Packet) Message {
			msg := new(ServoOutputRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_PARTIAL_LIST: func(pkt *Packet) Message {
			msg := new(MissionRequestPartialList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_WRITE_PARTIAL_LIST: func(pkt *Packet) Message {
			msg := new(MissionWritePartialList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM: func(pkt *Packet) Message {
			msg := new(MissionItem)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST: func(pkt *Packet) Message {
			msg := new(MissionRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_SET_CURRENT: func(pkt *Packet) Message {
			msg := new(MissionSetCurrent)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CURRENT: func(pkt *Packet) Message {
			msg := new(MissionCurrent)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(MissionRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_COUNT: func(pkt *Packet) Message {
			msg := new(MissionCount)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_CLEAR_ALL: func(pkt *Packet) Message {
			msg := new(MissionClearAll)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM_REACHED: func(pkt *Packet) Message {
			msg := new(MissionItemReached)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ACK: func(pkt *Packet) Message {
			msg := new(MissionAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_GPS_GLOBAL_ORIGIN: func(pkt *Packet) Message {
			msg := new(SetGpsGlobalOrigin)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_GLOBAL_ORIGIN: func(pkt *Packet) Message {
			msg := new(GpsGlobalOrigin)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PARAM_MAP_RC: func(pkt *Packet) Message {
			msg := new(ParamMapRc)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_REQUEST_INT: func(pkt *Packet) Message {
			msg := new(MissionRequestInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SAFETY_SET_ALLOWED_AREA: func(pkt *Packet) Message {
			msg := new(SafetySetAllowedArea)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SAFETY_ALLOWED_AREA: func(pkt *Packet) Message {
			msg := new(SafetyAllowedArea)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_QUATERNION_COV: func(pkt *Packet) Message {
			msg := new(AttitudeQuaternionCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAV_CONTROLLER_OUTPUT: func(pkt *Packet) Message {
			msg := new(NavControllerOutput)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_POSITION_INT_COV: func(pkt *Packet) Message {
			msg := new(GlobalPositionIntCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED_COV: func(pkt *Packet) Message {
			msg := new(LocalPositionNedCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS: func(pkt *Packet) Message {
			msg := new(RcChannels)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REQUEST_DATA_STREAM: func(pkt *Packet) Message {
			msg := new(RequestDataStream)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA_STREAM: func(pkt *Packet) Message {
			msg := new(DataStream)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MANUAL_CONTROL: func(pkt *Packet) Message {
			msg := new(ManualControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RC_CHANNELS_OVERRIDE: func(pkt *Packet) Message {
			msg := new(RcChannelsOverride)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MISSION_ITEM_INT: func(pkt *Packet) Message {
			msg := new(MissionItemInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VFR_HUD: func(pkt *Packet) Message {
			msg := new(VfrHud)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_INT: func(pkt *Packet) Message {
			msg := new(CommandInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_LONG: func(pkt *Packet) Message {
			msg := new(CommandLong)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMMAND_ACK: func(pkt *Packet) Message {
			msg := new(CommandAck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MANUAL_SETPOINT: func(pkt *Packet) Message {
			msg := new(ManualSetpoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_ATTITUDE_TARGET: func(pkt *Packet) Message {
			msg := new(SetAttitudeTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATTITUDE_TARGET: func(pkt *Packet) Message {
			msg := new(AttitudeTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_POSITION_TARGET_LOCAL_NED: func(pkt *Packet) Message {
			msg := new(SetPositionTargetLocalNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POSITION_TARGET_LOCAL_NED: func(pkt *Packet) Message {
			msg := new(PositionTargetLocalNed)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_POSITION_TARGET_GLOBAL_INT: func(pkt *Packet) Message {
			msg := new(SetPositionTargetGlobalInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POSITION_TARGET_GLOBAL_INT: func(pkt *Packet) Message {
			msg := new(PositionTargetGlobalInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET: func(pkt *Packet) Message {
			msg := new(LocalPositionNedSystemGlobalOffset)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_STATE: func(pkt *Packet) Message {
			msg := new(HilState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_CONTROLS: func(pkt *Packet) Message {
			msg := new(HilControls)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_RC_INPUTS_RAW: func(pkt *Packet) Message {
			msg := new(HilRcInputsRaw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_ACTUATOR_CONTROLS: func(pkt *Packet) Message {
			msg := new(HilActuatorControls)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_OPTICAL_FLOW: func(pkt *Packet) Message {
			msg := new(OpticalFlow)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(GlobalVisionPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VISION_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(VisionPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VISION_SPEED_ESTIMATE: func(pkt *Packet) Message {
			msg := new(VisionSpeedEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VICON_POSITION_ESTIMATE: func(pkt *Packet) Message {
			msg := new(ViconPositionEstimate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGHRES_IMU: func(pkt *Packet) Message {
			msg := new(HighresImu)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_OPTICAL_FLOW_RAD: func(pkt *Packet) Message {
			msg := new(OpticalFlowRad)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_SENSOR: func(pkt *Packet) Message {
			msg := new(HilSensor)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SIM_STATE: func(pkt *Packet) Message {
			msg := new(SimState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO_STATUS: func(pkt *Packet) Message {
			msg := new(RadioStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FILE_TRANSFER_PROTOCOL: func(pkt *Packet) Message {
			msg := new(FileTransferProtocol)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TIMESYNC: func(pkt *Packet) Message {
			msg := new(Timesync)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_TRIGGER: func(pkt *Packet) Message {
			msg := new(CameraTrigger)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_GPS: func(pkt *Packet) Message {
			msg := new(HilGps)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_OPTICAL_FLOW: func(pkt *Packet) Message {
			msg := new(HilOpticalFlow)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIL_STATE_QUATERNION: func(pkt *Packet) Message {
			msg := new(HilStateQuaternion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU2: func(pkt *Packet) Message {
			msg := new(ScaledImu2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_LIST: func(pkt *Packet) Message {
			msg := new(LogRequestList)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_ENTRY: func(pkt *Packet) Message {
			msg := new(LogEntry)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_DATA: func(pkt *Packet) Message {
			msg := new(LogRequestData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_DATA: func(pkt *Packet) Message {
			msg := new(LogData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_ERASE: func(pkt *Packet) Message {
			msg := new(LogErase)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LOG_REQUEST_END: func(pkt *Packet) Message {
			msg := new(LogRequestEnd)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_INJECT_DATA: func(pkt *Packet) Message {
			msg := new(GpsInjectData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS2_RAW: func(pkt *Packet) Message {
			msg := new(Gps2Raw)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_POWER_STATUS: func(pkt *Packet) Message {
			msg := new(PowerStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SERIAL_CONTROL: func(pkt *Packet) Message {
			msg := new(SerialControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RTK: func(pkt *Packet) Message {
			msg := new(GpsRtk)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS2_RTK: func(pkt *Packet) Message {
			msg := new(Gps2Rtk)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_IMU3: func(pkt *Packet) Message {
			msg := new(ScaledImu3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA_TRANSMISSION_HANDSHAKE: func(pkt *Packet) Message {
			msg := new(DataTransmissionHandshake)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ENCAPSULATED_DATA: func(pkt *Packet) Message {
			msg := new(EncapsulatedData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DISTANCE_SENSOR: func(pkt *Packet) Message {
			msg := new(DistanceSensor)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_REQUEST: func(pkt *Packet) Message {
			msg := new(TerrainRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_DATA: func(pkt *Packet) Message {
			msg := new(TerrainData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_CHECK: func(pkt *Packet) Message {
			msg := new(TerrainCheck)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_TERRAIN_REPORT: func(pkt *Packet) Message {
			msg := new(TerrainReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE2: func(pkt *Packet) Message {
			msg := new(ScaledPressure2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ATT_POS_MOCAP: func(pkt *Packet) Message {
			msg := new(AttPosMocap)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_ACTUATOR_CONTROL_TARGET: func(pkt *Packet) Message {
			msg := new(SetActuatorControlTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ACTUATOR_CONTROL_TARGET: func(pkt *Packet) Message {
			msg := new(ActuatorControlTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ALTITUDE: func(pkt *Packet) Message {
			msg := new(Altitude)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RESOURCE_REQUEST: func(pkt *Packet) Message {
			msg := new(ResourceRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SCALED_PRESSURE3: func(pkt *Packet) Message {
			msg := new(ScaledPressure3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FOLLOW_TARGET: func(pkt *Packet) Message {
			msg := new(FollowTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CONTROL_SYSTEM_STATE: func(pkt *Packet) Message {
			msg := new(ControlSystemState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_BATTERY_STATUS: func(pkt *Packet) Message {
			msg := new(BatteryStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTOPILOT_VERSION: func(pkt *Packet) Message {
			msg := new(AutopilotVersion)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LANDING_TARGET: func(pkt *Packet) Message {
			msg := new(LandingTarget)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ESTIMATOR_STATUS: func(pkt *Packet) Message {
			msg := new(EstimatorStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_WIND_COV: func(pkt *Packet) Message {
			msg := new(WindCov)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_INPUT: func(pkt *Packet) Message {
			msg := new(GpsInput)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GPS_RTCM_DATA: func(pkt *Packet) Message {
			msg := new(GpsRtcmData)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGH_LATENCY: func(pkt *Packet) Message {
			msg := new(HighLatency)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HIGH_LATENCY2: func(pkt *Packet) Message {
			msg := new(HighLatency2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_VIBRATION: func(pkt *Packet) Message {
			msg := new(Vibration)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HOME_POSITION: func(pkt *Packet) Message {
			msg := new(HomePosition)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_HOME_POSITION: func(pkt *Packet) Message {
			msg := new(SetHomePosition)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MESSAGE_INTERVAL: func(pkt *Packet) Message {
			msg := new(MessageInterval)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EXTENDED_SYS_STATE: func(pkt *Packet) Message {
			msg := new(ExtendedSysState)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_ADSB_VEHICLE: func(pkt *Packet) Message {
			msg := new(AdsbVehicle)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COLLISION: func(pkt *Packet) Message {
			msg := new(Collision)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_V2_EXTENSION: func(pkt *Packet) Message {
			msg := new(V2Extension)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MEMORY_VECT: func(pkt *Packet) Message {
			msg := new(MemoryVect)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEBUG_VECT: func(pkt *Packet) Message {
			msg := new(DebugVect)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAMED_VALUE_FLOAT: func(pkt *Packet) Message {
			msg := new(NamedValueFloat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_NAMED_VALUE_INT: func(pkt *Packet) Message {
			msg := new(NamedValueInt)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_STATUSTEXT: func(pkt *Packet) Message {
			msg := new(Statustext)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEBUG: func(pkt *Packet) Message {
			msg := new(Debug)
			msg.Unpack(pkt)
			return msg
		},
	},
}
