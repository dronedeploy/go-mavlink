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

// AccelcalVehiclePos:
const (
	ACCELCAL_VEHICLE_POS_LEVEL    = 1        //
	ACCELCAL_VEHICLE_POS_LEFT     = 2        //
	ACCELCAL_VEHICLE_POS_RIGHT    = 3        //
	ACCELCAL_VEHICLE_POS_NOSEDOWN = 4        //
	ACCELCAL_VEHICLE_POS_NOSEUP   = 5        //
	ACCELCAL_VEHICLE_POS_BACK     = 6        //
	ACCELCAL_VEHICLE_POS_SUCCESS  = 16777215 //
	ACCELCAL_VEHICLE_POS_FAILED   = 16777216 //
)

// MavCmd:
const (
	MAV_CMD_DO_GRIPPER                      = 211   // Mission command to operate EPM gripper.
	MAV_CMD_DO_AUTOTUNE_ENABLE              = 212   // Enable/disable autotune.
	MAV_CMD_NAV_ALTITUDE_WAIT               = 83    // Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up.
	MAV_CMD_POWER_OFF_INITIATED             = 42000 // A system wide power-off event has been initiated.
	MAV_CMD_SOLO_BTN_FLY_CLICK              = 42001 // FLY button has been clicked.
	MAV_CMD_SOLO_BTN_FLY_HOLD               = 42002 // FLY button has been held for 1.5 seconds.
	MAV_CMD_SOLO_BTN_PAUSE_CLICK            = 42003 // PAUSE button has been clicked.
	MAV_CMD_FIXED_MAG_CAL                   = 42004 // Magnetometer calibration based on fixed position         in earth field given by inclination, declination and intensity.
	MAV_CMD_FIXED_MAG_CAL_FIELD             = 42005 // Magnetometer calibration based on fixed expected field values in milliGauss.
	MAV_CMD_DO_START_MAG_CAL                = 42424 // Initiate a magnetometer calibration.
	MAV_CMD_DO_ACCEPT_MAG_CAL               = 42425 // Initiate a magnetometer calibration.
	MAV_CMD_DO_CANCEL_MAG_CAL               = 42426 // Cancel a running magnetometer calibration.
	MAV_CMD_ACCELCAL_VEHICLE_POS            = 42429 // Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in.
	MAV_CMD_DO_SEND_BANNER                  = 42428 // Reply with the version banner.
	MAV_CMD_SET_FACTORY_TEST_MODE           = 42427 // Command autopilot to get into factory test/diagnostic mode.
	MAV_CMD_GIMBAL_RESET                    = 42501 // Causes the gimbal to reset and boot as if it was just powered on.
	MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS  = 42502 // Reports progress and success or failure of gimbal axis calibration procedure.
	MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION = 42503 // Starts commutation calibration on the gimbal.
	MAV_CMD_GIMBAL_FULL_RESET               = 42505 // Erases gimbal application and parameters.
	MAV_CMD_DO_WINCH                        = 42600 // Command to operate winch.
	MAV_CMD_FLASH_BOOTLOADER                = 42650 // Update the bootloader
)

// LimitsState:
const (
	LIMITS_INIT       = 0 // Pre-initialization.
	LIMITS_DISABLED   = 1 // Disabled.
	LIMITS_ENABLED    = 2 // Checking limits.
	LIMITS_TRIGGERED  = 3 // A limit has been breached.
	LIMITS_RECOVERING = 4 // Taking action e.g. Return/RTL.
	LIMITS_RECOVERED  = 5 // We're no longer in breach of a limit.
)

// LimitModule:
const (
	LIMIT_GPSLOCK  = 1 // Pre-initialization.
	LIMIT_GEOFENCE = 2 // Disabled.
	LIMIT_ALTITUDE = 4 // Checking limits.
)

// RallyFlags: Flags in RALLY_POINT message.
const (
	FAVORABLE_WIND   = 1 // Flag set when requiring favorable winds for landing.
	LAND_IMMEDIATELY = 2 // Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land.
)

// ParachuteAction:
const (
	PARACHUTE_DISABLE = 0 // Disable parachute release.
	PARACHUTE_ENABLE  = 1 // Enable parachute release.
	PARACHUTE_RELEASE = 2 // Release parachute.
)

// GripperActions: Gripper actions.
const (
	GRIPPER_ACTION_RELEASE = 0 // Gripper release cargo.
	GRIPPER_ACTION_GRAB    = 1 // Gripper grab onto cargo.
)

// WinchActions: Winch actions.
const (
	WINCH_RELAXED                 = 0 // Relax winch.
	WINCH_RELATIVE_LENGTH_CONTROL = 1 // Winch unwinds or winds specified length of cable optionally using specified rate.
	WINCH_RATE_CONTROL            = 2 // Winch unwinds or winds cable at specified rate in meters/seconds.
)

// CameraStatusTypes:
const (
	CAMERA_STATUS_TYPE_HEARTBEAT  = 0 // Camera heartbeat, announce camera component ID at 1Hz.
	CAMERA_STATUS_TYPE_TRIGGER    = 1 // Camera image triggered.
	CAMERA_STATUS_TYPE_DISCONNECT = 2 // Camera connection lost.
	CAMERA_STATUS_TYPE_ERROR      = 3 // Camera unknown error.
	CAMERA_STATUS_TYPE_LOWBATT    = 4 // Camera battery low. Parameter p1 shows reported voltage.
	CAMERA_STATUS_TYPE_LOWSTORE   = 5 // Camera storage low. Parameter p1 shows reported shots remaining.
	CAMERA_STATUS_TYPE_LOWSTOREV  = 6 // Camera storage low. Parameter p1 shows reported video minutes remaining.
)

// CameraFeedbackFlags:
const (
	CAMERA_FEEDBACK_PHOTO       = 0 // Shooting photos, not video.
	CAMERA_FEEDBACK_VIDEO       = 1 // Shooting video, not stills.
	CAMERA_FEEDBACK_BADEXPOSURE = 2 // Unable to achieve requested exposure (e.g. shutter speed too low).
	CAMERA_FEEDBACK_CLOSEDLOOP  = 3 // Closed loop feedback from camera, we know for sure it has successfully taken a picture.
	CAMERA_FEEDBACK_OPENLOOP    = 4 // Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture.
)

// MavModeGimbal:
const (
	MAV_MODE_GIMBAL_UNINITIALIZED     = 0 // Gimbal is powered on but has not started initializing yet.
	MAV_MODE_GIMBAL_CALIBRATING_PITCH = 1 // Gimbal is currently running calibration on the pitch axis.
	MAV_MODE_GIMBAL_CALIBRATING_ROLL  = 2 // Gimbal is currently running calibration on the roll axis.
	MAV_MODE_GIMBAL_CALIBRATING_YAW   = 3 // Gimbal is currently running calibration on the yaw axis.
	MAV_MODE_GIMBAL_INITIALIZED       = 4 // Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter.
	MAV_MODE_GIMBAL_ACTIVE            = 5 // Gimbal is actively stabilizing.
	MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT  = 6 // Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command.
)

// GimbalAxis:
const (
	GIMBAL_AXIS_YAW   = 0 // Gimbal yaw axis.
	GIMBAL_AXIS_PITCH = 1 // Gimbal pitch axis.
	GIMBAL_AXIS_ROLL  = 2 // Gimbal roll axis.
)

// GimbalAxisCalibrationStatus:
const (
	GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS = 0 // Axis calibration is in progress.
	GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED   = 1 // Axis calibration succeeded.
	GIMBAL_AXIS_CALIBRATION_STATUS_FAILED      = 2 // Axis calibration failed.
)

// GimbalAxisCalibrationRequired:
const (
	GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN = 0 // Whether or not this axis requires calibration is unknown at this time.
	GIMBAL_AXIS_CALIBRATION_REQUIRED_TRUE    = 1 // This axis requires calibration.
	GIMBAL_AXIS_CALIBRATION_REQUIRED_FALSE   = 2 // This axis does not require calibration.
)

// GoproHeartbeatStatus:
const (
	GOPRO_HEARTBEAT_STATUS_DISCONNECTED = 0 // No GoPro connected.
	GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE = 1 // The detected GoPro is not HeroBus compatible.
	GOPRO_HEARTBEAT_STATUS_CONNECTED    = 2 // A HeroBus compatible GoPro is connected.
	GOPRO_HEARTBEAT_STATUS_ERROR        = 3 // An unrecoverable error was encountered with the connected GoPro, it may require a power cycle.
)

// GoproHeartbeatFlags:
const (
	GOPRO_FLAG_RECORDING = 1 // GoPro is currently recording.
)

// GoproRequestStatus:
const (
	GOPRO_REQUEST_SUCCESS = 0 // The write message with ID indicated succeeded.
	GOPRO_REQUEST_FAILED  = 1 // The write message with ID indicated failed.
)

// GoproCommand:
const (
	GOPRO_COMMAND_POWER                 = 0  // (Get/Set).
	GOPRO_COMMAND_CAPTURE_MODE          = 1  // (Get/Set).
	GOPRO_COMMAND_SHUTTER               = 2  // (___/Set).
	GOPRO_COMMAND_BATTERY               = 3  // (Get/___).
	GOPRO_COMMAND_MODEL                 = 4  // (Get/___).
	GOPRO_COMMAND_VIDEO_SETTINGS        = 5  // (Get/Set).
	GOPRO_COMMAND_LOW_LIGHT             = 6  // (Get/Set).
	GOPRO_COMMAND_PHOTO_RESOLUTION      = 7  // (Get/Set).
	GOPRO_COMMAND_PHOTO_BURST_RATE      = 8  // (Get/Set).
	GOPRO_COMMAND_PROTUNE               = 9  // (Get/Set).
	GOPRO_COMMAND_PROTUNE_WHITE_BALANCE = 10 // (Get/Set) Hero 3+ Only.
	GOPRO_COMMAND_PROTUNE_COLOUR        = 11 // (Get/Set) Hero 3+ Only.
	GOPRO_COMMAND_PROTUNE_GAIN          = 12 // (Get/Set) Hero 3+ Only.
	GOPRO_COMMAND_PROTUNE_SHARPNESS     = 13 // (Get/Set) Hero 3+ Only.
	GOPRO_COMMAND_PROTUNE_EXPOSURE      = 14 // (Get/Set) Hero 3+ Only.
	GOPRO_COMMAND_TIME                  = 15 // (Get/Set).
	GOPRO_COMMAND_CHARGING              = 16 // (Get/Set).
)

// GoproCaptureMode:
const (
	GOPRO_CAPTURE_MODE_VIDEO      = 0   // Video mode.
	GOPRO_CAPTURE_MODE_PHOTO      = 1   // Photo mode.
	GOPRO_CAPTURE_MODE_BURST      = 2   // Burst mode, Hero 3+ only.
	GOPRO_CAPTURE_MODE_TIME_LAPSE = 3   // Time lapse mode, Hero 3+ only.
	GOPRO_CAPTURE_MODE_MULTI_SHOT = 4   // Multi shot mode, Hero 4 only.
	GOPRO_CAPTURE_MODE_PLAYBACK   = 5   // Playback mode, Hero 4 only, silver only except when LCD or HDMI is connected to black.
	GOPRO_CAPTURE_MODE_SETUP      = 6   // Playback mode, Hero 4 only.
	GOPRO_CAPTURE_MODE_UNKNOWN    = 255 // Mode not yet known.
)

// GoproResolution:
const (
	GOPRO_RESOLUTION_480p            = 0  // 848 x 480 (480p).
	GOPRO_RESOLUTION_720p            = 1  // 1280 x 720 (720p).
	GOPRO_RESOLUTION_960p            = 2  // 1280 x 960 (960p).
	GOPRO_RESOLUTION_1080p           = 3  // 1920 x 1080 (1080p).
	GOPRO_RESOLUTION_1440p           = 4  // 1920 x 1440 (1440p).
	GOPRO_RESOLUTION_2_7k_17_9       = 5  // 2704 x 1440 (2.7k-17:9).
	GOPRO_RESOLUTION_2_7k_16_9       = 6  // 2704 x 1524 (2.7k-16:9).
	GOPRO_RESOLUTION_2_7k_4_3        = 7  // 2704 x 2028 (2.7k-4:3).
	GOPRO_RESOLUTION_4k_16_9         = 8  // 3840 x 2160 (4k-16:9).
	GOPRO_RESOLUTION_4k_17_9         = 9  // 4096 x 2160 (4k-17:9).
	GOPRO_RESOLUTION_720p_SUPERVIEW  = 10 // 1280 x 720 (720p-SuperView).
	GOPRO_RESOLUTION_1080p_SUPERVIEW = 11 // 1920 x 1080 (1080p-SuperView).
	GOPRO_RESOLUTION_2_7k_SUPERVIEW  = 12 // 2704 x 1520 (2.7k-SuperView).
	GOPRO_RESOLUTION_4k_SUPERVIEW    = 13 // 3840 x 2160 (4k-SuperView).
)

// GoproFrameRate:
const (
	GOPRO_FRAME_RATE_12   = 0  // 12 FPS.
	GOPRO_FRAME_RATE_15   = 1  // 15 FPS.
	GOPRO_FRAME_RATE_24   = 2  // 24 FPS.
	GOPRO_FRAME_RATE_25   = 3  // 25 FPS.
	GOPRO_FRAME_RATE_30   = 4  // 30 FPS.
	GOPRO_FRAME_RATE_48   = 5  // 48 FPS.
	GOPRO_FRAME_RATE_50   = 6  // 50 FPS.
	GOPRO_FRAME_RATE_60   = 7  // 60 FPS.
	GOPRO_FRAME_RATE_80   = 8  // 80 FPS.
	GOPRO_FRAME_RATE_90   = 9  // 90 FPS.
	GOPRO_FRAME_RATE_100  = 10 // 100 FPS.
	GOPRO_FRAME_RATE_120  = 11 // 120 FPS.
	GOPRO_FRAME_RATE_240  = 12 // 240 FPS.
	GOPRO_FRAME_RATE_12_5 = 13 // 12.5 FPS.
)

// GoproFieldOfView:
const (
	GOPRO_FIELD_OF_VIEW_WIDE   = 0 // 0x00: Wide.
	GOPRO_FIELD_OF_VIEW_MEDIUM = 1 // 0x01: Medium.
	GOPRO_FIELD_OF_VIEW_NARROW = 2 // 0x02: Narrow.
)

// GoproVideoSettingsFlags:
const (
	GOPRO_VIDEO_SETTINGS_TV_MODE = 1 // 0=NTSC, 1=PAL.
)

// GoproPhotoResolution:
const (
	GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM = 0 // 5MP Medium.
	GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM = 1 // 7MP Medium.
	GOPRO_PHOTO_RESOLUTION_7MP_WIDE   = 2 // 7MP Wide.
	GOPRO_PHOTO_RESOLUTION_10MP_WIDE  = 3 // 10MP Wide.
	GOPRO_PHOTO_RESOLUTION_12MP_WIDE  = 4 // 12MP Wide.
)

// GoproProtuneWhiteBalance:
const (
	GOPRO_PROTUNE_WHITE_BALANCE_AUTO  = 0 // Auto.
	GOPRO_PROTUNE_WHITE_BALANCE_3000K = 1 // 3000K.
	GOPRO_PROTUNE_WHITE_BALANCE_5500K = 2 // 5500K.
	GOPRO_PROTUNE_WHITE_BALANCE_6500K = 3 // 6500K.
	GOPRO_PROTUNE_WHITE_BALANCE_RAW   = 4 // Camera Raw.
)

// GoproProtuneColour:
const (
	GOPRO_PROTUNE_COLOUR_STANDARD = 0 // Auto.
	GOPRO_PROTUNE_COLOUR_NEUTRAL  = 1 // Neutral.
)

// GoproProtuneGain:
const (
	GOPRO_PROTUNE_GAIN_400  = 0 // ISO 400.
	GOPRO_PROTUNE_GAIN_800  = 1 // ISO 800 (Only Hero 4).
	GOPRO_PROTUNE_GAIN_1600 = 2 // ISO 1600.
	GOPRO_PROTUNE_GAIN_3200 = 3 // ISO 3200 (Only Hero 4).
	GOPRO_PROTUNE_GAIN_6400 = 4 // ISO 6400.
)

// GoproProtuneSharpness:
const (
	GOPRO_PROTUNE_SHARPNESS_LOW    = 0 // Low Sharpness.
	GOPRO_PROTUNE_SHARPNESS_MEDIUM = 1 // Medium Sharpness.
	GOPRO_PROTUNE_SHARPNESS_HIGH   = 2 // High Sharpness.
)

// GoproProtuneExposure:
const (
	GOPRO_PROTUNE_EXPOSURE_NEG_5_0 = 0  // -5.0 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_NEG_4_5 = 1  // -4.5 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_NEG_4_0 = 2  // -4.0 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_NEG_3_5 = 3  // -3.5 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_NEG_3_0 = 4  // -3.0 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_NEG_2_5 = 5  // -2.5 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_NEG_2_0 = 6  // -2.0 EV.
	GOPRO_PROTUNE_EXPOSURE_NEG_1_5 = 7  // -1.5 EV.
	GOPRO_PROTUNE_EXPOSURE_NEG_1_0 = 8  // -1.0 EV.
	GOPRO_PROTUNE_EXPOSURE_NEG_0_5 = 9  // -0.5 EV.
	GOPRO_PROTUNE_EXPOSURE_ZERO    = 10 // 0.0 EV.
	GOPRO_PROTUNE_EXPOSURE_POS_0_5 = 11 // +0.5 EV.
	GOPRO_PROTUNE_EXPOSURE_POS_1_0 = 12 // +1.0 EV.
	GOPRO_PROTUNE_EXPOSURE_POS_1_5 = 13 // +1.5 EV.
	GOPRO_PROTUNE_EXPOSURE_POS_2_0 = 14 // +2.0 EV.
	GOPRO_PROTUNE_EXPOSURE_POS_2_5 = 15 // +2.5 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_POS_3_0 = 16 // +3.0 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_POS_3_5 = 17 // +3.5 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_POS_4_0 = 18 // +4.0 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_POS_4_5 = 19 // +4.5 EV (Hero 3+ Only).
	GOPRO_PROTUNE_EXPOSURE_POS_5_0 = 20 // +5.0 EV (Hero 3+ Only).
)

// GoproCharging:
const (
	GOPRO_CHARGING_DISABLED = 0 // Charging disabled.
	GOPRO_CHARGING_ENABLED  = 1 // Charging enabled.
)

// GoproModel:
const (
	GOPRO_MODEL_UNKNOWN            = 0 // Unknown gopro model.
	GOPRO_MODEL_HERO_3_PLUS_SILVER = 1 // Hero 3+ Silver (HeroBus not supported by GoPro).
	GOPRO_MODEL_HERO_3_PLUS_BLACK  = 2 // Hero 3+ Black.
	GOPRO_MODEL_HERO_4_SILVER      = 3 // Hero 4 Silver.
	GOPRO_MODEL_HERO_4_BLACK       = 4 // Hero 4 Black.
)

// GoproBurstRate:
const (
	GOPRO_BURST_RATE_3_IN_1_SECOND  = 0 // 3 Shots / 1 Second.
	GOPRO_BURST_RATE_5_IN_1_SECOND  = 1 // 5 Shots / 1 Second.
	GOPRO_BURST_RATE_10_IN_1_SECOND = 2 // 10 Shots / 1 Second.
	GOPRO_BURST_RATE_10_IN_2_SECOND = 3 // 10 Shots / 2 Second.
	GOPRO_BURST_RATE_10_IN_3_SECOND = 4 // 10 Shots / 3 Second (Hero 4 Only).
	GOPRO_BURST_RATE_30_IN_1_SECOND = 5 // 30 Shots / 1 Second.
	GOPRO_BURST_RATE_30_IN_2_SECOND = 6 // 30 Shots / 2 Second.
	GOPRO_BURST_RATE_30_IN_3_SECOND = 7 // 30 Shots / 3 Second.
	GOPRO_BURST_RATE_30_IN_6_SECOND = 8 // 30 Shots / 6 Second.
)

// LedControlPattern:
const (
	LED_CONTROL_PATTERN_OFF            = 0   // LED patterns off (return control to regular vehicle control).
	LED_CONTROL_PATTERN_FIRMWAREUPDATE = 1   // LEDs show pattern during firmware update.
	LED_CONTROL_PATTERN_CUSTOM         = 255 // Custom Pattern using custom bytes fields.
)

// EkfStatusFlags: Flags in EKF_STATUS message.
const (
	EKF_ATTITUDE           = 1   // Set if EKF's attitude estimate is good.
	EKF_VELOCITY_HORIZ     = 2   // Set if EKF's horizontal velocity estimate is good.
	EKF_VELOCITY_VERT      = 4   // Set if EKF's vertical velocity estimate is good.
	EKF_POS_HORIZ_REL      = 8   // Set if EKF's horizontal position (relative) estimate is good.
	EKF_POS_HORIZ_ABS      = 16  // Set if EKF's horizontal position (absolute) estimate is good.
	EKF_POS_VERT_ABS       = 32  // Set if EKF's vertical position (absolute) estimate is good.
	EKF_POS_VERT_AGL       = 64  // Set if EKF's vertical position (above ground) estimate is good.
	EKF_CONST_POS_MODE     = 128 // EKF is in constant position mode and does not know it's absolute or relative position.
	EKF_PRED_POS_HORIZ_REL = 256 // Set if EKF's predicted horizontal position (relative) estimate is good.
	EKF_PRED_POS_HORIZ_ABS = 512 // Set if EKF's predicted horizontal position (absolute) estimate is good.
)

// PidTuningAxis:
const (
	PID_TUNING_ROLL    = 1 //
	PID_TUNING_PITCH   = 2 //
	PID_TUNING_YAW     = 3 //
	PID_TUNING_ACCZ    = 4 //
	PID_TUNING_STEER   = 5 //
	PID_TUNING_LANDING = 6 //
)

// MagCalStatus:
const (
	MAG_CAL_NOT_STARTED      = 0 //
	MAG_CAL_WAITING_TO_START = 1 //
	MAG_CAL_RUNNING_STEP_ONE = 2 //
	MAG_CAL_RUNNING_STEP_TWO = 3 //
	MAG_CAL_SUCCESS          = 4 //
	MAG_CAL_FAILED           = 5 //
	MAG_CAL_BAD_ORIENTATION  = 6 //
)

// MavRemoteLogDataBlockCommands: Special ACK block numbers control activation of dataflash log streaming.
const (
	MAV_REMOTE_LOG_DATA_BLOCK_STOP  = 2147483645 // UAV to stop sending DataFlash blocks.
	MAV_REMOTE_LOG_DATA_BLOCK_START = 2147483646 // UAV to start sending DataFlash blocks.
)

// MavRemoteLogDataBlockStatuses: Possible remote log data block statuses.
const (
	MAV_REMOTE_LOG_DATA_BLOCK_NACK = 0 // This block has NOT been received.
	MAV_REMOTE_LOG_DATA_BLOCK_ACK  = 1 // This block has been received.
)

// DeviceOpBustype: Bus types for device operations.
const (
	DEVICE_OP_BUSTYPE_I2C = 0 // I2C Device operation.
	DEVICE_OP_BUSTYPE_SPI = 1 // SPI Device operation.
)

// DeepstallStage: Deepstall flight stage.
const (
	DEEPSTALL_STAGE_FLY_TO_LANDING    = 0 // Flying to the landing point.
	DEEPSTALL_STAGE_ESTIMATE_WIND     = 1 // Building an estimate of the wind.
	DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT = 2 // Waiting to breakout of the loiter to fly the approach.
	DEEPSTALL_STAGE_FLY_TO_ARC        = 3 // Flying to the first arc point to turn around to the landing point.
	DEEPSTALL_STAGE_ARC               = 4 // Turning around back to the deepstall landing point.
	DEEPSTALL_STAGE_APPROACH          = 5 // Approaching the landing point.
	DEEPSTALL_STAGE_LAND              = 6 // Stalling and steering towards the land point.
)

// PlaneMode: A mapping of plane flight modes for custom_mode field of heartbeat.
const (
	PLANE_MODE_MANUAL        = 0  //
	PLANE_MODE_CIRCLE        = 1  //
	PLANE_MODE_STABILIZE     = 2  //
	PLANE_MODE_TRAINING      = 3  //
	PLANE_MODE_ACRO          = 4  //
	PLANE_MODE_FLY_BY_WIRE_A = 5  //
	PLANE_MODE_FLY_BY_WIRE_B = 6  //
	PLANE_MODE_CRUISE        = 7  //
	PLANE_MODE_AUTOTUNE      = 8  //
	PLANE_MODE_AUTO          = 10 //
	PLANE_MODE_RTL           = 11 //
	PLANE_MODE_LOITER        = 12 //
	PLANE_MODE_AVOID_ADSB    = 14 //
	PLANE_MODE_GUIDED        = 15 //
	PLANE_MODE_INITIALIZING  = 16 //
	PLANE_MODE_QSTABILIZE    = 17 //
	PLANE_MODE_QHOVER        = 18 //
	PLANE_MODE_QLOITER       = 19 //
	PLANE_MODE_QLAND         = 20 //
	PLANE_MODE_QRTL          = 21 //
)

// CopterMode: A mapping of copter flight modes for custom_mode field of heartbeat.
const (
	COPTER_MODE_STABILIZE    = 0  //
	COPTER_MODE_ACRO         = 1  //
	COPTER_MODE_ALT_HOLD     = 2  //
	COPTER_MODE_AUTO         = 3  //
	COPTER_MODE_GUIDED       = 4  //
	COPTER_MODE_LOITER       = 5  //
	COPTER_MODE_RTL          = 6  //
	COPTER_MODE_CIRCLE       = 7  //
	COPTER_MODE_LAND         = 9  //
	COPTER_MODE_DRIFT        = 11 //
	COPTER_MODE_SPORT        = 13 //
	COPTER_MODE_FLIP         = 14 //
	COPTER_MODE_AUTOTUNE     = 15 //
	COPTER_MODE_POSHOLD      = 16 //
	COPTER_MODE_BRAKE        = 17 //
	COPTER_MODE_THROW        = 18 //
	COPTER_MODE_AVOID_ADSB   = 19 //
	COPTER_MODE_GUIDED_NOGPS = 20 //
	COPTER_MODE_SMART_RTL    = 21 //
)

// SubMode: A mapping of sub flight modes for custom_mode field of heartbeat.
const (
	SUB_MODE_STABILIZE = 0  //
	SUB_MODE_ACRO      = 1  //
	SUB_MODE_ALT_HOLD  = 2  //
	SUB_MODE_AUTO      = 3  //
	SUB_MODE_GUIDED    = 4  //
	SUB_MODE_CIRCLE    = 7  //
	SUB_MODE_SURFACE   = 9  //
	SUB_MODE_POSHOLD   = 16 //
	SUB_MODE_MANUAL    = 19 //
)

// RoverMode: A mapping of rover flight modes for custom_mode field of heartbeat.
const (
	ROVER_MODE_MANUAL       = 0  //
	ROVER_MODE_ACRO         = 1  //
	ROVER_MODE_STEERING     = 3  //
	ROVER_MODE_HOLD         = 4  //
	ROVER_MODE_LOITER       = 5  //
	ROVER_MODE_AUTO         = 10 //
	ROVER_MODE_RTL          = 11 //
	ROVER_MODE_SMART_RTL    = 12 //
	ROVER_MODE_GUIDED       = 15 //
	ROVER_MODE_INITIALIZING = 16 //
)

// TrackerMode: A mapping of antenna tracker flight modes for custom_mode field of heartbeat.
const (
	TRACKER_MODE_MANUAL       = 0  //
	TRACKER_MODE_STOP         = 1  //
	TRACKER_MODE_SCAN         = 2  //
	TRACKER_MODE_SERVO_TEST   = 3  //
	TRACKER_MODE_AUTO         = 10 //
	TRACKER_MODE_INITIALIZING = 16 //
)

// Offsets and calibrations values for hardware sensors. This makes it easier to debug the calibration process.
type SensorOffsets struct {
	MagDeclination float32 `json:"magDeclination"` // Magnetic declination.
	RawPress       int32   `json:"rawPress"`       // Raw pressure from barometer.
	RawTemp        int32   `json:"rawTemp"`        // Raw temperature from barometer.
	GyroCalX       float32 `json:"gyroCalX"`       // Gyro X calibration.
	GyroCalY       float32 `json:"gyroCalY"`       // Gyro Y calibration.
	GyroCalZ       float32 `json:"gyroCalZ"`       // Gyro Z calibration.
	AccelCalX      float32 `json:"accelCalX"`      // Accel X calibration.
	AccelCalY      float32 `json:"accelCalY"`      // Accel Y calibration.
	AccelCalZ      float32 `json:"accelCalZ"`      // Accel Z calibration.
	MagOfsX        int16   `json:"magOfsX"`        // Magnetometer X offset.
	MagOfsY        int16   `json:"magOfsY"`        // Magnetometer Y offset.
	MagOfsZ        int16   `json:"magOfsZ"`        // Magnetometer Z offset.
}

func (self *SensorOffsets) MsgID() MessageID {
	return MSG_ID_SENSOR_OFFSETS
}

func (self *SensorOffsets) MsgName() string {
	return "SensorOffsets"
}

func (self *SensorOffsets) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.MagDeclination))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.RawPress))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.RawTemp))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.GyroCalX))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.GyroCalY))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.GyroCalZ))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.AccelCalX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.AccelCalY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.AccelCalZ))
	binary.LittleEndian.PutUint16(payload[36:], uint16(self.MagOfsX))
	binary.LittleEndian.PutUint16(payload[38:], uint16(self.MagOfsY))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.MagOfsZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SensorOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.MagDeclination = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.RawPress = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.RawTemp = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.GyroCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.GyroCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.GyroCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.AccelCalX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.AccelCalY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.AccelCalZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[36:]))
	self.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[38:]))
	self.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[40:]))
	return nil
}

func (self *SensorOffsets) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SensorOffsetsFromJSON(data []byte) (*SensorOffsets, error) {
	p := &SensorOffsets{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Set the magnetometer offsets
type SetMagOffsets struct {
	MagOfsX         int16 `json:"magOfsX"`         // Magnetometer X offset.
	MagOfsY         int16 `json:"magOfsY"`         // Magnetometer Y offset.
	MagOfsZ         int16 `json:"magOfsZ"`         // Magnetometer Z offset.
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
}

func (self *SetMagOffsets) MsgID() MessageID {
	return MSG_ID_SET_MAG_OFFSETS
}

func (self *SetMagOffsets) MsgName() string {
	return "SetMagOffsets"
}

func (self *SetMagOffsets) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.MagOfsX))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.MagOfsY))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.MagOfsZ))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *SetMagOffsets) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.MagOfsX = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.MagOfsY = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.MagOfsZ = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	return nil
}

func (self *SetMagOffsets) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SetMagOffsetsFromJSON(data []byte) (*SetMagOffsets, error) {
	p := &SetMagOffsets{}
	err := json.Unmarshal(data, p)
	return p, err
}

// State of APM memory.
type Meminfo struct {
	Brkval  uint16 `json:"brkval"`  // Heap top.
	Freemem uint16 `json:"freemem"` // Free memory.
}

func (self *Meminfo) MsgID() MessageID {
	return MSG_ID_MEMINFO
}

func (self *Meminfo) MsgName() string {
	return "Meminfo"
}

func (self *Meminfo) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Brkval))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Freemem))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Meminfo) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Brkval = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Freemem = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

func (self *Meminfo) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MeminfoFromJSON(data []byte) (*Meminfo, error) {
	p := &Meminfo{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Raw ADC output.
type ApAdc struct {
	Adc1 uint16 `json:"adc1"` // ADC output 1.
	Adc2 uint16 `json:"adc2"` // ADC output 2.
	Adc3 uint16 `json:"adc3"` // ADC output 3.
	Adc4 uint16 `json:"adc4"` // ADC output 4.
	Adc5 uint16 `json:"adc5"` // ADC output 5.
	Adc6 uint16 `json:"adc6"` // ADC output 6.
}

func (self *ApAdc) MsgID() MessageID {
	return MSG_ID_AP_ADC
}

func (self *ApAdc) MsgName() string {
	return "ApAdc"
}

func (self *ApAdc) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Adc1))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.Adc2))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.Adc3))
	binary.LittleEndian.PutUint16(payload[6:], uint16(self.Adc4))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Adc5))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.Adc6))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *ApAdc) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Adc1 = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.Adc2 = uint16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.Adc3 = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.Adc4 = uint16(binary.LittleEndian.Uint16(p.Payload[6:]))
	self.Adc5 = uint16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.Adc6 = uint16(binary.LittleEndian.Uint16(p.Payload[10:]))
	return nil
}

func (self *ApAdc) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func ApAdcFromJSON(data []byte) (*ApAdc, error) {
	p := &ApAdc{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Configure on-board Camera Control System.
type DigicamConfigure struct {
	ExtraValue      float32 `json:"extraValue"`      // Correspondent value to given extra_param.
	ShutterSpeed    uint16  `json:"shutterSpeed"`    // Divisor number //e.g. 1000 means 1/1000 (0 means ignore).
	TargetSystem    uint8   `json:"targetSystem"`    // System ID.
	TargetComponent uint8   `json:"targetComponent"` // Component ID.
	Mode            uint8   `json:"mode"`            // Mode enumeration from 1 to N //P, TV, AV, M, etc. (0 means ignore).
	Aperture        uint8   `json:"aperture"`        // F stop number x 10 //e.g. 28 means 2.8 (0 means ignore).
	Iso             uint8   `json:"iso"`             // ISO enumeration from 1 to N //e.g. 80, 100, 200, Etc (0 means ignore).
	ExposureType    uint8   `json:"exposureType"`    // Exposure type enumeration from 1 to N (0 means ignore).
	CommandId       uint8   `json:"commandId"`       // Command Identity (incremental loop: 0 to 255). //A command sent multiple times will be executed or pooled just once.
	EngineCutOff    uint8   `json:"engineCutOff"`    // Main engine cut-off time before camera trigger (0 means no cut-off).
	ExtraParam      uint8   `json:"extraParam"`      // Extra parameters enumeration (0 means ignore).
}

func (self *DigicamConfigure) MsgID() MessageID {
	return MSG_ID_DIGICAM_CONFIGURE
}

func (self *DigicamConfigure) MsgName() string {
	return "DigicamConfigure"
}

func (self *DigicamConfigure) Pack(p *Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ExtraValue))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.ShutterSpeed))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)
	payload[8] = byte(self.Mode)
	payload[9] = byte(self.Aperture)
	payload[10] = byte(self.Iso)
	payload[11] = byte(self.ExposureType)
	payload[12] = byte(self.CommandId)
	payload[13] = byte(self.EngineCutOff)
	payload[14] = byte(self.ExtraParam)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DigicamConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	self.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.ShutterSpeed = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	self.Mode = uint8(p.Payload[8])
	self.Aperture = uint8(p.Payload[9])
	self.Iso = uint8(p.Payload[10])
	self.ExposureType = uint8(p.Payload[11])
	self.CommandId = uint8(p.Payload[12])
	self.EngineCutOff = uint8(p.Payload[13])
	self.ExtraParam = uint8(p.Payload[14])
	return nil
}

func (self *DigicamConfigure) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DigicamConfigureFromJSON(data []byte) (*DigicamConfigure, error) {
	p := &DigicamConfigure{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Control on-board Camera Control System to take shots.
type DigicamControl struct {
	ExtraValue      float32 `json:"extraValue"`      // Correspondent value to given extra_param.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID.
	TargetComponent uint8   `json:"targetComponent"` // Component ID.
	Session         uint8   `json:"session"`         // 0: stop, 1: start or keep it up //Session control e.g. show/hide lens.
	ZoomPos         uint8   `json:"zoomPos"`         // 1 to N //Zoom's absolute position (0 means ignore).
	ZoomStep        int8    `json:"zoomStep"`        // -100 to 100 //Zooming step value to offset zoom from the current position.
	FocusLock       uint8   `json:"focusLock"`       // 0: unlock focus or keep unlocked, 1: lock focus or keep locked, 3: re-lock focus.
	Shot            uint8   `json:"shot"`            // 0: ignore, 1: shot or start filming.
	CommandId       uint8   `json:"commandId"`       // Command Identity (incremental loop: 0 to 255)//A command sent multiple times will be executed or pooled just once.
	ExtraParam      uint8   `json:"extraParam"`      // Extra parameters enumeration (0 means ignore).
}

func (self *DigicamControl) MsgID() MessageID {
	return MSG_ID_DIGICAM_CONTROL
}

func (self *DigicamControl) MsgName() string {
	return "DigicamControl"
}

func (self *DigicamControl) Pack(p *Packet) error {
	payload := make([]byte, 13)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.ExtraValue))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)
	payload[6] = byte(self.Session)
	payload[7] = byte(self.ZoomPos)
	payload[8] = byte(self.ZoomStep)
	payload[9] = byte(self.FocusLock)
	payload[10] = byte(self.Shot)
	payload[11] = byte(self.CommandId)
	payload[12] = byte(self.ExtraParam)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *DigicamControl) Unpack(p *Packet) error {
	if len(p.Payload) < 13 {
		return fmt.Errorf("payload too small")
	}
	self.ExtraValue = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	self.Session = uint8(p.Payload[6])
	self.ZoomPos = uint8(p.Payload[7])
	self.ZoomStep = int8(p.Payload[8])
	self.FocusLock = uint8(p.Payload[9])
	self.Shot = uint8(p.Payload[10])
	self.CommandId = uint8(p.Payload[11])
	self.ExtraParam = uint8(p.Payload[12])
	return nil
}

func (self *DigicamControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DigicamControlFromJSON(data []byte) (*DigicamControl, error) {
	p := &DigicamControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message to configure a camera mount, directional antenna, etc.
type MountConfigure struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
	MountMode       uint8 `json:"mountMode"`       // Mount operating mode.
	StabRoll        uint8 `json:"stabRoll"`        // (1 = yes, 0 = no).
	StabPitch       uint8 `json:"stabPitch"`       // (1 = yes, 0 = no).
	StabYaw         uint8 `json:"stabYaw"`         // (1 = yes, 0 = no).
}

func (self *MountConfigure) MsgID() MessageID {
	return MSG_ID_MOUNT_CONFIGURE
}

func (self *MountConfigure) MsgName() string {
	return "MountConfigure"
}

func (self *MountConfigure) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.MountMode)
	payload[3] = byte(self.StabRoll)
	payload[4] = byte(self.StabPitch)
	payload[5] = byte(self.StabYaw)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountConfigure) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.MountMode = uint8(p.Payload[2])
	self.StabRoll = uint8(p.Payload[3])
	self.StabPitch = uint8(p.Payload[4])
	self.StabYaw = uint8(p.Payload[5])
	return nil
}

func (self *MountConfigure) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MountConfigureFromJSON(data []byte) (*MountConfigure, error) {
	p := &MountConfigure{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message to control a camera mount, directional antenna, etc.
type MountControl struct {
	InputA          int32 `json:"inputA"`          // Pitch (centi-degrees) or lat (degE7), depending on mount mode.
	InputB          int32 `json:"inputB"`          // Roll (centi-degrees) or lon (degE7) depending on mount mode.
	InputC          int32 `json:"inputC"`          // Yaw (centi-degrees) or alt (cm) depending on mount mode.
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
	SavePosition    uint8 `json:"savePosition"`    // If "1" it will save current trimmed position on EEPROM (just valid for NEUTRAL and LANDING).
}

func (self *MountControl) MsgID() MessageID {
	return MSG_ID_MOUNT_CONTROL
}

func (self *MountControl) MsgName() string {
	return "MountControl"
}

func (self *MountControl) Pack(p *Packet) error {
	payload := make([]byte, 15)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.InputA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.InputB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.InputC))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)
	payload[14] = byte(self.SavePosition)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountControl) Unpack(p *Packet) error {
	if len(p.Payload) < 15 {
		return fmt.Errorf("payload too small")
	}
	self.InputA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.InputB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.InputC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	self.SavePosition = uint8(p.Payload[14])
	return nil
}

func (self *MountControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MountControlFromJSON(data []byte) (*MountControl, error) {
	p := &MountControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Message with some status from APM to GCS about camera or antenna mount.
type MountStatus struct {
	PointingA       int32 `json:"pointingA"`       // Pitch.
	PointingB       int32 `json:"pointingB"`       // Roll.
	PointingC       int32 `json:"pointingC"`       // Yaw.
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
}

func (self *MountStatus) MsgID() MessageID {
	return MSG_ID_MOUNT_STATUS
}

func (self *MountStatus) MsgName() string {
	return "MountStatus"
}

func (self *MountStatus) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.PointingA))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.PointingB))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.PointingC))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MountStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.PointingA = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PointingB = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PointingC = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	return nil
}

func (self *MountStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MountStatusFromJSON(data []byte) (*MountStatus, error) {
	p := &MountStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// A fence point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.
type FencePoint struct {
	Lat             float32 `json:"lat"`             // Latitude of point.
	Lng             float32 `json:"lng"`             // Longitude of point.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID.
	TargetComponent uint8   `json:"targetComponent"` // Component ID.
	Idx             uint8   `json:"idx"`             // Point index (first point is 1, 0 is for return point).
	Count           uint8   `json:"count"`           // Total number of points (for sanity checking).
}

func (self *FencePoint) MsgID() MessageID {
	return MSG_ID_FENCE_POINT
}

func (self *FencePoint) MsgName() string {
	return "FencePoint"
}

func (self *FencePoint) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Lng))
	payload[8] = byte(self.TargetSystem)
	payload[9] = byte(self.TargetComponent)
	payload[10] = byte(self.Idx)
	payload[11] = byte(self.Count)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FencePoint) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lng = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[8])
	self.TargetComponent = uint8(p.Payload[9])
	self.Idx = uint8(p.Payload[10])
	self.Count = uint8(p.Payload[11])
	return nil
}

func (self *FencePoint) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func FencePointFromJSON(data []byte) (*FencePoint, error) {
	p := &FencePoint{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a current fence point from MAV.
type FenceFetchPoint struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
	Idx             uint8 `json:"idx"`             // Point index (first point is 1, 0 is for return point).
}

func (self *FenceFetchPoint) MsgID() MessageID {
	return MSG_ID_FENCE_FETCH_POINT
}

func (self *FenceFetchPoint) MsgName() string {
	return "FenceFetchPoint"
}

func (self *FenceFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Idx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FenceFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Idx = uint8(p.Payload[2])
	return nil
}

func (self *FenceFetchPoint) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func FenceFetchPointFromJSON(data []byte) (*FenceFetchPoint, error) {
	p := &FenceFetchPoint{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of geo-fencing. Sent in extended status stream when fencing enabled.
type FenceStatus struct {
	BreachTime   uint32 `json:"breachTime"`   // Time (since boot) of last breach.
	BreachCount  uint16 `json:"breachCount"`  // Number of fence breaches.
	BreachStatus uint8  `json:"breachStatus"` // Breach status (0 if currently inside fence, 1 if outside).
	BreachType   uint8  `json:"breachType"`   // Last breach type.
}

func (self *FenceStatus) MsgID() MessageID {
	return MSG_ID_FENCE_STATUS
}

func (self *FenceStatus) MsgName() string {
	return "FenceStatus"
}

func (self *FenceStatus) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.BreachTime))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.BreachCount))
	payload[6] = byte(self.BreachStatus)
	payload[7] = byte(self.BreachType)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *FenceStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.BreachTime = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.BreachStatus = uint8(p.Payload[6])
	self.BreachType = uint8(p.Payload[7])
	return nil
}

func (self *FenceStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func FenceStatusFromJSON(data []byte) (*FenceStatus, error) {
	p := &FenceStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of DCM attitude estimator.
type Ahrs struct {
	Omegaix     float32 `json:"omegaix"`     // X gyro drift estimate.
	Omegaiy     float32 `json:"omegaiy"`     // Y gyro drift estimate.
	Omegaiz     float32 `json:"omegaiz"`     // Z gyro drift estimate.
	AccelWeight float32 `json:"accelWeight"` // Average accel_weight.
	RenormVal   float32 `json:"renormVal"`   // Average renormalisation value.
	ErrorRp     float32 `json:"errorRp"`     // Average error_roll_pitch value.
	ErrorYaw    float32 `json:"errorYaw"`    // Average error_yaw value.
}

func (self *Ahrs) MsgID() MessageID {
	return MSG_ID_AHRS
}

func (self *Ahrs) MsgName() string {
	return "Ahrs"
}

func (self *Ahrs) Pack(p *Packet) error {
	payload := make([]byte, 28)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Omegaix))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Omegaiy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Omegaiz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.AccelWeight))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.RenormVal))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.ErrorRp))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.ErrorYaw))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs) Unpack(p *Packet) error {
	if len(p.Payload) < 28 {
		return fmt.Errorf("payload too small")
	}
	self.Omegaix = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Omegaiy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Omegaiz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.AccelWeight = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.RenormVal = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.ErrorRp = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.ErrorYaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	return nil
}

func (self *Ahrs) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AhrsFromJSON(data []byte) (*Ahrs, error) {
	p := &Ahrs{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of simulation environment, if used.
type Simstate struct {
	Roll  float32 `json:"roll"`  // Roll angle.
	Pitch float32 `json:"pitch"` // Pitch angle.
	Yaw   float32 `json:"yaw"`   // Yaw angle.
	Xacc  float32 `json:"xacc"`  // X acceleration.
	Yacc  float32 `json:"yacc"`  // Y acceleration.
	Zacc  float32 `json:"zacc"`  // Z acceleration.
	Xgyro float32 `json:"xgyro"` // Angular speed around X axis.
	Ygyro float32 `json:"ygyro"` // Angular speed around Y axis.
	Zgyro float32 `json:"zgyro"` // Angular speed around Z axis.
	Lat   int32   `json:"lat"`   // Latitude.
	Lng   int32   `json:"lng"`   // Longitude.
}

func (self *Simstate) MsgID() MessageID {
	return MSG_ID_SIMSTATE
}

func (self *Simstate) MsgName() string {
	return "Simstate"
}

func (self *Simstate) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Xacc))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Yacc))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Zacc))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Xgyro))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Ygyro))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Zgyro))
	binary.LittleEndian.PutUint32(payload[36:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[40:], uint32(self.Lng))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Simstate) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Xacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Yacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Zacc = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Xgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Ygyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Zgyro = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[40:]))
	return nil
}

func (self *Simstate) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func SimstateFromJSON(data []byte) (*Simstate, error) {
	p := &Simstate{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of key hardware.
type Hwstatus struct {
	Vcc    uint16 `json:"vcc"`    // Board voltage.
	I2cerr uint8  `json:"i2cerr"` // I2C error count.
}

func (self *Hwstatus) MsgID() MessageID {
	return MSG_ID_HWSTATUS
}

func (self *Hwstatus) MsgName() string {
	return "Hwstatus"
}

func (self *Hwstatus) Pack(p *Packet) error {
	payload := make([]byte, 3)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Vcc))
	payload[2] = byte(self.I2cerr)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Hwstatus) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.Vcc = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.I2cerr = uint8(p.Payload[2])
	return nil
}

func (self *Hwstatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func HwstatusFromJSON(data []byte) (*Hwstatus, error) {
	p := &Hwstatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status generated by radio.
type Radio struct {
	Rxerrors uint16 `json:"rxerrors"` // Receive errors.
	Fixed    uint16 `json:"fixed"`    // Count of error corrected packets.
	Rssi     uint8  `json:"rssi"`     // Local signal strength.
	Remrssi  uint8  `json:"remrssi"`  // Remote signal strength.
	Txbuf    uint8  `json:"txbuf"`    // How full the tx buffer is.
	Noise    uint8  `json:"noise"`    // Background noise level.
	Remnoise uint8  `json:"remnoise"` // Remote background noise level.
}

func (self *Radio) MsgID() MessageID {
	return MSG_ID_RADIO
}

func (self *Radio) MsgName() string {
	return "Radio"
}

func (self *Radio) Pack(p *Packet) error {
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

func (self *Radio) Unpack(p *Packet) error {
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

func (self *Radio) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RadioFromJSON(data []byte) (*Radio, error) {
	p := &Radio{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of AP_Limits. Sent in extended status stream when AP_Limits is enabled.
type LimitsStatus struct {
	LastTrigger   uint32 `json:"lastTrigger"`   // Time (since boot) of last breach.
	LastAction    uint32 `json:"lastAction"`    // Time (since boot) of last recovery action.
	LastRecovery  uint32 `json:"lastRecovery"`  // Time (since boot) of last successful recovery.
	LastClear     uint32 `json:"lastClear"`     // Time (since boot) of last all-clear.
	BreachCount   uint16 `json:"breachCount"`   // Number of fence breaches.
	LimitsState   uint8  `json:"limitsState"`   // State of AP_Limits.
	ModsEnabled   uint8  `json:"modsEnabled"`   // AP_Limit_Module bitfield of enabled modules.
	ModsRequired  uint8  `json:"modsRequired"`  // AP_Limit_Module bitfield of required modules.
	ModsTriggered uint8  `json:"modsTriggered"` // AP_Limit_Module bitfield of triggered modules.
}

func (self *LimitsStatus) MsgID() MessageID {
	return MSG_ID_LIMITS_STATUS
}

func (self *LimitsStatus) MsgName() string {
	return "LimitsStatus"
}

func (self *LimitsStatus) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.LastTrigger))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.LastAction))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.LastRecovery))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.LastClear))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.BreachCount))
	payload[18] = byte(self.LimitsState)
	payload[19] = byte(self.ModsEnabled)
	payload[20] = byte(self.ModsRequired)
	payload[21] = byte(self.ModsTriggered)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LimitsStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.LastTrigger = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LastAction = uint32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.LastRecovery = uint32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.LastClear = uint32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.BreachCount = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.LimitsState = uint8(p.Payload[18])
	self.ModsEnabled = uint8(p.Payload[19])
	self.ModsRequired = uint8(p.Payload[20])
	self.ModsTriggered = uint8(p.Payload[21])
	return nil
}

func (self *LimitsStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LimitsStatusFromJSON(data []byte) (*LimitsStatus, error) {
	p := &LimitsStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Wind estimation.
type Wind struct {
	Direction float32 `json:"direction"` // Wind direction (that wind is coming from).
	Speed     float32 `json:"speed"`     // Wind speed in ground plane.
	SpeedZ    float32 `json:"speedZ"`    // Vertical wind speed.
}

func (self *Wind) MsgID() MessageID {
	return MSG_ID_WIND
}

func (self *Wind) MsgName() string {
	return "Wind"
}

func (self *Wind) Pack(p *Packet) error {
	payload := make([]byte, 12)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Direction))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Speed))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.SpeedZ))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Wind) Unpack(p *Packet) error {
	if len(p.Payload) < 12 {
		return fmt.Errorf("payload too small")
	}
	self.Direction = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Speed = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.SpeedZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	return nil
}

func (self *Wind) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func WindFromJSON(data []byte) (*Wind, error) {
	p := &Wind{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data packet, size 16.
type Data16 struct {
	Type uint8     `json:"type"` // Data type.
	Len  uint8     `json:"len"`  // Data length.
	Data [16]uint8 `json:"data"` // Raw data.
}

func (self *Data16) MsgID() MessageID {
	return MSG_ID_DATA16
}

func (self *Data16) MsgName() string {
	return "Data16"
}

func (self *Data16) Pack(p *Packet) error {
	payload := make([]byte, 18)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data16) Unpack(p *Packet) error {
	if len(p.Payload) < 18 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:18])
	return nil
}

func (self *Data16) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Data16FromJSON(data []byte) (*Data16, error) {
	p := &Data16{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data packet, size 32.
type Data32 struct {
	Type uint8     `json:"type"` // Data type.
	Len  uint8     `json:"len"`  // Data length.
	Data [32]uint8 `json:"data"` // Raw data.
}

func (self *Data32) MsgID() MessageID {
	return MSG_ID_DATA32
}

func (self *Data32) MsgName() string {
	return "Data32"
}

func (self *Data32) Pack(p *Packet) error {
	payload := make([]byte, 34)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data32) Unpack(p *Packet) error {
	if len(p.Payload) < 34 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:34])
	return nil
}

func (self *Data32) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Data32FromJSON(data []byte) (*Data32, error) {
	p := &Data32{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data packet, size 64.
type Data64 struct {
	Type uint8     `json:"type"` // Data type.
	Len  uint8     `json:"len"`  // Data length.
	Data [64]uint8 `json:"data"` // Raw data.
}

func (self *Data64) MsgID() MessageID {
	return MSG_ID_DATA64
}

func (self *Data64) MsgName() string {
	return "Data64"
}

func (self *Data64) Pack(p *Packet) error {
	payload := make([]byte, 66)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data64) Unpack(p *Packet) error {
	if len(p.Payload) < 66 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:66])
	return nil
}

func (self *Data64) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Data64FromJSON(data []byte) (*Data64, error) {
	p := &Data64{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Data packet, size 96.
type Data96 struct {
	Type uint8     `json:"type"` // Data type.
	Len  uint8     `json:"len"`  // Data length.
	Data [96]uint8 `json:"data"` // Raw data.
}

func (self *Data96) MsgID() MessageID {
	return MSG_ID_DATA96
}

func (self *Data96) MsgName() string {
	return "Data96"
}

func (self *Data96) Pack(p *Packet) error {
	payload := make([]byte, 98)
	payload[0] = byte(self.Type)
	payload[1] = byte(self.Len)
	copy(payload[2:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Data96) Unpack(p *Packet) error {
	if len(p.Payload) < 98 {
		return fmt.Errorf("payload too small")
	}
	self.Type = uint8(p.Payload[0])
	self.Len = uint8(p.Payload[1])
	copy(self.Data[:], p.Payload[2:98])
	return nil
}

func (self *Data96) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Data96FromJSON(data []byte) (*Data96, error) {
	p := &Data96{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Rangefinder reporting.
type Rangefinder struct {
	Distance float32 `json:"distance"` // Distance.
	Voltage  float32 `json:"voltage"`  // Raw voltage if available, zero otherwise.
}

func (self *Rangefinder) MsgID() MessageID {
	return MSG_ID_RANGEFINDER
}

func (self *Rangefinder) MsgName() string {
	return "Rangefinder"
}

func (self *Rangefinder) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Distance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Voltage))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Rangefinder) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Distance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Voltage = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

func (self *Rangefinder) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RangefinderFromJSON(data []byte) (*Rangefinder, error) {
	p := &Rangefinder{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Airspeed auto-calibration.
type AirspeedAutocal struct {
	Vx           float32 `json:"vx"`           // GPS velocity north.
	Vy           float32 `json:"vy"`           // GPS velocity east.
	Vz           float32 `json:"vz"`           // GPS velocity down.
	DiffPressure float32 `json:"diffPressure"` // Differential pressure.
	Eas2tas      float32 `json:"eas2tas"`      // Estimated to true airspeed ratio.
	Ratio        float32 `json:"ratio"`        // Airspeed ratio.
	StateX       float32 `json:"stateX"`       // EKF state x.
	StateY       float32 `json:"stateY"`       // EKF state y.
	StateZ       float32 `json:"stateZ"`       // EKF state z.
	Pax          float32 `json:"pax"`          // EKF Pax.
	Pby          float32 `json:"pby"`          // EKF Pby.
	Pcz          float32 `json:"pcz"`          // EKF Pcz.
}

func (self *AirspeedAutocal) MsgID() MessageID {
	return MSG_ID_AIRSPEED_AUTOCAL
}

func (self *AirspeedAutocal) MsgName() string {
	return "AirspeedAutocal"
}

func (self *AirspeedAutocal) Pack(p *Packet) error {
	payload := make([]byte, 48)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Vx))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Vy))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Vz))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.DiffPressure))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.Eas2tas))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.Ratio))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.StateX))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.StateY))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.StateZ))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.Pax))
	binary.LittleEndian.PutUint32(payload[40:], math.Float32bits(self.Pby))
	binary.LittleEndian.PutUint32(payload[44:], math.Float32bits(self.Pcz))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AirspeedAutocal) Unpack(p *Packet) error {
	if len(p.Payload) < 48 {
		return fmt.Errorf("payload too small")
	}
	self.Vx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Vy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Vz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.DiffPressure = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Eas2tas = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Ratio = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.StateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.StateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.StateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Pax = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.Pby = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[40:]))
	self.Pcz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[44:]))
	return nil
}

func (self *AirspeedAutocal) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AirspeedAutocalFromJSON(data []byte) (*AirspeedAutocal, error) {
	p := &AirspeedAutocal{}
	err := json.Unmarshal(data, p)
	return p, err
}

// A rally point. Used to set a point when from GCS -> MAV. Also used to return a point from MAV -> GCS.
type RallyPoint struct {
	Lat             int32  `json:"lat"`             // Latitude of point.
	Lng             int32  `json:"lng"`             // Longitude of point.
	Alt             int16  `json:"alt"`             // Transit / loiter altitude relative to home.
	BreakAlt        int16  `json:"breakAlt"`        // Break altitude relative to home.
	LandDir         uint16 `json:"landDir"`         // Heading to aim for when landing.
	TargetSystem    uint8  `json:"targetSystem"`    // System ID.
	TargetComponent uint8  `json:"targetComponent"` // Component ID.
	Idx             uint8  `json:"idx"`             // Point index (first point is 0).
	Count           uint8  `json:"count"`           // Total number of points (for sanity checking).
	Flags           uint8  `json:"flags"`           // Configuration flags.
}

func (self *RallyPoint) MsgID() MessageID {
	return MSG_ID_RALLY_POINT
}

func (self *RallyPoint) MsgName() string {
	return "RallyPoint"
}

func (self *RallyPoint) Pack(p *Packet) error {
	payload := make([]byte, 19)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.Lng))
	binary.LittleEndian.PutUint16(payload[8:], uint16(self.Alt))
	binary.LittleEndian.PutUint16(payload[10:], uint16(self.BreakAlt))
	binary.LittleEndian.PutUint16(payload[12:], uint16(self.LandDir))
	payload[14] = byte(self.TargetSystem)
	payload[15] = byte(self.TargetComponent)
	payload[16] = byte(self.Idx)
	payload[17] = byte(self.Count)
	payload[18] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RallyPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 19 {
		return fmt.Errorf("payload too small")
	}
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Alt = int16(binary.LittleEndian.Uint16(p.Payload[8:]))
	self.BreakAlt = int16(binary.LittleEndian.Uint16(p.Payload[10:]))
	self.LandDir = uint16(binary.LittleEndian.Uint16(p.Payload[12:]))
	self.TargetSystem = uint8(p.Payload[14])
	self.TargetComponent = uint8(p.Payload[15])
	self.Idx = uint8(p.Payload[16])
	self.Count = uint8(p.Payload[17])
	self.Flags = uint8(p.Payload[18])
	return nil
}

func (self *RallyPoint) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RallyPointFromJSON(data []byte) (*RallyPoint, error) {
	p := &RallyPoint{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a current rally point from MAV. MAV should respond with a RALLY_POINT message. MAV should not respond if the request is invalid.
type RallyFetchPoint struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
	Idx             uint8 `json:"idx"`             // Point index (first point is 0).
}

func (self *RallyFetchPoint) MsgID() MessageID {
	return MSG_ID_RALLY_FETCH_POINT
}

func (self *RallyFetchPoint) MsgName() string {
	return "RallyFetchPoint"
}

func (self *RallyFetchPoint) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Idx)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RallyFetchPoint) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Idx = uint8(p.Payload[2])
	return nil
}

func (self *RallyFetchPoint) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RallyFetchPointFromJSON(data []byte) (*RallyFetchPoint, error) {
	p := &RallyFetchPoint{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of compassmot calibration.
type CompassmotStatus struct {
	Current       float32 `json:"current"`       // Current.
	Compensationx float32 `json:"compensationx"` // Motor Compensation X.
	Compensationy float32 `json:"compensationy"` // Motor Compensation Y.
	Compensationz float32 `json:"compensationz"` // Motor Compensation Z.
	Throttle      uint16  `json:"throttle"`      // Throttle.
	Interference  uint16  `json:"interference"`  // Interference.
}

func (self *CompassmotStatus) MsgID() MessageID {
	return MSG_ID_COMPASSMOT_STATUS
}

func (self *CompassmotStatus) MsgName() string {
	return "CompassmotStatus"
}

func (self *CompassmotStatus) Pack(p *Packet) error {
	payload := make([]byte, 20)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Current))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Compensationx))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Compensationy))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Compensationz))
	binary.LittleEndian.PutUint16(payload[16:], uint16(self.Throttle))
	binary.LittleEndian.PutUint16(payload[18:], uint16(self.Interference))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CompassmotStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 20 {
		return fmt.Errorf("payload too small")
	}
	self.Current = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Compensationx = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Compensationy = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Compensationz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Throttle = uint16(binary.LittleEndian.Uint16(p.Payload[16:]))
	self.Interference = uint16(binary.LittleEndian.Uint16(p.Payload[18:]))
	return nil
}

func (self *CompassmotStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CompassmotStatusFromJSON(data []byte) (*CompassmotStatus, error) {
	p := &CompassmotStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of secondary AHRS filter if available.
type Ahrs2 struct {
	Roll     float32 `json:"roll"`     // Roll angle.
	Pitch    float32 `json:"pitch"`    // Pitch angle.
	Yaw      float32 `json:"yaw"`      // Yaw angle.
	Altitude float32 `json:"altitude"` // Altitude (MSL).
	Lat      int32   `json:"lat"`      // Latitude.
	Lng      int32   `json:"lng"`      // Longitude.
}

func (self *Ahrs2) MsgID() MessageID {
	return MSG_ID_AHRS2
}

func (self *Ahrs2) MsgName() string {
	return "Ahrs2"
}

func (self *Ahrs2) Pack(p *Packet) error {
	payload := make([]byte, 24)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Lng))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs2) Unpack(p *Packet) error {
	if len(p.Payload) < 24 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	return nil
}

func (self *Ahrs2) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Ahrs2FromJSON(data []byte) (*Ahrs2, error) {
	p := &Ahrs2{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Camera Event.
type CameraStatus struct {
	TimeUsec     uint64  `json:"timeUsec"`     // Image timestamp (since UNIX epoch, according to camera clock).
	P1           float32 `json:"p1"`           // Parameter 1 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	P2           float32 `json:"p2"`           // Parameter 2 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	P3           float32 `json:"p3"`           // Parameter 3 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	P4           float32 `json:"p4"`           // Parameter 4 (meaning depends on event_id, see CAMERA_STATUS_TYPES enum).
	ImgIdx       uint16  `json:"imgIdx"`       // Image index.
	TargetSystem uint8   `json:"targetSystem"` // System ID.
	CamIdx       uint8   `json:"camIdx"`       // Camera ID.
	EventId      uint8   `json:"eventId"`      // Event type.
}

func (self *CameraStatus) MsgID() MessageID {
	return MSG_ID_CAMERA_STATUS
}

func (self *CameraStatus) MsgName() string {
	return "CameraStatus"
}

func (self *CameraStatus) Pack(p *Packet) error {
	payload := make([]byte, 29)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.P1))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P2))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.P3))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.P4))
	binary.LittleEndian.PutUint16(payload[24:], uint16(self.ImgIdx))
	payload[26] = byte(self.TargetSystem)
	payload[27] = byte(self.CamIdx)
	payload[28] = byte(self.EventId)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.P1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.P3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.P4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[24:]))
	self.TargetSystem = uint8(p.Payload[26])
	self.CamIdx = uint8(p.Payload[27])
	self.EventId = uint8(p.Payload[28])
	return nil
}

func (self *CameraStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CameraStatusFromJSON(data []byte) (*CameraStatus, error) {
	p := &CameraStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Camera Capture Feedback.
type CameraFeedback struct {
	TimeUsec     uint64  `json:"timeUsec"`     // Image timestamp (since UNIX epoch), as passed in by CAMERA_STATUS message (or autopilot if no CCB).
	Lat          int32   `json:"lat"`          // Latitude.
	Lng          int32   `json:"lng"`          // Longitude.
	AltMsl       float32 `json:"altMsl"`       // Altitude (MSL).
	AltRel       float32 `json:"altRel"`       // Altitude (Relative to HOME location).
	Roll         float32 `json:"roll"`         // Camera Roll angle (earth frame, +-180).
	Pitch        float32 `json:"pitch"`        // Camera Pitch angle (earth frame, +-180).
	Yaw          float32 `json:"yaw"`          // Camera Yaw (earth frame, 0-360, true).
	FocLen       float32 `json:"focLen"`       // Focal Length.
	ImgIdx       uint16  `json:"imgIdx"`       // Image index.
	TargetSystem uint8   `json:"targetSystem"` // System ID.
	CamIdx       uint8   `json:"camIdx"`       // Camera ID.
	Flags        uint8   `json:"flags"`        // Feedback flags.
}

func (self *CameraFeedback) MsgID() MessageID {
	return MSG_ID_CAMERA_FEEDBACK
}

func (self *CameraFeedback) MsgName() string {
	return "CameraFeedback"
}

func (self *CameraFeedback) Pack(p *Packet) error {
	payload := make([]byte, 45)
	binary.LittleEndian.PutUint64(payload[0:], uint64(self.TimeUsec))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.Lng))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.AltMsl))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.AltRel))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.FocLen))
	binary.LittleEndian.PutUint16(payload[40:], uint16(self.ImgIdx))
	payload[42] = byte(self.TargetSystem)
	payload[43] = byte(self.CamIdx)
	payload[44] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *CameraFeedback) Unpack(p *Packet) error {
	if len(p.Payload) < 45 {
		return fmt.Errorf("payload too small")
	}
	self.TimeUsec = uint64(binary.LittleEndian.Uint64(p.Payload[0:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.AltMsl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.AltRel = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.FocLen = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.ImgIdx = uint16(binary.LittleEndian.Uint16(p.Payload[40:]))
	self.TargetSystem = uint8(p.Payload[42])
	self.CamIdx = uint8(p.Payload[43])
	self.Flags = uint8(p.Payload[44])
	return nil
}

func (self *CameraFeedback) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func CameraFeedbackFromJSON(data []byte) (*CameraFeedback, error) {
	p := &CameraFeedback{}
	err := json.Unmarshal(data, p)
	return p, err
}

// 2nd Battery status
type Battery2 struct {
	Voltage        uint16 `json:"voltage"`        // Voltage.
	CurrentBattery int16  `json:"currentBattery"` // Battery current, -1: autopilot does not measure the current.
}

func (self *Battery2) MsgID() MessageID {
	return MSG_ID_BATTERY2
}

func (self *Battery2) MsgName() string {
	return "Battery2"
}

func (self *Battery2) Pack(p *Packet) error {
	payload := make([]byte, 4)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.Voltage))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.CurrentBattery))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Battery2) Unpack(p *Packet) error {
	if len(p.Payload) < 4 {
		return fmt.Errorf("payload too small")
	}
	self.Voltage = uint16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.CurrentBattery = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	return nil
}

func (self *Battery2) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Battery2FromJSON(data []byte) (*Battery2, error) {
	p := &Battery2{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Status of third AHRS filter if available. This is for ANU research group (Ali and Sean).
type Ahrs3 struct {
	Roll     float32 `json:"roll"`     // Roll angle.
	Pitch    float32 `json:"pitch"`    // Pitch angle.
	Yaw      float32 `json:"yaw"`      // Yaw angle.
	Altitude float32 `json:"altitude"` // Altitude (MSL).
	Lat      int32   `json:"lat"`      // Latitude.
	Lng      int32   `json:"lng"`      // Longitude.
	V1       float32 `json:"v1"`       // Test variable1.
	V2       float32 `json:"v2"`       // Test variable2.
	V3       float32 `json:"v3"`       // Test variable3.
	V4       float32 `json:"v4"`       // Test variable4.
}

func (self *Ahrs3) MsgID() MessageID {
	return MSG_ID_AHRS3
}

func (self *Ahrs3) MsgName() string {
	return "Ahrs3"
}

func (self *Ahrs3) Pack(p *Packet) error {
	payload := make([]byte, 40)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Roll))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Pitch))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Yaw))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.Altitude))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.Lat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.Lng))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.V1))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.V2))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.V3))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.V4))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Ahrs3) Unpack(p *Packet) error {
	if len(p.Payload) < 40 {
		return fmt.Errorf("payload too small")
	}
	self.Roll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Pitch = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Yaw = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.Lat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Lng = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.V1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.V2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.V3 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.V4 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	return nil
}

func (self *Ahrs3) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func Ahrs3FromJSON(data []byte) (*Ahrs3, error) {
	p := &Ahrs3{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request the autopilot version from the system/component.
type AutopilotVersionRequest struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
}

func (self *AutopilotVersionRequest) MsgID() MessageID {
	return MSG_ID_AUTOPILOT_VERSION_REQUEST
}

func (self *AutopilotVersionRequest) MsgName() string {
	return "AutopilotVersionRequest"
}

func (self *AutopilotVersionRequest) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *AutopilotVersionRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	return nil
}

func (self *AutopilotVersionRequest) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func AutopilotVersionRequestFromJSON(data []byte) (*AutopilotVersionRequest, error) {
	p := &AutopilotVersionRequest{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send a block of log data to remote location.
type RemoteLogDataBlock struct {
	Seqno           uint32     `json:"seqno"`           // Log data block sequence number.
	TargetSystem    uint8      `json:"targetSystem"`    // System ID.
	TargetComponent uint8      `json:"targetComponent"` // Component ID.
	Data            [200]uint8 `json:"data"`            // Log data block.
}

func (self *RemoteLogDataBlock) MsgID() MessageID {
	return MSG_ID_REMOTE_LOG_DATA_BLOCK
}

func (self *RemoteLogDataBlock) MsgName() string {
	return "RemoteLogDataBlock"
}

func (self *RemoteLogDataBlock) Pack(p *Packet) error {
	payload := make([]byte, 206)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Seqno))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)
	copy(payload[6:], self.Data[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RemoteLogDataBlock) Unpack(p *Packet) error {
	if len(p.Payload) < 206 {
		return fmt.Errorf("payload too small")
	}
	self.Seqno = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	copy(self.Data[:], p.Payload[6:206])
	return nil
}

func (self *RemoteLogDataBlock) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RemoteLogDataBlockFromJSON(data []byte) (*RemoteLogDataBlock, error) {
	p := &RemoteLogDataBlock{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Send Status of each log block that autopilot board might have sent.
type RemoteLogBlockStatus struct {
	Seqno           uint32 `json:"seqno"`           // Log data block sequence number.
	TargetSystem    uint8  `json:"targetSystem"`    // System ID.
	TargetComponent uint8  `json:"targetComponent"` // Component ID.
	Status          uint8  `json:"status"`          // Log data block status.
}

func (self *RemoteLogBlockStatus) MsgID() MessageID {
	return MSG_ID_REMOTE_LOG_BLOCK_STATUS
}

func (self *RemoteLogBlockStatus) MsgName() string {
	return "RemoteLogBlockStatus"
}

func (self *RemoteLogBlockStatus) Pack(p *Packet) error {
	payload := make([]byte, 7)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.Seqno))
	payload[4] = byte(self.TargetSystem)
	payload[5] = byte(self.TargetComponent)
	payload[6] = byte(self.Status)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *RemoteLogBlockStatus) Unpack(p *Packet) error {
	if len(p.Payload) < 7 {
		return fmt.Errorf("payload too small")
	}
	self.Seqno = uint32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.TargetSystem = uint8(p.Payload[4])
	self.TargetComponent = uint8(p.Payload[5])
	self.Status = uint8(p.Payload[6])
	return nil
}

func (self *RemoteLogBlockStatus) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RemoteLogBlockStatusFromJSON(data []byte) (*RemoteLogBlockStatus, error) {
	p := &RemoteLogBlockStatus{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Control vehicle LEDs.
type LedControl struct {
	TargetSystem    uint8     `json:"targetSystem"`    // System ID.
	TargetComponent uint8     `json:"targetComponent"` // Component ID.
	Instance        uint8     `json:"instance"`        // Instance (LED instance to control or 255 for all LEDs).
	Pattern         uint8     `json:"pattern"`         // Pattern (see LED_PATTERN_ENUM).
	CustomLen       uint8     `json:"customLen"`       // Custom Byte Length.
	CustomBytes     [24]uint8 `json:"customBytes"`     // Custom Bytes.
}

func (self *LedControl) MsgID() MessageID {
	return MSG_ID_LED_CONTROL
}

func (self *LedControl) MsgName() string {
	return "LedControl"
}

func (self *LedControl) Pack(p *Packet) error {
	payload := make([]byte, 29)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.Instance)
	payload[3] = byte(self.Pattern)
	payload[4] = byte(self.CustomLen)
	copy(payload[5:], self.CustomBytes[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *LedControl) Unpack(p *Packet) error {
	if len(p.Payload) < 29 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.Instance = uint8(p.Payload[2])
	self.Pattern = uint8(p.Payload[3])
	self.CustomLen = uint8(p.Payload[4])
	copy(self.CustomBytes[:], p.Payload[5:29])
	return nil
}

func (self *LedControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func LedControlFromJSON(data []byte) (*LedControl, error) {
	p := &LedControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reports progress of compass calibration.
type MagCalProgress struct {
	DirectionX     float32   `json:"directionX"`     // Body frame direction vector for display.
	DirectionY     float32   `json:"directionY"`     // Body frame direction vector for display.
	DirectionZ     float32   `json:"directionZ"`     // Body frame direction vector for display.
	CompassId      uint8     `json:"compassId"`      // Compass being calibrated.
	CalMask        uint8     `json:"calMask"`        // Bitmask of compasses being calibrated.
	CalStatus      uint8     `json:"calStatus"`      // Calibration Status.
	Attempt        uint8     `json:"attempt"`        // Attempt number.
	CompletionPct  uint8     `json:"completionPct"`  // Completion percentage.
	CompletionMask [10]uint8 `json:"completionMask"` // Bitmask of sphere sections (see http://en.wikipedia.org/wiki/Geodesic_grid).
}

func (self *MagCalProgress) MsgID() MessageID {
	return MSG_ID_MAG_CAL_PROGRESS
}

func (self *MagCalProgress) MsgName() string {
	return "MagCalProgress"
}

func (self *MagCalProgress) Pack(p *Packet) error {
	payload := make([]byte, 27)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.DirectionX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.DirectionY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.DirectionZ))
	payload[12] = byte(self.CompassId)
	payload[13] = byte(self.CalMask)
	payload[14] = byte(self.CalStatus)
	payload[15] = byte(self.Attempt)
	payload[16] = byte(self.CompletionPct)
	copy(payload[17:], self.CompletionMask[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MagCalProgress) Unpack(p *Packet) error {
	if len(p.Payload) < 27 {
		return fmt.Errorf("payload too small")
	}
	self.DirectionX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.DirectionY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.DirectionZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.CompassId = uint8(p.Payload[12])
	self.CalMask = uint8(p.Payload[13])
	self.CalStatus = uint8(p.Payload[14])
	self.Attempt = uint8(p.Payload[15])
	self.CompletionPct = uint8(p.Payload[16])
	copy(self.CompletionMask[:], p.Payload[17:27])
	return nil
}

func (self *MagCalProgress) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MagCalProgressFromJSON(data []byte) (*MagCalProgress, error) {
	p := &MagCalProgress{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
type MagCalReport struct {
	Fitness   float32 `json:"fitness"`   // RMS milligauss residuals.
	OfsX      float32 `json:"ofsX"`      // X offset.
	OfsY      float32 `json:"ofsY"`      // Y offset.
	OfsZ      float32 `json:"ofsZ"`      // Z offset.
	DiagX     float32 `json:"diagX"`     // X diagonal (matrix 11).
	DiagY     float32 `json:"diagY"`     // Y diagonal (matrix 22).
	DiagZ     float32 `json:"diagZ"`     // Z diagonal (matrix 33).
	OffdiagX  float32 `json:"offdiagX"`  // X off-diagonal (matrix 12 and 21).
	OffdiagY  float32 `json:"offdiagY"`  // Y off-diagonal (matrix 13 and 31).
	OffdiagZ  float32 `json:"offdiagZ"`  // Z off-diagonal (matrix 32 and 23).
	CompassId uint8   `json:"compassId"` // Compass being calibrated.
	CalMask   uint8   `json:"calMask"`   // Bitmask of compasses being calibrated.
	CalStatus uint8   `json:"calStatus"` // Calibration Status.
	Autosaved uint8   `json:"autosaved"` // 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
}

func (self *MagCalReport) MsgID() MessageID {
	return MSG_ID_MAG_CAL_REPORT
}

func (self *MagCalReport) MsgName() string {
	return "MagCalReport"
}

func (self *MagCalReport) Pack(p *Packet) error {
	payload := make([]byte, 44)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Fitness))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.OfsX))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.OfsY))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.OfsZ))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.DiagX))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.DiagY))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.DiagZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.OffdiagX))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.OffdiagY))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.OffdiagZ))
	payload[40] = byte(self.CompassId)
	payload[41] = byte(self.CalMask)
	payload[42] = byte(self.CalStatus)
	payload[43] = byte(self.Autosaved)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *MagCalReport) Unpack(p *Packet) error {
	if len(p.Payload) < 44 {
		return fmt.Errorf("payload too small")
	}
	self.Fitness = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.OfsX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.OfsY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.OfsZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.DiagX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.DiagY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.DiagZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.OffdiagX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.OffdiagY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.OffdiagZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.CompassId = uint8(p.Payload[40])
	self.CalMask = uint8(p.Payload[41])
	self.CalStatus = uint8(p.Payload[42])
	self.Autosaved = uint8(p.Payload[43])
	return nil
}

func (self *MagCalReport) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func MagCalReportFromJSON(data []byte) (*MagCalReport, error) {
	p := &MagCalReport{}
	err := json.Unmarshal(data, p)
	return p, err
}

// EKF Status message including flags and variances.
type EkfStatusReport struct {
	VelocityVariance   float32 `json:"velocityVariance"`   // Velocity variance.
	PosHorizVariance   float32 `json:"posHorizVariance"`   // Horizontal Position variance.
	PosVertVariance    float32 `json:"posVertVariance"`    // Vertical Position variance.
	CompassVariance    float32 `json:"compassVariance"`    // Compass variance.
	TerrainAltVariance float32 `json:"terrainAltVariance"` // Terrain Altitude variance.
	Flags              uint16  `json:"flags"`              // Flags.
}

func (self *EkfStatusReport) MsgID() MessageID {
	return MSG_ID_EKF_STATUS_REPORT
}

func (self *EkfStatusReport) MsgName() string {
	return "EkfStatusReport"
}

func (self *EkfStatusReport) Pack(p *Packet) error {
	payload := make([]byte, 22)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.VelocityVariance))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.PosHorizVariance))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.PosVertVariance))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.CompassVariance))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.TerrainAltVariance))
	binary.LittleEndian.PutUint16(payload[20:], uint16(self.Flags))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *EkfStatusReport) Unpack(p *Packet) error {
	if len(p.Payload) < 22 {
		return fmt.Errorf("payload too small")
	}
	self.VelocityVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.PosHorizVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PosVertVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.CompassVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.TerrainAltVariance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.Flags = uint16(binary.LittleEndian.Uint16(p.Payload[20:]))
	return nil
}

func (self *EkfStatusReport) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func EkfStatusReportFromJSON(data []byte) (*EkfStatusReport, error) {
	p := &EkfStatusReport{}
	err := json.Unmarshal(data, p)
	return p, err
}

// PID tuning information.
type PidTuning struct {
	Desired  float32 `json:"desired"`  // Desired rate.
	Achieved float32 `json:"achieved"` // Achieved rate.
	Ff       float32 `json:"ff"`       // FF component.
	P        float32 `json:"p"`        // P component.
	I        float32 `json:"i"`        // I component.
	D        float32 `json:"d"`        // D component.
	Axis     uint8   `json:"axis"`     // Axis.
}

func (self *PidTuning) MsgID() MessageID {
	return MSG_ID_PID_TUNING
}

func (self *PidTuning) MsgName() string {
	return "PidTuning"
}

func (self *PidTuning) Pack(p *Packet) error {
	payload := make([]byte, 25)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Desired))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Achieved))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.Ff))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.P))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.I))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.D))
	payload[24] = byte(self.Axis)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *PidTuning) Unpack(p *Packet) error {
	if len(p.Payload) < 25 {
		return fmt.Errorf("payload too small")
	}
	self.Desired = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Achieved = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.Ff = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.P = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.I = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.D = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Axis = uint8(p.Payload[24])
	return nil
}

func (self *PidTuning) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func PidTuningFromJSON(data []byte) (*PidTuning, error) {
	p := &PidTuning{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Deepstall path planning.
type Deepstall struct {
	LandingLat             int32   `json:"landingLat"`             // Landing latitude.
	LandingLon             int32   `json:"landingLon"`             // Landing longitude.
	PathLat                int32   `json:"pathLat"`                // Final heading start point, latitude.
	PathLon                int32   `json:"pathLon"`                // Final heading start point, longitude.
	ArcEntryLat            int32   `json:"arcEntryLat"`            // Arc entry point, latitude.
	ArcEntryLon            int32   `json:"arcEntryLon"`            // Arc entry point, longitude.
	Altitude               float32 `json:"altitude"`               // Altitude.
	ExpectedTravelDistance float32 `json:"expectedTravelDistance"` // Distance the aircraft expects to travel during the deepstall.
	CrossTrackError        float32 `json:"crossTrackError"`        // Deepstall cross track error (only valid when in DEEPSTALL_STAGE_LAND).
	Stage                  uint8   `json:"stage"`                  // Deepstall stage.
}

func (self *Deepstall) MsgID() MessageID {
	return MSG_ID_DEEPSTALL
}

func (self *Deepstall) MsgName() string {
	return "Deepstall"
}

func (self *Deepstall) Pack(p *Packet) error {
	payload := make([]byte, 37)
	binary.LittleEndian.PutUint32(payload[0:], uint32(self.LandingLat))
	binary.LittleEndian.PutUint32(payload[4:], uint32(self.LandingLon))
	binary.LittleEndian.PutUint32(payload[8:], uint32(self.PathLat))
	binary.LittleEndian.PutUint32(payload[12:], uint32(self.PathLon))
	binary.LittleEndian.PutUint32(payload[16:], uint32(self.ArcEntryLat))
	binary.LittleEndian.PutUint32(payload[20:], uint32(self.ArcEntryLon))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.Altitude))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.ExpectedTravelDistance))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.CrossTrackError))
	payload[36] = byte(self.Stage)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Deepstall) Unpack(p *Packet) error {
	if len(p.Payload) < 37 {
		return fmt.Errorf("payload too small")
	}
	self.LandingLat = int32(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.LandingLon = int32(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.PathLat = int32(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.PathLon = int32(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.ArcEntryLat = int32(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.ArcEntryLon = int32(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.Altitude = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.ExpectedTravelDistance = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.CrossTrackError = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.Stage = uint8(p.Payload[36])
	return nil
}

func (self *Deepstall) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func DeepstallFromJSON(data []byte) (*Deepstall, error) {
	p := &Deepstall{}
	err := json.Unmarshal(data, p)
	return p, err
}

// 3 axis gimbal measurements.
type GimbalReport struct {
	DeltaTime       float32 `json:"deltaTime"`       // Time since last update.
	DeltaAngleX     float32 `json:"deltaAngleX"`     // Delta angle X.
	DeltaAngleY     float32 `json:"deltaAngleY"`     // Delta angle Y.
	DeltaAngleZ     float32 `json:"deltaAngleZ"`     // Delta angle X.
	DeltaVelocityX  float32 `json:"deltaVelocityX"`  // Delta velocity X.
	DeltaVelocityY  float32 `json:"deltaVelocityY"`  // Delta velocity Y.
	DeltaVelocityZ  float32 `json:"deltaVelocityZ"`  // Delta velocity Z.
	JointRoll       float32 `json:"jointRoll"`       // Joint ROLL.
	JointEl         float32 `json:"jointEl"`         // Joint EL.
	JointAz         float32 `json:"jointAz"`         // Joint AZ.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID.
	TargetComponent uint8   `json:"targetComponent"` // Component ID.
}

func (self *GimbalReport) MsgID() MessageID {
	return MSG_ID_GIMBAL_REPORT
}

func (self *GimbalReport) MsgName() string {
	return "GimbalReport"
}

func (self *GimbalReport) Pack(p *Packet) error {
	payload := make([]byte, 42)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.DeltaTime))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.DeltaAngleX))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.DeltaAngleY))
	binary.LittleEndian.PutUint32(payload[12:], math.Float32bits(self.DeltaAngleZ))
	binary.LittleEndian.PutUint32(payload[16:], math.Float32bits(self.DeltaVelocityX))
	binary.LittleEndian.PutUint32(payload[20:], math.Float32bits(self.DeltaVelocityY))
	binary.LittleEndian.PutUint32(payload[24:], math.Float32bits(self.DeltaVelocityZ))
	binary.LittleEndian.PutUint32(payload[28:], math.Float32bits(self.JointRoll))
	binary.LittleEndian.PutUint32(payload[32:], math.Float32bits(self.JointEl))
	binary.LittleEndian.PutUint32(payload[36:], math.Float32bits(self.JointAz))
	payload[40] = byte(self.TargetSystem)
	payload[41] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalReport) Unpack(p *Packet) error {
	if len(p.Payload) < 42 {
		return fmt.Errorf("payload too small")
	}
	self.DeltaTime = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.DeltaAngleX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.DeltaAngleY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.DeltaAngleZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[12:]))
	self.DeltaVelocityX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[16:]))
	self.DeltaVelocityY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[20:]))
	self.DeltaVelocityZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[24:]))
	self.JointRoll = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[28:]))
	self.JointEl = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[32:]))
	self.JointAz = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[36:]))
	self.TargetSystem = uint8(p.Payload[40])
	self.TargetComponent = uint8(p.Payload[41])
	return nil
}

func (self *GimbalReport) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GimbalReportFromJSON(data []byte) (*GimbalReport, error) {
	p := &GimbalReport{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Control message for rate gimbal.
type GimbalControl struct {
	DemandedRateX   float32 `json:"demandedRateX"`   // Demanded angular rate X.
	DemandedRateY   float32 `json:"demandedRateY"`   // Demanded angular rate Y.
	DemandedRateZ   float32 `json:"demandedRateZ"`   // Demanded angular rate Z.
	TargetSystem    uint8   `json:"targetSystem"`    // System ID.
	TargetComponent uint8   `json:"targetComponent"` // Component ID.
}

func (self *GimbalControl) MsgID() MessageID {
	return MSG_ID_GIMBAL_CONTROL
}

func (self *GimbalControl) MsgName() string {
	return "GimbalControl"
}

func (self *GimbalControl) Pack(p *Packet) error {
	payload := make([]byte, 14)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.DemandedRateX))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.DemandedRateY))
	binary.LittleEndian.PutUint32(payload[8:], math.Float32bits(self.DemandedRateZ))
	payload[12] = byte(self.TargetSystem)
	payload[13] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalControl) Unpack(p *Packet) error {
	if len(p.Payload) < 14 {
		return fmt.Errorf("payload too small")
	}
	self.DemandedRateX = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.DemandedRateY = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	self.DemandedRateZ = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[8:]))
	self.TargetSystem = uint8(p.Payload[12])
	self.TargetComponent = uint8(p.Payload[13])
	return nil
}

func (self *GimbalControl) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GimbalControlFromJSON(data []byte) (*GimbalControl, error) {
	p := &GimbalControl{}
	err := json.Unmarshal(data, p)
	return p, err
}

// 100 Hz gimbal torque command telemetry.
type GimbalTorqueCmdReport struct {
	RlTorqueCmd     int16 `json:"rlTorqueCmd"`     // Roll Torque Command.
	ElTorqueCmd     int16 `json:"elTorqueCmd"`     // Elevation Torque Command.
	AzTorqueCmd     int16 `json:"azTorqueCmd"`     // Azimuth Torque Command.
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
}

func (self *GimbalTorqueCmdReport) MsgID() MessageID {
	return MSG_ID_GIMBAL_TORQUE_CMD_REPORT
}

func (self *GimbalTorqueCmdReport) MsgName() string {
	return "GimbalTorqueCmdReport"
}

func (self *GimbalTorqueCmdReport) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint16(payload[0:], uint16(self.RlTorqueCmd))
	binary.LittleEndian.PutUint16(payload[2:], uint16(self.ElTorqueCmd))
	binary.LittleEndian.PutUint16(payload[4:], uint16(self.AzTorqueCmd))
	payload[6] = byte(self.TargetSystem)
	payload[7] = byte(self.TargetComponent)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GimbalTorqueCmdReport) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.RlTorqueCmd = int16(binary.LittleEndian.Uint16(p.Payload[0:]))
	self.ElTorqueCmd = int16(binary.LittleEndian.Uint16(p.Payload[2:]))
	self.AzTorqueCmd = int16(binary.LittleEndian.Uint16(p.Payload[4:]))
	self.TargetSystem = uint8(p.Payload[6])
	self.TargetComponent = uint8(p.Payload[7])
	return nil
}

func (self *GimbalTorqueCmdReport) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GimbalTorqueCmdReportFromJSON(data []byte) (*GimbalTorqueCmdReport, error) {
	p := &GimbalTorqueCmdReport{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Heartbeat from a HeroBus attached GoPro.
type GoproHeartbeat struct {
	Status      uint8 `json:"status"`      // Status.
	CaptureMode uint8 `json:"captureMode"` // Current capture mode.
	Flags       uint8 `json:"flags"`       // Additional status bits.
}

func (self *GoproHeartbeat) MsgID() MessageID {
	return MSG_ID_GOPRO_HEARTBEAT
}

func (self *GoproHeartbeat) MsgName() string {
	return "GoproHeartbeat"
}

func (self *GoproHeartbeat) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.Status)
	payload[1] = byte(self.CaptureMode)
	payload[2] = byte(self.Flags)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproHeartbeat) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.Status = uint8(p.Payload[0])
	self.CaptureMode = uint8(p.Payload[1])
	self.Flags = uint8(p.Payload[2])
	return nil
}

func (self *GoproHeartbeat) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GoproHeartbeatFromJSON(data []byte) (*GoproHeartbeat, error) {
	p := &GoproHeartbeat{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request a GOPRO_COMMAND response from the GoPro.
type GoproGetRequest struct {
	TargetSystem    uint8 `json:"targetSystem"`    // System ID.
	TargetComponent uint8 `json:"targetComponent"` // Component ID.
	CmdId           uint8 `json:"cmdId"`           // Command ID.
}

func (self *GoproGetRequest) MsgID() MessageID {
	return MSG_ID_GOPRO_GET_REQUEST
}

func (self *GoproGetRequest) MsgName() string {
	return "GoproGetRequest"
}

func (self *GoproGetRequest) Pack(p *Packet) error {
	payload := make([]byte, 3)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.CmdId)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproGetRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 3 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.CmdId = uint8(p.Payload[2])
	return nil
}

func (self *GoproGetRequest) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GoproGetRequestFromJSON(data []byte) (*GoproGetRequest, error) {
	p := &GoproGetRequest{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Response from a GOPRO_COMMAND get request.
type GoproGetResponse struct {
	CmdId  uint8    `json:"cmdId"`  // Command ID.
	Status uint8    `json:"status"` // Status.
	Value  [4]uint8 `json:"value"`  // Value.
}

func (self *GoproGetResponse) MsgID() MessageID {
	return MSG_ID_GOPRO_GET_RESPONSE
}

func (self *GoproGetResponse) MsgName() string {
	return "GoproGetResponse"
}

func (self *GoproGetResponse) Pack(p *Packet) error {
	payload := make([]byte, 6)
	payload[0] = byte(self.CmdId)
	payload[1] = byte(self.Status)
	copy(payload[2:], self.Value[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproGetResponse) Unpack(p *Packet) error {
	if len(p.Payload) < 6 {
		return fmt.Errorf("payload too small")
	}
	self.CmdId = uint8(p.Payload[0])
	self.Status = uint8(p.Payload[1])
	copy(self.Value[:], p.Payload[2:6])
	return nil
}

func (self *GoproGetResponse) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GoproGetResponseFromJSON(data []byte) (*GoproGetResponse, error) {
	p := &GoproGetResponse{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Request to set a GOPRO_COMMAND with a desired.
type GoproSetRequest struct {
	TargetSystem    uint8    `json:"targetSystem"`    // System ID.
	TargetComponent uint8    `json:"targetComponent"` // Component ID.
	CmdId           uint8    `json:"cmdId"`           // Command ID.
	Value           [4]uint8 `json:"value"`           // Value.
}

func (self *GoproSetRequest) MsgID() MessageID {
	return MSG_ID_GOPRO_SET_REQUEST
}

func (self *GoproSetRequest) MsgName() string {
	return "GoproSetRequest"
}

func (self *GoproSetRequest) Pack(p *Packet) error {
	payload := make([]byte, 7)
	payload[0] = byte(self.TargetSystem)
	payload[1] = byte(self.TargetComponent)
	payload[2] = byte(self.CmdId)
	copy(payload[3:], self.Value[:])

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproSetRequest) Unpack(p *Packet) error {
	if len(p.Payload) < 7 {
		return fmt.Errorf("payload too small")
	}
	self.TargetSystem = uint8(p.Payload[0])
	self.TargetComponent = uint8(p.Payload[1])
	self.CmdId = uint8(p.Payload[2])
	copy(self.Value[:], p.Payload[3:7])
	return nil
}

func (self *GoproSetRequest) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GoproSetRequestFromJSON(data []byte) (*GoproSetRequest, error) {
	p := &GoproSetRequest{}
	err := json.Unmarshal(data, p)
	return p, err
}

// Response from a GOPRO_COMMAND set request.
type GoproSetResponse struct {
	CmdId  uint8 `json:"cmdId"`  // Command ID.
	Status uint8 `json:"status"` // Status.
}

func (self *GoproSetResponse) MsgID() MessageID {
	return MSG_ID_GOPRO_SET_RESPONSE
}

func (self *GoproSetResponse) MsgName() string {
	return "GoproSetResponse"
}

func (self *GoproSetResponse) Pack(p *Packet) error {
	payload := make([]byte, 2)
	payload[0] = byte(self.CmdId)
	payload[1] = byte(self.Status)

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *GoproSetResponse) Unpack(p *Packet) error {
	if len(p.Payload) < 2 {
		return fmt.Errorf("payload too small")
	}
	self.CmdId = uint8(p.Payload[0])
	self.Status = uint8(p.Payload[1])
	return nil
}

func (self *GoproSetResponse) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func GoproSetResponseFromJSON(data []byte) (*GoproSetResponse, error) {
	p := &GoproSetResponse{}
	err := json.Unmarshal(data, p)
	return p, err
}

// RPM sensor output.
type Rpm struct {
	Rpm1 float32 `json:"rpm1"` // RPM Sensor1.
	Rpm2 float32 `json:"rpm2"` // RPM Sensor2.
}

func (self *Rpm) MsgID() MessageID {
	return MSG_ID_RPM
}

func (self *Rpm) MsgName() string {
	return "Rpm"
}

func (self *Rpm) Pack(p *Packet) error {
	payload := make([]byte, 8)
	binary.LittleEndian.PutUint32(payload[0:], math.Float32bits(self.Rpm1))
	binary.LittleEndian.PutUint32(payload[4:], math.Float32bits(self.Rpm2))

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *Rpm) Unpack(p *Packet) error {
	if len(p.Payload) < 8 {
		return fmt.Errorf("payload too small")
	}
	self.Rpm1 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[0:]))
	self.Rpm2 = math.Float32frombits(binary.LittleEndian.Uint32(p.Payload[4:]))
	return nil
}

func (self *Rpm) ToJSON() ([]byte, error) {
	return json.Marshal(self)
}

func RpmFromJSON(data []byte) (*Rpm, error) {
	p := &Rpm{}
	err := json.Unmarshal(data, p)
	return p, err
}

func init() {
	Messages["sensoroffsets"] = &SensorOffsets{}
	MessageIDs[150] = &SensorOffsets{}
	Messages["setmagoffsets"] = &SetMagOffsets{}
	MessageIDs[151] = &SetMagOffsets{}
	Messages["meminfo"] = &Meminfo{}
	MessageIDs[152] = &Meminfo{}
	Messages["apadc"] = &ApAdc{}
	MessageIDs[153] = &ApAdc{}
	Messages["digicamconfigure"] = &DigicamConfigure{}
	MessageIDs[154] = &DigicamConfigure{}
	Messages["digicamcontrol"] = &DigicamControl{}
	MessageIDs[155] = &DigicamControl{}
	Messages["mountconfigure"] = &MountConfigure{}
	MessageIDs[156] = &MountConfigure{}
	Messages["mountcontrol"] = &MountControl{}
	MessageIDs[157] = &MountControl{}
	Messages["mountstatus"] = &MountStatus{}
	MessageIDs[158] = &MountStatus{}
	Messages["fencepoint"] = &FencePoint{}
	MessageIDs[160] = &FencePoint{}
	Messages["fencefetchpoint"] = &FenceFetchPoint{}
	MessageIDs[161] = &FenceFetchPoint{}
	Messages["fencestatus"] = &FenceStatus{}
	MessageIDs[162] = &FenceStatus{}
	Messages["ahrs"] = &Ahrs{}
	MessageIDs[163] = &Ahrs{}
	Messages["simstate"] = &Simstate{}
	MessageIDs[164] = &Simstate{}
	Messages["hwstatus"] = &Hwstatus{}
	MessageIDs[165] = &Hwstatus{}
	Messages["radio"] = &Radio{}
	MessageIDs[166] = &Radio{}
	Messages["limitsstatus"] = &LimitsStatus{}
	MessageIDs[167] = &LimitsStatus{}
	Messages["wind"] = &Wind{}
	MessageIDs[168] = &Wind{}
	Messages["data16"] = &Data16{}
	MessageIDs[169] = &Data16{}
	Messages["data32"] = &Data32{}
	MessageIDs[170] = &Data32{}
	Messages["data64"] = &Data64{}
	MessageIDs[171] = &Data64{}
	Messages["data96"] = &Data96{}
	MessageIDs[172] = &Data96{}
	Messages["rangefinder"] = &Rangefinder{}
	MessageIDs[173] = &Rangefinder{}
	Messages["airspeedautocal"] = &AirspeedAutocal{}
	MessageIDs[174] = &AirspeedAutocal{}
	Messages["rallypoint"] = &RallyPoint{}
	MessageIDs[175] = &RallyPoint{}
	Messages["rallyfetchpoint"] = &RallyFetchPoint{}
	MessageIDs[176] = &RallyFetchPoint{}
	Messages["compassmotstatus"] = &CompassmotStatus{}
	MessageIDs[177] = &CompassmotStatus{}
	Messages["ahrs2"] = &Ahrs2{}
	MessageIDs[178] = &Ahrs2{}
	Messages["camerastatus"] = &CameraStatus{}
	MessageIDs[179] = &CameraStatus{}
	Messages["camerafeedback"] = &CameraFeedback{}
	MessageIDs[180] = &CameraFeedback{}
	Messages["battery2"] = &Battery2{}
	MessageIDs[181] = &Battery2{}
	Messages["ahrs3"] = &Ahrs3{}
	MessageIDs[182] = &Ahrs3{}
	Messages["autopilotversionrequest"] = &AutopilotVersionRequest{}
	MessageIDs[183] = &AutopilotVersionRequest{}
	Messages["remotelogdatablock"] = &RemoteLogDataBlock{}
	MessageIDs[184] = &RemoteLogDataBlock{}
	Messages["remotelogblockstatus"] = &RemoteLogBlockStatus{}
	MessageIDs[185] = &RemoteLogBlockStatus{}
	Messages["ledcontrol"] = &LedControl{}
	MessageIDs[186] = &LedControl{}
	Messages["magcalprogress"] = &MagCalProgress{}
	MessageIDs[191] = &MagCalProgress{}
	Messages["magcalreport"] = &MagCalReport{}
	MessageIDs[192] = &MagCalReport{}
	Messages["ekfstatusreport"] = &EkfStatusReport{}
	MessageIDs[193] = &EkfStatusReport{}
	Messages["pidtuning"] = &PidTuning{}
	MessageIDs[194] = &PidTuning{}
	Messages["deepstall"] = &Deepstall{}
	MessageIDs[195] = &Deepstall{}
	Messages["gimbalreport"] = &GimbalReport{}
	MessageIDs[200] = &GimbalReport{}
	Messages["gimbalcontrol"] = &GimbalControl{}
	MessageIDs[201] = &GimbalControl{}
	Messages["gimbaltorquecmdreport"] = &GimbalTorqueCmdReport{}
	MessageIDs[214] = &GimbalTorqueCmdReport{}
	Messages["goproheartbeat"] = &GoproHeartbeat{}
	MessageIDs[215] = &GoproHeartbeat{}
	Messages["goprogetrequest"] = &GoproGetRequest{}
	MessageIDs[216] = &GoproGetRequest{}
	Messages["goprogetresponse"] = &GoproGetResponse{}
	MessageIDs[217] = &GoproGetResponse{}
	Messages["goprosetrequest"] = &GoproSetRequest{}
	MessageIDs[218] = &GoproSetRequest{}
	Messages["goprosetresponse"] = &GoproSetResponse{}
	MessageIDs[219] = &GoproSetResponse{}
	Messages["rpm"] = &Rpm{}
	MessageIDs[226] = &Rpm{}

}

// Message IDs
const (
	MSG_ID_SENSOR_OFFSETS            MessageID = 150
	MSG_ID_SET_MAG_OFFSETS           MessageID = 151
	MSG_ID_MEMINFO                   MessageID = 152
	MSG_ID_AP_ADC                    MessageID = 153
	MSG_ID_DIGICAM_CONFIGURE         MessageID = 154
	MSG_ID_DIGICAM_CONTROL           MessageID = 155
	MSG_ID_MOUNT_CONFIGURE           MessageID = 156
	MSG_ID_MOUNT_CONTROL             MessageID = 157
	MSG_ID_MOUNT_STATUS              MessageID = 158
	MSG_ID_FENCE_POINT               MessageID = 160
	MSG_ID_FENCE_FETCH_POINT         MessageID = 161
	MSG_ID_FENCE_STATUS              MessageID = 162
	MSG_ID_AHRS                      MessageID = 163
	MSG_ID_SIMSTATE                  MessageID = 164
	MSG_ID_HWSTATUS                  MessageID = 165
	MSG_ID_RADIO                     MessageID = 166
	MSG_ID_LIMITS_STATUS             MessageID = 167
	MSG_ID_WIND                      MessageID = 168
	MSG_ID_DATA16                    MessageID = 169
	MSG_ID_DATA32                    MessageID = 170
	MSG_ID_DATA64                    MessageID = 171
	MSG_ID_DATA96                    MessageID = 172
	MSG_ID_RANGEFINDER               MessageID = 173
	MSG_ID_AIRSPEED_AUTOCAL          MessageID = 174
	MSG_ID_RALLY_POINT               MessageID = 175
	MSG_ID_RALLY_FETCH_POINT         MessageID = 176
	MSG_ID_COMPASSMOT_STATUS         MessageID = 177
	MSG_ID_AHRS2                     MessageID = 178
	MSG_ID_CAMERA_STATUS             MessageID = 179
	MSG_ID_CAMERA_FEEDBACK           MessageID = 180
	MSG_ID_BATTERY2                  MessageID = 181
	MSG_ID_AHRS3                     MessageID = 182
	MSG_ID_AUTOPILOT_VERSION_REQUEST MessageID = 183
	MSG_ID_REMOTE_LOG_DATA_BLOCK     MessageID = 184
	MSG_ID_REMOTE_LOG_BLOCK_STATUS   MessageID = 185
	MSG_ID_LED_CONTROL               MessageID = 186
	MSG_ID_MAG_CAL_PROGRESS          MessageID = 191
	MSG_ID_MAG_CAL_REPORT            MessageID = 192
	MSG_ID_EKF_STATUS_REPORT         MessageID = 193
	MSG_ID_PID_TUNING                MessageID = 194
	MSG_ID_DEEPSTALL                 MessageID = 195
	MSG_ID_GIMBAL_REPORT             MessageID = 200
	MSG_ID_GIMBAL_CONTROL            MessageID = 201
	MSG_ID_GIMBAL_TORQUE_CMD_REPORT  MessageID = 214
	MSG_ID_GOPRO_HEARTBEAT           MessageID = 215
	MSG_ID_GOPRO_GET_REQUEST         MessageID = 216
	MSG_ID_GOPRO_GET_RESPONSE        MessageID = 217
	MSG_ID_GOPRO_SET_REQUEST         MessageID = 218
	MSG_ID_GOPRO_SET_RESPONSE        MessageID = 219
	MSG_ID_RPM                       MessageID = 226
)

// DialectArdupilotmega is the dialect represented by ardupilotmega.xml
var DialectArdupilotmega *Dialect = &Dialect{
	Name: "ardupilotmega",
	crcExtras: map[MessageID]uint8{
		MSG_ID_SENSOR_OFFSETS:            134,
		MSG_ID_SET_MAG_OFFSETS:           219,
		MSG_ID_MEMINFO:                   208,
		MSG_ID_AP_ADC:                    188,
		MSG_ID_DIGICAM_CONFIGURE:         84,
		MSG_ID_DIGICAM_CONTROL:           22,
		MSG_ID_MOUNT_CONFIGURE:           19,
		MSG_ID_MOUNT_CONTROL:             21,
		MSG_ID_MOUNT_STATUS:              134,
		MSG_ID_FENCE_POINT:               78,
		MSG_ID_FENCE_FETCH_POINT:         68,
		MSG_ID_FENCE_STATUS:              189,
		MSG_ID_AHRS:                      127,
		MSG_ID_SIMSTATE:                  154,
		MSG_ID_HWSTATUS:                  21,
		MSG_ID_RADIO:                     21,
		MSG_ID_LIMITS_STATUS:             144,
		MSG_ID_WIND:                      1,
		MSG_ID_DATA16:                    234,
		MSG_ID_DATA32:                    73,
		MSG_ID_DATA64:                    181,
		MSG_ID_DATA96:                    22,
		MSG_ID_RANGEFINDER:               83,
		MSG_ID_AIRSPEED_AUTOCAL:          167,
		MSG_ID_RALLY_POINT:               138,
		MSG_ID_RALLY_FETCH_POINT:         234,
		MSG_ID_COMPASSMOT_STATUS:         240,
		MSG_ID_AHRS2:                     47,
		MSG_ID_CAMERA_STATUS:             189,
		MSG_ID_CAMERA_FEEDBACK:           52,
		MSG_ID_BATTERY2:                  174,
		MSG_ID_AHRS3:                     229,
		MSG_ID_AUTOPILOT_VERSION_REQUEST: 85,
		MSG_ID_REMOTE_LOG_DATA_BLOCK:     159,
		MSG_ID_REMOTE_LOG_BLOCK_STATUS:   186,
		MSG_ID_LED_CONTROL:               72,
		MSG_ID_MAG_CAL_PROGRESS:          92,
		MSG_ID_MAG_CAL_REPORT:            36,
		MSG_ID_EKF_STATUS_REPORT:         71,
		MSG_ID_PID_TUNING:                98,
		MSG_ID_DEEPSTALL:                 120,
		MSG_ID_GIMBAL_REPORT:             134,
		MSG_ID_GIMBAL_CONTROL:            205,
		MSG_ID_GIMBAL_TORQUE_CMD_REPORT:  69,
		MSG_ID_GOPRO_HEARTBEAT:           101,
		MSG_ID_GOPRO_GET_REQUEST:         50,
		MSG_ID_GOPRO_GET_RESPONSE:        202,
		MSG_ID_GOPRO_SET_REQUEST:         17,
		MSG_ID_GOPRO_SET_RESPONSE:        162,
		MSG_ID_RPM:                       207,
	},
	messageConstructorByMsgId: map[MessageID]func(*Packet) Message{
		MSG_ID_SENSOR_OFFSETS: func(pkt *Packet) Message {
			msg := new(SensorOffsets)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SET_MAG_OFFSETS: func(pkt *Packet) Message {
			msg := new(SetMagOffsets)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MEMINFO: func(pkt *Packet) Message {
			msg := new(Meminfo)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AP_ADC: func(pkt *Packet) Message {
			msg := new(ApAdc)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DIGICAM_CONFIGURE: func(pkt *Packet) Message {
			msg := new(DigicamConfigure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DIGICAM_CONTROL: func(pkt *Packet) Message {
			msg := new(DigicamControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_CONFIGURE: func(pkt *Packet) Message {
			msg := new(MountConfigure)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_CONTROL: func(pkt *Packet) Message {
			msg := new(MountControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MOUNT_STATUS: func(pkt *Packet) Message {
			msg := new(MountStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_POINT: func(pkt *Packet) Message {
			msg := new(FencePoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_FETCH_POINT: func(pkt *Packet) Message {
			msg := new(FenceFetchPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_FENCE_STATUS: func(pkt *Packet) Message {
			msg := new(FenceStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS: func(pkt *Packet) Message {
			msg := new(Ahrs)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_SIMSTATE: func(pkt *Packet) Message {
			msg := new(Simstate)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_HWSTATUS: func(pkt *Packet) Message {
			msg := new(Hwstatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RADIO: func(pkt *Packet) Message {
			msg := new(Radio)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LIMITS_STATUS: func(pkt *Packet) Message {
			msg := new(LimitsStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_WIND: func(pkt *Packet) Message {
			msg := new(Wind)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA16: func(pkt *Packet) Message {
			msg := new(Data16)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA32: func(pkt *Packet) Message {
			msg := new(Data32)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA64: func(pkt *Packet) Message {
			msg := new(Data64)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DATA96: func(pkt *Packet) Message {
			msg := new(Data96)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RANGEFINDER: func(pkt *Packet) Message {
			msg := new(Rangefinder)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AIRSPEED_AUTOCAL: func(pkt *Packet) Message {
			msg := new(AirspeedAutocal)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RALLY_POINT: func(pkt *Packet) Message {
			msg := new(RallyPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RALLY_FETCH_POINT: func(pkt *Packet) Message {
			msg := new(RallyFetchPoint)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_COMPASSMOT_STATUS: func(pkt *Packet) Message {
			msg := new(CompassmotStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS2: func(pkt *Packet) Message {
			msg := new(Ahrs2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_STATUS: func(pkt *Packet) Message {
			msg := new(CameraStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_CAMERA_FEEDBACK: func(pkt *Packet) Message {
			msg := new(CameraFeedback)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_BATTERY2: func(pkt *Packet) Message {
			msg := new(Battery2)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AHRS3: func(pkt *Packet) Message {
			msg := new(Ahrs3)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_AUTOPILOT_VERSION_REQUEST: func(pkt *Packet) Message {
			msg := new(AutopilotVersionRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REMOTE_LOG_DATA_BLOCK: func(pkt *Packet) Message {
			msg := new(RemoteLogDataBlock)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_REMOTE_LOG_BLOCK_STATUS: func(pkt *Packet) Message {
			msg := new(RemoteLogBlockStatus)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_LED_CONTROL: func(pkt *Packet) Message {
			msg := new(LedControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MAG_CAL_PROGRESS: func(pkt *Packet) Message {
			msg := new(MagCalProgress)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_MAG_CAL_REPORT: func(pkt *Packet) Message {
			msg := new(MagCalReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_EKF_STATUS_REPORT: func(pkt *Packet) Message {
			msg := new(EkfStatusReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_PID_TUNING: func(pkt *Packet) Message {
			msg := new(PidTuning)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_DEEPSTALL: func(pkt *Packet) Message {
			msg := new(Deepstall)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_REPORT: func(pkt *Packet) Message {
			msg := new(GimbalReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_CONTROL: func(pkt *Packet) Message {
			msg := new(GimbalControl)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GIMBAL_TORQUE_CMD_REPORT: func(pkt *Packet) Message {
			msg := new(GimbalTorqueCmdReport)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_HEARTBEAT: func(pkt *Packet) Message {
			msg := new(GoproHeartbeat)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_GET_REQUEST: func(pkt *Packet) Message {
			msg := new(GoproGetRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_GET_RESPONSE: func(pkt *Packet) Message {
			msg := new(GoproGetResponse)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_SET_REQUEST: func(pkt *Packet) Message {
			msg := new(GoproSetRequest)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_GOPRO_SET_RESPONSE: func(pkt *Packet) Message {
			msg := new(GoproSetResponse)
			msg.Unpack(pkt)
			return msg
		},
		MSG_ID_RPM: func(pkt *Packet) Message {
			msg := new(Rpm)
			msg.Unpack(pkt)
			return msg
		},
	},
}
