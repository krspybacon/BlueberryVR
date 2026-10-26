#pragma once

//////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Display_Section = "Display";

static const char* const k_pch_Sample_WindowX_Int32 = "windowX";
static const char* const k_pch_Sample_WindowY_Int32 = "windowY";
static const char* const k_pch_Sample_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_Sample_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_Sample_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_Sample_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_Sample_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_Sample_DisplayFrequency_Float = "displayFrequency";
static const char* const k_pch_Sample_ViewportZoom_Float = "ViewportZoom";
static const char* const k_pch_Sample_FOV_Float = "FOV";
static const char* const k_pch_Sample_IPD_Float = "IPD";
static const char* const k_pch_Sample_EyeReliefMeters_Float = "EyeReliefMeters";
static const char* const k_pch_Sample_DistanceBetweenLenses_Float = "DistanceBetweenLenses";
static const char* const k_pch_Sample_DistanceBetweenViews_Float = "DistanceBetweenViews";
static const char* const k_pch_Sample_DisplayWidth_Float = "DisplayWidth";
static const char* const k_pch_Sample_IsSinglePanel_Bool = "IsSinglePanel";
static const char* const k_pch_Sample_ScreenOffsetX_Int32 = "ScreenOffsetX";
static const char* const k_pch_Sample_ScreenOffsetY_Int32 = "ScreenOffsetY";
static const char* const k_pch_Sample_Stereo_Bool = "Stereo";
static const char* const k_pch_Sample_DisplayOnDesktop = "IsDisplayOnDesktop";
static const char* const k_pch_Sample_DisplayReal = "IsDisplayReal";
static const char* const k_pch_Sample_DebugMode_Bool = "DebugMode";

static const char* const k_pch_Sample_EyeRightYawAngle_Float = "RightEyeYawAngle";
static const char* const k_pch_Sample_EyeRightPitchAngle_Float = "RightEyePitchAngle";
static const char* const k_pch_Sample_EyeRightRollAngle_Float = "RightEyeRollAngle";

static const char* const k_pch_Sample_EyeLeftYawAngle_Float = "LeftEyeYawAngle";
static const char* const k_pch_Sample_EyeLeftPitchAngle_Float = "LeftEyePitchAngle";
static const char* const k_pch_Sample_EyeLeftRollAngle_Float = "LeftEyeRollAngle";

static const char* const k_pch_Sample_ViewportPixelOffset = "JankUVOffset";
static const char* const k_pch_Sample_Mirror_Bool = "MirrorImage";

//////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Distortion_Section = "Distortion";

static const char* const k_pch_Distortion_Red_K1_Float = "Red_K1";
static const char* const k_pch_Distortion_Red_K2_Float = "Red_K2";
static const char* const k_pch_Distortion_Red_K3_Float = "Red_K3";
static const char* const k_pch_Distortion_Green_K1_Float = "Green_K1";
static const char* const k_pch_Distortion_Green_K2_Float = "Green_K2";
static const char* const k_pch_Distortion_Green_K3_Float = "Green_K3";
static const char* const k_pch_Distortion_Blue_K1_Float = "Blue_K1";
static const char* const k_pch_Distortion_Blue_K2_Float = "Blue_K2";
static const char* const k_pch_Distortion_Blue_K3_Float = "Blue_K3";

//////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Driver_Section = "Driver";

static const char* const k_pch_HID_PID_Int32 = "HID_PID";
static const char* const k_pch_HID_VID_Int32 = "HID_VID";
static const char* const k_pch_DirectMode_EDID_PID_Int32 = "EDID_PID";
static const char* const k_pch_DirectMode_EDID_VID_Int32 = "EDID_VID";
static const char* const k_pch_DirectModeEnable_Bool = "EnableDirectMode";

static const char* const k_pch_headlessMode_Bool = "HeadlessMode";
static const char* const k_pch_TransportMode_String = "TransportMode";
static const char* const k_pch_UART_Port = "UART_Port";
static const char* const k_pch_UART_Baudrate = "UART_Baudrate";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_HMD_Section = "HMD";

static const char* const k_pch_HMD_SerialNumber_String = "serialNumber";
static const char* const k_pch_HMD_ModelNumber_String = "modelNumber";
static const char* const k_pch_HMD_Enable_Bool = "EnableHMD";
static const char* const k_pch_HMD_MinFilterBeta_Float = "MinFilterBeta";
static const char* const k_pch_HMD_MaxFilterBeta_Float = "MaxFilterBeta";

static const char* const k_pch_HMD_YawOffset_Float = "HMDYawOffset";
static const char* const k_pch_HMD_PitchOffset_Float = "HMDPitchOffset";
static const char* const k_pch_HMD_RollOffset_Float = "HMDRollOffset";

static const char* const k_pch_HMD_XOffset_Float = "HMDXOffset";
static const char* const k_pch_HMD_YOffset_Float = "HMDYOffset";
static const char* const k_pch_HMD_ZOffset_Float = "HMDZOffset";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Controllers_Section = "Controllers";

static const char* const k_pch_Controller_Enable_Bool = "EnableControllers";
static const char* const k_pch_Controller_Mode_Int32 = "ControllerMode";

static const char* const k_pch_ControllerRight_YawOffset_Float = "CTRLRightYawOffset";
static const char* const k_pch_ControllerRight_PitchOffset_Float = "CTRLRightPitchOffset";
static const char* const k_pch_ControllerRight_RollOffset_Float = "CTRLRightRollOffset";

static const char* const k_pch_ControllerRight_XOffset_Float = "CTRLRightXOffset";
static const char* const k_pch_ControllerRight_YOffset_Float = "CTRLRightYOffset";
static const char* const k_pch_ControllerRight_ZOffset_Float = "CTRLRightZOffset";

static const char* const k_pch_ControllerLeft_YawOffset_Float = "CTRLLeftYawOffset";
static const char* const k_pch_ControllerLeft_PitchOffset_Float = "CTRLLeftPitchOffset";
static const char* const k_pch_ControllerLeft_RollOffset_Float = "CTRLLeftRollOffset";

static const char* const k_pch_ControllerLeft_XOffset_Float = "CTRLLeftXOffset";
static const char* const k_pch_ControllerLeft_YOffset_Float = "CTRLLeftYOffset";
static const char* const k_pch_ControllerLeft_ZOffset_Float = "CTRLLeftZOffset";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Tracker_Section = "Trackers";

static const char* const k_pch_Tracker_Enable_Bool = "EnableTrackers";
static const char* const k_pch_Tracker_Mode_Int32 = "TrackerMode";

///////////////////////////////////////////////////////////////////////////////////////////////////

static const char* const k_pch_Experimental_Section = "Experimental";

static const char* const k_pch_EnableCorrection_Bool = "EnableDriftCorrection";
static const char* const k_pch_Drift_LowVelThreshold_Float = "LowerVelocityTreshold";
static const char* const k_pch_Drift_HighVelThreshold_Float = "UpperVelocityTreshold";
static const char* const k_pch_Drift_MeasurementUncertainty_Float = "Measurement_Uncertainty";
static const char* const k_pch_Drift_EstimationUncertainty_Float = "Estimation_Uncertainty";
static const char* const k_pch_Drift_HMDP_Noise_Float = "HMD_Process_Noise";
static const char* const k_pch_Drift_ContP_Noise_Float = "Controller_Process_Noise";

///////////////////////////////////////////////////////////////////////////////////////////////////
//general use
static const char* const k_pch_Tracking_AccelEnable_Bool = "UseAccelerometers";
static const char* const k_pch_Camera_Kalman_Meas_err_Float = "Camera_Kalman_Measurement_Uncertainty";
static const char* const k_pch_Camera_Kalman_Estim_err_Float = "Camera_Kalman_Estimation_Uncertainty";
static const char* const k_pch_Camera_Kalman_Proc_noise_Float = "Camera_Kalman_Process_Noise";
static const char* const k_pch_IMU_Kalman_Meas_err_Float = "IMU_Kalman_Measurement_Uncertainty";
static const char* const k_pch_IMU_Kalman_Estim_err_Float = "IMU_Kalman_Estimation_Uncertainty";
static const char* const k_pch_IMU_Kalman_Proc_noise_Float = "IMU_Kalman_Process_Noise";

///////////////////////////////////////////////////////////////////////////////////////////////////
//this all goes in the steamvr.vrsettings file
static const char* const k_pch_Calibration_Section = "HadesVR_Calibration";

static const char* const k_pch_Calibration_HMD = "HMD";
static const char* const k_pch_Calibration_HMDW_Float = "HMDW";
static const char* const k_pch_Calibration_HMDY_Float = "HMDY";

static const char* const k_pch_Calibration_CONTRight = "CONTRight";
static const char* const k_pch_Calibration_CONTRightW_Float = "CONTRightW";
static const char* const k_pch_Calibration_CONTRightY_Float = "CONTRightY";


static const char* const k_pch_Calibration_CONTLeft = "CONTLeft";
static const char* const k_pch_Calibration_CONTLeftW_Float = "CONTLeftW";
static const char* const k_pch_Calibration_CONTLeftY_Float = "CONTLeftY";


static const char* const k_pch_Calibration_TRKWaist = "TRKWaist";
static const char* const k_pch_Calibration_TRKWaistW_Float = "TRKWaistW";
static const char* const k_pch_Calibration_TRKWaistY_Float = "TRKWaistY";


static const char* const k_pch_Calibration_TRKLeft = "TRKLeft";
static const char* const k_pch_Calibration_TRKLeftW_Float = "TRKLeftW";
static const char* const k_pch_Calibration_TRKLeftY_Float = "TRKLeftY";


static const char* const k_pch_Calibration_TRKRight = "TRKRight";
static const char* const k_pch_Calibration_TRKRightW_Float = "TRKRightW";
static const char* const k_pch_Calibration_TRKRightY_Float = "TRKRightY";

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
//headset camera SLAM + controller LED tracking

static const char* const k_pch_Slam_Section = "Slam";

static const char* const k_pch_Slam_Enable_Bool = "EnableSlam";
static const char* const k_pch_Slam_CameraIndex_Int32 = "CameraIndex";
static const char* const k_pch_Slam_CameraWidth_Int32 = "CameraWidth";
static const char* const k_pch_Slam_CameraHeight_Int32 = "CameraHeight";
static const char* const k_pch_Slam_CameraFps_Int32 = "CameraFps";
static const char* const k_pch_Slam_AutoExposure_Bool = "AutoExposure";
static const char* const k_pch_Slam_Exposure_Float = "Exposure";

static const char* const k_pch_Slam_Fx_Float = "Fx";
static const char* const k_pch_Slam_Fy_Float = "Fy";
static const char* const k_pch_Slam_Cx_Float = "Cx";
static const char* const k_pch_Slam_Cy_Float = "Cy";
static const char* const k_pch_Slam_HFOV_Float = "HorizontalFOV";
static const char* const k_pch_Slam_DistK1_Float = "DistortionK1";
static const char* const k_pch_Slam_DistK2_Float = "DistortionK2";
static const char* const k_pch_Slam_DistP1_Float = "DistortionP1";
static const char* const k_pch_Slam_DistP2_Float = "DistortionP2";
static const char* const k_pch_Slam_DistK3_Float = "DistortionK3";

static const char* const k_pch_Slam_CameraPitch_Float = "CameraPitchAngle";
static const char* const k_pch_Slam_CameraYaw_Float = "CameraYawAngle";
static const char* const k_pch_Slam_CameraRoll_Float = "CameraRollAngle";

static const char* const k_pch_Slam_LEDDiameter_Float = "LEDDiameter";
static const char* const k_pch_Slam_LEDDepthScale_Float = "LEDDepthScale";
static const char* const k_pch_Slam_LEDRightHue_Int32 = "LEDRightHue";
static const char* const k_pch_Slam_LEDLeftHue_Int32 = "LEDLeftHue";
static const char* const k_pch_Slam_LEDHueTolerance_Int32 = "LEDHueTolerance";
static const char* const k_pch_Slam_LEDMinSaturation_Int32 = "LEDMinSaturation";
static const char* const k_pch_Slam_LEDMinValue_Int32 = "LEDMinValue";

static const char* const k_pch_Slam_MaxFeatures_Int32 = "MaxFeatures";
static const char* const k_pch_Slam_MaxMapPoints_Int32 = "MaxMapPoints";
static const char* const k_pch_Slam_FallbackDepth_Float = "FallbackSceneDepth";
static const char* const k_pch_Slam_OriginX_Float = "OriginX";
static const char* const k_pch_Slam_OriginY_Float = "OriginY";
static const char* const k_pch_Slam_OriginZ_Float = "OriginZ";
static const char* const k_pch_Slam_MapFile_String = "MapFile";
static const char* const k_pch_Slam_DebugImage_String = "DebugImage";
static const char* const k_pch_Slam_AutoCalibrate_Bool = "AutoCalibrate";
static const char* const k_pch_Slam_CalibrationFile_String = "CalibrationFile";
