# The authoritative surface, transcribed from psdk_ros2_topics_services.pdf
TELEMETRY = """acceleration_body_fused acceleration_body_raw acceleration_ground_fused
altitude_barometric altitude_sea_level angular_rate_body_raw angular_rate_ground_fused attitude
control_mode display_mode esc_data flight_anomaly flight_status gps_details gps_position
gps_position_fused gps_signal_level gps_velocity height_above_ground home_point home_point_status
imu magnetic_field main_camera_stream motor_start_error perception_stereo_left_stream
perception_stereo_right_stream position_fused rc rc_connection_status relative_obstacle_info
rtk_connection_status rtk_position rtk_position_info rtk_velocity rtk_yaw rtk_yaw_info
single_battery_index1 single_battery_index2 velocity_ground_fused visual_odometry""".split()
COMMAND = """flight_control_setpoint_ENUposition_yaw flight_control_setpoint_ENUvelocity_yawrate
flight_control_setpoint_FLUvelocity_yawrate flight_control_setpoint_generic
flight_control_setpoint_rollpitch_yawrate_thrust""".split()
NO_DATA = """battery fpv_camera_stream gimbal_angles gimbal_status gps_control_level
home_point_altitude landing_gear_status perception_camera_parameters""".split()
SERVICES = """camera_format_sd_card camera_get_aperture camera_get_exposure_mode_ev
camera_get_file_list_info camera_get_focus_mode camera_get_focus_ring_range
camera_get_focus_ring_value camera_get_focus_target camera_get_iso camera_get_laser_ranging_info
camera_get_optical_zoom camera_get_sd_storage_info camera_get_shutter_speed camera_get_type
camera_record_video camera_set_aperture camera_set_exposure_mode_ev camera_set_focus_mode
camera_set_focus_ring_value camera_set_focus_target camera_set_infrared_zoom camera_set_iso
camera_set_optical_zoom camera_set_shutter_speed camera_setup_streaming camera_shoot_burst_photo
camera_shoot_interval_photo camera_shoot_single_photo camera_stop_shoot_photo cancel_go_home
cancel_landing get_downwards_vo_obstacle_avoidance get_go_home_altitude
get_horizontal_radar_obstacle_avoidance get_horizontal_vo_obstacle_avoidance
get_upwards_radar_obstacle_avoidance get_upwards_vo_obstacle_avoidance land obtain_ctrl_authority
release_ctrl_authority set_downwards_vo_obstacle_avoidance set_go_home_altitude
set_home_from_current_location set_home_from_gps set_horizontal_radar_obstacle_avoidance
set_horizontal_vo_obstacle_avoidance set_local_position_ref set_upwards_radar_obstacle_avoidance
set_upwards_vo_obstacle_avoidance start_confirm_landing start_force_landing start_go_home
start_perception takeoff turn_off_motors turn_on_motors""".split()
if __name__ == "__main__":
    print(len(TELEMETRY), len(COMMAND), len(NO_DATA), len(SERVICES))


# ── how the live surface is allowed to differ, and why ───────────────────────

# Opt-in by design (plan §12.4): an uncompressed 1080p frame is ~6 MB and two
# feeds saturate the transport, starving control and telemetry. These appear
# when a camera group is enabled, not before.
VIDEO_OPT_IN = [
    "main_camera_stream",
    "fpv_camera_stream",
    "perception_stereo_left_stream",
    "perception_stereo_right_stream",
]

# Topics a real M4E publishes that this simulation does not yet synthesise.
# V1 did not synthesise them either -- they arrived from the aircraft's own
# hardware and the bridge merely mirrored them, so there was never any
# conversion to inherit. Tracked explicitly so the gap is reported rather than
# rediscovered by a mission waiting on a topic that will never arrive.
SYNTHESIS_GAP = [
    "acceleration_body_fused", "acceleration_body_raw",
    "acceleration_ground_fused", "angular_rate_ground_fused",
    "esc_data", "gps_control_level", "gps_signal_level",
    "landing_gear_status", "magnetic_field", "motor_start_error",
    "perception_camera_parameters",
    "rtk_connection_status", "rtk_position_info", "rtk_velocity",
    "rtk_yaw", "rtk_yaw_info",
]

# Command topics with no honest PX4 equivalent: PX4 takes attitude and thrust
# on a different message entirely, and `generic` is a raw PSDK flag byte. A
# route that accepted them and quietly did nothing would be worse than absent.
COMMAND_UNCONVERTIBLE = [
    "flight_control_setpoint_generic",
    "flight_control_setpoint_rollpitch_yawrate_thrust",
]

# Publish rates from the PDF, for the topics worth holding to a number.
EXPECTED_HZ = {
    "attitude": 50.0, "imu": 50.0, "position_fused": 50.0,
    "velocity_ground_fused": 50.0, "angular_rate_body_raw": 50.0,
    "altitude_barometric": 70.0, "altitude_sea_level": 70.0,
    "flight_status": 25.0, "display_mode": 25.0, "control_mode": 24.0,
    "height_above_ground": 25.0, "gps_position": 25.0, "gps_details": 25.0,
    "rc": 25.0, "rc_connection_status": 25.0, "relative_obstacle_info": 25.0,
    "home_point": 25.0, "home_point_status": 24.0, "flight_anomaly": 25.0,
    "gps_velocity": 25.0, "gps_position_fused": 30.0,
    "single_battery_index2": 21.0, "visual_odometry": 50.0,
}
