#include "Copter.h"

#if MODE_CNDN_ENABLED == ENABLED

const AP_Param::GroupInfo ModeCNDN::var_info[] = {
    // @Param: METHOD
    // @DisplayName: Mode using method
    // @Description: Mode using method of CNDN & ETRI Mission computer
    // @Values: 0: Disable, 1: All enable, 2: Take picture only, 2: Edge follow only, 3: Take picture after Edge following
    // @User: Advance
    AP_GROUPINFO_FLAGS("METHOD", 0, ModeCNDN, _method, 2, AP_PARAM_FLAG_ENABLE),

    // @Param: TAKE_ALT
    // @DisplayName: Take picture altitute
    // @Description: Altitute of take picture
    // @Units: cm
    // @Range: 100 2000
    // @User: Advance
    AP_GROUPINFO("TAKEOFF_ALT", 1, ModeCNDN, _take_alt_cm, 200),

    // @Param: MISSION_ALT
    // @DisplayName: Mission altitute
    // @Description: Altitute of mission planning
    // @Units: cm
    // @Range: 200 1000
    // @User: Advance
    AP_GROUPINFO("MISSION_ALT", 2, ModeCNDN, _mission_alt_cm, 300),

    // @Param: SPRAY_WIDTH
    // @DisplayName: Spray width
    // @Description: Mission planning width of spraying
    // @Units: cm
    // @Range: 3000 8000
    // @User: Advance
    AP_GROUPINFO("SPRAY_WIDTH", 3, ModeCNDN, _spray_width_cm, 400),

    // @Param: DIS_EDGE
    // @DisplayName: Distance Edge
    // @Description: Distance from Edge
    // @Units: cms
    // @Range: 100 800
    // @User: Advance
    AP_GROUPINFO("DIS_EDGE", 4, ModeCNDN, _dst_eg_cm, 250),

    // @Param: SPD_EDGE mission
    // @DisplayName: Speed edge
    // @Description: Mission speed for Edge
    // @Units: cm
    // @Range: 100 1000
    // @User: Advance
    AP_GROUPINFO("SPD_EDGE", 5, ModeCNDN, _spd_edge_cm, 350),

    // @Param: SPD_AUTO
    // @DisplayName: Speed auto mission
    // @Description: Mission speed for Auto
    // @Units: cm
    // @Range: 100 1000
    // @User: Advance
    AP_GROUPINFO("SPD_AUTO", 6, ModeCNDN, _spd_auto_cm, 500),

    // @Param: RADAR_FLT_HZ
    // @DisplayName: RADAR Filter Herz
    // @Description: Radar low pass filter frequency
    // @Units: Hz
    // @Range: 0.0 1.0
    // @User: Advance
    AP_GROUPINFO("RADAR_HZ", 7, ModeCNDN, _radar_flt_hz, 0.25),

    // @Param: LEVEL_PIN
    // @DisplayName: Level sensor gpio pin
    // @Description: Level sensor gpio pin
    // @Range: 0.0 1.0
    // @User: Advance
    AP_GROUPINFO("LEVEL_PIN", 8, ModeCNDN, _sensor_pin, 59),

    // @Param: AVOID_CM
    // @DisplayName: AVOIDANCE DISTANCE CM
    // @Description: Avoidance distance for break mode
    // @Range: 200 1000
    // @User: Advance
    AP_GROUPINFO("AVOID_CM", 9, ModeCNDN, _avoid_cm, 600),

    // @Param: MC8_OPTION
    // @DisplayName: MC8_OPTION
    // @Description: Mission computer options
    // @Range: 0
    // @User: Advance
    AP_GROUPINFO("MC8_OPTION", 10, ModeCNDN, _mc8_option, 0),

    AP_GROUPEND
};

class GPIOSensor
{
private:
    GPIOSensor() {
        toTICK.reset(0);
    }
#ifdef HAL_PUMP_SENSOR_PIN
    uint8_t u_pin = HAL_PUMP_SENSOR_PIN;
    uint8_t l_pin = HAL_PUMP_SENSOR_PIN;
#else
    uint8_t u_pin = 0;
    uint8_t l_pin = 0;
#endif
    uint32_t l_ms = 0;
    uint32_t l_cn = 0;
    uint32_t l_en = 0;
    uint32_t l_dt = 0;
    uint32_t m_count = 0;
    uint32_t l_count = 0;
    bool m_init = false;
    bool m_set = false;
    bool m_pin_state = false;

public:
    CNTimeout toTICK;

    static GPIOSensor& get() {
        static GPIOSensor sgpio;
        return sgpio;
    }
 
    void set_pin(uint8_t pin) {
        u_pin = pin;
        if (l_pin != u_pin) {
            l_pin = u_pin;
            m_init = false;
        }
    }

    bool isTimeout(uint32_t now, uint32_t tout) {
        if (l_ms == 0) l_ms = now;
        return (now - l_ms) > tout;
    }

    void resetTimeout(uint32_t now) { l_ms = now; }

    bool stateChanged(bool bSet) {
        if (m_set != bSet) {
            m_set = bSet;
            return true;
        }
        return false;
    }

    uint32_t getPulse() { return 0;/*AP::rpm()->get_counter(0);*/ }

    void resetCount() { /*AP::rpm()->reset_counter(0);*/ }

    float getCount() {
        if (u_pin) {
            // ensure we are in input mode
            hal.gpio->pinMode(u_pin, HAL_GPIO_INPUT);
            bool bState = hal.gpio->read(u_pin); // Active Low
            return bState ? 0.0f : 1000.0f;
        }
        return 0.0f;
    }

    float getRPM() {
        return 0.0f;/*AP::rpm()->get_rpm(0);*/
    }
};
/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeCNDN::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

#if PRECISION_LANDING == ENABLED
    _precision_loiter_active = false;
#endif

    return true;
}

#if PRECISION_LANDING == ENABLED
bool ModeCNDN::do_precision_loiter()
{
    if (!_precision_loiter_enabled) {
        return false;
    }
    if (copter.ap.land_complete_maybe) {
        return false;        // don't move on the ground
    }
    // if the pilot *really* wants to move the vehicle, let them....
    if (loiter_nav->get_pilot_desired_acceleration().length() > 50.0f) {
        return false;
    }
    if (!copter.precland.target_acquired()) {
        return false; // we don't have a good vector
    }
    return true;
}

void ModeCNDN::precision_loiter_xy()
{
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos = inertial_nav.get_position_xy_cm();
    }
    // get the velocity of the target
    copter.precland.get_target_velocity_cms(inertial_nav.get_velocity_xy_cms(), target_vel);

    Vector2f zero;
    Vector2p landing_pos = target_pos.topostype();
    // target vel will remain zero if landing target is stationary
    pos_control->input_pos_vel_accel_xy(landing_pos, target_vel, zero);
    // run pos controller
    pos_control->update_xy_controller();
}
#endif

void ModeCNDN::init_speed()
{
    wp_nav->wp_and_spline_init();
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeCNDN::run()
{
    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine Determination
    AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);

    // Loiter State Machine
    switch (loiter_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        loiter_nav->init_target();
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // run loiter controller
        loiter_nav->update();

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if PRECISION_LANDING == ENABLED
        bool precision_loiter_old_state = _precision_loiter_active;
        if (do_precision_loiter()) {
            precision_loiter_xy();
            _precision_loiter_active = true;
        } else {
            _precision_loiter_active = false;
        }
        if (precision_loiter_old_state && !_precision_loiter_active) {
            // prec loiter was active, not any more, let's init again as user takes control
            loiter_nav->init_target();
        }
        // run loiter controller if we are not doing prec loiter
        if (!_precision_loiter_active) {
            loiter_nav->update();
        }
#else
        loiter_nav->update();
#endif

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

uint32_t ModeCNDN::wp_distance() const
{
    return loiter_nav->get_distance_to_target();
}

int32_t ModeCNDN::wp_bearing() const
{
    return loiter_nav->get_bearing_to_target();
}

// return manual control to the pilot
void ModeCNDN::return_to_manual_control(bool maintain_target)
{
    cmd_mode = 0;

    copter.rangefinder_state.enabled = false;

    if (stage != MANUAL) {
        stage = MANUAL;
        loiter_nav->clear_pilot_desired_acceleration();
        if (maintain_target) {
            const Vector3f wp_dest = wp_nav->get_wp_destination();
            loiter_nav->init_target(wp_dest.xy());
            if (wp_nav->origin_and_destination_are_terrain_alt()) {
                copter.surface_tracking.set_target_alt_cm(wp_dest.z);
            }
        } else {
            loiter_nav->init_target();
        }
        auto_yaw.set_mode(Mode::AutoYaw::Mode::HOLD);
    }
}

void ModeCNDN::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_NAMED_VALUE_INT: {
            mavlink_named_value_int_t m;
            mavlink_msg_named_value_int_decode(&msg, &m);
            if (strncmp(m.name, "BLINKR", 10) == 0) {
            } else if (strncmp(m.name, "BLINKG", 10) == 0) {
            } else if (strncmp(m.name, "VNOTIFY", 10) == 0) {
            } else if (strncmp(m.name, "WIFICHAN", 10) == 0) {
            } else if (strncmp(m.name, "LOGDISARM", 10) == 0) {
            }
        } break;
    }
}

void ModeCNDN::mission_trigger(uint8_t dest_num)
{
    logdebug("mission_trigger(%d)", dest_num);

    bool bControlled = copter.flightmode == &copter.mode_auto/* || copter.flightmode == &copter.mode_zigzag*/;

    if (bControlled) { // 속도/분사 제어
        float mss, prr;
        switch (dest_num) {
            case 6: // CNDN_SPD_UP
                if (copter.flightmode == &copter.mode_auto && edge_mode) {
                    mss = _spd_edge_cm.get() + 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_edge_cm.set_and_save(mss);
                } else {
                    mss = _spd_auto_cm.get() + 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_auto_cm.set_and_save(mss);
                }
                gcsdebug("속도 증가: %0.2f m/s", mss * 1e-2f);
            break;

            case 7: // CNDN_SPD_DN
                if (copter.flightmode == &copter.mode_auto && edge_mode) {
                    mss = _spd_edge_cm.get() - 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_edge_cm.set_and_save(mss);
                } else {
                    mss = _spd_auto_cm.get() - 50.0f;
                    mss = constrain_float(mss, 100.0f, 1500.0f);
                    copter.wp_nav->set_speed_xy(mss);
                    _spd_auto_cm.set_and_save(mss);
                }
                gcsdebug("속도 감소: %0.2f m/s", mss * 1e-2f);
            break;

#if HAL_SPRAYER_ENABLED == ENABLED
            case 8: // CNDN_SPR_UP
                prr = copter.sprayer.inc_pump_rate(+2.5f);
                gcsdebug("분무량 증가: %0.1f%%", prr);
            break;
            case 9: // CNDN_SPR_DN
                prr = copter.sprayer.inc_pump_rate(-2.5f);
                gcsdebug("분무량 감소: %0.1f%%", prr);
            break;
#endif

#if MODE_ZIGZAG_ENABLED == ENABLED
            case 0: case 1: case 2:
                if (copter.flightmode == &copter.mode_zigzag)
                    copter.mode_zigzag.save_or_move_to_destination(dest_num);
            break;
#endif
        }
        return;
    }

    switch (dest_num) {
        case 20: {
            Location loc(copter.current_loc);
            Location home(AP::ahrs().get_home());
            int32_t yaws = wrap_180_cd(ahrs.yaw_sensor);
            gcs().send_cndn_trigger(home, loc, 0, 0, 2, yaws);
        } return;
    }

    if (dest_num > 2)
        return;

    // handle state machine changes
    // if (copter.flightmode != &copter.mode_cndn) {
    //     // 씨엔디엔 모드가 아닐 때
    //     return;
    // }

    switch (stage) {
    case MANUAL:
        if (_method.get() == 0)
            break;

        if (dest_num > 0) {
            if (copter.flightmode != &copter.mode_cndn)
                break;

            cmd_mode = dest_num;
            init_speed();
            Vector3f stopping_point;
            wp_nav->get_wp_stopping_point(stopping_point);
            wp_nav->set_wp_destination(stopping_point, false);

            Location loc(copter.current_loc);
            Location home(AP::ahrs().get_home());
            int32_t yaws = wrap_180_cd(ahrs.yaw_sensor);
            gcs().send_cndn_trigger(home, loc, _dst_eg_cm.get(), _spray_width_cm.get(), m_bZigZag?1:0, yaws);
            gcsdebug("[방제검색] %d,%d,%d", (int)loc.lat, (int)loc.lng, (int)yaws);

            float alt_cm = 0.0f;
            if (wp_nav->get_terrain_offset(alt_cm)) {
                copter.surface_tracking.set_target_alt_cm(alt_cm);
            }
        }
        break;

    case PREPARE_AUTO:
        if (_method.get() == 0){
            init_speed();
            return_to_manual_control(false);
            return;
        }

        if (dest_num == 2)
            cmd_mode = dest_num;

        if (dest_num == 0) {
            init_speed();
            return_to_manual_control(false);
            return;
        }
        break;

    // case AUTO:
    // case PREPARE_ABLINE:
    // case FINISHED:
    default:
        if (dest_num == 0) {
            init_speed();
            return_to_manual_control(false);
            return;
        }
        break;
    }
}

void ModeCNDN::check_avoidance()
{
}

void ModeCNDN::check_sensors()
{
}

void ModeCNDN::check_spraying()
{
}

#endif
