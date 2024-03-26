#pragma once

#ifndef __MODE_CNDN_HPP__
#define __MODE_CNDN_HPP__

#include    "Copter.h"

# define gcsdebug(FORM, ...)    gcs().send_text(MAV_SEVERITY_DEBUG, FORM, __VA_ARGS__)
# define gcsinfo(FORM)          gcs().send_text(MAV_SEVERITY_INFO, FORM)
# define gcswarning(FORM)       gcs().send_text(MAV_SEVERITY_WARNING, FORM)
# define logdebug(FORM, args ...)    hal.console->printf(FORM "\n", ## args)

#define CNDN_TONE_STARTUP    { "MFT200L4O4CL8GAL2F", false }

#define CASE_MODE_NUMBER_CNDN()     \
        case Mode::Number::CNDN:    \
            ret = &mode_cndn;       \
            break;

#define CASE_MODE_NUMBER_CNDN2()    \
    case Mode::Number::CNDN: ret = &mode_cndn; mode_cndn.setZigZag(false);  break; \
    case Mode::Number::AWZIGZAG: ret = &mode_cndn; mode_cndn.setZigZag(true); break;

#define CASE_AUX_FUNC_INIT_CNDN()   \
        case AUX_FUNC::CNDN: case AUX_FUNC::CNDN_AUTO:  case AUX_FUNC::CNDN_PUMP: \
        case AUX_FUNC::CNDN_SPD_UP: case AUX_FUNC::CNDN_SPD_DN: case AUX_FUNC::CNDN_SPR_UP: \
        case AUX_FUNC::CNDN_SPR_DN: case AUX_FUNC::CNDN_SPR_FF:

#define CASE_AUX_FUNC_CNDN()                            \
    case AUX_FUNC::CNDN:                                \
        copter.mode_cndn.mission_trigger(3 + (uint8_t)ch_flag);  \
    break;                                              \
    case AUX_FUNC::CNDN_AUTO: {                         \
        switch (ch_flag) {                              \
        case AuxSwitchPos::LOW      : copter.mode_cndn.mission_trigger(0); break;    \
        case AuxSwitchPos::MIDDLE   : copter.mode_cndn.mission_trigger(1); break;    \
        case AuxSwitchPos::HIGH     : copter.mode_cndn.mission_trigger(2); break;    \
        }                                                           \
    } break;                                                        \
    case AUX_FUNC::CNDN_SPD_UP: if (ch_flag != AuxSwitchPos::LOW) copter.mode_cndn.mission_trigger(6); break;\
    case AUX_FUNC::CNDN_SPD_DN: if (ch_flag != AuxSwitchPos::LOW) copter.mode_cndn.mission_trigger(7); break;\
    case AUX_FUNC::CNDN_SPR_UP: if (ch_flag != AuxSwitchPos::LOW) copter.mode_cndn.mission_trigger(8); break;\
    case AUX_FUNC::CNDN_SPR_DN: if (ch_flag != AuxSwitchPos::LOW) copter.mode_cndn.mission_trigger(9); break;\
    case AUX_FUNC::CNDN_PUMP: break; \
    case AUX_FUNC::CNDN_TRIG02: if (ch_flag == AuxSwitchPos::HIGH) copter.mode_cndn.mission_trigger(20); break;

#define CNDN_HANDLE_MESSAGE() \
    copter.mode_cndn.handle_message(msg);

struct CNMIS {
    float   yaw_deg;
    float   spdcm;
    uint8_t spryr;
    uint8_t edge;
    bool    addNew;
    uint16_t curr_idx;
    uint16_t repl_idx;
    uint16_t jump_idx;
    Location loctg;
    int32_t  misAlt;
    Location::AltFrame misFrame;
};

class ModeCNDN : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CNDN; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }

#if PRECISION_LANDING == ENABLED
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif
    void handle_message(const mavlink_message_t &msg);
    void mission_trigger(uint8_t dest_num);
    void check_avoidance();
    void check_sensors();
    void check_spraying();

    static const struct AP_Param::GroupInfo var_info[];
protected:

    const char *name() const override { return "CNDN"; }
    const char *name4() const override { return "CNDN"; }

    uint32_t wp_distance() const override;
    int32_t wp_bearing() const override;
    float crosstrack_error() const override { return pos_control->crosstrack_error();}

#if PRECISION_LANDING == ENABLED
    bool do_precision_loiter();
    void precision_loiter_xy();
#endif
private:

#if PRECISION_LANDING == ENABLED
    bool _precision_loiter_enabled;
    bool _precision_loiter_active; // true if user has switched on prec loiter
#endif

private:
    void return_to_manual_control(bool maintain_target);

    void init_speed();
    void pos_control_start();
    void auto_control();
    void manual_control();
    void zigzag_manual();
    bool reached_destination();
    void set_yaw_state(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle);
    bool processArea();
    bool processAB();
    bool getResume(CNMIS& dat);
    void setResume(CNMIS& dat, bool bRemote = false);
    bool isOwnMission();
    bool hasResume(uint16_t &resumeIdx);

   enum cndn_state
    {
        MANUAL,         // pilot toggle the switch to middle position, has manual control
        PREPARE_AUTO,
        AUTO,
        SET_AUTO,
        PREPARE_ABLINE,
        FINISHED,
    } stage;

    float           yaw_deg = 0;
    uint32_t        reach_wp_time_ms = 0; // time since vehicle reached destination (or zero if not yet reached)
    uint8_t*        data_buff = NULL;
    uint16_t        data_size = 0;
    uint16_t        data_wpos = 0;
    uint8_t         data_sysid = 0;
    uint8_t         data_cmpid = 0;
    float           last_yaw_deg = 0.0f;
    bool            edge_mode = false;
    bool            m_bZigZag;
    uint8_t         cmd_mode;
    uint8_t         m_lockStick = 0;
    Vector2f        m_target_pos;
    uint32_t        _rate_dt = 0;
    Location        resumeLoc;
    CNTimeout       toYAW, toBAT;
    uint32_t        m_missionReset = 0;

    // parameters
    AP_Int8         _method;                ///< CNDN Method 0: Disable, 1: Take Picture, 2: Edge following and auto mission, 3: Mission 
    AP_Int16        _take_alt_cm;           ///< Takeoff Altitute
    AP_Int16        _mission_alt_cm;        ///< Mission altitute
    AP_Int16        _spray_width_cm;        ///< Spray width cm
    AP_Int16        _dst_eg_cm;             ///< Edge distance cm
    AP_Int16        _spd_edge_cm;           ///< Edge speed cm/s
    AP_Int16        _spd_auto_cm;           ///< Auto speed cm/s
    AP_Float        _radar_flt_hz;          ///< RADAR Lowpass filter apply frequency
    AP_Int16        _spray_backs;           ///< Sprayer minimum steps
    AP_Int8         _sensor_pin;            ///< CNDN Level sensor gpio pin
    AP_Int16        _avoid_cm;              ///< avoid 10cm
    AP_Int16        _mc8_option;            ///< CNDN Mission computer options
};

#endif