#pragma once

#ifndef __MODE_CNDN_HPP__
#define __MODE_CNDN_HPP__

#include    "Copter.h"

#define CASE_MODE_NUMBER_CNDN()     \
        case Mode::Number::CNDN:    \
            ret = &mode_cndn;       \
            break;

class ModeCNDN : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LOITER; }

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
    void check_avoidance();
    void check_sensors();
    void check_spraying();

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

};

#endif