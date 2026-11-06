// include/power_limiter.hpp
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include "config.hpp"

// Safety cap on the motor's power (Config::POWER_LIMIT_W, 72 W).
//
// Feed it every NEW power reading (electrical power from the wattmeter, or torque x speed when there is no wattmeter) together
// with the PWM that is applied right now. It returns, through max_pwm(), the highest PWM the motor may get:
//   power > aim (92 % of the limit, ~66 W) -> the PWM ceiling is cut in proportion (aim / power), so the next readings come back to it.
//                              Regulating a little below 72 W leaves room for the ~30 ms the wattmeter needs per reading, so a short
//                              peak (start-up, a load change) still stays under the 72 W ceiling.
//   power < 95 % of the aim  -> the ceiling is slowly released again (POWER_LIMIT_RECOVER_PCT_S of full scale per second)
//   power > trip level (125 %) for POWER_TRIP_READINGS readings in a row (a short, a sudden overload)
//                            -> tripped(): the motor must be switched OFF and stays off until reset() (the operator stops the motor)
//   check_stall(): PWM > STALL_PWM_PCT but the shaft < STALL_RPM for STALL_TIME_S (a jammed impeller, which the power cap alone
//                            would otherwise hold at 72 W forever) -> tripped() as well
// In PI mode the ceiling goes into PIController::set_max_pwm(), whose anti-windup uses the same value, so the integrator does not
// wind up while the power is being limited. In open loop the PWM is simply clamped to it.
class PowerLimiter {
public:
    void reset() {
        cap_ = 4095.0;
        over_ = 0;
        tripped_ = false;
        limiting_ = false;
        primed_ = false;
        stall_timing_ = false;
        reason_ = "";
    }

    // Call every control loop with the applied PWM and the measured speed.
    void check_stall(int pwm_now, double rpm) {
        const auto now = std::chrono::steady_clock::now();
        const bool stalled = pwm_now > Config::STALL_PWM_PCT / 100.0 * 4095.0 && std::fabs(rpm) < Config::STALL_RPM;
        if (!stalled) { stall_timing_ = false; return; }
        if (!stall_timing_) { stall_timing_ = true; stall_since_ = now; return; }
        if (std::chrono::duration<double>(now - stall_since_).count() >= Config::STALL_TIME_S && !tripped_) {
            tripped_ = true;
            reason_ = "STALL";
        }
    }

    // power_w: the new reading (NaN = none). pwm_now: PWM counts applied while it was measured.
    void update(double power_w, int pwm_now) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = primed_ ? std::chrono::duration<double>(now - last_).count() : 0.0;
        last_ = now;
        primed_ = true;
        if (!std::isfinite(power_w)) return;
        last_power_ = power_w;

        const double limit = Config::POWER_LIMIT_W;
        if (power_w > limit * Config::POWER_TRIP_FACTOR) {
            if (++over_ >= Config::POWER_TRIP_READINGS && !tripped_) { tripped_ = true; reason_ = "OVERLOAD"; }
        } else {
            over_ = 0;
        }

        const double aim = limit * Config::POWER_LIMIT_AIM;
        if (power_w > aim) {
            const double base = std::min(cap_, static_cast<double>(std::max(pwm_now, 0)));
            cap_ = std::max(0.0, base * (aim / power_w));              // proportional cut: lands back at the aim
            limiting_ = true;
        } else if (power_w < 0.95 * aim) {
            cap_ = std::min(4095.0, cap_ + 4095.0 * Config::POWER_LIMIT_RECOVER_PCT_S / 100.0 * std::min(dt, 0.5));
            if (cap_ >= 4095.0) limiting_ = false;
        }
    }

    int max_pwm() const { return static_cast<int>(cap_); }
    bool limiting() const { return limiting_; }      // the ceiling is below full scale because of the power limit
    bool tripped() const { return tripped_; }
    double last_power() const { return last_power_; }
    const char* trip_reason() const { return reason_; }   // "OVERLOAD" or "STALL"

private:
    double cap_ = 4095.0;
    int over_ = 0;
    bool tripped_ = false;
    bool limiting_ = false;
    bool primed_ = false;
    double last_power_ = 0.0;
    std::chrono::steady_clock::time_point last_{};
    bool stall_timing_ = false;
    std::chrono::steady_clock::time_point stall_since_{};
    const char* reason_ = "";
};