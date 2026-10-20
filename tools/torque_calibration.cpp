// tools/torque_calibration.cpp
//
// C++ port of read_loadcell.py: torque sensor calibration test. You tell it the real weight and the lever
// arm, it records the sensor for each trial and saves a CSV. Same workflow, same question order, same CSV
// layout as the Python original -- just compiled, and reusing TorqueSensor (include/torquesensor.hpp), the
// same class the daemon uses to log torque_nm into its own CSVs (see main.cpp --torque / --test-torque).
//
// You are asked for:
//   1. the lever arm length (distance from the sensor axis to the line of the hanging weight)
//   2. how many trials to do for each weight
//   3. the REAL weight of each load you attach (from a scale), one load after another; type 'done' when finished
//
// For every trial the amplifier is read over Modbus (averaging a few samples) and a row is appended to a CSV
// file (one file per run, named loadcell_test_<date>_<time>.csv, written after every trial so nothing is lost).
//
// Decimals are accepted everywhere, with a point or a comma (150.5, 150,5mm, 0.25kg, 0,25kg).
//
// Run it from a real terminal:                              ./torque_calibration
// Watch the zero / noise / drift live (nothing attached):    ./torque_calibration --watch           (add --duration=120 to stop by itself)
// Scan registers to find the right one (apply/remove load):  ./torque_calibration --probe
// Rehearsal without hardware:                                ./torque_calibration --simulate   (also works with --watch)
//
// Build:  g++ -std=c++17 -O2 -I../include -o torque_calibration torque_calibration.cpp

#include "torquesensor.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// ------------------------------------------------------------------------ settings (same defaults as the original script)
struct Settings {
    std::string port = "/dev/ttyUSB0";
    int baud = 9600;
    int slave_id = 1;
    unsigned reg_addr = 0;              // PLACEHOLDER: take from the manual's register map (find it with --probe)
    unsigned reg_count = 2;             // PLACEHOLDER: depends on 16-bit vs 32-bit weight
    std::string value_format = "int32"; // int16 | uint16 | int32 | uint32 | float32
    std::string word_order = "big";     // 32-bit values: "big" = first register holds the high word
    double scale = 0.001;               // reading = decoded number * scale (e.g. 0.001 if the amplifier counts in mN*m)
    double offset = 0.0;
    std::string unit = "Nm";
    double sensor_capacity_nm = 1.0;    // PLACEHOLDER: the capacity of YOUR sensor; used for an overload warning
    double settle_s = 3.0;              // wait this long after you press Enter, so the reading settles
    int n_samples = 10;                 // readings averaged for one trial
    double sample_interval_s = 0.2;
    bool simulate = false;
    bool watch = false;
    bool probe = false;
    double watch_duration_s = -1.0;     // < 0 = until Ctrl+C
};

constexpr double G_GRAVITY = 9.80665;   // local gravity, m/s^2

struct FixedItem { std::string name; double mass_g; double dist_mm; };
// Parts that stay on the sensor and add their own torque (used only if you zeroed the bare sensor before mounting them).
static std::vector<FixedItem> default_fixed_items() {
    return {{"printed lever arm", 9.01, 43.91}, {"nut and bolt at the load point", 2.4, 102.0}};
}

// ------------------------------------------------------------------------ small helpers
static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

static std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

// float() that also accepts a decimal comma: "0,25" -> 0.25
static double num(const std::string& text) {
    std::string t = trim(text);
    std::replace(t.begin(), t.end(), ',', '.');
    size_t pos = 0;
    const double v = std::stod(t, &pos);
    if (pos != t.size()) throw std::invalid_argument("not a number: " + text);
    return v;
}

static bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static std::string strip_spaces(std::string s) {
    s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
    return s;
}

// "150.5", "150,5mm", "15.5cm", "0.155m" -> metres. A bare number is ALWAYS millimetres.
static double parse_length_m(const std::string& text) {
    std::string t = lower(strip_spaces(trim(text)));
    std::replace(t.begin(), t.end(), ',', '.');
    if (ends_with(t, "mm")) return num(t.substr(0, t.size() - 2)) * 0.001;
    if (ends_with(t, "cm")) return num(t.substr(0, t.size() - 2)) * 0.01;
    if (ends_with(t, "m")) return num(t.substr(0, t.size() - 1)) * 1.0;
    return num(t) / 1000.0;
}

// "250", "250.5g", "0,25kg" -> (grams, was_ambiguous). A bare number is grams.
static std::pair<double, bool> parse_weight_g(const std::string& text) {
    std::string t = lower(strip_spaces(trim(text)));
    if (ends_with(t, "kg")) return {num(t.substr(0, t.size() - 2)) * 1000.0, false};
    if (ends_with(t, "g")) return {num(t.substr(0, t.size() - 1)), false};
    const double v = num(t);
    return {v, (v > 0.0 && v < 10.0)};
}

static std::string ask_text(const std::string& prompt) {
    std::cout << prompt << std::flush;
    std::string line;
    if (!std::getline(std::cin, line)) throw std::runtime_error("EOF");
    return trim(line);
}

static bool ask_yes(const std::string& prompt, bool def = false) {
    const std::string s = lower(ask_text(prompt + (def ? " [Y/n]: " : " [y/N]: ")));
    if (s.empty()) return def;
    return s[0] == 'y';
}

static int ask_int(const std::string& prompt, int def) {
    while (true) {
        const std::string s = ask_text(prompt + " [" + std::to_string(def) + "]: ");
        if (s.empty()) return def;
        try {
            const double f = num(s);
            if (f >= 1 && f == static_cast<int>(f)) return static_cast<int>(f);
        } catch (...) {}
        std::cout << "  please enter a whole number, 1 or more (a trial count can't be a fraction)\n";
    }
}

static std::string now_stamp() {
    const auto t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    return buf;
}

static std::string now_iso() {
    const auto t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
    return buf;
}

// "How was the amplifier zeroed (tared)?" -> (baseline torque in Nm, description text)
static std::pair<double, std::string> ask_zero_method() {
    std::cout << "\nHow was the amplifier zeroed (tared)?\n"
              << "  1 = WITH the lever arm, nut and bolt mounted and nothing hanging  -> their torque is cancelled; enter only what you hang on it  (recommended)\n"
              << "  2 = with the BARE sensor, before the arm was mounted             -> the arm's own torque is part of every reading\n";
    std::string choice;
    while (true) {
        choice = ask_text("Choice [1]: ");
        if (choice.empty()) choice = "1";
        if (choice == "1" || choice == "2") break;
        std::cout << "  please type 1 or 2\n";
    }
    if (choice == "1") return {0.0, "zeroed with the arm mounted (arm torque cancelled)"};

    auto items = default_fixed_items();
    std::cout << "  Parts left on the sensor:\n";
    for (const auto& it : items) {
        const double tq = it.mass_g / 1000.0 * G_GRAVITY * it.dist_mm / 1000.0;
        std::cout << "    " << it.name << ": " << it.mass_g << " g at " << it.dist_mm << " mm = " << tq << " N.m\n";
    }
    if (!ask_yes("  Are these the right parts and numbers?", true)) {
        items.clear();
        std::cout << "  Enter each part as: mass_in_g distance_from_axis_in_mm  (for example  9.01 43.91). Press Enter on an empty line to finish.\n";
        while (true) {
            const std::string t = ask_text("    part: ");
            if (t.empty()) break;
            try {
                std::string tt = t;
                std::replace(tt.begin(), tt.end(), ';', ' ');
                std::istringstream iss(tt.find(',') != std::string::npos && tt.find(' ') == std::string::npos
                                            ? [&] { std::string c = tt; std::replace(c.begin(), c.end(), ',', ' '); return c; }()
                                            : tt);
                double m, d;
                if (!(iss >> m >> d)) throw std::invalid_argument("bad part line");
                items.push_back({"part", m, d});
            } catch (...) {
                std::cout << "    type two numbers, e.g.  2.4 102\n";
            }
        }
    }
    double base = 0.0;
    for (const auto& it : items) base += it.mass_g / 1000.0 * G_GRAVITY * it.dist_mm / 1000.0;
    std::cout << "  -> torque already on the sensor at zero added weight: " << base << " N.m (added to every expected torque)\n";
    std::ostringstream desc;
    desc << "zeroed bare; fixed parts add " << base << " N.m";
    return {base, desc.str()};
}

// ------------------------------------------------------------------------ sensor backend (real hardware, or --simulate)
class ISensorBackend {
public:
    virtual ~ISensorBackend() = default;
    virtual bool available() const = 0;
    virtual bool read(double& torque_out) = 0;
    virtual bool read_raw(unsigned addr, unsigned count, std::vector<uint16_t>& out) const = 0;
    virtual void set_simulated_torque(double) {}    // no-op on the real backend; --simulate uses this to declare "what's hanging now"
};

class RealBackend : public ISensorBackend {
    TorqueSensor sensor_;
public:
    explicit RealBackend(const TorqueSensor::Settings& s) : sensor_(s) {}
    bool available() const override { return sensor_.available(); }
    bool read(double& t) override { return sensor_.read(t); }
    bool read_raw(unsigned a, unsigned c, std::vector<uint16_t>& out) const override { return sensor_.read_raw(a, c, out); }
};

// Stand-in for --simulate: reads roughly the torque you declared, with a little gain error, offset and noise
// (same model as the Python FakeClient: v = 1.004*T + 0.003 + 0.000004*reads + gauss(0, 0.0007)).
class FakeBackend : public ISensorBackend {
    double torque_ = 0.0;
    long reads_ = 0;
    std::mt19937 rng_{std::random_device{}()};
public:
    bool available() const override { return true; }
    void set_simulated_torque(double t) override { torque_ = t; }
    bool read(double& out) override {
        ++reads_;
        std::normal_distribution<double> noise(0.0, 0.0007);
        out = 1.004 * torque_ + 0.003 + 0.000004 * static_cast<double>(reads_) + noise(rng_);
        return true;
    }
    bool read_raw(unsigned, unsigned, std::vector<uint16_t>&) const override { return false; }   // --probe needs real hardware
};

// ------------------------------------------------------------------------ one averaged measurement
struct Measurement { double mean; double sd; int n; };

static std::optional<Measurement> measure(ISensorBackend& sensor, const Settings& cfg) {
    std::vector<double> vals;
    for (int i = 0; i < cfg.n_samples; ++i) {
        double v = 0.0;
        if (sensor.read(v)) vals.push_back(v);
        if (!cfg.simulate) std::this_thread::sleep_for(std::chrono::duration<double>(cfg.sample_interval_s));
    }
    if (static_cast<int>(vals.size()) < std::max(3, cfg.n_samples / 2)) return std::nullopt;
    double mean = 0.0; for (double v : vals) mean += v; mean /= static_cast<double>(vals.size());
    double sd = 0.0; for (double v : vals) sd += (v - mean) * (v - mean);
    sd = vals.size() > 1 ? std::sqrt(sd / static_cast<double>(vals.size())) : 0.0;
    return Measurement{mean, sd, static_cast<int>(vals.size())};
}

// ------------------------------------------------------------------------ one trial row (for the CSV + the final summary)
struct TrialRow {
    std::string timestamp;
    double weight_g;
    double lever_arm_mm;
    int trial;
    double baseline_torque_nm;
    double expected_torque_nm;
    double reading;
    double reading_std;
    int n_samples;
    std::string source;   // "modbus" | "simulated" | "typed"
};

static void write_header_if_new(std::ofstream& f, bool is_new) {
    if (!is_new) return;
    f << "timestamp,weight_g,lever_arm_mm,trial,baseline_torque_Nm,expected_torque_Nm,reading,reading_std,n_samples,source,unit\n";
}

static void write_row(std::ofstream& f, const TrialRow& r, const std::string& unit) {
    f << r.timestamp << ',' << r.weight_g << ',' << r.lever_arm_mm << ',' << r.trial << ','
      << r.baseline_torque_nm << ',' << r.expected_torque_nm << ',' << r.reading << ',' << r.reading_std << ','
      << r.n_samples << ',' << r.source << ',' << unit << '\n';
    f.flush();
}

// ------------------------------------------------------------------------ summary at the end
static void summarise(const std::vector<TrialRow>& rows, const std::string& unit) {
    if (rows.empty()) return;
    std::cout << "\n================ SUMMARY ================\n";
    std::map<double, std::vector<TrialRow>> groups;
    for (const auto& r : rows) groups[r.weight_g].push_back(r);

    std::cout << std::right << std::setw(15) << "weight g" << std::setw(7) << "trials"
              << std::setw(22) << ("expected " + unit) << std::setw(22) << "mean reading" << std::setw(22) << "spread (std)\n";

    std::vector<std::pair<double, double>> pts;   // (expected, mean reading) per weight, for the linear fit
    for (const auto& [w, rs] : groups) {
        double mean = 0.0; for (const auto& r : rs) mean += r.reading; mean /= static_cast<double>(rs.size());
        double sd = 0.0; for (const auto& r : rs) sd += (r.reading - mean) * (r.reading - mean);
        sd = rs.size() > 1 ? std::sqrt(sd / static_cast<double>(rs.size())) : 0.0;
        const double T = rs.front().expected_torque_nm;
        std::cout << std::setw(15) << w << std::setw(7) << rs.size() << std::setw(22) << T << std::setw(22) << mean << std::setw(22) << sd << '\n';
        pts.emplace_back(T, mean);
    }

    if (pts.size() >= 3) {
        const auto n = static_cast<double>(pts.size());
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (const auto& [x, y] : pts) { sx += x; sy += y; sxx += x * x; sxy += x * y; }
        const double den = n * sxx - sx * sx;
        if (den > 0) {
            const double gain = (n * sxy - sx * sy) / den;
            const double off = (sy - gain * sx) / n;
            const double ybar = sy / n;
            double ss_tot = 0, ss_res = 0;
            for (const auto& [x, y] : pts) { ss_tot += (y - ybar) * (y - ybar); const double pred = gain * x + off; ss_res += (y - pred) * (y - pred); }
            const double r2 = ss_tot > 0 ? 1.0 - ss_res / ss_tot : std::numeric_limits<double>::quiet_NaN();
            std::cout << "\nStraight-line fit of the weight averages:  reading = " << gain << " x torque " << (off >= 0 ? "+" : "") << off
                      << "   (R2 = " << r2 << ")\n"
                      << "  gain error " << std::abs(gain - 1.0) * 100.0 << " % and zero offset " << (off >= 0 ? "+" : "") << off << " " << unit
                      << " (the ideal is gain 1.0, offset 0 if the reading is in " << unit << ")\n";
        }
    }
}

// ------------------------------------------------------------------------ --probe: scan registers to find the right one
static void run_probe(const Settings& cfg) {
    if (cfg.simulate) { std::cout << "--probe needs real hardware; it scans live registers.\n"; return; }
    TorqueSensor::Settings s;
    s.port = cfg.port; s.baud = cfg.baud; s.slave_id = cfg.slave_id; s.timeout_ms = 300;
    s.reg_addr = 0; s.reg_count = 1;
    TorqueSensor sensor(s);
    std::cout << "=== Register probe on " << cfg.port << ", slave " << cfg.slave_id << " ===\n"
              << "Scanning registers 0-19 (one at a time, function 0x03). Apply and remove a known weight while\n"
              << "watching which register changes -- that is your REG_ADDR. A register whose value does not move\n"
              << "at all is probably unrelated (temperature, status flags, a constant).\n\n"
              << "Press Enter to take one scan, or type 'q' then Enter to stop.\n";
    while (true) {
        const std::string in = ask_text("> ");
        if (!in.empty() && (in[0] == 'q' || in[0] == 'Q')) break;
        std::cout << std::left << std::setw(5) << "reg" << std::setw(10) << "raw(hex)" << std::setw(10) << "raw(dec)"
                  << std::setw(14) << "as int16" << std::setw(16) << "as int32(+next)" << "as float32(+next)\n";
        for (unsigned addr = 0; addr < 20; ++addr) {
            std::vector<uint16_t> r1, r2;
            const bool ok1 = sensor.read_raw(addr, 1, r1);
            const bool ok2 = sensor.read_raw(addr, 2, r2);
            if (!ok1 && !ok2) continue;    // this address doesn't answer at all: skip it, rather than print noise
            std::cout << std::left << std::setw(5) << addr;
            if (ok1) {
                char hexbuf[8];
                std::snprintf(hexbuf, sizeof(hexbuf), "0x%04X", r1[0]);
                std::cout << std::setw(10) << hexbuf << std::setw(10) << r1[0]
                          << std::setw(14) << sensor.decode_as(r1, "int16", "big");
            } else {
                std::cout << std::setw(10) << "-" << std::setw(10) << "-" << std::setw(14) << "-";
            }
            if (ok2) {
                std::cout << std::setw(16) << sensor.decode_as(r2, "int32", "big") << sensor.decode_as(r2, "float32", "big");
            } else {
                std::cout << std::setw(16) << "-" << "-";
            }
            std::cout << '\n';
        }
        std::cout << '\n';
    }
}

// ------------------------------------------------------------------------ --watch: is the zero stable?
static void run_watch(ISensorBackend& sensor, const Settings& cfg) {
    const std::string out = "loadcell_watch_" + now_stamp() + ".csv";
    std::cout << "=== Live reading: leave the sensor completely untouched (no hand on the arm, no moving cables) ===\n"
              << "Ctrl+C stops it" << (cfg.watch_duration_s > 0 ? (" (or it stops after " + std::to_string(cfg.watch_duration_s) + " s)") : "")
              << ". Every sample is saved to " << out << "\n\n";
    std::ofstream f(out);
    f << "t_s,reading," << cfg.unit << "\n";   // header kept close to the Python version's intent; raw registers are not reliably available on the simulated backend

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::pair<double, double>> data;   // (t, v)
    double last_print = 0.0;
    long bad = 0;
    try {
        while (true) {
            const double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (cfg.watch_duration_s > 0 && t >= cfg.watch_duration_s) break;
            double v = 0.0;
            if (sensor.read(v)) {
                data.emplace_back(t, v);
                f << std::fixed << std::setprecision(6) << t << ',' << v << '\n';
                f.flush();
                if (t - last_print >= 1.0) {
                    last_print = t;
                    const size_t from = data.size() > 20 ? data.size() - 20 : 0;
                    double m = 0.0; for (size_t i = from; i < data.size(); ++i) m += data[i].second; m /= static_cast<double>(data.size() - from);
                    double sdv = 0.0; for (size_t i = from; i < data.size(); ++i) sdv += (data[i].second - m) * (data[i].second - m);
                    sdv = std::sqrt(sdv / static_cast<double>(data.size() - from));
                    std::cout << "  " << static_cast<int>(t) / 60 << ":" << std::setw(2) << std::setfill('0') << static_cast<int>(t) % 60 << std::setfill(' ')
                              << "   " << std::fixed << std::setprecision(5) << v << " " << cfg.unit
                              << "    last " << (data.size() - from) << ": mean " << m << "  std " << sdv << '\n';
                }
            } else {
                ++bad;
            }
            std::this_thread::sleep_for(std::chrono::duration<double>(cfg.simulate ? 0.02 : 0.2));
        }
    } catch (...) {}

    if (data.size() < 5) {
        std::cout << "\nToo few readings (" << data.size() << ", " << bad << " errors) to summarise. Check the wiring and the register settings.\n";
        return;
    }
    const double n = static_cast<double>(data.size());
    double tm = 0.0, vm = 0.0;
    for (const auto& [t, v] : data) { tm += t; vm += v; }
    tm /= n; vm /= n;
    double num_ = 0.0, den = 0.0, vsd = 0.0, vmin = data[0].second, vmax = data[0].second;
    for (const auto& [t, v] : data) {
        num_ += (t - tm) * (v - vm);
        den += (t - tm) * (t - tm);
        vsd += (v - vm) * (v - vm);
        vmin = std::min(vmin, v);
        vmax = std::max(vmax, v);
    }
    const double slope = den > 1e-12 ? num_ / den : 0.0;
    vsd = std::sqrt(vsd / n);
    const double duration = data.back().first;

    std::cout << "\n================ ZERO CHECK ================\n"
              << data.size() << " readings over " << duration << " s (" << bad << " errors)\n"
              << "mean " << vm << " " << cfg.unit << "     std (noise) " << vsd << "     min " << vmin << "     max " << vmax << '\n'
              << "drift: " << (slope * 60.0) << " " << cfg.unit << " per minute   (total change over the run about " << (slope * duration) << ")\n";
    if (cfg.sensor_capacity_nm > 0) {
        std::cout << "as a share of the " << cfg.sensor_capacity_nm << " " << cfg.unit << " capacity: offset "
                  << std::abs(vm) / cfg.sensor_capacity_nm * 100.0 << " % FS, noise " << vsd / cfg.sensor_capacity_nm * 100.0
                  << " % FS (datasheet: zero balance +-2 % FS, repeatability 0.05 % FS)\n";
    }
    std::cout << "A steady offset is fine: zero (tare) the amplifier with the lever arm and empty hanger mounted. "
              << "A reading that keeps wandering is the real problem:\n"
              << "let the sensor and amplifier warm up for 15-30 minutes, keep the cable still, and repeat.\n";
}

// ------------------------------------------------------------------------ the interactive calibration run
static void run_main(ISensorBackend& sensor, const Settings& cfg) {
    std::cout << "=== Torque sensor test ===\n";
    double L = 0.0;
    while (true) {
        try {
            L = parse_length_m(ask_text("Lever arm length (distance from the sensor axis to the hanging weight), e.g. 150, 150.5mm, 15,5cm or 0.155m (bare number = mm): "));
            if (L > 0) break;
        } catch (...) {}
        std::cout << "  please type a length such as 150.5mm\n";
    }
    std::cout << "  -> lever arm = " << (L * 1000.0) << " mm (" << L << " m)\n";
    const int trials = ask_int("How many trials per weight?", 3);
    const auto [baseline, zero_text] = ask_zero_method();

    const std::string out = "loadcell_test_" + now_stamp() + ".csv";
    std::vector<TrialRow> rows;
    std::cout << "\nResults will be saved to: " << out << "\n"
              << "Zero convention: " << zero_text << ".\n"
              << "For each load, type the TOTAL mass of everything that is hanging and was NOT there when you zeroed "
              << "(for example your water bottle + string, weighed on the scale).\n";
    try {
        ask_text("Press Enter when you are ready to start ... ");
        while (true) {
            const std::string s = lower(ask_text("\nReal weight of the load now attached (from your scale, e.g. 250, 250.5g or 0.25kg), or 'done': "));
            if (s == "done" || s == "d" || s == "q" || s == "quit" || s == "exit") break;
            double w_g = 0.0;
            bool ambiguous = false;
            try {
                std::tie(w_g, ambiguous) = parse_weight_g(s);
            } catch (...) {
                std::cout << "  please type a weight such as 250 (grams) or 0.25kg\n";
                continue;
            }
            if (w_g < 0) { std::cout << "  the weight cannot be negative\n"; continue; }
            if (ambiguous && !ask_yes("  You typed " + s + " with no unit: is that " + std::to_string(w_g) + " grams?")) {
                std::cout << "  please type the weight again, with a unit (for example 0.5kg or 500g)\n";
                continue;
            }
            const double T = w_g / 1000.0 * G_GRAVITY * L + baseline;
            std::cout << "  -> " << w_g << " g at " << (L * 1000.0) << " mm = " << T << " N.m expected";
            if (baseline != 0.0) std::cout << " (includes " << baseline << " N.m from the arm and parts)";
            std::cout << " (" << (T / cfg.sensor_capacity_nm * 100.0) << " % of a " << cfg.sensor_capacity_nm << " N.m sensor)\n";
            if (T > cfg.sensor_capacity_nm && !ask_yes("  !!! This is ABOVE the sensor capacity (120 % is the safe overload). Continue anyway?")) continue;

            sensor.set_simulated_torque(T);
            for (int k = 1; k <= trials; ++k) {
                ask_text("  Trial " + std::to_string(k) + " of " + std::to_string(trials) +
                          ": attach the load (re-attach it for each trial if you want to test repeatability), then press Enter to record ... ");
                if (!cfg.simulate) {
                    std::cout << "    settling " << cfg.settle_s << " s ...\n";
                    std::this_thread::sleep_for(std::chrono::duration<double>(cfg.settle_s));
                }
                const auto res = measure(sensor, cfg);
                TrialRow row;
                row.timestamp = now_iso();
                row.weight_g = w_g;
                row.lever_arm_mm = L * 1000.0;
                row.trial = k;
                row.baseline_torque_nm = baseline;
                row.expected_torque_nm = T;
                if (!res) {
                    std::cout << "    !! no usable reply from the amplifier (check wiring, baud rate, slave ID, register settings).\n";
                    double v = std::numeric_limits<double>::quiet_NaN();
                    try { v = num(ask_text("    Type the value the amplifier shows in " + cfg.unit + " (or just press Enter to skip this trial): ")); }
                    catch (...) {}
                    if (!std::isfinite(v)) continue;
                    row.reading = v; row.reading_std = 0.0; row.n_samples = 1; row.source = "typed";
                } else {
                    row.reading = res->mean; row.reading_std = res->sd; row.n_samples = res->n;
                    row.source = cfg.simulate ? "simulated" : "modbus";
                }
                std::cout << "    reading = " << row.reading << " " << cfg.unit;
                if (row.n_samples > 1) std::cout << " (+-" << row.reading_std << ", " << row.n_samples << " samples)";
                if (cfg.unit == "Nm" || cfg.unit == "N.m") {
                    std::cout << "   expected " << T << "   difference " << (row.reading - T >= 0 ? "+" : "") << (row.reading - T);
                }
                std::cout << '\n';
                rows.push_back(row);
                const bool is_new = !std::ifstream(out).good();
                std::ofstream f(out, std::ios::app);
                write_header_if_new(f, is_new);
                write_row(f, row, cfg.unit);
            }
        }
    } catch (...) {
        std::cout << "\nStopped.\n";
    }
    summarise(rows, cfg.unit);
    if (!rows.empty()) std::cout << "\n" << rows.size() << " trial(s) saved to " << out << "\n";
    else std::cout << "\nNothing was recorded.\n";
}

// ------------------------------------------------------------------------ main
int main(int argc, char** argv) {
    Settings cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--simulate") cfg.simulate = true;
        else if (a == "--watch") cfg.watch = true;
        else if (a == "--probe") cfg.probe = true;
        else if (a.rfind("--port=", 0) == 0) cfg.port = a.substr(7);
        else if (a.rfind("--baud=", 0) == 0) cfg.baud = std::stoi(a.substr(7));
        else if (a.rfind("--slave=", 0) == 0) cfg.slave_id = std::stoi(a.substr(8));
        else if (a.rfind("--reg=", 0) == 0) cfg.reg_addr = static_cast<unsigned>(std::stoul(a.substr(6)));
        else if (a.rfind("--regcount=", 0) == 0) cfg.reg_count = static_cast<unsigned>(std::stoul(a.substr(11)));
        else if (a.rfind("--format=", 0) == 0) cfg.value_format = a.substr(9);
        else if (a.rfind("--word-order=", 0) == 0) cfg.word_order = a.substr(13);
        else if (a.rfind("--scale=", 0) == 0) cfg.scale = std::stod(a.substr(8));
        else if (a.rfind("--offset=", 0) == 0) cfg.offset = std::stod(a.substr(9));
        else if (a.rfind("--capacity=", 0) == 0) cfg.sensor_capacity_nm = std::stod(a.substr(11));
        else if (a.rfind("--settle=", 0) == 0) cfg.settle_s = std::stod(a.substr(9));
        else if (a.rfind("--samples=", 0) == 0) cfg.n_samples = std::stoi(a.substr(10));
        else if (a.rfind("--interval=", 0) == 0) cfg.sample_interval_s = std::stod(a.substr(11));
        else if (a.rfind("--duration=", 0) == 0) cfg.watch_duration_s = std::stod(a.substr(11));
        else if (a == "--help" || a == "-h") {
            std::cout << "Usage: torque_calibration [--simulate] [--watch [--duration=S]] [--probe]\n"
                      << "                           [--port=/dev/ttyUSB0] [--baud=9600] [--slave=1]\n"
                      << "                           [--reg=0] [--regcount=2] [--format=int32] [--word-order=big]\n"
                      << "                           [--scale=0.001] [--offset=0] [--capacity=1.0]\n"
                      << "                           [--settle=3] [--samples=10] [--interval=0.2]\n";
            return 0;
        }
    }
    if (cfg.simulate) std::cout << "*** SIMULATION: no real sensor is being read ***\n\n";

    if (cfg.probe) { run_probe(cfg); return 0; }

    std::unique_ptr<ISensorBackend> sensor;
    if (cfg.simulate) {
        sensor = std::make_unique<FakeBackend>();
    } else {
        TorqueSensor::Settings s;
        s.port = cfg.port; s.baud = cfg.baud; s.slave_id = cfg.slave_id;
        s.reg_addr = cfg.reg_addr; s.reg_count = cfg.reg_count;
        s.value_format = cfg.value_format; s.word_order = cfg.word_order;
        s.scale = cfg.scale; s.offset = cfg.offset; s.unit = cfg.unit;
        sensor = std::make_unique<RealBackend>(s);
        if (!sensor->available()) {
            std::cerr << "Could not get a valid response from the amplifier on " << cfg.port << ".\n"
                      << "  Check the port name, baud rate, slave ID and register settings, or try --simulate for a rehearsal.\n"
                      << "  'sudo fuser -v " << cfg.port << "' shows if something else still has the port open.\n";
            return 1;
        }
    }

    if (cfg.watch) run_watch(*sensor, cfg);
    else run_main(*sensor, cfg);
    return 0;
}