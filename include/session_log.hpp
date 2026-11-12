// include/session_log.hpp
// Mode 1 session recorder: collects one row per control loop while the motor is started from the knob / KO key, and writes a .csv and a
// .mat file when the run ends (KO stop, safety trip, dashboard connects, MATLAB takes over, daemon exit).
// The columns, their order, the file names and the .mat variable names are those of the dashboard's export, so the same scripts read both.
// The .mat file is a MATLAB Level 5 file written here directly (no library): one double column vector per variable plus the impeller tag.
#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace SessionLog {

constexpr size_t NCOL = 13;
constexpr size_t MAX_ROWS = 1000000;                       // 100 Hz -> 2.8 h; beyond this the run keeps going but is no longer recorded

using Row = std::array<double, NCOL>;

inline const char* const CSV_HEADERS[NCOL] = {"t (s)", "Raw RPM", "Filtered RPM", "Revolutions", "Torque", "Power (W)", "N_Re", "N_Po",
                                              "PWM (%)", "Target RPM", "Bus (V)", "Current (mA)", "Elec. Power (W)"};
inline const char* const MAT_NAMES[NCOL] = {"time_s", "Raw_RPM", "Filtered_RPM", "Revolutions", "Torque_Nm", "Power_W", "N_Re", "N_Po",
                                            "PWM_pct", "Target_RPM", "Bus_V", "Current_mA", "Elec_Power_W"};

// Re = rho n D^2 / mu, Po = P / (rho n^3 D^5), P = torque * 2 pi n   (same as the dashboard; a missing torque gives power and Po 0)
inline void metrics(double rpm, double torque_nm, double rho, double mu, double d, double& power_w, double& n_re, double& n_po) {
    const double n = rpm / 60.0;
    power_w = std::isfinite(torque_nm) ? torque_nm * n * 2.0 * 3.14159265358979323846 : 0.0;
    n_re = n > 0 ? rho * n * d * d / mu : 0.0;
    n_po = n > 0 ? power_w / (rho * n * n * n * std::pow(d, 5)) : 0.0;
}

inline std::string timestamp_now() {
    std::time_t t = std::time(nullptr); std::tm tmv{};
    localtime_r(&t, &tmv);
    char b[32]; std::strftime(b, sizeof b, "%Y%m%d_%H%M%S", &tmv);
    return b;
}

// ---- MATLAB Level 5 writer ----
namespace mat {
inline void put32(std::string& s, uint32_t v) { for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFF)); }
inline void put16(std::string& s, uint16_t v) { s.push_back(static_cast<char>(v & 0xFF)); s.push_back(static_cast<char>(v >> 8)); }
inline void pad8(std::string& s) { while (s.size() % 8) s.push_back('\0'); }
// one sub-element: tag (type, bytes) + data, padded to 8 bytes
inline void element(std::string& s, uint32_t type, const std::string& data) { put32(s, type); put32(s, static_cast<uint32_t>(data.size())); s += data; pad8(s); }

inline std::string header() {
    std::string h = "MATLAB 5.0 MAT-file, written by mixr1_daemon (Mode 1 session log)";
    h.resize(116, ' ');
    h.append(8, '\0');                    // subsystem data offset
    put16(h, 0x0100);                     // version
    h += "IM";                            // little-endian marker
    return h;
}
// miMATRIX holding an array of class `cls` (6 = double, 4 = char) with data of type `dtype`
inline void matrix(std::string& out, const std::string& name, uint32_t cls, uint32_t rows, uint32_t cols, uint32_t dtype, const std::string& data) {
    std::string body;
    std::string flags; put32(flags, cls); put32(flags, 0); element(body, 6, flags);          // array flags (miUINT32)
    std::string dims; put32(dims, rows); put32(dims, cols); element(body, 5, dims);          // dimensions (miINT32)
    element(body, 1, name);                                                                  // name (miINT8)
    element(body, dtype, data);                                                              // real part
    put32(out, 14); put32(out, static_cast<uint32_t>(body.size())); out += body;             // miMATRIX
}
inline std::string doubles(const std::vector<Row>& rows, size_t col) {
    std::string d; d.reserve(rows.size() * 8);
    for (const auto& r : rows) { double v = r[col]; char b[8]; std::memcpy(b, &v, 8); d.append(b, 8); }   // host is little-endian (Raspberry Pi)
    return d;
}
}  // namespace mat

inline bool write_csv(const std::string& path, const std::vector<Row>& rows, const std::string& tag) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    for (size_t c = 0; c < NCOL; ++c) std::fprintf(f, "%s,", CSV_HEADERS[c]);
    std::fprintf(f, "impeller,mode\n");
    static const int prec[NCOL] = {2, 2, 2, 0, 3, 3, 2, 3, 2, 2, 2, 0, 2};
    for (const auto& r : rows) {
        for (size_t c = 0; c < NCOL; ++c) {
            if (std::isfinite(r[c])) std::fprintf(f, "%.*f,", prec[c], r[c]); else std::fprintf(f, "nan,");
        }
        std::fprintf(f, "%s,MODE1\n", tag.c_str());
    }
    return std::fclose(f) == 0;
}

inline bool write_mat(const std::string& path, const std::vector<Row>& rows, const std::string& tag) {
    std::string out = mat::header();
    for (size_t c = 0; c < NCOL; ++c) mat::matrix(out, MAT_NAMES[c], 6, static_cast<uint32_t>(rows.size()), 1, 9, mat::doubles(rows, c));
    std::string chars; for (char ch : tag) mat::put16(chars, static_cast<uint8_t>(ch));
    mat::matrix(out, "Impeller", 4, 1, static_cast<uint32_t>(tag.size()), 4, chars);             // char array, stored as miUINT16
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
    return std::fclose(f) == 0 && ok;
}

// Writes <dir>/mixr1_<tag>_<stamp>_mode1.csv and .mat. Returns the CSV path (empty on failure).
inline std::string save(const std::string& dir, const std::string& tag, const std::string& stamp, const std::vector<Row>& rows) {
    if (rows.empty()) return "";
    std::error_code ec; std::filesystem::create_directories(dir, ec);
    const std::string base = dir + "/mixr1_" + tag + "_" + stamp + "_mode1";
    const bool a = write_csv(base + ".csv", rows, tag), b = write_mat(base + ".mat", rows, tag);
    return (a && b) ? base + ".csv" : "";
}

}  // namespace SessionLog
