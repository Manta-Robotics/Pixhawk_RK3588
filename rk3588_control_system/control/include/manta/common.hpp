#pragma once
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace manta {
using Json = nlohmann::json;
constexpr double pi = 3.14159265358979323846;
constexpr double radians = pi / 180.0;
constexpr double missing = std::numeric_limits<double>::quiet_NaN();
inline double wrap(double x) { return x - 360.0 * std::floor((x + 180.0) / 360.0); }
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double number(const Json& j, const std::string& k, double fallback = missing) {
    const auto it = j.find(k);
    return it != j.end() && it->is_number() && std::isfinite(it->get<double>()) ? it->get<double>() : fallback;
}
inline Json nullable(double x) { return std::isfinite(x) ? Json(x) : Json(nullptr); }
inline double squared(double x) { return x * x; }
struct Vec {
    double x = 0, y = 0;
    Vec operator+(Vec b) const { return {x+b.x, y+b.y}; }
    Vec operator-(Vec b) const { return {x-b.x, y-b.y}; }
    Vec operator*(double s) const { return {x*s, y*s}; }
    Vec operator/(double s) const { return *this * (1/s); }
    double norm() const { return std::hypot(x,y); }
    double angle() const { return std::atan2(y,x)/radians; }
};
struct Point { double at; Vec z; double variance = 0; };
struct Fit { Vec position, velocity; double denominator, mean_time; };
inline std::optional<Fit> fit(const std::vector<Point>& points, double at) {
    if (points.empty()) return {};
    double mt=0, denominator=0; Vec mean, numerator;
    for (const auto& p:points) { mt+=(p.at-at)/1000; mean=mean+p.z; }
    mt/=points.size(); mean=mean/double(points.size());
    for (const auto& p:points) {
        const double d=(p.at-at)/1000-mt;
        denominator+=d*d; numerator=numerator+(p.z-mean)*d;
    }
    if (denominator<=0) return {};
    const Vec velocity=numerator/denominator;
    return Fit{mean-velocity*mt,velocity,denominator,mt};
}
inline void trim(std::vector<Point>& points, double at, double age, size_t count) {
    std::erase_if(points,[=](const Point& p){return at<p.at || at-p.at>age;});
    if(points.size()>count) points.erase(points.begin(),points.end()-count);
}
}
