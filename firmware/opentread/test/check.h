// check.h - minimal test macros (no framework). Host tests only.
// CHECK(cond), CHECK_EQ(a, b), CHECK_NEAR(a, b, tol). Failures print
// file:line and the values, and are counted; check::summary() returns the
// process exit code. Compare like-typed values (the build uses -Werror with
// -Wsign-compare / -Wconversion).
#pragma once

#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

namespace check {
inline long& passes() { static long n = 0; return n; }
inline long& failures() { static long n = 0; return n; }
template <class T> std::string str(const T& v) { std::ostringstream o; o.precision(9); o << +v; return o.str(); }
inline std::string str(const std::string& v) { return "\"" + v + "\""; }
inline std::string str(const char* v) { return str(std::string(v)); }
inline bool report(bool ok, const char* file, int line, const std::string& what) {
  if (ok) { ++passes(); return true; }
  ++failures();
  if (failures() <= 200) std::fprintf(stderr, "FAIL %s:%d: %s\n", file, line, what.c_str());
  return false;
}
inline int summary(const char* name) {
  std::printf("[%s] %ld checks passed, %ld failed\n", name, passes(), failures());
  return failures() == 0 ? 0 : 1;
}
}  // namespace check

#define CHECK(c) check::report(static_cast<bool>(c), __FILE__, __LINE__, "CHECK(" #c ")")
#define CHECK_EQ(a, b)                                                                           \
  ([&]() { const auto va_ = (a); const auto vb_ = (b);                                           \
     return check::report(va_ == vb_, __FILE__, __LINE__,                                        \
         "CHECK_EQ(" #a ", " #b "): " + check::str(va_) + " != " + check::str(vb_)); }())
#define CHECK_NEAR(a, b, tol)                                                                    \
  ([&]() { const double va_ = static_cast<double>(a), vb_ = static_cast<double>(b);              \
     const double tl_ = static_cast<double>(tol);                                                \
     return check::report(std::fabs(va_ - vb_) <= tl_, __FILE__, __LINE__,                       \
         "CHECK_NEAR(" #a ", " #b ", " #tol "): " + check::str(va_) + " vs " + check::str(vb_)); }())
