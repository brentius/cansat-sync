// GPL-3.0-only. C ABI glue to Zig 0.16 standard math / compiler_rt.
// Algorithms and their original notices remain in the installed Zig sources.
// No local approximations. build.py pins/checks the Zig version and writes
// the upstream MIT/Sun notices beside the linked image.
const std = @import("std");
export fn acos(x: f64) callconv(.c) f64 {
    return std.math.acos(x);
}
export fn ldexp(x: f64, n: c_int) callconv(.c) f64 {
    return std.math.ldexp(x, n);
}
