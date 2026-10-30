// GPL-3.0-only. Host C ABI probes of the same Zig runtime used by target.
// Linked without libc: transcendental calls resolve to Zig compiler_rt.
const rt = @import("runtime");
const std = @import("std");
comptime { _ = rt; }
export fn rt_sin(x: f64) callconv(.c) f64 { return @sin(x); }
export fn rt_cos(x: f64) callconv(.c) f64 { return @cos(x); }
export fn rt_sqrt(x: f64) callconv(.c) f64 { return @sqrt(x); }
export fn rt_log(x: f64) callconv(.c) f64 { return @log(x); }
export fn rt_exp(x: f64) callconv(.c) f64 { return @exp(x); }
export fn rt_acos(x: f64) callconv(.c) f64 { return std.math.acos(x); }
export fn rt_floor(x: f64) callconv(.c) f64 { return @floor(x); }
export fn rt_ceil(x: f64) callconv(.c) f64 { return @ceil(x); }
export fn rt_fmod(x: f64, y: f64) callconv(.c) f64 {
    return @extern(*const fn (f64,f64) callconv(.c) f64, .{.name="fmod"})(x,y);
}
export fn rt_ldexp(x: f64, n: c_int) callconv(.c) f64 { return std.math.ldexp(x,n); }
export fn rt_logf(x: f32) callconv(.c) f32 { return @log(x); }
export fn rt_floorf(x: f32) callconv(.c) f32 { return @floor(x); }
