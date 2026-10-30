/* GPL-3.0-only. C twin of link.ImuAccumulator and protocol section 6.
 * q and u are double precision running integrals. Do not mix synchronous and
 * asynchronous calls without init. All times modulo 2^32, ordered within each
 * sensor stream with adjacent intervals < 2^31 us. Synchronous h obeys the
 * same upper limit in seconds (2147.483648), and must be positive after priming.
 * Inputs SI, body axes.
 * Invalid/out-of-order samples return false and latch fault/missed flags.
 *
 * Asynchronous method: retain bounded gyro attitudes; SLERP q to each accel
 * tick, rotate force there, integrate trapezoidally. A snapshot requires an
 * integrated accel on/beyond its gyro tick; split that force trapezoid exactly
 * at the requested tick. Thus no stale u or extrapolation is transmitted.
 * snapshot_latest selects the newest fully bracketed gyro tick (normally a
 * delay of 1-2 gyro periods). Caller must feed both streams frequently: 64
 * entries per history are retained, overflow/rejection is counted and flagged.
 * snapshot(tick) requires an exact retained gyro tick in async mode; returns
 * false when not yet bracketed or older than the preceding snapshot.
 * Counts cover captures through snapshot tick; future bracketing samples are
 * counted next time. Flags from those samples are conservatively included.
 * Gyro/accel histories are rings (oldest at *_first), so retiring an entry
 * costs O(1) instead of shifting the whole history on every sample.
 */
#ifndef CS_IMU_H
#define CS_IMU_H
#include "cs_link.h"
#define CS_IMU_HISTORY 64u /* power of two: ring indices are masked */
typedef struct { uint32_t tick; double q[4]; uint64_t count; uint8_t flags; } cs_imu_gyro_node_t;
typedef struct { uint32_t tick; double f[3], u[3]; uint64_t count; uint8_t flags; } cs_imu_accel_node_t;
typedef struct { uint32_t tick; float force[3]; uint8_t flags; } cs_imu_pending_t;
typedef struct {
    double q[4], u[3], rate[3], force[3];
    uint64_t n_gyro, n_accel, sent_gyro, sent_accel;
    uint8_t epoch, flags, mode;
    bool primed, have_snapshot, have_accel_tick;
    uint32_t last_gyro_tick, last_accel_tick, snapshot_tick, rejected;
    cs_imu_gyro_node_t gyro[CS_IMU_HISTORY];
    cs_imu_accel_node_t accel[CS_IMU_HISTORY];
    cs_imu_pending_t pending[CS_IMU_HISTORY];
    size_t gyro_count, accel_count, pending_count;
    size_t gyro_first, accel_first;
} cs_imu_t;
void cs_imu_init(cs_imu_t *imu, uint8_t epoch);
bool cs_imu_add_sample(cs_imu_t *imu, const float rate[3], const float force[3], double h, uint8_t flags);
bool cs_imu_add_gyro(cs_imu_t *imu, uint32_t tick, const float rate[3], uint8_t flags);
bool cs_imu_add_accel(cs_imu_t *imu, uint32_t tick, const float force[3], uint8_t flags);
bool cs_imu_snapshot(cs_imu_t *imu, uint32_t tick, cs_link_imu_t *out);
bool cs_imu_snapshot_latest(cs_imu_t *imu, cs_link_imu_t *out);
#endif
