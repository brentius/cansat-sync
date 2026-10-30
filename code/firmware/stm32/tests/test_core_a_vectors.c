/* GPL-3.0-only. Python golden vectors, fragmented/malformed streams and IMU model.
 * CS_SOURCES: cs_crc16.c cs_cobs.c cs_link.c cs_imu.c cs_telemetry.c */
#include "cs_test.h"
#include "cs_crc16.h"
#include "cs_cobs.h"
#include "cs_link.h"
#include "cs_imu.h"
#include "cs_telemetry.h"
static size_t received;
static uint8_t ids[256], seqs[256];
static void capture(void *ctx,uint8_t id,uint8_t seq,const uint8_t *p,size_t n) {
    (void)ctx; (void)p; (void)n;
    if(received<256) { ids[received]=id;seqs[received]=seq; } received++;
}
#include "core_a_vectors.inc"
CS_MAIN(crc_and_cobs,message_0,message_1,message_2,message_3,message_4,message_5,message_6,message_7,message_8,message_9,message_10,message_11,message_12,message_13,message_14,message_15,message_16,message_17,message_18,message_19,message_20,message_21,message_22,message_23,message_24,message_25,message_26,message_27,message_28,message_29,message_30,message_31,message_32,message_33,message_34,message_35,message_36,message_37,message_38,message_39,message_40,message_41,message_42,message_43,message_44,message_45,message_46,message_47,message_48,message_49,message_50,message_51,message_52,message_53,message_54,message_55,message_56,message_57,message_58,message_59,stream_0,stream_1,stream_2,stream_3,stream_4,stream_5,stream_6,stream_7,stream_8,stream_9,stream_10,stream_11,stream_12,stream_13,stream_14,stream_15,stream_16,stream_17,stream_18,stream_19,stream_20,stream_21,stream_22,stream_23,stream_24,stream_25,stream_26,stream_27,stream_28,stream_29,stream_30,stream_31,stream_32,stream_33,stream_34,stream_35,stream_36,stream_37,stream_38,stream_39,stream_40,stream_41,stream_42,stream_43,stream_44,stream_45,stream_46,stream_47,stream_48,imu_model_0,telemetry_vectors)
