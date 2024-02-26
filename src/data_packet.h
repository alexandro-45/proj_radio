#ifndef REB_CONTROL_DATA_PACKET_H
#define REB_CONTROL_DATA_PACKET_H

#include <stdint.h>
#include <stdbool.h>

struct RadioDataPacket {
    union {
        struct {
            uint16_t ch1;
            uint16_t ch2;
            uint16_t ch3;
            uint16_t ch4;
        };
        uint16_t channels[4];
    };
    bool gen_on;
    bool force_gen;
    uint8_t hash;
};

struct ControlDataPacket {
    bool force_gen;
    uint8_t hash;
};

bool rdp_check_hash(struct RadioDataPacket *packet);
void rdp_gen_hash(struct RadioDataPacket *packet);
bool cdp_check_hash(struct ControlDataPacket *packet);
void cdp_gen_hash(struct ControlDataPacket *packet);

#endif //REB_CONTROL_DATA_PACKET_H
