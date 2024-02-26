#include "data_packet.h"

bool rdp_check_hash(struct RadioDataPacket *packet) {
    return packet->hash == (packet->ch1 + packet->ch2 + packet->ch3 + packet->ch4 + packet->gen_on + packet->force_gen) % 99;
}

void rdp_gen_hash(struct RadioDataPacket *packet) {
    packet->hash = (packet->ch1 + packet->ch2 + packet->ch3 + packet->ch4 + packet->gen_on + packet->force_gen) % 99;
}

bool cdp_check_hash(struct ControlDataPacket *packet) {
    return packet->hash == packet->force_gen;
}

void cdp_gen_hash(struct ControlDataPacket *packet) {
    packet->hash = packet->force_gen;
}