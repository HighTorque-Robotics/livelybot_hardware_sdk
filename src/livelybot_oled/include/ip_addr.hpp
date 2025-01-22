#include <cstdint>

#define LO_NET      "lo"
#define ETHERNET    "enp86s0"
#define WLAN        "p2p0"

void update_ip_addr(void);
uint32_t get_ip_data_u32(uint8_t i);
uint32_t* get_ip_data_u32_all(void);

