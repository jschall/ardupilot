#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB

#include "AP_Networking_Hub.h"
#include "AP_Networking.h"
#include <AP_HAL/AP_HAL.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

AP_Networking_Hub::AP_Networking_Hub()
{
#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        mac_table[i].port_idx = -1;
    }
#endif
}

int8_t AP_Networking_Hub::register_port(AP_Networking_HubPort *port)
{
    if (port == nullptr) {
        return -1;
    }
    WITH_SEMAPHORE(route_sem);
    if (num_ports >= ARRAY_SIZE(ports)) {
        return -1;
    }
    ports[num_ports] = port;
    return (int8_t)num_ports++;
}

void AP_Networking_Hub::unregister_port(AP_Networking_HubPort *port)
{
    if (port == nullptr) {
        return;
    }
    WITH_SEMAPHORE(route_sem);
    if (num_ports == 0) {
        return;
    }
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] == port) {
#if AP_NETWORKING_HUB_SWITCHING_ENABLED
            fixup_mac_entries_for_removed_port(i);
#endif
            for (uint8_t j = i; j + 1 < num_ports; j++) {
                ports[j] = ports[j + 1];
            }
            ports[num_ports - 1] = nullptr;
            num_ports--;
            break;
        }
    }
}

void AP_Networking_Hub::route_frame(AP_Networking_HubPort *source, const uint8_t *frame, size_t len)
{
    WITH_SEMAPHORE(route_sem);
    if (frame == nullptr || len < 14 || len > MAX_ETH_FRAME) {
        frames_dropped++;
        return;
    }

    // Ethernet header: dest MAC (6) + src MAC (6) + ethertype (2)
    const uint8_t *dst_mac = frame;
    const uint8_t *src_mac = frame + 6;

#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    // Find source port index
    int8_t src_port_idx = -1;
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] == source) {
            src_port_idx = (int8_t)i;
            break;
        }
    }

    // Learn source MAC
    if (src_port_idx >= 0 && !is_broadcast_or_multicast(src_mac)) {
        learn_mac(src_mac, src_port_idx);
    }

    // Lookup destination
    int8_t dst_port_idx = -1;
    if (!is_broadcast_or_multicast(dst_mac)) {
        dst_port_idx = lookup_mac(dst_mac);
    }

    if (dst_port_idx >= 0 && dst_port_idx < (int8_t)num_ports) {
        // Unicast to known port
        AP_Networking_HubPort *port = ports[dst_port_idx];
        if (port != nullptr && port != source && port->can_receive()) {
            port->deliver_frame(frame, len);
        }
    } else {
        // Flood: broadcast, multicast, or unknown unicast
        for (uint8_t i = 0; i < num_ports; i++) {
            AP_Networking_HubPort *port = ports[i];
            if (port == nullptr || port == source) {
                continue;
            }
            if (!port->can_receive()) {
                continue;
            }
            port->deliver_frame(frame, len);
        }
    }
#else
    // Pure hub: flood to all ports except source
    (void)dst_mac;
    (void)src_mac;
    for (uint8_t i = 0; i < num_ports; i++) {
        AP_Networking_HubPort *port = ports[i];
        if (port == nullptr || port == source) {
            continue;
        }
        if (!port->can_receive()) {
            continue;
        }
        port->deliver_frame(frame, len);
    }
#endif // AP_NETWORKING_HUB_SWITCHING_ENABLED

    frames_routed++;
}

void AP_Networking_Hub::update()
{
    for (uint8_t i = 0; i < num_ports; i++) {
        if (ports[i] != nullptr) {
            ports[i]->update();
        }
    }
#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    {
        WITH_SEMAPHORE(route_sem);
        age_mac_table();
    }
#endif
}

uint8_t AP_Networking_Hub::get_num_ports_link_up() const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < num_ports; i++) {
        const AP_Networking_HubPort *port = ports[i];
        if (port != nullptr && port->is_link_up()) {
            count++;
        }
    }
    return count;
}

#if AP_NETWORKING_HUB_SWITCHING_ENABLED
void AP_Networking_Hub::learn_mac(const uint8_t *mac, int8_t port_idx)
{
    uint32_t now = AP_HAL::millis();
    int16_t oldest_idx = -1;
    uint32_t oldest_age = 0;
    int16_t empty_idx = -1;

    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx < 0) {
            if (empty_idx < 0) {
                empty_idx = (int16_t)i;
            }
            continue;
        }
        if (memcmp(e.mac, mac, 6) == 0) {
            // Update existing entry
            e.port_idx = port_idx;
            e.last_seen_ms = now;
            return;
        }
        uint32_t age = now - e.last_seen_ms;
        if (age > oldest_age) {
            oldest_age = age;
            oldest_idx = (int16_t)i;
        }
    }

    // Insert new entry
    int16_t idx = (empty_idx >= 0) ? empty_idx : oldest_idx;
    if (idx >= 0) {
        MACEntry &e = mac_table[idx];
        memcpy(e.mac, mac, 6);
        e.port_idx = port_idx;
        e.last_seen_ms = now;
    }
}

int8_t AP_Networking_Hub::lookup_mac(const uint8_t *mac) const
{
    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        const MACEntry &e = mac_table[i];
        if (e.port_idx >= 0 && memcmp(e.mac, mac, 6) == 0) {
            return e.port_idx;
        }
    }
    return -1;
}

void AP_Networking_Hub::age_mac_table()
{
    uint32_t now = AP_HAL::millis();
    if ((now - last_age_ms) < 1000) {
        return;
    }
    last_age_ms = now;

    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx >= 0 && (now - e.last_seen_ms) > AP_NETWORKING_HUB_MAC_AGE_MS) {
            e.port_idx = -1;
        }
    }
}

void AP_Networking_Hub::fixup_mac_entries_for_removed_port(uint8_t removed_idx)
{
    for (uint16_t i = 0; i < ARRAY_SIZE(mac_table); i++) {
        MACEntry &e = mac_table[i];
        if (e.port_idx < 0) {
            continue;
        }
        if ((uint8_t)e.port_idx == removed_idx) {
            e.port_idx = -1;
        } else if ((uint8_t)e.port_idx > removed_idx) {
            e.port_idx--;
        }
    }
}
#endif // AP_NETWORKING_HUB_SWITCHING_ENABLED

#endif // AP_NETWORKING_BACKEND_HUB


