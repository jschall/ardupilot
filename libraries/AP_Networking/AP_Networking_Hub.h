#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_HUB

#include <stdint.h>
#include <stddef.h>
#include <AP_HAL/Semaphores.h>

class AP_Networking_HubPort;

/*
  Abstract hub port interface
*/
class AP_Networking_HubPort
{
public:
    virtual ~AP_Networking_HubPort() {}

    // Called by hub to deliver a frame to this port
    virtual void deliver_frame(const uint8_t *frame, size_t len) = 0;

    // Called by hub to check if port is ready to receive
    virtual bool can_receive() const
    {
        return true;
    }

    // Link state reporting for switch aggregation
    // Ports should override to reflect their own link availability
    virtual bool is_link_up() const
    {
        return false;
    }

    // Optional update hook, called by hub->update()
    virtual void update() {}

    // Port identifier (for debugging)
    virtual const char *get_name() const = 0;
};

/*
  Layer-2 switch/hub that routes frames between registered ports.
  When switching is enabled, learns source MACs and forwards unicast
  to the learned port. Broadcasts/multicasts and unknown unicasts flood.
*/
class AP_Networking_Hub
{
public:
    AP_Networking_Hub();

    // Register a port with the hub
    // Returns: port ID (0-based index) or -1 on failure
    int8_t register_port(AP_Networking_HubPort *port);

    // Unregister a port
    void unregister_port(AP_Networking_HubPort *port);

    // Route a frame from source port to destination port(s)
    void route_frame(AP_Networking_HubPort *source, const uint8_t *frame, size_t len);

    // Update all ports and age MAC table
    void update();

    // Statistics
    uint32_t get_frames_routed() const
    {
        return frames_routed;
    }
    uint32_t get_frames_dropped() const
    {
        return frames_dropped;
    }

    // Number of ports currently reporting link up (including lwIP if enabled)
    uint8_t get_num_ports_link_up() const;

    // Maximum supported Ethernet frame size that hub will route (includes VLAN)
    static constexpr size_t MAX_ETH_FRAME = 1522;

private:
    AP_Networking_HubPort *ports[AP_NETWORKING_HUB_PORT_MAX_INSTANCES];
    uint8_t num_ports;
    uint32_t frames_routed;
    uint32_t frames_dropped;
    HAL_Semaphore route_sem;

#if AP_NETWORKING_HUB_SWITCHING_ENABLED
    struct MACEntry {
        uint8_t mac[6];
        int8_t port_idx;
        uint32_t last_seen_ms;
    };
    MACEntry mac_table[AP_NETWORKING_HUB_MAC_TABLE_SIZE];

    // Learn source MAC on ingress
    void learn_mac(const uint8_t *mac, int8_t port_idx);

    // Lookup destination MAC, returns port index or -1 if not found
    int8_t lookup_mac(const uint8_t *mac) const;

    // Invalidate/adjust MAC entries when port at given index is removed
    void fixup_mac_entries_for_removed_port(uint8_t removed_idx);

    // Age out stale entries
    void age_mac_table();

    // Check if MAC is broadcast or multicast
    static bool is_broadcast_or_multicast(const uint8_t *mac)
    {
        return (mac[0] & 0x01) != 0;
    }

    uint32_t last_age_ms;
#endif // AP_NETWORKING_HUB_SWITCHING_ENABLED
};

#endif // AP_NETWORKING_BACKEND_HUB


