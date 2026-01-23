#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_CHIBIOS
#include "AP_Networking_Backend.h"

class AP_Networking_ChibiOS : public AP_Networking_Backend
{
public:
    friend class BL_Network;
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_ChibiOS);

    bool init() override;
    void update() override;

#if AP_NETWORKING_BACKEND_HUB
    // Frame hook callback type
    using frame_hook_f = void (*)(const uint8_t *frame, size_t len);

    // Set hook for MAC RX frames (called by Port_Ethernet)
    static void set_rx_hook(frame_hook_f hook);

    // Set hook for lwIP TX frames (called by Port_lwIP)
    static void set_tx_hook(frame_hook_f hook);

    // Inject a frame into lwIP as if received from MAC (called by Port_lwIP)
    static void inject_frame_to_lwip(const uint8_t *frame, size_t len);

    // Send a frame to MAC (called by Port_Ethernet)
    static bool send_frame_to_mac(const uint8_t *frame, size_t len);

    // Get MAC link status (called by Port_Ethernet)
    static bool get_link_status();
#endif

private:
    static bool allocate_buffers(void);
    void thread(void);
    static void link_up_cb(void*);
    static void link_down_cb(void*);
    static int8_t ethernetif_init(struct netif *netif);
    static int8_t low_level_output(struct netif *netif, struct pbuf *p);
    static bool low_level_input(struct netif *netif, struct pbuf **pbuf);
#if AP_NETWORKING_CAPTURE_ENABLED
    void start_capture(void);
    void stop_capture(void);
    static void capture_pbuf(struct pbuf *p);
    struct {
        HAL_Semaphore sem;
        int fd = -1;
    } capture;
#endif

    struct lwipthread_opts *lwip_options;
    uint8_t macaddr[6];

    struct netif *thisif;
};

#endif // AP_NETWORKING_BACKEND_CHIBIOS
