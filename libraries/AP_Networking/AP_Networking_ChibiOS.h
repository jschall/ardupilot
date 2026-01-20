#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_CHIBIOS
#include "AP_Networking_Backend.h"
#include <stddef.h>

class AP_Networking_ChibiOS : public AP_Networking_Backend
{
public:
    friend class BL_Network;
    using AP_Networking_Backend::AP_Networking_Backend;

    // Switch interface callbacks for integrating an external Layer-2 switch
    using rx_get_frame_f = bool (*)(uint8_t *buf, size_t *len, size_t max_len);
    using tx_send_frame_f = bool (*)(const uint8_t *frame, size_t len);

    // Register/unregister switch interface; when active, lwIP RX/TX route via these
    static void set_switch_interface(rx_get_frame_f rx_cb, tx_send_frame_f tx_cb);
    static bool switch_interface_active();

    // Allocate MAC DMA buffers (called by Port_Ethernet and BL_Network)
    static bool allocate_buffers();

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_ChibiOS);

    bool init() override;
    void update() override;

private:
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

