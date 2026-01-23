
#include "AP_Networking_Config.h"

#if AP_NETWORKING_BACKEND_CHIBIOS

#include "AP_Networking_ChibiOS.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <lwip/udp.h>
#include <lwip/ip_addr.h>
#include <lwip/tcpip.h>
#include <lwip/netifapi.h>
#if LWIP_DHCP
#include <lwip/dhcp.h>
#endif
#include <lwip/etharp.h>
#include <hal.h>
#include "../../modules/ChibiOS/os/various/evtimer.h"
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>

extern const AP_HAL::HAL& hal;

#ifndef STM32_ETH_BUFFERS_EXTERN
#error "Must use external ethernet buffers"
#endif

/*
  these are referenced as globals inside lwip
*/
stm32_eth_rx_descriptor_t *__eth_rd;
stm32_eth_tx_descriptor_t *__eth_td;
uint32_t *__eth_rb[STM32_MAC_RECEIVE_BUFFERS];
uint32_t *__eth_tb[STM32_MAC_TRANSMIT_BUFFERS];

#define LWIP_SEND_TIMEOUT_MS 50
#define LWIP_NETIF_MTU       1500
#define LWIP_LINK_POLL_INTERVAL TIME_S2I(5)

#define PERIODIC_TIMER_ID       1
#define FRAME_RECEIVED_ID       2
#if AP_NETWORKING_BACKEND_HUB
#define INJECTED_FRAME_ID       4
#endif

#if AP_NETWORKING_BACKEND_HUB
// Frame hooks for hub integration (each port registers its own)
static AP_Networking_ChibiOS::frame_hook_f s_rx_hook = nullptr;  // MAC RX → Port_Ethernet
static AP_Networking_ChibiOS::frame_hook_f s_tx_hook = nullptr;  // lwIP TX → Port_lwIP

// Queue for frames injected from hub (e.g., from COBS ports)
#define INJECT_QUEUE_SIZE 8
#define INJECT_FRAME_MAX 1522
static struct {
    uint8_t buf[INJECT_FRAME_MAX];
    size_t len;
} s_inject_queue[INJECT_QUEUE_SIZE];
static volatile uint8_t s_inject_head = 0;
static volatile uint8_t s_inject_tail = 0;
static event_source_t s_inject_evt;
static bool s_inject_evt_initialized = false;

void AP_Networking_ChibiOS::set_rx_hook(frame_hook_f hook)
{
    s_rx_hook = hook;
}

void AP_Networking_ChibiOS::set_tx_hook(frame_hook_f hook)
{
    s_tx_hook = hook;
}

void AP_Networking_ChibiOS::inject_frame_to_lwip(const uint8_t *frame, size_t len)
{
    if (len > INJECT_FRAME_MAX) {
        return;
    }
    uint8_t next_head = (s_inject_head + 1) % INJECT_QUEUE_SIZE;
    if (next_head == s_inject_tail) {
        return; // queue full
    }
    memcpy(s_inject_queue[s_inject_head].buf, frame, len);
    s_inject_queue[s_inject_head].len = len;
    s_inject_head = next_head;
    if (s_inject_evt_initialized) {
        chEvtBroadcastFlags(&s_inject_evt, INJECTED_FRAME_ID);
    }
}

bool AP_Networking_ChibiOS::send_frame_to_mac(const uint8_t *frame, size_t len)
{
    MACTransmitDescriptor td;
    if (macWaitTransmitDescriptor(&ETHD1, &td, TIME_MS2I(LWIP_SEND_TIMEOUT_MS)) != MSG_OK) {
        return false;
    }
    macWriteTransmitDescriptor(&td, (uint8_t *)frame, len);
    macReleaseTransmitDescriptorX(&td);
    return true;
}

bool AP_Networking_ChibiOS::get_link_status()
{
    return macPollLinkStatus(&ETHD1);
}
#endif // AP_NETWORKING_BACKEND_HUB

/*
  allocate buffers for LWIP
*/
bool AP_Networking_ChibiOS::allocate_buffers()
{
#define AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE ((((STM32_MAC_BUFFERS_SIZE - 1) | 3) + 1) / 4) // typically == 381
    // check total size of buffers
    const uint32_t total_size = sizeof(stm32_eth_rx_descriptor_t)*STM32_MAC_RECEIVE_BUFFERS +
        sizeof(stm32_eth_tx_descriptor_t)*STM32_MAC_TRANSMIT_BUFFERS +
        sizeof(uint32_t)*STM32_MAC_RECEIVE_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE +
        sizeof(uint32_t)*STM32_MAC_TRANSMIT_BUFFERS*AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE; // typically == 9240

    // ensure that we allocate 32-bit aligned memory, and mark it non-cacheable
    uint32_t size = 1;
    uint8_t rasr = 0;
    // find size closest to power of 2
    while (size < total_size) {
        size = size << 1;
        rasr++;
    }
    void *mem = malloc_eth_safe(size);
    if (mem == nullptr) {
        return false;
    }

#ifndef HAL_BOOTLOADER_BUILD
    // ensure our memory is aligned
    // ref. Cortex-M7 peripherals PM0253, section 4.6.4 MPU region base address register
    if (((uint32_t)mem) % size) {
        AP_HAL::panic("Bad alignment of ETH memory");
    }
#endif

    // for total_size == 9240, size should be 16384 and (rasr-1) should be 13 (MPU_RASR_SIZE_16K)
    const uint32_t rasr_size = MPU_RASR_SIZE(rasr-1);

    // set up MPU region for buffers
    mpuConfigureRegion(STM32_NOCACHE_MPU_REGION_ETH,
                       (uint32_t)mem,
                       MPU_RASR_ATTR_AP_RW_RW |
                       MPU_RASR_ATTR_NON_CACHEABLE |
                       MPU_RASR_ATTR_S |
                       rasr_size |
                       MPU_RASR_ENABLE);
    mpuEnable(MPU_CTRL_PRIVDEFENA);
    SCB_CleanInvalidateDCache();

    // assign buffers
    __eth_rd = (stm32_eth_rx_descriptor_t *)mem;
    __eth_td = (stm32_eth_tx_descriptor_t *)&__eth_rd[STM32_MAC_RECEIVE_BUFFERS];
    __eth_rb[0] = (uint32_t*)&__eth_td[STM32_MAC_TRANSMIT_BUFFERS];
    for (uint16_t i = 1; i < STM32_MAC_RECEIVE_BUFFERS; i++) {
        __eth_rb[i] = &(__eth_rb[i-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    }
    __eth_tb[0] = &(__eth_rb[STM32_MAC_RECEIVE_BUFFERS-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    for (uint16_t i = 1; i < STM32_MAC_TRANSMIT_BUFFERS; i++) {
        __eth_tb[i] = &(__eth_tb[i-1][AP_NETWORKING_EXTERN_MAC_BUFFER_SIZE]);
    }
    return true;
}

/*
  initialise ChibiOS network backend using LWIP
*/
bool AP_Networking_ChibiOS::init()
{
#ifdef HAL_GPIO_ETH_ENABLE
    hal.gpio->pinMode(HAL_GPIO_ETH_ENABLE, HAL_GPIO_OUTPUT);
    hal.gpio->write(HAL_GPIO_ETH_ENABLE, frontend.param.enabled ? 1 : 0);
#endif

    if (!allocate_buffers()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: Failed to allocate buffers");
        return false;
    }

    if (!macInit()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NET: macInit failed");
        return false;
    }

#if LWIP_IGMP
    if (ETH != nullptr) {
        // enbale "permit multicast" so we can receive multicast packets
        ETH->MACPFR |= ETH_MACPFR_PM;
    }
#endif

    thisif = NEW_NOTHROW netif;
    if (thisif == nullptr) {
        return false;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Networking_ChibiOS::thread, void),
                                      "network",
                                      2048, AP_HAL::Scheduler::PRIORITY_NET, 0)) {
        return false;
    }
    
    return true;
}

void AP_Networking_ChibiOS::link_up_cb(void *p)
{
#if LWIP_DHCP
    auto *driver = (AP_Networking_ChibiOS *)p;
    if (driver->frontend.get_dhcp_enabled()) {
        dhcp_start(driver->thisif);
    }
#endif
}

void AP_Networking_ChibiOS::link_down_cb(void *p)
{
#if LWIP_DHCP
    auto *driver = (AP_Networking_ChibiOS *)p;
    if (driver->frontend.get_dhcp_enabled()) {
        dhcp_stop(driver->thisif);
    }
#endif
}


#if AP_NETWORKING_CAPTURE_ENABLED
/*
  capture all data in a pbuf chain
 */
void AP_Networking_ChibiOS::capture_pbuf(struct pbuf *p)
{
    auto *front = AP_Networking::singleton;
    if (!front->option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
        return;
    }
    auto &driver = *(AP_Networking_ChibiOS*)front->backend;
    WITH_SEMAPHORE(driver.capture.sem);
    if (driver.capture.fd == -1) {
        return;
    }
    driver.capture_header(driver.capture.fd, p->tot_len);
    auto &fs = AP::FS();
    for (auto *pp = p; pp != nullptr; pp = pp->next) {
        fs.write(driver.capture.fd, (const uint8_t *)pp->payload, pp->len);
    }
}
#endif

/*
 * This function does the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
int8_t AP_Networking_ChibiOS::low_level_output(struct netif *netif, struct pbuf *p)
{
    struct pbuf *q;
    MACTransmitDescriptor td;
    (void)netif;

    if (macWaitTransmitDescriptor(&ETHD1, &td, TIME_MS2I(LWIP_SEND_TIMEOUT_MS)) != MSG_OK) {
        return ERR_TIMEOUT;
    }

#if AP_NETWORKING_CAPTURE_ENABLED
    capture_pbuf(p);
#endif

#if ETH_PAD_SIZE
    pbuf_header(p, -ETH_PAD_SIZE);        /* drop the padding word */
#endif

    /* Iterates through the pbuf chain. */
    for(q = p; q != NULL; q = q->next) {
        macWriteTransmitDescriptor(&td, (uint8_t *)q->payload, (size_t)q->len);
    }
    macReleaseTransmitDescriptorX(&td);

#if AP_NETWORKING_BACKEND_HUB
    // If hub hook is registered, also send the frame to hub for routing to other ports
    if (s_tx_hook != nullptr) {
        // Flatten pbuf chain into contiguous buffer for hook
        static uint8_t tx_buf[1522];
        size_t ofs = 0;
        for (q = p; q != NULL && ofs < sizeof(tx_buf); q = q->next) {
            size_t to_copy = (ofs + q->len <= sizeof(tx_buf)) ? q->len : (sizeof(tx_buf) - ofs);
            memcpy(&tx_buf[ofs], q->payload, to_copy);
            ofs += to_copy;
        }
        s_tx_hook(tx_buf, ofs);
    }
#endif

#if ETH_PAD_SIZE
    pbuf_header(p, ETH_PAD_SIZE);         /* reclaim the padding word */
#endif

    return ERR_OK;
}

/*
 * Receives a frame.
 * Allocates a pbuf and transfers the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
bool AP_Networking_ChibiOS::low_level_input(struct netif *netif, struct pbuf **pbuf)
{
    MACReceiveDescriptor rd;
    struct pbuf *q;
    u16_t len;

    (void)netif;

    if (macWaitReceiveDescriptor(&ETHD1, &rd, TIME_IMMEDIATE) != MSG_OK) {
        return false;
    }

    len = (u16_t)rd.size;

#if ETH_PAD_SIZE
    len += ETH_PAD_SIZE;        /* allow room for Ethernet padding */
#endif

    /* We allocate a pbuf chain of pbufs from the pool. */
    *pbuf = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);

    if (*pbuf != nullptr) {
#if ETH_PAD_SIZE
        pbuf_header(*pbuf, -ETH_PAD_SIZE); /* drop the padding word */
#endif

        /* Iterates through the pbuf chain. */
        for(q = *pbuf; q != NULL; q = q->next) {
            macReadReceiveDescriptor(&rd, (uint8_t *)q->payload, (size_t)q->len);
        }
        macReleaseReceiveDescriptorX(&rd);

#if ETH_PAD_SIZE
        pbuf_header(*pbuf, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

#if AP_NETWORKING_CAPTURE_ENABLED
        capture_pbuf(*pbuf);
#endif

#if AP_NETWORKING_BACKEND_HUB
        // If hub hook is registered, also send the frame to hub for routing
        if (s_rx_hook != nullptr) {
            // Flatten pbuf chain into contiguous buffer for hook
            static uint8_t rx_buf[1522];
            size_t ofs = 0;
            for (q = *pbuf; q != NULL && ofs < sizeof(rx_buf); q = q->next) {
                size_t to_copy = (ofs + q->len <= sizeof(rx_buf)) ? q->len : (sizeof(rx_buf) - ofs);
                memcpy(&rx_buf[ofs], q->payload, to_copy);
                ofs += to_copy;
            }
            s_rx_hook(rx_buf, ofs);
        }
#endif

    } else {
        macReleaseReceiveDescriptorX(&rd);     // Drop packet
    }
  
    return true;
}

int8_t AP_Networking_ChibiOS::ethernetif_init(struct netif *netif)
{
    netif->state = NULL;
    netif->name[0] = 'm';
    netif->name[1] = 's';
    netif->output = etharp_output;
    netif->linkoutput = low_level_output;

    /* set MAC hardware address length */
    netif->hwaddr_len = ETHARP_HWADDR_LEN;

    /* maximum transfer unit */
    netif->mtu = LWIP_NETIF_MTU;

    /* device capabilities */
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;

#if LWIP_IGMP
    // also enable multicast
    netif->flags |= NETIF_FLAG_IGMP;
#endif

    return ERR_OK;
}

#if AP_NETWORKING_CAPTURE_ENABLED
// start a pcap network capture
void AP_Networking_ChibiOS::start_capture(void)
{
    if (capture.fd != -1) {
        // called at 1Hz, flush the file
        AP::FS().fsync(capture.fd);
        return;
    }
    const struct pcap_hdr {
        uint32_t magic_number;   // 0xa1b2c3d4
        uint16_t version_major;  // 2
        uint16_t version_minor;  // 4
        int32_t  thiszone;       // GMT to local correction
        uint32_t sigfigs;        // accuracy of timestamps
        uint32_t snaplen;        // max length of captured packets, in octets
        uint32_t network;        // data link type (1 for Ethernet)
    } hdr = {
        0xa1b2c3d4, 2, 4, 0, 0, 1500, 1
    };
    const char *fname = "eth0.cap";
    WITH_SEMAPHORE(capture.sem);
    auto &fs = AP::FS();
    capture.fd = fs.open(fname, O_WRONLY|O_CREAT|O_TRUNC);
    if (capture.fd != -1) {
        fs.write(capture.fd, (const void *)&hdr, sizeof(hdr));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Capturing to %s", fname);
    }
}

// stop a pcap network capture
void AP_Networking_ChibiOS::stop_capture(void)
{
    int fd = capture.fd;
    if (fd != -1) {
        capture.fd = -1;
        AP::FS().close(fd);
    }
}
#endif // AP_NETWORKING_CAPTURE_ENABLED

#if AP_NETWORKING_BACKEND_HUB
/*
  Process frames injected from hub (e.g., from COBS ports) into lwIP
*/
static void process_injected_frame(struct netif *netif)
{
    while (s_inject_tail != s_inject_head) {
        const uint8_t *frame = s_inject_queue[s_inject_tail].buf;
        size_t len = s_inject_queue[s_inject_tail].len;
        
        struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
        if (p != nullptr) {
            pbuf_take(p, frame, len);
            struct eth_hdr *ethhdr = (struct eth_hdr *)p->payload;
            switch (htons(ethhdr->type)) {
            case ETHTYPE_IP:
            case ETHTYPE_ARP:
                if (netif->input(p, netif) == ERR_OK) {
                    break;
                }
                /* Falls through */
            default:
                pbuf_free(p);
            }
        }
        s_inject_tail = (s_inject_tail + 1) % INJECT_QUEUE_SIZE;
    }
}
#endif // AP_NETWORKING_BACKEND_HUB

/*
  networking thread
*/
void AP_Networking_ChibiOS::thread()
{
    while (!hal.scheduler->is_system_initialized()) {
        hal.scheduler->delay_microseconds(1000);
    }

    /* start tcpip thread */
    tcpip_init(NULL, NULL);

    frontend.param.macaddr.get_address(thisif->hwaddr);

    struct {
        ip4_addr_t ip, gateway, netmask;
    } addr {};

    if (!frontend.get_dhcp_enabled()) {
        addr.ip.addr = htonl(frontend.get_ip_param());
        addr.gateway.addr = htonl(frontend.get_gateway_param());
        addr.netmask.addr = htonl(frontend.get_netmask_param());
    }

    const MACConfig mac_config = {thisif->hwaddr};
    macStart(&ETHD1, &mac_config);

    /* Add interface. */
    auto result = netifapi_netif_add(thisif, &addr.ip, &addr.netmask, &addr.gateway, NULL, ethernetif_init, tcpip_input);
    if (result != ERR_OK) {
        AP_HAL::panic("Failed to initialise netif");
    }

    netifapi_netif_set_default(thisif);
    netifapi_netif_set_up(thisif);

    /* Setup event sources.*/
    event_timer_t evt;
    event_listener_t el0, el1;
    
    evtObjectInit(&evt, LWIP_LINK_POLL_INTERVAL);
    evtStart(&evt);
    chEvtRegisterMask(&evt.et_es, &el0, PERIODIC_TIMER_ID);
    chEvtRegisterMaskWithFlags(macGetEventSource(&ETHD1), &el1,
                               FRAME_RECEIVED_ID, MAC_FLAGS_RX);

#if AP_NETWORKING_BACKEND_HUB
    event_listener_t el2;
    chEvtObjectInit(&s_inject_evt);
    s_inject_evt_initialized = true;
    chEvtRegisterMask(&s_inject_evt, &el2, INJECTED_FRAME_ID);
    chEvtAddEvents(PERIODIC_TIMER_ID | FRAME_RECEIVED_ID | INJECTED_FRAME_ID);
#else
    chEvtAddEvents(PERIODIC_TIMER_ID | FRAME_RECEIVED_ID);
#endif

    while (true) {
#if AP_NETWORKING_CAPTURE_ENABLED
        eventmask_t mask = chEvtWaitAnyTimeout(ALL_EVENTS, chTimeMS2I(1000));
#else
        eventmask_t mask = chEvtWaitAny(ALL_EVENTS);
#endif
        if (mask & PERIODIC_TIMER_ID) {
            bool current_link_status = macPollLinkStatus(&ETHD1);
            if (current_link_status != netif_is_link_up(thisif)) {
                if (current_link_status) {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_up, thisif, 0);
                    tcpip_callback_with_block(link_up_cb, this, 0);
                }
                else {
                    tcpip_callback_with_block((tcpip_callback_fn) netif_set_link_down, thisif, 0);
                    tcpip_callback_with_block(link_down_cb, this, 0);
                }
            }
        }

        if (mask & FRAME_RECEIVED_ID) {
            struct pbuf *p;
            while (low_level_input(thisif, &p)) {
                if (p != NULL) {
                    struct eth_hdr *ethhdr = (struct eth_hdr *)p->payload;
                    switch (htons(ethhdr->type)) {
                        /* IP or ARP packet? */
                    case ETHTYPE_IP:
                    case ETHTYPE_ARP:
                        /* full packet send to tcpip_thread to process */
                        if (thisif->input(p, thisif) == ERR_OK) {
                            break;
                        }
                        /* Falls through */
                    default:
                        pbuf_free(p);
                    }
                }
            }
        }

#if AP_NETWORKING_BACKEND_HUB
        if (mask & INJECTED_FRAME_ID) {
            process_injected_frame(thisif);
        }
#endif

#if AP_NETWORKING_CAPTURE_ENABLED
        if (frontend.option_is_set(AP_Networking::OPTION::CAPTURE_PACKETS)) {
            start_capture();
        } else {
            stop_capture();
        }
#endif
    }
}

/*
  update called at 10Hz
*/
void AP_Networking_ChibiOS::update()
{
    const uint32_t ip = ntohl(thisif->ip_addr.addr);
    const uint32_t nm = ntohl(thisif->netmask.addr);
    const uint32_t gw = ntohl(thisif->gw.addr);

    if (ip != activeSettings.ip ||
        nm != activeSettings.nm ||
        gw != activeSettings.gw) {
        activeSettings.ip = ip;
        activeSettings.gw = gw;
        activeSettings.nm = nm;
        activeSettings.last_change_ms = AP_HAL::millis();
    }
}

#endif // AP_NETWORKING_BACKEND_CHIBIOS
