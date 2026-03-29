#include "zf_common_headfile.h"

#include "driver_dshot600.h"

#define DRIVER_DSHOT600_COMMAND_MAX           (47U)

#define DRIVER_DSHOT600_MOTOR_PIN_SHIFT       (0U)
#define DRIVER_DSHOT600_MOTOR_PORT_INDEX      (5U)

#define DRIVER_DSHOT600_GPIO_BASE_ADDR        (0x40310000UL)
#define DRIVER_DSHOT600_GPIO_PORT_STRIDE      (0x80UL)
#define DRIVER_DSHOT600_GPIO_OUT_CLR_OFFSET   (0x04UL)
#define DRIVER_DSHOT600_GPIO_OUT_SET_OFFSET   (0x08UL)

#define DRIVER_DSHOT600_BIT_NS                (1667UL)
#define DRIVER_DSHOT600_T0H_NS                (625UL)
#define DRIVER_DSHOT600_T1H_NS                (1250UL)
#define DRIVER_DSHOT600_RESET_US              (10UL)

typedef struct
{
    uint8 initialized;
    uint16 throttle[DRIVER_DSHOT600_MOTOR_COUNT];
    uint8 telemetry[DRIVER_DSHOT600_MOTOR_COUNT];
    uint32 gpio_set_addr;
    uint32 gpio_clr_addr;
    uint32 gpio_all_mask;
    uint32 core_clock_hz;
    uint32 bit_cycles;
    uint32 t0h_cycles;
    uint32 t1h_cycles;
    uint32 reset_cycles;
} driver_dshot600_state_struct;

static driver_dshot600_state_struct g_dshot600 = {0};

static uint32 driver_dshot600_ns_to_cycles (uint32 ns, uint32 core_clock_hz)
{
    uint64 cycles = ((uint64)ns * (uint64)core_clock_hz + 999999999ULL) / 1000000000ULL;
    return (uint32)cycles;
}

static uint32 driver_dshot600_us_to_cycles (uint32 us, uint32 core_clock_hz)
{
    uint64 cycles = ((uint64)us * (uint64)core_clock_hz + 999999ULL) / 1000000ULL;
    return (uint32)cycles;
}

static uint16 driver_dshot600_build_packet (uint16 value, uint8 telemetry)
{
    uint16 packet = (uint16)((value << 1U) | (telemetry ? 1U : 0U));
    uint16 csum = (uint16)((packet ^ (packet >> 4U) ^ (packet >> 8U)) & 0x0FU);
    return (uint16)((packet << 4U) | csum);
}

static void driver_dshot600_set_all_motor_low (void)
{
    (*(volatile uint32 *)g_dshot600.gpio_clr_addr) = g_dshot600.gpio_all_mask;
}

static void driver_dshot600_wait_until_cycle (uint32 target_cycle)
{
    while ((int32)(DWT->CYCCNT - target_cycle) < 0)
    {
    }
}

static uint8 driver_dshot600_send_packets (uint16 p0, uint16 p1, uint16 p2, uint16 p3)
{
    uint16 bit_mask = 0U;
    uint8 bit_index = 0U;
    uint32 start_cycle = 0U;
    uint32 clear_0_masks[16] = {0};

    if (!g_dshot600.initialized)
    {
        return 1U;
    }

    for (bit_mask = 0x8000U, bit_index = 0U; bit_mask != 0U; bit_mask >>= 1U, bit_index++)
    {
        uint32 clear_0_mask = 0U;

        if ((p0 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 0U));
        if ((p1 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 1U));
        if ((p2 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 2U));
        if ((p3 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 3U));

        clear_0_masks[bit_index] = clear_0_mask;
    }

    __disable_irq();

    for (bit_mask = 0x8000U, bit_index = 0U; bit_mask != 0U; bit_mask >>= 1U, bit_index++)
    {
        start_cycle = DWT->CYCCNT;

        (*(volatile uint32 *)g_dshot600.gpio_set_addr) = g_dshot600.gpio_all_mask;

        driver_dshot600_wait_until_cycle(start_cycle + g_dshot600.t0h_cycles);

        if (clear_0_masks[bit_index] != 0U)
        {
            (*(volatile uint32 *)g_dshot600.gpio_clr_addr) = clear_0_masks[bit_index];
        }

        driver_dshot600_wait_until_cycle(start_cycle + g_dshot600.t1h_cycles);

        (*(volatile uint32 *)g_dshot600.gpio_clr_addr) = g_dshot600.gpio_all_mask;

        driver_dshot600_wait_until_cycle(start_cycle + g_dshot600.bit_cycles);
    }

    driver_dshot600_wait_until_cycle(DWT->CYCCNT + g_dshot600.reset_cycles);

    __enable_irq();
    return 0U;
}

uint8 driver_dshot600_init (void)
{
    uint8 i = 0U;

    gpio_init(P5_0, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P5_1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P5_2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P5_3, GPO, GPIO_LOW, GPO_PUSH_PULL);

    g_dshot600.gpio_set_addr = DRIVER_DSHOT600_GPIO_BASE_ADDR +
                               ((uint32)DRIVER_DSHOT600_MOTOR_PORT_INDEX * DRIVER_DSHOT600_GPIO_PORT_STRIDE) +
                               DRIVER_DSHOT600_GPIO_OUT_SET_OFFSET;
    g_dshot600.gpio_clr_addr = DRIVER_DSHOT600_GPIO_BASE_ADDR +
                               ((uint32)DRIVER_DSHOT600_MOTOR_PORT_INDEX * DRIVER_DSHOT600_GPIO_PORT_STRIDE) +
                               DRIVER_DSHOT600_GPIO_OUT_CLR_OFFSET;
    g_dshot600.gpio_all_mask = (uint32)(0x0FUL << DRIVER_DSHOT600_MOTOR_PIN_SHIFT);

    SystemCoreClockUpdate();
    g_dshot600.core_clock_hz = SystemCoreClock;
    g_dshot600.bit_cycles = driver_dshot600_ns_to_cycles(DRIVER_DSHOT600_BIT_NS, g_dshot600.core_clock_hz);
    g_dshot600.t0h_cycles = driver_dshot600_ns_to_cycles(DRIVER_DSHOT600_T0H_NS, g_dshot600.core_clock_hz);
    g_dshot600.t1h_cycles = driver_dshot600_ns_to_cycles(DRIVER_DSHOT600_T1H_NS, g_dshot600.core_clock_hz);
    g_dshot600.reset_cycles = driver_dshot600_us_to_cycles(DRIVER_DSHOT600_RESET_US, g_dshot600.core_clock_hz);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    for (i = 0U; i < DRIVER_DSHOT600_MOTOR_COUNT; i++)
    {
        g_dshot600.throttle[i] = 0U;
        g_dshot600.telemetry[i] = 0U;
    }

    driver_dshot600_set_all_motor_low();
    g_dshot600.initialized = 1U;
    return 0U;
}

uint8 driver_dshot600_set_throttle (uint8 motor_index, uint16 throttle, uint8 telemetry)
{
    if (motor_index >= DRIVER_DSHOT600_MOTOR_COUNT)
    {
        return 1U;
    }

    if (throttle > DRIVER_DSHOT600_THROTTLE_MAX)
    {
        throttle = DRIVER_DSHOT600_THROTTLE_MAX;
    }
    if ((throttle != 0U) && (throttle < DRIVER_DSHOT600_THROTTLE_MIN))
    {
        throttle = DRIVER_DSHOT600_THROTTLE_MIN;
    }

    g_dshot600.throttle[motor_index] = throttle;
    g_dshot600.telemetry[motor_index] = telemetry ? 1U : 0U;
    return 0U;
}

uint8 driver_dshot600_set_throttle_all (uint16 m1, uint16 m2, uint16 m3, uint16 m4, uint8 telemetry)
{
    (void)driver_dshot600_set_throttle(0U, m1, telemetry);
    (void)driver_dshot600_set_throttle(1U, m2, telemetry);
    (void)driver_dshot600_set_throttle(2U, m3, telemetry);
    (void)driver_dshot600_set_throttle(3U, m4, telemetry);
    return 0U;
}

uint8 driver_dshot600_send_frame (void)
{
    uint16 p0 = driver_dshot600_build_packet(g_dshot600.throttle[0], g_dshot600.telemetry[0]);
    uint16 p1 = driver_dshot600_build_packet(g_dshot600.throttle[1], g_dshot600.telemetry[1]);
    uint16 p2 = driver_dshot600_build_packet(g_dshot600.throttle[2], g_dshot600.telemetry[2]);
    uint16 p3 = driver_dshot600_build_packet(g_dshot600.throttle[3], g_dshot600.telemetry[3]);

    return driver_dshot600_send_packets(p0, p1, p2, p3);
}

uint8 driver_dshot600_send_command (uint16 command, uint8 repeat, uint8 telemetry)
{
    uint8 i = 0U;
    uint16 packet = 0U;

    if (command > DRIVER_DSHOT600_COMMAND_MAX)
    {
        return 1U;
    }

    if (repeat == 0U)
    {
        repeat = 1U;
    }

    packet = driver_dshot600_build_packet(command, telemetry ? 1U : 0U);

    for (i = 0U; i < repeat; i++)
    {
        (void)driver_dshot600_send_packets(packet, packet, packet, packet);
    }

    return 0U;
}

void driver_dshot600_stop_all (void)
{
    uint8 i = 0U;

    for (i = 0U; i < DRIVER_DSHOT600_MOTOR_COUNT; i++)
    {
        g_dshot600.throttle[i] = 0U;
        g_dshot600.telemetry[i] = 0U;
    }

    (void)driver_dshot600_send_frame();
}

uint8 driver_dshot600_is_ready (void)
{
    return g_dshot600.initialized;
}
