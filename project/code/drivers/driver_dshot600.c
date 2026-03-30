#include "zf_common_headfile.h"

#include "driver_dshot600.h"

#include "sysclk/cy_sysclk.h"
#include "tcpwm/cy_tcpwm_counter.h"
#include "dma/cy_pdma.h"
#include "trigmux/cy_trigmux.h"

#define DRIVER_DSHOT600_MOTOR_PIN_SHIFT       (3U)
#define DRIVER_DSHOT600_MOTOR_PORT_INDEX      (18U)

#define DRIVER_DSHOT600_GPIO_BASE_ADDR        (0x40310000UL)
#define DRIVER_DSHOT600_GPIO_PORT_STRIDE      (0x80UL)
#define DRIVER_DSHOT600_GPIO_OUT_CLR_OFFSET   (0x04UL)
#define DRIVER_DSHOT600_GPIO_OUT_SET_OFFSET   (0x08UL)

#define DRIVER_DSHOT600_TIMER_CH              (50U)

#define DRIVER_DSHOT600_SLOT_HZ               (3000000UL)
#define DRIVER_DSHOT600_SLOT_PER_BIT          (5U)
#define DRIVER_DSHOT600_BIT_COUNT             (16U)
#define DRIVER_DSHOT600_RESET_SLOTS           (30U)
#define DRIVER_DSHOT600_TOTAL_SLOTS           ((DRIVER_DSHOT600_BIT_COUNT * DRIVER_DSHOT600_SLOT_PER_BIT) + DRIVER_DSHOT600_RESET_SLOTS)

#define DRIVER_DSHOT600_CLR0_SLOT_OFFSET      (2U)
#define DRIVER_DSHOT600_CLR1_SLOT_OFFSET      (4U)

#define DRIVER_DSHOT600_PDMA_CH_SET           (8U)
#define DRIVER_DSHOT600_PDMA_CH_CLR           (9U)

#define DRIVER_DSHOT600_TIMEOUT_US            (200U)

#define DRIVER_DSHOT600_COMMAND_MAX           (47U)

typedef struct
{
    uint8 initialized;
    uint8 dma_ready;
    uint16 throttle[DRIVER_DSHOT600_MOTOR_COUNT];
    uint8 telemetry[DRIVER_DSHOT600_MOTOR_COUNT];
    uint32 gpio_set_addr;
    uint32 gpio_clr_addr;
    uint32 gpio_all_mask;
    cy_stc_pdma_descr_t set_descr;
    cy_stc_pdma_descr_t clr_descr;
    uint32 set_buffer[DRIVER_DSHOT600_TOTAL_SLOTS];
    uint32 clr_buffer[DRIVER_DSHOT600_TOTAL_SLOTS];
} driver_dshot600_state_struct;

static driver_dshot600_state_struct g_dshot600 = {0};

static uint8 driver_dshot600_dma_wait_done (void)
{
    uint32 start_tick = systick_get_us();

    while (0U == Cy_PDMA_Chnl_GetInterruptStatusMasked(DW0, DRIVER_DSHOT600_PDMA_CH_CLR))
    {
        if ((systick_get_us() - start_tick) > DRIVER_DSHOT600_TIMEOUT_US)
        {
            return 1U;
        }
    }

    Cy_PDMA_Chnl_ClearInterrupt(DW0, DRIVER_DSHOT600_PDMA_CH_CLR);
    return 0U;
}

static void driver_dshot600_build_slot_buffers (uint16 p0, uint16 p1, uint16 p2, uint16 p3)
{
    uint16 bit_mask = 0U;
    uint8 bit_index = 0U;

    for (bit_index = 0U; bit_index < DRIVER_DSHOT600_TOTAL_SLOTS; bit_index++)
    {
        g_dshot600.set_buffer[bit_index] = 0U;
        g_dshot600.clr_buffer[bit_index] = 0U;
    }

    for (bit_mask = 0x8000U, bit_index = 0U; bit_mask != 0U; bit_mask >>= 1U, bit_index++)
    {
        uint32 base = (uint32)bit_index * DRIVER_DSHOT600_SLOT_PER_BIT;
        uint32 clear_0_mask = 0U;
        uint32 clear_1_mask = 0U;

        if ((p0 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 0U));
        else                        clear_1_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 0U));

        if ((p1 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 1U));
        else                        clear_1_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 1U));

        if ((p2 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 2U));
        else                        clear_1_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 2U));

        if ((p3 & bit_mask) == 0U) clear_0_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 3U));
        else                        clear_1_mask |= (1UL << (DRIVER_DSHOT600_MOTOR_PIN_SHIFT + 3U));

        g_dshot600.set_buffer[base] = g_dshot600.gpio_all_mask;
        g_dshot600.clr_buffer[base + DRIVER_DSHOT600_CLR0_SLOT_OFFSET] = clear_0_mask;
        g_dshot600.clr_buffer[base + DRIVER_DSHOT600_CLR1_SLOT_OFFSET] = clear_1_mask;
    }
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

static uint8 driver_dshot600_dma_init (void)
{
    cy_stc_pdma_descr_config_t set_descr_cfg;
    cy_stc_pdma_descr_config_t clr_descr_cfg;
    cy_stc_pdma_chnl_config_t set_chnl_cfg;
    cy_stc_pdma_chnl_config_t clr_chnl_cfg;

    memset(&set_descr_cfg, 0, sizeof(set_descr_cfg));
    memset(&clr_descr_cfg, 0, sizeof(clr_descr_cfg));
    memset(&set_chnl_cfg, 0, sizeof(set_chnl_cfg));
    memset(&clr_chnl_cfg, 0, sizeof(clr_chnl_cfg));

    set_descr_cfg.deact = CY_PDMA_TRIG_DEACT_NO_WAIT;
    set_descr_cfg.intrType = CY_PDMA_INTR_DESCR_CMPLT;
    set_descr_cfg.trigoutType = CY_PDMA_TRIGOUT_DESCR_CMPLT;
    set_descr_cfg.chStateAtCmplt = CY_PDMA_CH_DISABLED;
    set_descr_cfg.triginType = CY_PDMA_TRIGIN_1ELEMENT;
    set_descr_cfg.dataSize = CY_PDMA_WORD;
    set_descr_cfg.srcTxfrSize = CY_PDMA_TXFR_SIZE_WORD;
    set_descr_cfg.destTxfrSize = CY_PDMA_TXFR_SIZE_WORD;
    set_descr_cfg.descrType = CY_PDMA_1D_TRANSFER;
    set_descr_cfg.srcAddr = (void *)g_dshot600.set_buffer;
    set_descr_cfg.destAddr = (void *)g_dshot600.gpio_set_addr;
    set_descr_cfg.srcXincr = (int32)sizeof(uint32);
    set_descr_cfg.destXincr = 0;
    set_descr_cfg.xCount = DRIVER_DSHOT600_TOTAL_SLOTS;
    set_descr_cfg.descrNext = NULL;

    clr_descr_cfg = set_descr_cfg;
    clr_descr_cfg.srcAddr = (void *)g_dshot600.clr_buffer;
    clr_descr_cfg.destAddr = (void *)g_dshot600.gpio_clr_addr;

    if (CY_PDMA_SUCCESS != Cy_PDMA_Descr_Init(&g_dshot600.set_descr, &set_descr_cfg))
    {
        return 1U;
    }
    if (CY_PDMA_SUCCESS != Cy_PDMA_Descr_Init(&g_dshot600.clr_descr, &clr_descr_cfg))
    {
        return 1U;
    }

    set_chnl_cfg.PDMA_Descriptor = &g_dshot600.set_descr;
    set_chnl_cfg.preemptable = 0U;
    set_chnl_cfg.priority = 1U;
    set_chnl_cfg.enable = 0U;

    clr_chnl_cfg.PDMA_Descriptor = &g_dshot600.clr_descr;
    clr_chnl_cfg.preemptable = 0U;
    clr_chnl_cfg.priority = 1U;
    clr_chnl_cfg.enable = 0U;

    if (CY_PDMA_SUCCESS != Cy_PDMA_Chnl_Init(DW0, DRIVER_DSHOT600_PDMA_CH_SET, &set_chnl_cfg))
    {
        return 1U;
    }
    if (CY_PDMA_SUCCESS != Cy_PDMA_Chnl_Init(DW0, DRIVER_DSHOT600_PDMA_CH_CLR, &clr_chnl_cfg))
    {
        return 1U;
    }

    Cy_PDMA_Chnl_ClearInterrupt(DW0, DRIVER_DSHOT600_PDMA_CH_SET);
    Cy_PDMA_Chnl_ClearInterrupt(DW0, DRIVER_DSHOT600_PDMA_CH_CLR);
    Cy_PDMA_Chnl_SetInterruptMask(DW0, DRIVER_DSHOT600_PDMA_CH_CLR);
    Cy_PDMA_Enable(DW0);

    if (CY_TRIGMUX_SUCCESS != Cy_TrigMux_Connect(TRIG_IN_MUX_3_TCPWM_16_TR_OUT050,
                                                 TRIG_OUT_MUX_3_PDMA0_TR_IN8,
                                                 CY_TR_MUX_TR_INV_DISABLE,
                                                 TRIGGER_TYPE_EDGE,
                                                 0U))
    {
        return 1U;
    }

    if (CY_TRIGMUX_SUCCESS != Cy_TrigMux_Connect(TRIG_IN_MUX_3_TCPWM_16_TR_OUT050,
                                                 TRIG_OUT_MUX_3_PDMA0_TR_IN9,
                                                 CY_TR_MUX_TR_INV_DISABLE,
                                                 TRIGGER_TYPE_EDGE,
                                                 0U))
    {
        return 1U;
    }

    g_dshot600.dma_ready = 1U;
    return 0U;
}

static uint8 driver_dshot600_send_packets (uint16 p0, uint16 p1, uint16 p2, uint16 p3)
{
    if ((!g_dshot600.initialized) || (!g_dshot600.dma_ready))
    {
        return 1U;
    }

    driver_dshot600_build_slot_buffers(p0, p1, p2, p3);

    Cy_PDMA_Descr_SetSrcAddr(&g_dshot600.set_descr, (const void *)g_dshot600.set_buffer);
    Cy_PDMA_Descr_SetSrcAddr(&g_dshot600.clr_descr, (const void *)g_dshot600.clr_buffer);
    Cy_PDMA_Chnl_SetDescr(DW0, DRIVER_DSHOT600_PDMA_CH_SET, &g_dshot600.set_descr);
    Cy_PDMA_Chnl_SetDescr(DW0, DRIVER_DSHOT600_PDMA_CH_CLR, &g_dshot600.clr_descr);

    Cy_PDMA_Chnl_ClearInterrupt(DW0, DRIVER_DSHOT600_PDMA_CH_SET);
    Cy_PDMA_Chnl_ClearInterrupt(DW0, DRIVER_DSHOT600_PDMA_CH_CLR);

    Cy_PDMA_Chnl_Enable(DW0, DRIVER_DSHOT600_PDMA_CH_SET);
    Cy_PDMA_Chnl_Enable(DW0, DRIVER_DSHOT600_PDMA_CH_CLR);

    if (driver_dshot600_dma_wait_done())
    {
        Cy_PDMA_Chnl_Disable(DW0, DRIVER_DSHOT600_PDMA_CH_SET);
        Cy_PDMA_Chnl_Disable(DW0, DRIVER_DSHOT600_PDMA_CH_CLR);
        driver_dshot600_set_all_motor_low();
        return 1U;
    }

    driver_dshot600_set_all_motor_low();
    return 0U;
}

uint8 driver_dshot600_init (void)
{
    uint8 i = 0U;
    cy_stc_tcpwm_counter_config_t timer_cfg;

    gpio_init(P18_3, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P18_4, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P18_5, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P18_6, GPO, GPIO_LOW, GPO_PUSH_PULL);

    g_dshot600.gpio_set_addr = DRIVER_DSHOT600_GPIO_BASE_ADDR +
                               ((uint32)DRIVER_DSHOT600_MOTOR_PORT_INDEX * DRIVER_DSHOT600_GPIO_PORT_STRIDE) +
                               DRIVER_DSHOT600_GPIO_OUT_SET_OFFSET;
    g_dshot600.gpio_clr_addr = DRIVER_DSHOT600_GPIO_BASE_ADDR +
                               ((uint32)DRIVER_DSHOT600_MOTOR_PORT_INDEX * DRIVER_DSHOT600_GPIO_PORT_STRIDE) +
                               DRIVER_DSHOT600_GPIO_OUT_CLR_OFFSET;
    g_dshot600.gpio_all_mask = (uint32)(0x0FUL << DRIVER_DSHOT600_MOTOR_PIN_SHIFT);

    Cy_SysClk_PeriphAssignDivider((en_clk_dst_t)(DRIVER_DSHOT600_TIMER_CH + PCLK_TCPWM0_CLOCKS0),
                                  (cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT,
                                  0ul);
    Cy_SysClk_PeriphSetDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul, 0u);
    Cy_SysClk_PeriphEnableDivider((cy_en_divider_types_t)CY_SYSCLK_DIV_16_BIT, 0ul);

    memset(&timer_cfg, 0, sizeof(timer_cfg));
    timer_cfg.period = (CY_INITIAL_TARGET_PERI_FREQ / DRIVER_DSHOT600_SLOT_HZ) - 1U;
    timer_cfg.clockPrescaler = CY_TCPWM_PRESCALER_DIVBY_1;
    timer_cfg.runMode = CY_TCPWM_COUNTER_CONTINUOUS;
    timer_cfg.countDirection = CY_TCPWM_COUNTER_COUNT_UP;
    timer_cfg.debug_pause = false;
    timer_cfg.compareOrCapture = CY_TCPWM_COUNTER_MODE_COMPARE;
    timer_cfg.compare0 = 0U;
    timer_cfg.compare0_buff = 0U;
    timer_cfg.enableCompare0Swap = false;
    timer_cfg.interruptSources = CY_TCPWM_INT_NONE;
    timer_cfg.capture0InputMode = CY_TCPWM_INPUT_LEVEL;
    timer_cfg.capture0Input = 0x3FU;
    timer_cfg.compare1 = 0U;
    timer_cfg.compare1_buff = 0U;
    timer_cfg.enableCompare1Swap = false;
    timer_cfg.capture1InputMode = CY_TCPWM_INPUT_LEVEL;
    timer_cfg.capture1Input = 0x3FU;
    timer_cfg.reloadInputMode = CY_TCPWM_INPUT_LEVEL;
    timer_cfg.reloadInput = 0x3FU;
    timer_cfg.startInputMode = CY_TCPWM_INPUT_LEVEL;
    timer_cfg.startInput = 0x3FU;
    timer_cfg.stopInputMode = CY_TCPWM_INPUT_LEVEL;
    timer_cfg.stopInput = 0x3FU;
    timer_cfg.countInputMode = CY_TCPWM_INPUT_LEVEL;
    timer_cfg.countInput = 1uL;
    timer_cfg.trigger0EventCfg = CY_TCPWM_COUNTER_OVERFLOW;
    timer_cfg.trigger1EventCfg = CY_TCPWM_COUNTER_DISABLED;

    if (0U != Cy_Tcpwm_Counter_Init((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[DRIVER_DSHOT600_TIMER_CH], &timer_cfg))
    {
        return 1U;
    }

    Cy_Tcpwm_Counter_Enable((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[DRIVER_DSHOT600_TIMER_CH]);
    Cy_Tcpwm_TriggerStart((volatile stc_TCPWM_GRP_CNT_t *)&TCPWM0->GRP[0].CNT[DRIVER_DSHOT600_TIMER_CH]);

    if (driver_dshot600_dma_init())
    {
        return 1U;
    }

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
