#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/pm.h>

#include <cmsis_core.h>
#include <pmu_manager.h>
#include <utils.h>
#include <platform_rtc.h>
#include <rom_api_for_zephyr.h>
#include <pm.h>

#include <trace.h>

#include <zephyr/logging/log.h>
#include <power_manager_slave.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);


#ifdef CONFIG_PM_POLICY_CUSTOM
extern void power_manager_slave_send_idle_check_request(void);
extern void power_manager_slave_receive_m2s_inact_msg(PMM2SInactivateMessage *p_pm_m2s_inact_msg);
bool power_manager_slave_inact_action_handler_zephyr(void)
{
    bool ret = false;

    if (!power_manager_slave_system.suspended)
    {
        /* 1. disable IRQ */
        __disable_irq();

        bool is_interrupt = false;

        /* 2. send PM_ACTION_IDLE_CHECK_REQUEST to master */
        power_manager_slave_send_idle_check_request();

        /* 3. continuously receive message */
        while (1)
        {
            PMM2SInactivateMessage pm_m2s_inact_msg;
            power_manager_slave_receive_m2s_inact_msg(&pm_m2s_inact_msg);

            if (!power_manager_slave_inact_msg_handler(&pm_m2s_inact_msg, &is_interrupt))
            {
                break;
            }
        }

        if (power_manager_interface_get_unit_status(PM_SLAVE_0, PM_UNIT_PLATFORM) == PM_UNIT_INACTIVE)
        {
            ret = true;
        }
        else
        {
            /* 4. enable IRQ */
            __enable_irq();
        }
    }

    return ret;
}

const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
    ARG_UNUSED(cpu);
    ARG_UNUSED(ticks);

    static const struct pm_state_info state[] =
    {
        {.state = PM_STATE_RUNTIME_IDLE},
        {.state = PM_STATE_SUSPEND_TO_RAM, .substate_id = 0},
        {.state = PM_STATE_SUSPEND_TO_RAM, .substate_id = 1},
        {.state = PM_STATE_SOFT_OFF},
    };

    extern void (*thermal_meter_read)(void);
    thermal_meter_read();

    extern void log_buffer_trigger_schedule_in_km4_idle_task(void);
    log_buffer_trigger_schedule_in_km4_idle_task();

    if (power_manager_slave_inact_action_handler_zephyr())
    {
        switch (platform_pm_get_power_mode())
        {
        case PLATFORM_POWERDOWN:
            return &state[3];
            break;

        case PLATFORM_DLPS_PFM:
            return &state[1];
            break;

        case PLATFORM_DLPS_RET:
            return &state[2];
            break;

        default:
            break;
        }
    }

    return &state[0];
}
#endif


void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    if (state == PM_STATE_RUNTIME_IDLE)
       return;
    k_sched_unlock(); // Corresponds to the "k_sched_lock" in pm_system_suspend() function. Becasue in rtk flow, we have locked it once.

    __set_BASEPRI(0); //clear the basepri, may not need
    __DSB();

    switch (state)
    {
    case PM_STATE_SUSPEND_TO_RAM:
        switch (substate_id)
        {
        case 0:
            lop_setting(PLATFORM_DLPS_PFM);
            break;

        case 1:
            lop_setting(PLATFORM_DLPS_RET);
            break;

        default:
            break;
        }
        break;

    case PM_STATE_SOFT_OFF:
        lop_setting(PLATFORM_POWERDOWN);
        AON_REG_WRITE_BITFIELD(AON_NS_REG0X_FW_GENERAL_NS, km4_pon_boot_done, 0);
        break;

    default:
        LOG_DBG("Unsupported power state %u", state);
        break;
    }

    AON_REG2X_SYS_TYPE reg2x_sys = {.d32 = AON_REG_READ(AON_REG2X_SYS)};
    reg2x_sys.FW_enter_lps = true; // trigger AON FSM power off sequence
    AON_REG_WRITE(AON_REG2X_SYS, reg2x_sys.d32);


    while (1) {};
}

/* Handle SOC specific activity after Low Power Mode Exit */
void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(state);
    ARG_UNUSED(substate_id);

}

extern void NMI_Handler(void);
/* Initialize power system */
static int rtl87x2g_power_init(void)
{
    int ret = 0;

    printk("in rtl87x2g_power_init");

    bt_power_mode_set(BTPOWER_DEEP_SLEEP);
    //bt_power_mode_set(BTPOWER_ACTIVE);

    power_mode_set(POWER_DLPS_MODE);
    //power_mode_set(POWER_ACTIVE_MODE);

    z_arm_nmi_set_handler(NMI_Handler);

    return ret;
}

SYS_INIT(rtl87x2g_power_init, APPLICATION, 1); //do it after lowerstack entry
