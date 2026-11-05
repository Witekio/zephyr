/*
 * Copyright (c) 2026 Witekio
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/opamp.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <stm32_ll_opamp.h>
#include <stm32_ll_pwr.h>

LOG_MODULE_REGISTER(opamp_stm32u5, CONFIG_OPAMP_LOG_LEVEL);

#define DT_DRV_COMPAT st_stm32u5_opamp

#define STM32_OPAMP_TRIM_DELAY_MS      1    /* tOFFTRIM, see device datasheet */
#define STM32_OPAMP_TRIM_VAL_MAX       0x1f /* maximum allowed trimming value */
#define STM32_OPAMP_TRIM_VAL_UNDEFINED 0xff /* undefined trimming value */

#define OPAMP_INM_NC    LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP_INM_VINM0 LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP_INM_VINM1 LL_OPAMP_INPUT_INVERT_IO1

#define OPAMP_INP_VINP0 LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP_INP_DAC   LL_OPAMP_INPUT_NONINVERT_DAC

struct stm32_opamp_config {
	OPAMP_TypeDef *opamp;
	struct stm32_pclken *pclken;
	const size_t pclk_len;
	const struct pinctrl_dev_config *pincfg;
	const uint32_t inm;
	const uint32_t inp;
	uint32_t power_mode;
	uint8_t functional_mode;
	uint8_t pmos_trimming_value;
	uint8_t nmos_trimming_value;
	bool self_calibration;
};

struct stm32_opamp_data {
	struct k_mutex dev_mtx;
};

static void stm32_opamp_config_log_dbg(const struct device *dev)
{
	const struct stm32_opamp_config *cfg = dev->config;
	const char *self_calib_str = cfg->self_calibration ? "true" : "false";

	LOG_DBG("%s config:\n"
		"  functional_mode: 0x%x\n"
		"  power_mode: 0x%x\n"
		"  inm: 0x%08x\n"
		"  inp: 0x%08x\n"
		"  self_calibration: %s\n"
		"  pmos_trimming_value: 0x%02x\n"
		"  nmos_trimming_value: 0x%02x\n"
		"  OPAMPx_CSR: 0x%08x\n",
		dev->name, cfg->functional_mode, cfg->power_mode, cfg->inm, cfg->inp,
		self_calib_str, (uint32_t)cfg->pmos_trimming_value,
		(uint32_t)cfg->nmos_trimming_value, (uint32_t)cfg->opamp->CSR);
}

static int stm32_opamp_pm_callback(const struct device *dev, enum pm_device_action action)
{
	const struct stm32_opamp_config *cfg = dev->config;
	OPAMP_TypeDef *opamp = cfg->opamp;

	if (action == PM_DEVICE_ACTION_RESUME) {
		LL_OPAMP_Enable(opamp);
	}

	if (action == PM_DEVICE_ACTION_SUSPEND) {
		LL_OPAMP_Disable(opamp);
	}

	return 0;
}

static int stm32_opamp_set_gain(const struct device *dev, enum opamp_gain gain)
{
	const struct stm32_opamp_config *cfg = dev->config;
	struct stm32_opamp_data *data = dev->data;
	OPAMP_TypeDef *opamp = cfg->opamp;
	uint32_t ll_gain;

	k_mutex_lock(&data->dev_mtx, K_FOREVER);

	/* Gain only applies in PGA (non_inverting) mode */
	if (cfg->functional_mode != OPAMP_FUNCTIONAL_MODE_NON_INVERTING) {
		LOG_DBG("%s: gain not supported in functional mode %u", dev->name,
			cfg->functional_mode);
		k_mutex_unlock(&data->dev_mtx);
		return 0;
	}

	switch (gain) {
	case OPAMP_GAIN_2:
		ll_gain = LL_OPAMP_PGA_GAIN_2;
		break;
	case OPAMP_GAIN_4:
		ll_gain = LL_OPAMP_PGA_GAIN_4;
		break;
	case OPAMP_GAIN_8:
		ll_gain = LL_OPAMP_PGA_GAIN_8;
		break;
	case OPAMP_GAIN_16:
		ll_gain = LL_OPAMP_PGA_GAIN_16;
		break;
	default:
		LOG_ERR("%s: invalid gain %d", dev->name, gain);
		k_mutex_unlock(&data->dev_mtx);
		return -EINVAL;
	}

	LL_OPAMP_SetPGAGain(opamp, ll_gain);

	k_mutex_unlock(&data->dev_mtx);
	return 0;
}

static int stm32_opamp_set_functional_mode(const struct device *dev)
{
	const struct stm32_opamp_config *cfg = dev->config;
	OPAMP_TypeDef *opamp = cfg->opamp;
	uint32_t ll_functional_mode = UINT32_MAX;

	switch (cfg->functional_mode) {
	case OPAMP_FUNCTIONAL_MODE_STANDALONE:
		LOG_DBG("%s: functional_mode: standalone", dev->name);
		/* Standalone mode: external feedback network defines the gain
		 *	INP selected by DTS (OPAMPx_CSR.VP_SEL)
		 *	INM selected by DTS, VINM0 or VINM1 (OPAMPx_CSR.VM_SEL = b00 or b01)
		 */
		LL_OPAMP_SetInputNonInverting(opamp, cfg->inp);
		LL_OPAMP_SetInputInverting(opamp, cfg->inm);

		ll_functional_mode = LL_OPAMP_MODE_STANDALONE;
		break;
	case OPAMP_FUNCTIONAL_MODE_FOLLOWER:
		LOG_DBG("%s: functional_mode: follower", dev->name);
		/* Follower mode:
		 *	INP selected by DTS (OPAMPx_CSR.VP_SEL)
		 *	INM internally connected to VOUT (OPAMPx_CSR.OPAMODE = b11),
		 *	not externally connected (OPAMPx_CSR.VM_SEL = b1x)
		 */
		LL_OPAMP_SetInputNonInverting(opamp, cfg->inp);
		LL_OPAMP_SetInputInverting(opamp, OPAMP_INM_NC);

		ll_functional_mode = LL_OPAMP_MODE_FOLLOWER;
		break;
	case OPAMP_FUNCTIONAL_MODE_NON_INVERTING:
		LOG_DBG("%s: functional_mode: non_inverting", dev->name);
		/* PGA mode: gain set by the internal resistor network
		 *	INP selected by DTS (OPAMPx_CSR.VP_SEL)
		 *	INM not externally connected (OPAMPx_CSR.VM_SEL = b1x), or connected to
		 *	VINM0/VINM1 for filtering (OPAMPx_CSR.VM_SEL = b00 or b01)
		 */
		LL_OPAMP_SetInputNonInverting(opamp, cfg->inp);
		LL_OPAMP_SetInputInverting(opamp, cfg->inm);

		ll_functional_mode = LL_OPAMP_MODE_PGA;
		break;
	default:
		LOG_ERR("%s: invalid functional_mode: %d", dev->name, cfg->functional_mode);
		return -EINVAL;
	}

	LL_OPAMP_SetMode(opamp, LL_OPAMP_MODE_FUNCTIONAL);
	LL_OPAMP_SetPGAGain(opamp, 0); /* gain reset value */
	LL_OPAMP_SetFunctionalMode(opamp, ll_functional_mode);

	return 0;
}

static uint32_t stm32_opamp_calibrate_pair(OPAMP_TypeDef *opamp, uint32_t power_mode, uint32_t pair)
{
	uint32_t trim = 16U;

	LL_OPAMP_SetCalibrationSelection(opamp, pair);

	/* Binary search for the trim value where CALOUT switches from 0 to 1 */
	for (uint32_t step = 8U; step > 0U; step >>= 1) {
		LL_OPAMP_SetTrimmingValue(opamp, power_mode, pair, trim);
		k_msleep(STM32_OPAMP_TRIM_DELAY_MS);

		if (LL_OPAMP_IsCalibrationOutputSet(opamp) != 0U) {
			trim -= step;
		} else {
			trim += step;
		}
	}

	/* The search ends either on the switching point or one step below it, capped at 0x1f */
	LL_OPAMP_SetTrimmingValue(opamp, power_mode, pair, trim);
	k_msleep(STM32_OPAMP_TRIM_DELAY_MS);
	if (LL_OPAMP_IsCalibrationOutputSet(opamp) == 0U && trim < STM32_OPAMP_TRIM_VAL_MAX) {
		trim++;
		LL_OPAMP_SetTrimmingValue(opamp, power_mode, pair, trim);
	}

	return trim;
}

static void stm32_opamp_self_calibration(const struct device *dev)
{
	const struct stm32_opamp_config *cfg = dev->config;
	OPAMP_TypeDef *opamp = cfg->opamp;
	uint32_t trim_n;
	uint32_t trim_p;

	/* RM0456 OPAMP calibration: OPAMP disabled, power mode already set */
	LL_OPAMP_Disable(opamp);
	LL_OPAMP_SetFunctionalMode(opamp, LL_OPAMP_MODE_STANDALONE);
	LL_OPAMP_SetTrimmingMode(opamp, LL_OPAMP_TRIMMING_USER);
	LL_OPAMP_SetMode(opamp, LL_OPAMP_MODE_CALIBRATION);
	LL_OPAMP_Enable(opamp);

	trim_n = stm32_opamp_calibrate_pair(opamp, cfg->power_mode, LL_OPAMP_TRIMMING_NMOS);
	trim_p = stm32_opamp_calibrate_pair(opamp, cfg->power_mode, LL_OPAMP_TRIMMING_PMOS);

	LL_OPAMP_Disable(opamp);
	LL_OPAMP_SetMode(opamp, LL_OPAMP_MODE_FUNCTIONAL);
	LL_OPAMP_SetCalibrationSelection(opamp, LL_OPAMP_TRIMMING_NMOS); /* CALSEL reset value */

	LOG_DBG("%s: trimming NMOS 0x%02x PMOS 0x%02x", dev->name, trim_n, trim_p);
}

static int stm32_opamp_init(const struct device *dev)
{
	const struct device *const clk = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);
	const struct stm32_opamp_config *cfg = dev->config;
	struct stm32_opamp_data *data = dev->data;
	OPAMP_TypeDef *opamp = cfg->opamp;
	int ret = 0;

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	/* Enable OPAMP bus clock */
	ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->pclken[0]);
	if (ret != 0) {
		LOG_ERR("%s clock op failed (%d)", dev->name, ret);
		return ret;
	}

	/* Configure OPAMP inputs as specified in Device Tree */
	ret = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s pinctrl setup failed (%d)", dev->name, ret);
		return ret;
	}

	k_mutex_lock(&data->dev_mtx, K_FOREVER);

	LL_PWR_EnableVddA();

	/* OPA_RANGE is common to both OPAMPs and must be set before the first one is enabled.
	 * Only the high range is valid on STM32U5, low is reserved (RM0456 38.5.1).
	 */
	if (LL_OPAMP_GetCommonPowerRange(OPAMP12_COMMON) != LL_OPAMP_POWERSUPPLY_RANGE_HIGH) {
		LL_OPAMP_SetCommonPowerRange(OPAMP12_COMMON, LL_OPAMP_POWERSUPPLY_RANGE_HIGH);
	}

	LL_OPAMP_SetPowerMode(opamp, cfg->power_mode);

	if (cfg->self_calibration) {
		stm32_opamp_self_calibration(dev);
	}

	if (cfg->pmos_trimming_value != STM32_OPAMP_TRIM_VAL_UNDEFINED) {
		LL_OPAMP_SetTrimmingMode(opamp, LL_OPAMP_TRIMMING_USER);
		LL_OPAMP_SetTrimmingValue(opamp, cfg->power_mode, LL_OPAMP_TRIMMING_PMOS,
					  cfg->pmos_trimming_value);
	}

	if (cfg->nmos_trimming_value != STM32_OPAMP_TRIM_VAL_UNDEFINED) {
		LL_OPAMP_SetTrimmingMode(opamp, LL_OPAMP_TRIMMING_USER);
		LL_OPAMP_SetTrimmingValue(opamp, cfg->power_mode, LL_OPAMP_TRIMMING_NMOS,
					  cfg->nmos_trimming_value);
	}

	ret = stm32_opamp_set_functional_mode(dev);
	if (ret != 0) {
		k_mutex_unlock(&data->dev_mtx);
		return ret;
	}

	stm32_opamp_config_log_dbg(dev);

	k_mutex_unlock(&data->dev_mtx);
	return pm_device_driver_init(dev, stm32_opamp_pm_callback);
}

/* The STM32U5 OPAMP supports the standalone, follower and non-inverting (PGA) modes */
#define STM32_OPAMP_MODE_IS(inst, mode) (DT_INST_ENUM_IDX(inst, functional_mode) == (mode))

#define STM32_OPAMP_FUNC_MODE_ASSERT(inst)                                                         \
	BUILD_ASSERT(STM32_OPAMP_MODE_IS(inst, OPAMP_FUNCTIONAL_MODE_STANDALONE) ||                \
			     STM32_OPAMP_MODE_IS(inst, OPAMP_FUNCTIONAL_MODE_FOLLOWER) ||          \
			     STM32_OPAMP_MODE_IS(inst, OPAMP_FUNCTIONAL_MODE_NON_INVERTING),       \
		     "Unsupported functional mode")

/* Standalone mode needs an external inverting input, VINM0 or VINM1 */
#define STM32_OPAMP_INM_ASSERT(inst)                                                               \
	BUILD_ASSERT(DT_INST_ENUM_IDX(inst, functional_mode) !=                                    \
				     OPAMP_FUNCTIONAL_MODE_STANDALONE ||                           \
			     DT_INST_NODE_HAS_PROP(inst, inm),                                     \
		     "inm is required in standalone mode")

#define STM32_OPAMP_DT_POWER_MODE(inst)                                                            \
	CONCAT(LL_OPAMP_POWERMODE_, DT_INST_STRING_TOKEN(inst, st_power_mode))

#define STM32_OPAMP_DT_INM(inst) CONCAT(OPAMP_INM_, DT_INST_STRING_TOKEN_OR(inst, inm, NC))

#define STM32_OPAMP_DT_INP(inst) CONCAT(OPAMP_INP_, DT_INST_STRING_TOKEN(inst, inp))

#define STM32_OPAMP_TRIMMING_ASSERT_IMPL(inst, value)                                              \
	BUILD_ASSERT(DT_INST_PROP(inst, value) <= STM32_OPAMP_TRIM_VAL_MAX,                        \
		     "The value exceeds maximum allowed trimming value STM32_OPAMP_TRIM_VAL_MAX")

/* This assert guarantees a value is always in allowed range at compile time */
#define STM32_OPAMP_TRIMMING_ASSERT(inst, value)                                                   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, value),                                             \
		    (STM32_OPAMP_TRIMMING_ASSERT_IMPL(inst, value)))

#define STM32_OPAMP_DT_PMOS_TRIMMING(inst)                                                         \
	DT_INST_PROP_OR(inst, st_pmos_trimming_value, STM32_OPAMP_TRIM_VAL_UNDEFINED)

#define STM32_OPAMP_DT_NMOS_TRIMMING(inst)                                                         \
	DT_INST_PROP_OR(inst, st_nmos_trimming_value, STM32_OPAMP_TRIM_VAL_UNDEFINED)

static DEVICE_API(opamp, opamp_api) = {
	.set_gain = stm32_opamp_set_gain,
};

#define STM32_OPAMP_DEVICE(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static struct stm32_pclken stm32_pclken_##inst[] = STM32_DT_INST_CLOCKS(inst);             \
                                                                                                   \
	STM32_OPAMP_TRIMMING_ASSERT(inst, st_pmos_trimming_value);                                 \
	STM32_OPAMP_TRIMMING_ASSERT(inst, st_nmos_trimming_value);                                 \
	STM32_OPAMP_FUNC_MODE_ASSERT(inst);                                                        \
	STM32_OPAMP_INM_ASSERT(inst);                                                              \
                                                                                                   \
	static const struct stm32_opamp_config stm32_opamp_config_##inst = {                       \
		.opamp = (OPAMP_TypeDef *)DT_INST_REG_ADDR(inst),                                  \
		.functional_mode = DT_INST_ENUM_IDX(inst, functional_mode),                        \
		.power_mode = STM32_OPAMP_DT_POWER_MODE(inst),                                     \
		.inp = STM32_OPAMP_DT_INP(inst),                                                   \
		.inm = STM32_OPAMP_DT_INM(inst),                                                   \
		.self_calibration = DT_INST_PROP(inst, st_enable_self_calibration),                \
		.pmos_trimming_value = STM32_OPAMP_DT_PMOS_TRIMMING(inst),                         \
		.nmos_trimming_value = STM32_OPAMP_DT_NMOS_TRIMMING(inst),                         \
		.pclken = stm32_pclken_##inst,                                                     \
		.pclk_len = DT_INST_NUM_CLOCKS(inst),                                              \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
	};                                                                                         \
                                                                                                   \
	static struct stm32_opamp_data stm32_opamp_data_##inst = {                                 \
		.dev_mtx = Z_MUTEX_INITIALIZER(stm32_opamp_data_##inst.dev_mtx),                   \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, stm32_opamp_pm_callback);                                   \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, stm32_opamp_init, PM_DEVICE_DT_INST_GET(inst),                 \
			      &stm32_opamp_data_##inst, &stm32_opamp_config_##inst, POST_KERNEL,   \
			      CONFIG_OPAMP_INIT_PRIORITY, &opamp_api);

DT_INST_FOREACH_STATUS_OKAY(STM32_OPAMP_DEVICE)
