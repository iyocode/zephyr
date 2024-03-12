/*
 * Copyright (c) 2016 Freescale Semiconductor, Inc.
 * Copyright (c) 2019, 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc_i2c

#include <errno.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/clock_control.h>
#include <fsl_i2c.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(mcux_flexcomm);

#include "i2c-priv.h"

struct mcux_flexcomm_config {
	I2C_Type *base;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	void (*irq_config_func)(const struct device *dev);
	uint32_t bitrate;
	const struct pinctrl_dev_config *pincfg;
};

struct mcux_flexcomm_data {
	i2c_master_handle_t handle;
	struct k_sem device_sync_sem;
	struct k_sem lock;
	status_t callback_status;
	uint32_t dev_config_raw;
#ifdef CONFIG_I2C_TARGET
	i2c_slave_handle_t target_handle;
	struct i2c_target_config *target_cfg;
	bool target_attached;
	bool first_read;
	bool first_write;
	bool is_write;
#endif
};

static int mcux_flexcomm_configure(const struct device *dev,
				   uint32_t dev_config_raw)
{
	const struct mcux_flexcomm_config *config = dev->config;
	struct mcux_flexcomm_data *data = dev->data;
	I2C_Type *base = config->base;
	uint32_t clock_freq;
	uint32_t baudrate;

	if (!(I2C_MODE_CONTROLLER & dev_config_raw)) {
		return -EINVAL;
	}

	if (I2C_ADDR_10_BITS & dev_config_raw) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		baudrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		baudrate = KHZ(400);
		break;
	case I2C_SPEED_FAST_PLUS:
		baudrate = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	/* Get the clock frequency */
	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	k_sem_take(&data->lock, K_FOREVER);
	I2C_MasterSetBaudRate(base, baudrate, clock_freq);
	k_sem_give(&data->lock);
	data->dev_config_raw = dev_config_raw;

	return 0;
}

static int mcux_flexcomm_get_config(const struct device *dev,
				   uint32_t* dev_config_raw)
{
	struct mcux_flexcomm_data *data = dev->data;
	*dev_config_raw = data->dev_config_raw;

	return 0;
}

static void mcux_flexcomm_master_transfer_callback(I2C_Type *base,
						   i2c_master_handle_t *handle,
						   status_t status,
						   void *userData)
{
	struct mcux_flexcomm_data *data = userData;

	ARG_UNUSED(handle);
	ARG_UNUSED(base);

	data->callback_status = status;
	k_sem_give(&data->device_sync_sem);
}

static uint32_t mcux_flexcomm_convert_flags(int msg_flags)
{
	uint32_t flags = 0U;

	if (!(msg_flags & I2C_MSG_STOP)) {
		flags |= kI2C_TransferNoStopFlag;
	}

	if (msg_flags & I2C_MSG_RESTART) {
		flags |= kI2C_TransferRepeatedStartFlag;
	}

	return flags;
}

static int mcux_flexcomm_transfer(const struct device *dev,
				  struct i2c_msg *msgs,
				  uint8_t num_msgs, uint16_t addr)
{
	const struct mcux_flexcomm_config *config = dev->config;
	struct mcux_flexcomm_data *data = dev->data;
	I2C_Type *base = config->base;
	i2c_master_transfer_t transfer;
	status_t status;
	int ret = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* Iterate over all the messages */
	for (int i = 0; i < num_msgs; i++) {
		if (I2C_MSG_ADDR_10_BITS & msgs->flags) {
			ret = -ENOTSUP;
			break;
		}

		/* Initialize the transfer descriptor */
		transfer.flags = mcux_flexcomm_convert_flags(msgs->flags);

		/* Prevent the controller to send a start condition between
		 * messages, except if explicitly requested.
		 */
		if (i != 0 && !(msgs->flags & I2C_MSG_RESTART)) {
			transfer.flags |= kI2C_TransferNoStartFlag;
		}

		transfer.slaveAddress = addr;
		transfer.direction = (msgs->flags & I2C_MSG_READ)
			? kI2C_Read : kI2C_Write;
		transfer.subaddress = 0;
		transfer.subaddressSize = 0;
		transfer.data = msgs->buf;
		transfer.dataSize = msgs->len;

		//void I2C_MasterSetTimeoutValue(I2C_Type *base, uint8_t timeout_Ms, uint32_t srcClock_Hz);

		/* Start the transfer */
		status = I2C_MasterTransferNonBlocking(base,
				&data->handle, &transfer);

		/* Return an error if the transfer didn't start successfully
		 * e.g., if the bus was busy
		 */
		if (status != kStatus_Success) {
			I2C_MasterTransferAbort(base, &data->handle);
			ret = -EIO;
			break;
		}

		/* Wait for the transfer to complete */
		k_sem_take(&data->device_sync_sem, K_FOREVER);

		/* Return an error if the transfer didn't complete
		 * successfully. e.g., nak, timeout, lost arbitration
		 */
		if (data->callback_status != kStatus_Success) {
			I2C_MasterTransferAbort(base, &data->handle);
			ret = -EIO;
			break;
		}

		/* Move to the next message */
		msgs++;
	}

	k_sem_give(&data->lock);

	return ret;
}

static int mcux_flexcomm_recover_bus(const struct device *dev)
{
	const struct mcux_flexcomm_config *config = dev->config;
	int ret = 0;

	const struct pinctrl_state *pin_state;
	ret = pinctrl_lookup_state(config->pincfg, PINCTRL_STATE_DEFAULT, &pin_state);
	if (ret != 0)
	{
		return ret;
	}

	int gpio_pin_nums[2];
	int gpio_port_nums[2];
	int num_pins = (pin_state->pin_cnt < 2) ? pin_state->pin_cnt : 2;
	for (uint8_t i =0; i < num_pins; i++)
	{
		pinctrl_soc_pin_t pin = pin_state->pins[i];
		int pin_num = ((pin & 0xFFF00000) >> 20);
		gpio_port_nums[i] = pin_num / 32;
		gpio_pin_nums[i] = pin_num % 32;
		
		IOPCTL->PIO[gpio_port_nums[i]][gpio_pin_nums[i]] = IOPCTL_PIO_FSEL(0) | IOPCTL_PIO_IBENA(0) | IOPCTL_PIO_ODENA(1);
		GPIO->DIR[gpio_port_nums[i]] |= BIT(gpio_pin_nums[i]);
	}
	LOG_INF("toggle gpio port %d, pin %d, port %d, pin %d", gpio_port_nums[0], gpio_pin_nums[0], gpio_port_nums[1], gpio_pin_nums[1]);
	for (uint8_t j = 0; j < 15; j++)
	{
		for (uint8_t i =0; i < num_pins; i++)
		{
			GPIO->CLR[gpio_port_nums[i]] = BIT(gpio_pin_nums[i]);
		}
		k_busy_wait(5);
		for (uint8_t i =0; i < num_pins; i++)
		{
			GPIO->SET[gpio_port_nums[i]] = BIT(gpio_pin_nums[i]);
		}
		k_busy_wait(5);
	}

	for (uint8_t i =0; i < num_pins; i++)
	{
		IOPCTL->PIO[gpio_port_nums[i]][gpio_pin_nums[i]] = IOPCTL_PIO_FSEL(0) | IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(0) | IOPCTL_PIO_ODENA(1);
		GPIO->DIR[gpio_port_nums[i]] &= ~(BIT(gpio_pin_nums[i]));
	}
	k_busy_wait(50);
	//reset back to I2C mode
	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	return ret;
}

#if defined(CONFIG_I2C_TARGET)
static void i2c_target_transfer_callback(I2C_Type *base,
		volatile i2c_slave_transfer_t *transfer, void *userData)
{
	struct mcux_flexcomm_data *data = userData;
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;
	static uint8_t rxVal, txVal;

	ARG_UNUSED(base);

	switch (transfer->event) {
	case kI2C_SlaveTransmitEvent:
		/* request to provide data to transmit */
		if (data->first_read && target_cb->read_requested) {
			data->first_read = false;
			target_cb->read_requested(data->target_cfg, &txVal);
		} else if (target_cb->read_processed) {
			target_cb->read_processed(data->target_cfg, &txVal);
		}

		transfer->txData = &txVal;
		transfer->txSize = 1;
		break;

	case kI2C_SlaveReceiveEvent:
		/* request to provide a buffer in which to place received data */
		if (data->first_write && target_cb->write_requested) {
			target_cb->write_requested(data->target_cfg);
			data->first_write = false;
		}

		transfer->rxData = &rxVal;
		transfer->rxSize = 1;
		data->is_write = true;
		break;

	case kI2C_SlaveCompletionEvent:
		/* called after every transferred byte */
		if (data->is_write && target_cb->write_received) {
			target_cb->write_received(data->target_cfg, rxVal);
			data->is_write = false;
		}
		break;

	case kI2C_SlaveDeselectedEvent:
		if (target_cb->stop) {
			target_cb->stop(data->target_cfg);
		}

		data->first_read = true;
		data->first_write = true;
		break;

	default:
		LOG_INF("Unhandled event: %d", transfer->event);
		break;
	}
}

int mcux_flexcomm_target_register(const struct device *dev,
			     struct i2c_target_config *target_config)
{
	const struct mcux_flexcomm_config *config = dev->config;
	struct mcux_flexcomm_data *data = dev->data;
	I2C_Type *base = config->base;
	uint32_t clock_freq;
	i2c_slave_config_t i2c_cfg;

	I2C_MasterDeinit(base);

	/* Get the clock frequency */
	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	if (!target_config) {
		return -EINVAL;
	}

	if (data->target_attached) {
		return -EBUSY;
	}

	data->target_cfg = target_config;
	data->target_attached = true;
	data->first_read = true;
	data->first_write = true;

	I2C_SlaveGetDefaultConfig(&i2c_cfg);
	i2c_cfg.address0.address = target_config->address;

	I2C_SlaveInit(base, &i2c_cfg, clock_freq);
	I2C_SlaveTransferCreateHandle(base, &data->target_handle,
			i2c_target_transfer_callback, data);
	I2C_SlaveTransferNonBlocking(base, &data->target_handle,
			kI2C_SlaveCompletionEvent | kI2C_SlaveTransmitEvent |
			kI2C_SlaveReceiveEvent | kI2C_SlaveDeselectedEvent);

	return 0;
}

int mcux_flexcomm_target_unregister(const struct device *dev,
			       struct i2c_target_config *target_config)
{
	const struct mcux_flexcomm_config *config = dev->config;
	struct mcux_flexcomm_data *data = dev->data;
	I2C_Type *base = config->base;

	if (!data->target_attached) {
		return -EINVAL;
	}

	data->target_cfg = NULL;
	data->target_attached = false;

	I2C_SlaveDeinit(base);

	return 0;
}
#endif

static void mcux_flexcomm_isr(const struct device *dev)
{
	const struct mcux_flexcomm_config *config = dev->config;
	struct mcux_flexcomm_data *data = dev->data;
	I2C_Type *base = config->base;

#if defined(CONFIG_I2C_TARGET)
	if (data->target_attached) {
		I2C_SlaveTransferHandleIRQ(base, &data->target_handle);
		return;
	}
#endif

	I2C_MasterTransferHandleIRQ(base, &data->handle);
}

static int mcux_flexcomm_init(const struct device *dev)
{
	const struct mcux_flexcomm_config *config = dev->config;
	struct mcux_flexcomm_data *data = dev->data;
	I2C_Type *base = config->base;
	uint32_t clock_freq, bitrate_cfg;
	i2c_master_config_t master_config;
	int error;

	error = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (error) {
		return error;
	}

	k_sem_init(&data->lock, 1, 1);
	k_sem_init(&data->device_sync_sem, 0, K_SEM_MAX_LIMIT);

	if (!device_is_ready(config->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Get the clock frequency */
	if (clock_control_get_rate(config->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	I2C_MasterGetDefaultConfig(&master_config);
	master_config.enableTimeout = true;
	master_config.timeout_Ms = 255;
	I2C_MasterInit(base, &master_config, clock_freq);
	I2C_MasterTransferCreateHandle(base, &data->handle,
				       mcux_flexcomm_master_transfer_callback,
				       data);

	// bitrate_cfg = i2c_map_dt_bitrate(config->bitrate);

	// error = mcux_flexcomm_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);
	// if (error) {
	// 	return error;
	// }

	if (config->bitrate <= I2C_BITRATE_STANDARD) {
		bitrate_cfg = I2C_SPEED_STANDARD << I2C_SPEED_SHIFT;
	} else if (config->bitrate <= I2C_BITRATE_FAST) {
		bitrate_cfg = I2C_SPEED_FAST << I2C_SPEED_SHIFT;
	} else if (config->bitrate <= I2C_BITRATE_FAST_PLUS) {
		bitrate_cfg = I2C_SPEED_FAST_PLUS << I2C_SPEED_SHIFT;
	} else if (config->bitrate <= I2C_BITRATE_HIGH) {
		bitrate_cfg = I2C_SPEED_HIGH << I2C_SPEED_SHIFT;
	} else {
		bitrate_cfg = I2C_SPEED_ULTRA << I2C_SPEED_SHIFT;
	}

	k_sem_take(&data->lock, K_FOREVER);
	I2C_MasterSetBaudRate(base, config->bitrate, clock_freq);
	k_sem_give(&data->lock);
	data->dev_config_raw = I2C_MODE_CONTROLLER | bitrate_cfg;

	config->irq_config_func(dev);

	return 0;
}

static const struct i2c_driver_api mcux_flexcomm_driver_api = {
	.configure = mcux_flexcomm_configure,
	.get_config = mcux_flexcomm_get_config,
	.transfer = mcux_flexcomm_transfer,
#if defined(CONFIG_I2C_TARGET)
	.target_register = mcux_flexcomm_target_register,
	.target_unregister = mcux_flexcomm_target_unregister,
#endif
	.recover_bus = mcux_flexcomm_recover_bus,
};

#define I2C_MCUX_FLEXCOMM_DEVICE(id)					\
	PINCTRL_DT_INST_DEFINE(id);					\
	static void mcux_flexcomm_config_func_##id(const struct device *dev); \
	static const struct mcux_flexcomm_config mcux_flexcomm_config_##id = {	\
		.base = (I2C_Type *) DT_INST_REG_ADDR(id),		\
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(id)),	\
		.clock_subsys =				\
		(clock_control_subsys_t)DT_INST_CLOCKS_CELL(id, name),\
		.irq_config_func = mcux_flexcomm_config_func_##id,	\
		.bitrate = DT_INST_PROP(id, clock_frequency),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(id),		\
	};								\
	static struct mcux_flexcomm_data mcux_flexcomm_data_##id;	\
	I2C_DEVICE_DT_INST_DEFINE(id,					\
			    mcux_flexcomm_init,				\
			    NULL,					\
			    &mcux_flexcomm_data_##id,			\
			    &mcux_flexcomm_config_##id,			\
			    POST_KERNEL,				\
			    CONFIG_I2C_INIT_PRIORITY,			\
			    &mcux_flexcomm_driver_api);			\
	static void mcux_flexcomm_config_func_##id(const struct device *dev) \
	{								\
		IRQ_CONNECT(DT_INST_IRQN(id),				\
			    DT_INST_IRQ(id, priority),			\
			    mcux_flexcomm_isr,				\
			    DEVICE_DT_INST_GET(id),			\
			    0);						\
		irq_enable(DT_INST_IRQN(id));				\
	}								\

DT_INST_FOREACH_STATUS_OKAY(I2C_MCUX_FLEXCOMM_DEVICE)
