// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

/*
 * AW86017 is a 10-bit DAC driver, capable of sinking up to 100mA.
 *
 * DW9817 is a bidirectional 10-bit driver, driving up to +/- 100mA.
 * Operationally it is identical to AW86017, except that the idle position is
 * the mid-point, not 0.
 */

#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>

#define AW86017_MAX_FOCUS_POS	1023 /* 10-bit DAC, max DAC value */
/*
 * This sets the minimum granularity for the focus positions.
 * A value of 1 gives maximum accuracy for a desired focus position.
 */
#define AW86017_FOCUS_STEPS	1
/*
 * This acts as the minimum granularity of lens movement.
 * Keep this value power of 2, so the control steps can be
 * uniformly adjusted for gradual lens movement, with desired
 * number of control steps.
 */
#define AW86017_CTRL_STEPS	16
#define AW86017_CTRL_DELAY_US	1000

#define AW86017_CTL_ADDR		0x02
/*
 * AW86017 separates two registers to control the VCM position.
 * One for MSB value, another is LSB value.
 */
#define AW86017_MSB_ADDR		0x03
#define AW86017_LSB_ADDR		0x04
#define AW86017_STATUS_ADDR	0x05
#define AW86017_MODE_ADDR	0x06
#define AW86017_RESONANCE_ADDR	0x07

#define MAX_RETRY		10

#define AW86017_PW_MIN_DELAY_US		100
#define AW86017_PW_DELAY_RANGE_US	10

struct aw86017_cfg {
	unsigned int idle_pos;
	unsigned int default_pos;
};

struct aw86017_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_subdev sd;
	u16 current_val;
	u16 idle_pos;
	struct regulator *vdd;
	struct notifier_block notifier;
	bool first;
};

static inline struct aw86017_device *sd_to_aw86017_vcm(
					struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct aw86017_device, sd);
}

static int aw86017_i2c_check(struct i2c_client *client)
{
	const char status_addr = AW86017_STATUS_ADDR;
	char status_result;
	int ret;

	ret = i2c_master_send(client, &status_addr, sizeof(status_addr));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write STATUS address fail ret = %d\n",
			ret);
		return ret;
	}

	ret = i2c_master_recv(client, &status_result, sizeof(status_result));
	if (ret < 0) {
		dev_err(&client->dev, "I2C read STATUS value fail ret = %d\n",
			ret);
		return ret;
	}

	return status_result;
}

/* Set 10-bit DAC Code */
static int aw86017_set_dac(struct i2c_client *client, u16 data)
{
	/*
	 * Output current = CODE[9:0] x (120mA / 1023) [mA]
	 */
	const char tx_data[3] = {
		AW86017_MSB_ADDR, ((data >> 8) & 0x03), (data & 0xff)
	};
	int val, ret;

	/*
	 * According to the datasheet, need to check the bus status before we
	 * write VCM position. This ensure that we really write the value
	 * into the register
	 */
	ret = readx_poll_timeout(aw86017_i2c_check, client, val, val <= 0,
			AW86017_CTRL_DELAY_US, MAX_RETRY * AW86017_CTRL_DELAY_US);

	if (ret || val < 0) {
		if (ret) {
			dev_warn(&client->dev,
				"Cannot do the write operation because VCM is busy\n");
		}

		return ret ? -EBUSY : val;
	}

	/* Write VCM position to registers */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev,
			"I2C write MSB fail ret=%d\n", ret);

		return ret;
	}

	return 0;
}

/*
 * The lens position is gradually moved in units of AW86017_CTRL_STEPS,
 * to make the movements smoothly. In all cases, even when "start" and
 * "end" are the same, the lens will be set to the "end" position.
 *
 * (We don't use hardware slew rate control, because it differs widely
 * between otherwise-compatible ICs, and may need lens-specific tuning.)
 */
static int aw86017_ramp(struct i2c_client *client, int start, int end)
{
	int step, val, ret;

	if (start < end)
		step = AW86017_CTRL_STEPS;
	else
		step = -AW86017_CTRL_STEPS;

	val = start;
	while (true) {
		val += step;
		if (step * (val - end) >= 0)
			val = end;
		ret = aw86017_set_dac(client, val);
		if (ret)
			dev_err_ratelimited(&client->dev, "%s I2C failure: %d",
					    __func__, ret);
		if (val == end)
			break;
		usleep_range(AW86017_CTRL_DELAY_US, AW86017_CTRL_DELAY_US + 10);
	}

	return ret;
}

/* 使能，进入 work 模式，马达从空闲位置移动到目标位置 */
static int aw86017_active(struct aw86017_device *aw86017_dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&aw86017_dev->sd);
	char tx_data[2] = { 0xed, 0xab }; // Enter Advanced Mode
	int ret;

	/* Enter Advanced Mode */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
		return ret;
	}

	/* Direct mode */
	tx_data[0] = AW86017_MODE_ADDR; // reg:0x06
	tx_data[1] = 0x00; /* bit[7]=0, Direct mode */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
		return ret;
	}

	/* Power on */
	tx_data[0] = AW86017_CTL_ADDR; // reg:0x02
	tx_data[1] = 0x00; /* bit[0]=0, work mode */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
		return ret;
	}

	aw86017_dev->first = true;

	return aw86017_ramp(client, aw86017_dev->idle_pos, aw86017_dev->current_val);
}

/* 进入 PD 模式，马达从目标位置移动到空闲位置*/
static int aw86017_standby(struct aw86017_device *aw86017_dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&aw86017_dev->sd);
	char tx_data[2] = { 0xed, 0xab }; // Enter Advanced Mode
	int ret;

	/* Enter Advanced Mode */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
		return ret;
	}

	if (abs(aw86017_dev->current_val - aw86017_dev->idle_pos) > AW86017_CTRL_STEPS)
		aw86017_ramp(client, aw86017_dev->current_val, aw86017_dev->idle_pos);

	/* Power down */
	tx_data[0] = AW86017_CTL_ADDR; // reg:0x02
	tx_data[1] = 0x01; /* bit[0]=1, Enter Power Down Mode */
	ret = i2c_master_send(client, tx_data, sizeof(tx_data));
	if (ret < 0) {
		dev_err(&client->dev, "I2C write CTL fail ret = %d\n", ret);
		return ret;
	}

	return 0;
}

static int aw86017_regulator_event(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct aw86017_device *aw86017_dev =
		container_of(nb, struct aw86017_device, notifier);

	if (action & REGULATOR_EVENT_ENABLE) {
		/*
		 * Initialisation delay between VDD low->high and the moment
		 * when the i2c command is available.
		 * From the datasheet, it should be 10ms + 2ms (max power
		 * up sequence duration)
		 */
		usleep_range(AW86017_PW_MIN_DELAY_US,
			     AW86017_PW_MIN_DELAY_US +
			     AW86017_PW_DELAY_RANGE_US);

		aw86017_active(aw86017_dev);
	} else if (action & REGULATOR_EVENT_PRE_DISABLE) {
		aw86017_standby(aw86017_dev);
	}

	return 0;
}

static int aw86017_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct aw86017_device *dev_vcm = container_of(ctrl->handler,
		struct aw86017_device, ctrls_vcm);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
		int start = (dev_vcm->first) ? dev_vcm->current_val : ctrl->val;

		dev_vcm->first = false;
		dev_vcm->current_val = ctrl->val;
		return aw86017_ramp(client, start, ctrl->val);
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops aw86017_vcm_ctrl_ops = {
	.s_ctrl = aw86017_set_ctrl,
};

static int aw86017_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return pm_runtime_resume_and_get(sd->dev);
}

static int aw86017_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops aw86017_int_ops = {
	.open = aw86017_open,
	.close = aw86017_close,
};

static const struct v4l2_subdev_ops aw86017_ops = { };

static void aw86017_subdev_cleanup(struct aw86017_device *aw86017_dev)
{
	v4l2_async_unregister_subdev(&aw86017_dev->sd);
	v4l2_ctrl_handler_free(&aw86017_dev->ctrls_vcm);
	media_entity_cleanup(&aw86017_dev->sd.entity);
}

static int aw86017_init_controls(struct aw86017_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &aw86017_vcm_ctrl_ops;
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);

	v4l2_ctrl_handler_init(hdl, 1);

	v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
			  0, AW86017_MAX_FOCUS_POS, AW86017_FOCUS_STEPS,
			  dev_vcm->current_val);

	dev_vcm->sd.ctrl_handler = hdl;
	if (hdl->error) {
		dev_err(&client->dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
		return hdl->error;
	}

	return 0;
}

/* Compatible devices; in fact there are many similar chips.
 * "data" holds the powered-off (zero current) lens position and a
 * default/initial control value (which need not be the same as the powered-off
 * value).
 */
static const struct aw86017_cfg aw86017_cfg = {
	.idle_pos = 0,
	.default_pos = 0
};

static const struct of_device_id aw86017_of_table[] = {
	{ .compatible = "awinic,aw86017", .data = &aw86017_cfg },
	{ /* sentinel */ }
};

static int aw86017_probe(struct i2c_client *client)
{
	struct aw86017_device *aw86017_dev;
	const struct of_device_id *match;
	const struct aw86017_cfg *cfg;
	int rval;

	aw86017_dev = devm_kzalloc(&client->dev, sizeof(*aw86017_dev),
				  GFP_KERNEL);
	if (aw86017_dev == NULL)
		return -ENOMEM;

	aw86017_dev->vdd = devm_regulator_get_optional(&client->dev, "VDD");
	if (IS_ERR(aw86017_dev->vdd)) {
		if (PTR_ERR(aw86017_dev->vdd) != -ENODEV)
			return PTR_ERR(aw86017_dev->vdd);

		aw86017_dev->vdd = NULL;
	} else {
		aw86017_dev->notifier.notifier_call = aw86017_regulator_event;

		rval = regulator_register_notifier(aw86017_dev->vdd,
						   &aw86017_dev->notifier);
		if (rval) {
			dev_err(&client->dev,
				"could not register regulator notifier\n");
			return rval;
		}
	}

	match = i2c_of_match_device(aw86017_of_table, client);
	if (match) {
		cfg = (const struct aw86017_cfg *)match->data;
		aw86017_dev->idle_pos = cfg->idle_pos;
		aw86017_dev->current_val = cfg->default_pos;
	}

	v4l2_i2c_subdev_init(&aw86017_dev->sd, client, &aw86017_ops);
	aw86017_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	aw86017_dev->sd.internal_ops = &aw86017_int_ops;

	rval = aw86017_init_controls(aw86017_dev);
	if (rval)
		goto err_cleanup;

	rval = media_entity_pads_init(&aw86017_dev->sd.entity, 0, NULL);
	if (rval < 0)
		goto err_cleanup;

	aw86017_dev->sd.entity.function = MEDIA_ENT_F_LENS;

	rval = v4l2_async_register_subdev(&aw86017_dev->sd);
	if (rval < 0)
		goto err_cleanup;

	if (!aw86017_dev->vdd)
		pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	dev_info(&client->dev, "vcm aw86017 probe\n");

	return 0;

err_cleanup:
	v4l2_ctrl_handler_free(&aw86017_dev->ctrls_vcm);
	media_entity_cleanup(&aw86017_dev->sd.entity);

	return rval;
}

static void aw86017_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct aw86017_device *aw86017_dev = sd_to_aw86017_vcm(sd);

	if (aw86017_dev->vdd)
		regulator_unregister_notifier(aw86017_dev->vdd,
					      &aw86017_dev->notifier);

	pm_runtime_disable(&client->dev);

	aw86017_subdev_cleanup(aw86017_dev);
}

/*
 * This function sets the vcm position, so it consumes least current
 * The lens position is gradually moved in units of AW86017_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int __maybe_unused aw86017_vcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct aw86017_device *aw86017_dev = sd_to_aw86017_vcm(sd);

	if (aw86017_dev->vdd)
		return regulator_disable(aw86017_dev->vdd);

	return aw86017_standby(aw86017_dev);
}

/*
 * This function sets the vcm position to the value set by the user
 * through v4l2_ctrl_ops s_ctrl handler
 * The lens position is gradually moved in units of AW86017_CTRL_STEPS,
 * to make the movements smoothly.
 */
static int  __maybe_unused aw86017_vcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct aw86017_device *aw86017_dev = sd_to_aw86017_vcm(sd);

	if (aw86017_dev->vdd)
		return regulator_enable(aw86017_dev->vdd);

	return aw86017_active(aw86017_dev);
}

MODULE_DEVICE_TABLE(of, aw86017_of_table);

static const struct dev_pm_ops aw86017_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(aw86017_vcm_suspend, aw86017_vcm_resume)
	SET_RUNTIME_PM_OPS(aw86017_vcm_suspend, aw86017_vcm_resume, NULL)
};

static struct i2c_driver aw86017_i2c_driver = {
	.driver = {
		.name = "aw86017",
		.pm = &aw86017_pm_ops,
		.of_match_table = aw86017_of_table,
	},
	.probe = aw86017_probe,
	.remove = aw86017_remove,
};

module_i2c_driver(aw86017_i2c_driver);

MODULE_AUTHOR("waveshare team");
MODULE_DESCRIPTION("AW86017 VCM driver");
MODULE_LICENSE("GPL");
