// SPDX-License-Identifier: GPL-2.0
/*
 * Best-effort reconstruction of the built-in SC233HGS sensor driver from:
 *   - boot/Image
 *   - System.map-5.10.160-rockchip-rk3588
 *   - embedded strings / static tables
 *
 * This is not the original source. It is a readable, source-level recovery of
 * the driver shape and the high-confidence behavior present in the binary.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/rk-camera-module.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#define DRIVER_VERSION                 KERNEL_VERSION(0, 1, 7)
#define SC233HGS_NAME                 "sc233hgs"
#define SC233HGS_LINK_FREQ_500M       500000000ULL
#define SC233HGS_LANES                2
#define SC233HGS_BITS_PER_SAMPLE      10
#define SC233HGS_PIXEL_RATE           (SC233HGS_LINK_FREQ_500M * 2 * \
				       SC233HGS_LANES / SC233HGS_BITS_PER_SAMPLE)
#define SC233HGS_XVCLK_FREQ           24000000
#define SC233HGS_CHIP_ID              0xcb61
#define SC233HGS_VTS_MAX              0x7fff

#define REG_NULL                      0xffff
#define SC233HGS_REG_VALUE_08BIT      1
#define SC233HGS_REG_VALUE_16BIT      2
#define SC233HGS_REG_VALUE_24BIT      3

#define SC233HGS_REG_CHIP_ID          0x3107
#define SC233HGS_REG_CTRL_MODE        0x2100
#define SC233HGS_REG_STANDBY          0x3000
#define SC233HGS_REG_XTMSTA           0x3002
#define SC233HGS_MODE_SW_STANDBY      0x00
#define SC233HGS_MODE_STREAMING       0x01

#define SC233HGS_REG_EXPOSURE_H       0x3e00
#define SC233HGS_REG_EXPOSURE_M       0x3e01
#define SC233HGS_REG_EXPOSURE_L       0x3e02
#define SC233HGS_REG_DIG_GAIN         0x3e06
#define SC233HGS_REG_DIG_FINE_GAIN    0x3e07
#define SC233HGS_REG_ANA_GAIN         0x3e08
#define SC233HGS_REG_ANA_FINE_GAIN    0x3e09
#define SC233HGS_REG_VTS              0x320e
#define SC233HGS_REG_TEST_PATTERN     0x4501
#define SC233HGS_REG_FLIP_MIRROR      0x3221

static const char * const sc233hgs_supply_names[] = {
	"avdd",
	"dovdd",
	"dvdd",
};

#define SC233HGS_NUM_SUPPLIES ARRAY_SIZE(sc233hgs_supply_names)

struct sc233hgs_regval {
	u16 addr;
	u8 val;
};

struct sc233hgs_gain_step {
	u16 gain_min;
	u16 gain_max;
	u16 ana_gain_min;
	u16 ana_gain_max;
	u16 reserved;
	u16 dig_gain_min;
	u16 dig_gain_max;
	u16 gain_step;
	u16 gain_div;
};

struct sc233hgs_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	u32 bus_fmt;
	const struct sc233hgs_regval *reg_list;
	u32 hdr_mode;
	u32 vc[4];
	u64 link_freq_idx;
};

struct sc233hgs {
	struct i2c_client *client;
	struct clk *xvclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *pwdn_gpio;
	struct regulator_bulk_data supplies[SC233HGS_NUM_SUPPLIES];
	const char *module_name;
	const char *len_name;
	const char *facing;
	u32 module_index;

	struct v4l2_subdev subdev;
	struct media_pad pad;
	struct fwnode_handle *endpoint;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *anal_gain;
	struct v4l2_ctrl *digi_gain;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *link_freq;
	struct mutex mutex;

	bool streaming;
	bool power_on;
	bool is_thunderboot;

	u32 cur_vts;
	const struct sc233hgs_mode *cur_mode;
};

static const char * const sc233hgs_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4",
};

/*
 * Sophgo linear SC233HGS 1920x1080 60fps mode table, kept inside the
 * Rockchip-style V4L2 driver so we can test whether this sensor programming
 * matches the Seeker camera modules better than the recovered firmware table.
 */
static const struct sc233hgs_regval sc233hgs_sophgo_1920x1080_60_regs[] = {
	{0x2100, 0x00}, {0x36e9, 0x80}, {0x37f9, 0x80}, {0x300c, 0x24},
	{0x3018, 0x3a}, {0x3019, 0x0c}, {0x301f, 0x9d}, {0x3044, 0x10},
	{0x3062, 0x00}, {0x3202, 0x00}, {0x3203, 0x3c}, {0x3206, 0x04},
	{0x3207, 0x83}, {0x320a, 0x04}, {0x320b, 0x38}, {0x320c, 0x04},
	{0x320d, 0x65}, {0x320e, 0x05}, {0x320f, 0xc9}, {0x321f, 0x0b},
	{0x322f, 0x00}, {0x3231, 0x00}, {0x3250, 0x03}, {0x32e2, 0x00},
	{0x3301, 0x18}, {0x3304, 0x68}, {0x3306, 0x48}, {0x3309, 0x88},
	{0x330b, 0xf0}, {0x330d, 0x30}, {0x3314, 0xc0}, {0x3315, 0x30},
	{0x3317, 0x30}, {0x3318, 0x1f}, {0x331d, 0x32}, {0x331f, 0x02},
	{0x3320, 0xc1}, {0x333b, 0x30}, {0x3352, 0x1a}, {0x3356, 0x1c},
	{0x3363, 0x0f}, {0x3385, 0x59}, {0x3387, 0x79}, {0x33b0, 0x00},
	{0x33ef, 0x04}, {0x33f8, 0x02}, {0x33fa, 0x02}, {0x341c, 0x06},
	{0x341e, 0x24}, {0x341f, 0x27}, {0x34ad, 0x05}, {0x34af, 0x02},
	{0x34de, 0x02}, {0x34f2, 0x00}, {0x3619, 0x20}, {0x361a, 0x90},
	{0x3630, 0x80}, {0x3633, 0x44}, {0x3637, 0x3e}, {0x363a, 0x00},
	{0x363b, 0x17}, {0x363c, 0x0f}, {0x363d, 0x1f}, {0x363e, 0xa8},
	{0x363f, 0x86}, {0x3648, 0x4a}, {0x364a, 0x22}, {0x3660, 0x02},
	{0x3661, 0x05}, {0x3662, 0x0b}, {0x3663, 0x17}, {0x3664, 0x17},
	{0x3665, 0x08}, {0x3666, 0x18}, {0x3667, 0x38}, {0x3668, 0x78},
	{0x3670, 0xf0}, {0x3671, 0xc8}, {0x3672, 0xa8}, {0x367c, 0x43},
	{0x367d, 0x43}, {0x367e, 0x43}, {0x36c0, 0x08}, {0x36c1, 0x38},
	{0x36c6, 0x08}, {0x36c7, 0x38}, {0x36ea, 0x0a}, {0x36eb, 0x0c},
	{0x36ec, 0x43}, {0x36ed, 0x14}, {0x3718, 0x04}, {0x3719, 0x06},
	{0x3722, 0x08}, {0x3728, 0xa0}, {0x372a, 0x10}, {0x3778, 0x08},
	{0x3779, 0x18}, {0x377a, 0xd8}, {0x3794, 0x00}, {0x3795, 0x00},
	{0x3796, 0x00}, {0x3798, 0x10}, {0x3799, 0x10}, {0x379a, 0x10},
	{0x37c4, 0x08}, {0x37c5, 0x38}, {0x37d2, 0x08}, {0x37d3, 0x38},
	{0x37d4, 0x08}, {0x37d5, 0x38}, {0x37fa, 0x04}, {0x37fb, 0xe5},
	{0x37fc, 0x01}, {0x37fd, 0x34}, {0x3900, 0x1d}, {0x3901, 0x06},
	{0x3903, 0x40}, {0x3904, 0x0a}, {0x3905, 0x4d}, {0x391f, 0x44},
	{0x3933, 0x00}, {0x3934, 0x20}, {0x3935, 0x00}, {0x3936, 0x3c},
	{0x3937, 0x7e}, {0x3938, 0x64}, {0x3939, 0x1f}, {0x393a, 0xaa},
	{0x393b, 0x00}, {0x393c, 0x20}, {0x39c1, 0x6c}, {0x3c0a, 0x2a},
	{0x3e00, 0x00}, {0x3e01, 0x5c}, {0x3e02, 0x30}, {0x3e03, 0x0b},
	{0x3e08, 0x00}, {0x3e09, 0x40}, {0x3e15, 0x00}, {0x3e1c, 0x00},
	{0x3e8e, 0x00}, {0x4330, 0x50}, {0x4331, 0x20}, {0x4333, 0x02},
	{0x4360, 0x07}, {0x4362, 0xf0}, {0x4364, 0xf0}, {0x4365, 0x18},
	{0x4366, 0x38}, {0x4368, 0x58}, {0x436a, 0x78}, {0x436b, 0x18},
	{0x436c, 0x38}, {0x4370, 0x20}, {0x4371, 0x10}, {0x4372, 0x18},
	{0x4373, 0x38}, {0x450d, 0x10}, {0x4837, 0x10}, {0x4b0a, 0x92},
	{0x5000, 0x38}, {0x5002, 0x00}, {0x502e, 0x21}, {0x5034, 0x01},
	{0x5104, 0x14}, {0x5105, 0x10}, {0x5106, 0x04}, {0x5107, 0x59},
	{0x510d, 0x44}, {0x5302, 0x00}, {0x5787, 0x0a}, {0x5788, 0x0a},
	{0x5789, 0x08}, {0x578a, 0x0a}, {0x578b, 0x0a}, {0x578c, 0x08},
	{0x578d, 0x40}, {0x5790, 0x08}, {0x5791, 0x04}, {0x5792, 0x04},
	{0x5793, 0x08}, {0x5794, 0x04}, {0x5795, 0x04}, {0x57aa, 0x2a},
	{0x57ab, 0x7f}, {0x57ac, 0x00}, {0x57ad, 0x00}, {0x36e9, 0x00},
	{0x37f9, 0x04}, {0x4412, 0x01}, {0x4402, 0x02}, {0x4403, 0x0a},
	{0x4404, 0x1e}, {0x440c, 0x32}, {0x440d, 0x32}, {0x440e, 0x26},
	{0x440f, 0x3f}, {0x4405, 0x28}, {0x4424, 0x01}, {0x4407, 0xa0},
	{0x2100, 0x01},
	{REG_NULL, 0x00},
};

static const s64 sc233hgs_link_freq_menu_items[] = {
	SC233HGS_LINK_FREQ_500M,
};

/*
 * Recovered from 0xffffffc0092213f8. The binary scans these seven entries to
 * pick the piecewise gain range, then linearly interpolates analogue and
 * digital gain register values from the requested gain.
 */
static const struct sc233hgs_gain_step sc233hgs_gain_table[] = {
	{ 0x0040, 0x0080, 0x0040, 0x007f, 0x0040, 0x0080, 0x0080, 0x0000, 0x0040 },
	{ 0x0080, 0x0100, 0x0140, 0x017f, 0x0080, 0x0080, 0x0080, 0x0000, 0x0040 },
	{ 0x0100, 0x0200, 0x0340, 0x037f, 0x0100, 0x0080, 0x0080, 0x0000, 0x0040 },
	{ 0x0200, 0x0400, 0x0740, 0x077f, 0x0200, 0x0080, 0x0080, 0x0000, 0x0040 },
	{ 0x0400, 0x0800, 0x077f, 0x077f, 0x0000, 0x0080, 0x00f8, 0x0400, 0x0079 },
	{ 0x0800, 0x1000, 0x077f, 0x077f, 0x0000, 0x0180, 0x01f8, 0x0800, 0x0079 },
	{ 0x1000, 0x2000, 0x077f, 0x077f, 0x0000, 0x0380, 0x03f8, 0x1000, 0x0079 },
};

static const struct sc233hgs_mode sc233hgs_supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = { .numerator = 10000, .denominator = 600000 },
		.hts_def = 2250,
		.vts_def = 1481,
		.exp_def = 400,
		.bus_fmt = MEDIA_BUS_FMT_SGRBG10_1X10,
		.reg_list = sc233hgs_sophgo_1920x1080_60_regs,
		.hdr_mode = NO_HDR,
		.vc = { 0, 1, 2, 3 },
		.link_freq_idx = 0,
	},
};

static int sc233hgs_write_reg(struct i2c_client *client, u16 reg, u32 len, u32 val)
{
	u8 buf[6];

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	if (len == 1) {
		buf[2] = val & 0xff;
	} else if (len == 2) {
		buf[2] = (val >> 8) & 0xff;
		buf[3] = val & 0xff;
	} else {
		buf[2] = (val >> 16) & 0xff;
		buf[3] = (val >> 8) & 0xff;
		buf[4] = val & 0xff;
	}

	return i2c_master_send(client, buf, len + 2) == len + 2 ? 0 : -EIO;
}

static int sc233hgs_read_reg(struct i2c_client *client, u16 reg, u32 len, u32 *val)
{
	struct i2c_msg msgs[2];
	__be16 reg_addr_be = cpu_to_be16(reg);
	__be32 data_be = 0;
	u8 *data_be_p = (u8 *)&data_be;
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;

	*val = be32_to_cpu(data_be);
	return 0;
}

static int sc233hgs_write_array(struct i2c_client *client,
				const struct sc233hgs_regval *regs)
{
	int ret = 0;

	for (; regs->addr != REG_NULL; regs++) {
		ret = sc233hgs_write_reg(client, regs->addr,
					 SC233HGS_REG_VALUE_08BIT, regs->val);
		if (ret)
			return ret;
	}

	return 0;
}

static int sc233hgs_set_stream_reg(struct sc233hgs *sc, bool on)
{
	int ret;

	if (on)
		return sc233hgs_write_reg(sc->client, SC233HGS_REG_CTRL_MODE,
					  SC233HGS_REG_VALUE_08BIT,
					  SC233HGS_MODE_STREAMING);

	ret = sc233hgs_write_reg(sc->client, SC233HGS_REG_STANDBY,
				 SC233HGS_REG_VALUE_08BIT, 0x01);
	ret |= sc233hgs_write_reg(sc->client, SC233HGS_REG_XTMSTA,
				  SC233HGS_REG_VALUE_08BIT, 0x01);
	return ret;
}

static void sc233hgs_map_gain(u32 req_gain, u16 *ana_gain, u16 *dig_gain)
{
	const struct sc233hgs_gain_step *step = &sc233hgs_gain_table[0];
	unsigned int i;
	u32 ana;
	u32 dig;

	for (i = 0; i < ARRAY_SIZE(sc233hgs_gain_table); i++) {
		if (req_gain >= sc233hgs_gain_table[i].gain_min &&
		    req_gain < sc233hgs_gain_table[i].gain_max) {
			step = &sc233hgs_gain_table[i];
			break;
		}
	}

	if (i == ARRAY_SIZE(sc233hgs_gain_table))
		step = &sc233hgs_gain_table[ARRAY_SIZE(sc233hgs_gain_table) - 1];

	if (!step->gain_step) {
		ana = step->ana_gain_min +
		      (req_gain - step->gain_min) / step->gain_div;
	} else {
		ana = step->ana_gain_min +
		      (req_gain - step->gain_min) / (step->gain_step / step->gain_div);
	}

	if (ana > step->ana_gain_max)
		ana = step->ana_gain_max;

	if (!step->gain_step) {
		dig = step->dig_gain_min;
	} else {
		dig = step->dig_gain_min +
		      (req_gain - step->gain_min) / (step->gain_step / step->gain_div);
	}

	if (dig > step->dig_gain_max)
		dig = step->dig_gain_max;

	*ana_gain = ana;
	*dig_gain = dig;
}

static int sc233hgs_configure_regulators(struct sc233hgs *sc)
{
	unsigned int i;

	for (i = 0; i < SC233HGS_NUM_SUPPLIES; i++)
		sc->supplies[i].supply = sc233hgs_supply_names[i];

	return devm_regulator_bulk_get(&sc->client->dev,
				       SC233HGS_NUM_SUPPLIES,
				       sc->supplies);
}

static int __sc233hgs_power_on(struct sc233hgs *sc)
{
	int ret;

	if (!IS_ERR_OR_NULL(sc->reset_gpio))
		gpiod_set_value_cansleep(sc->reset_gpio, 1);

	ret = clk_set_rate(sc->xvclk, SC233HGS_XVCLK_FREQ);
	if (ret)
		dev_warn(&sc->client->dev, "xvclk set rate failed\n");

	ret = clk_prepare_enable(sc->xvclk);
	if (ret)
		return ret;

	ret = regulator_bulk_enable(SC233HGS_NUM_SUPPLIES, sc->supplies);
	if (ret) {
		clk_disable_unprepare(sc->xvclk);
		return ret;
	}

	if (!IS_ERR_OR_NULL(sc->pwdn_gpio))
		gpiod_set_value_cansleep(sc->pwdn_gpio, 1);
	usleep_range(1000, 2000);

	if (!IS_ERR_OR_NULL(sc->reset_gpio))
		gpiod_set_value_cansleep(sc->reset_gpio, 0);

	usleep_range(0x130, 0x260);
	return 0;
}

static void __sc233hgs_power_off(struct sc233hgs *sc)
{
	if (!IS_ERR_OR_NULL(sc->reset_gpio))
		gpiod_set_value_cansleep(sc->reset_gpio, 1);
	if (!IS_ERR_OR_NULL(sc->pwdn_gpio))
		gpiod_set_value_cansleep(sc->pwdn_gpio, 0);

	clk_disable_unprepare(sc->xvclk);
	regulator_bulk_disable(SC233HGS_NUM_SUPPLIES, sc->supplies);
}

static int sc233hgs_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sc233hgs *sc =
		container_of(ctrl->handler, struct sc233hgs, ctrl_handler);
	struct i2c_client *client = sc->client;
	int ret = 0;
	u32 val = ctrl->val;
	u16 ana_gain = 0;
	u16 dig_gain = 0;

	if (!pm_runtime_get_if_in_use(&client->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ret = sc233hgs_write_reg(client, SC233HGS_REG_VTS,
					 SC233HGS_REG_VALUE_16BIT,
					 val + sc->cur_mode->height);
		if (!ret) {
			sc->cur_vts = val + sc->cur_mode->height;
			__v4l2_ctrl_modify_range(sc->exposure, sc->exposure->minimum,
						 sc->cur_vts - 6,
						 sc->exposure->step,
						 sc->exposure->default_value);
		}
		break;
	case V4L2_CID_EXPOSURE:
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_EXPOSURE_H, 1,
					  (val >> 12) & 0x0f);
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_EXPOSURE_M, 1,
					  (val >> 4) & 0xff);
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_EXPOSURE_L, 1,
					  (val & 0x0f) << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		sc233hgs_map_gain(val, &ana_gain, &dig_gain);
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_ANA_GAIN, 1,
					  (ana_gain >> 8) & 0xff);
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_ANA_FINE_GAIN, 1,
					  ana_gain & 0xff);
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_DIG_GAIN, 1,
					  (dig_gain >> 8) & 0xff);
		ret |= sc233hgs_write_reg(client, SC233HGS_REG_DIG_FINE_GAIN, 1,
					  dig_gain & 0xff);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = sc233hgs_write_reg(client, SC233HGS_REG_TEST_PATTERN, 1,
					 val ? (0xcc | (val - 1)) : 0xc4);
		break;
	default:
		break;
	}

	pm_runtime_put(&client->dev);
	return ret;
}

static const struct v4l2_ctrl_ops sc233hgs_ctrl_ops = {
	.s_ctrl = sc233hgs_set_ctrl,
};

static int sc233hgs_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);
	struct v4l2_mbus_framefmt *try_fmt;

	mutex_lock(&sc->mutex);
	try_fmt = v4l2_subdev_get_try_format(sd, fh->state, 0);
	try_fmt->width = sc->cur_mode->width;
	try_fmt->height = sc->cur_mode->height;
	try_fmt->code = sc->cur_mode->bus_fmt;
	try_fmt->field = V4L2_FIELD_NONE;
	mutex_unlock(&sc->mutex);

	return 0;
}

static int sc233hgs_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *fmt)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);
	const struct sc233hgs_mode *mode = &sc233hgs_supported_modes[0];
	s64 hblank, vblank;

	mutex_lock(&sc->mutex);

	fmt->format.code = mode->bus_fmt;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, fmt->pad) = fmt->format;
	} else {
		sc->cur_mode = mode;
		sc->cur_vts = mode->vts_def;
		hblank = mode->hts_def - mode->width;
		vblank = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(sc->hblank, hblank, hblank, 1, hblank);
		__v4l2_ctrl_modify_range(sc->vblank, vblank,
					 SC233HGS_VTS_MAX - mode->height,
					 1, vblank);
	}

	mutex_unlock(&sc->mutex);

	return 0;
}

static int sc233hgs_get_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_format *fmt)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);
	const struct sc233hgs_mode *mode = sc->cur_mode;

	mutex_lock(&sc->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt->format = *v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = mode->bus_fmt;
		fmt->format.field = V4L2_FIELD_NONE;
		fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&sc->mutex);

	return 0;
}

static int sc233hgs_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);

	if (code->index)
		return -EINVAL;

	code->code = sc->cur_mode->bus_fmt;
	return 0;
}

static int sc233hgs_enum_frame_sizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(sc233hgs_supported_modes))
		return -EINVAL;
	if (fse->code != sc233hgs_supported_modes[fse->index].bus_fmt)
		return -EINVAL;

	fse->min_width = sc233hgs_supported_modes[fse->index].width;
	fse->max_width = sc233hgs_supported_modes[fse->index].width;
	fse->min_height = sc233hgs_supported_modes[fse->index].height;
	fse->max_height = sc233hgs_supported_modes[fse->index].height;

	return 0;
}

static int sc233hgs_enum_frame_interval(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *sd_state,
					struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(sc233hgs_supported_modes))
		return -EINVAL;

	fie->code = sc233hgs_supported_modes[fie->index].bus_fmt;
	fie->width = sc233hgs_supported_modes[fie->index].width;
	fie->height = sc233hgs_supported_modes[fie->index].height;
	fie->interval = sc233hgs_supported_modes[fie->index].max_fps;

	return 0;
}

static int sc233hgs_g_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);

	fi->interval = sc->cur_mode->max_fps;
	return 0;
}

static int sc233hgs_g_mbus_config(struct v4l2_subdev *sd,
				  unsigned int pad_id,
				  struct v4l2_mbus_config *config)
{
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->bus.mipi_csi2.num_data_lanes = SC233HGS_LANES;

	return 0;
}

static int sc233hgs_s_stream(struct v4l2_subdev *sd, int on)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);
	int ret = 0;

	mutex_lock(&sc->mutex);
	on = !!on;
	if (sc->streaming == on)
		goto out;

	if (on) {
		ret = pm_runtime_resume_and_get(&sc->client->dev);
		if (ret < 0)
			goto out;

		ret = sc233hgs_write_array(sc->client, sc->cur_mode->reg_list);
		if (!ret)
			ret = __v4l2_ctrl_handler_setup(&sc->ctrl_handler);
		if (!ret && sc->cur_mode->hdr_mode == NO_HDR)
			ret = sc233hgs_set_stream_reg(sc, true);
		if (ret) {
			pm_runtime_put(&sc->client->dev);
			goto out;
		}
	} else {
		ret = sc233hgs_set_stream_reg(sc, false);
		pm_runtime_put(&sc->client->dev);
	}

	sc->streaming = on;
out:
	mutex_unlock(&sc->mutex);
	return ret;
}

static int sc233hgs_s_power(struct v4l2_subdev *sd, int on)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);
	int ret = 0;

	mutex_lock(&sc->mutex);
	if (sc->power_on == !!on)
		goto out;

	if (on) {
		ret = pm_runtime_resume_and_get(&sc->client->dev);
		if (!ret)
			sc->power_on = true;
	} else {
		pm_runtime_put(&sc->client->dev);
		sc->power_on = false;
	}
out:
	mutex_unlock(&sc->mutex);
	return ret;
}

static int sc233hgs_get_module_inf(struct sc233hgs *sc,
				   struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strscpy(inf->base.sensor, SC233HGS_NAME, sizeof(inf->base.sensor));
	strscpy(inf->base.module, sc->module_name ?: "", sizeof(inf->base.module));
	strscpy(inf->base.lens, sc->len_name ?: "", sizeof(inf->base.lens));
	return 0;
}

static long sc233hgs_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);
	struct rkmodule_hdr_cfg *hdr;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		return sc233hgs_get_module_inf(sc, arg);
	case RKMODULE_GET_HDR_CFG:
		hdr = arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = sc->cur_mode->hdr_mode;
		return 0;
	case RKMODULE_SET_QUICK_STREAM:
		return sc233hgs_set_stream_reg(sc, *(u32 *)arg);
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static long sc233hgs_compat_ioctl32(struct v4l2_subdev *sd,
				    unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_hdr_cfg *hdr;
	u32 quick;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf)
			return -ENOMEM;
		ret = sc233hgs_ioctl(sd, cmd, inf);
		if (!ret && copy_to_user(up, inf, sizeof(*inf)))
			ret = -EFAULT;
		kfree(inf);
		return ret;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr)
			return -ENOMEM;
		ret = sc233hgs_ioctl(sd, cmd, hdr);
		if (!ret && copy_to_user(up, hdr, sizeof(*hdr)))
			ret = -EFAULT;
		kfree(hdr);
		return ret;
	case RKMODULE_SET_QUICK_STREAM:
		if (copy_from_user(&quick, up, sizeof(quick)))
			return -EFAULT;
		return sc233hgs_ioctl(sd, cmd, &quick);
	default:
		return -ENOIOCTLCMD;
	}
}
#endif

static const struct v4l2_subdev_core_ops sc233hgs_core_ops = {
	.s_power = sc233hgs_s_power,
	.ioctl = sc233hgs_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = sc233hgs_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops sc233hgs_video_ops = {
	.s_stream = sc233hgs_s_stream,
	.g_frame_interval = sc233hgs_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops sc233hgs_pad_ops = {
	.enum_mbus_code = sc233hgs_enum_mbus_code,
	.enum_frame_size = sc233hgs_enum_frame_sizes,
	.enum_frame_interval = sc233hgs_enum_frame_interval,
	.get_fmt = sc233hgs_get_fmt,
	.set_fmt = sc233hgs_set_fmt,
	.get_mbus_config = sc233hgs_g_mbus_config,
};

static const struct v4l2_subdev_ops sc233hgs_subdev_ops = {
	.core = &sc233hgs_core_ops,
	.video = &sc233hgs_video_ops,
	.pad = &sc233hgs_pad_ops,
};

static const struct media_entity_operations sc233hgs_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static const struct v4l2_subdev_internal_ops sc233hgs_internal_ops = {
	.open = sc233hgs_open,
};

static int sc233hgs_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);

	return __sc233hgs_power_on(sc);
}

static int sc233hgs_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);

	__sc233hgs_power_off(sc);
	return 0;
}

static int sc233hgs_initialize_controls(struct sc233hgs *sc)
{
	const struct sc233hgs_mode *mode = sc->cur_mode;
	struct v4l2_ctrl_handler *handler = &sc->ctrl_handler;
	s64 hblank = mode->hts_def - mode->width;
	s64 vblank = mode->vts_def - mode->height;
	int ret;

	ret = v4l2_ctrl_handler_init(handler, 8);
	if (ret)
		return ret;

	handler->lock = &sc->mutex;

	sc->link_freq = v4l2_ctrl_new_int_menu(handler, &sc233hgs_ctrl_ops,
					       V4L2_CID_LINK_FREQ,
					       ARRAY_SIZE(sc233hgs_link_freq_menu_items) - 1,
					       mode->link_freq_idx,
					       sc233hgs_link_freq_menu_items);
	if (sc->link_freq)
		sc->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sc->pixel_rate = v4l2_ctrl_new_std(handler, &sc233hgs_ctrl_ops,
					   V4L2_CID_PIXEL_RATE, 0,
					   SC233HGS_PIXEL_RATE, 1,
					   SC233HGS_PIXEL_RATE);
	sc->hblank = v4l2_ctrl_new_std(handler, &sc233hgs_ctrl_ops,
				       V4L2_CID_HBLANK, hblank, hblank, 1,
				       hblank);
	if (sc->hblank)
		sc->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	sc->vblank = v4l2_ctrl_new_std(handler, &sc233hgs_ctrl_ops,
				       V4L2_CID_VBLANK, vblank,
				       SC233HGS_VTS_MAX - mode->height, 1,
				       vblank);
	sc->exposure = v4l2_ctrl_new_std(handler, &sc233hgs_ctrl_ops,
					 V4L2_CID_EXPOSURE, 1,
					 mode->vts_def - 6, 1, mode->exp_def);
	sc->anal_gain = v4l2_ctrl_new_std(handler, &sc233hgs_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN, 0x40,
					  0x1fff, 1, 0x40);
	sc->test_pattern =
		v4l2_ctrl_new_std_menu_items(handler, &sc233hgs_ctrl_ops,
					     V4L2_CID_TEST_PATTERN,
					     ARRAY_SIZE(sc233hgs_test_pattern_menu) - 1,
					     0, 0, sc233hgs_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		v4l2_ctrl_handler_free(handler);
		return ret;
	}

	sc->subdev.ctrl_handler = handler;
	return 0;
}

static int sc233hgs_check_sensor_id(struct sc233hgs *sc)
{
	struct device *dev = &sc->client->dev;
	u32 id = 0;
	int ret;

	ret = sc233hgs_read_reg(sc->client, SC233HGS_REG_CHIP_ID,
				SC233HGS_REG_VALUE_16BIT, &id);
	if (ret) {
		dev_err(dev, "failed to read sensor id: %d\n", ret);
		return ret;
	}

	if (id != SC233HGS_CHIP_ID) {
		dev_err(dev, "unexpected sensor id 0x%04x\n", id);
		return -ENODEV;
	}

	dev_info(dev, "Detected SC233HGS CHIP ID = 0x%04x sensor\n", id);
	return 0;
}

static int sc233hgs_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct sc233hgs *sc;
	struct v4l2_subdev *sd;
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x\n",
		 DRIVER_VERSION >> 16,
		 (DRIVER_VERSION & 0xff00) >> 8,
		 DRIVER_VERSION & 0xff);

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->client = client;
	sc->cur_mode = &sc233hgs_supported_modes[0];
	sc->cur_vts = sc->cur_mode->vts_def;
	mutex_init(&sc->mutex);

	sc->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(sc->xvclk))
		return dev_err_probe(dev, PTR_ERR(sc->xvclk), "failed to get xvclk\n");

	sc->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sc->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(sc->reset_gpio), "failed to get reset-gpios\n");

	sc->pwdn_gpio = devm_gpiod_get_optional(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(sc->pwdn_gpio))
		return dev_err_probe(dev, PTR_ERR(sc->pwdn_gpio), "failed to get pwdn-gpios\n");

	ret = sc233hgs_configure_regulators(sc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = of_property_read_u32(dev->of_node, RKMODULE_CAMERA_MODULE_INDEX,
				   &sc->module_index);
	ret |= of_property_read_string(dev->of_node, RKMODULE_CAMERA_MODULE_NAME,
				       &sc->module_name);
	ret |= of_property_read_string(dev->of_node, RKMODULE_CAMERA_LENS_NAME,
				       &sc->len_name);
	ret |= of_property_read_string(dev->of_node, RKMODULE_CAMERA_MODULE_FACING,
				       &sc->facing);
	if (ret)
		return dev_err_probe(dev, ret, "could not get module information\n");

	sd = &sc->subdev;
	v4l2_i2c_subdev_init(sd, client, &sc233hgs_subdev_ops);
	sc->endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!sc->endpoint) {
		ret = -EINVAL;
		dev_err_probe(dev, ret, "missing endpoint node\n");
		goto err_destroy_mutex;
	}
	sd->fwnode = sc->endpoint;

	ret = sc233hgs_initialize_controls(sc);
	if (ret)
		goto err_put_endpoint;

	ret = __sc233hgs_power_on(sc);
	if (ret)
		goto err_free_handler;

	ret = sc233hgs_check_sensor_id(sc);
	if (ret)
		goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &sc233hgs_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

	sc->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sd->entity.ops = &sc233hgs_entity_ops;
	ret = media_entity_pads_init(&sd->entity, 1, &sc->pad);
	if (ret)
		goto err_power_off;

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 sc->module_index, sc->facing[0] == 'b' ? "b" : "f",
		 SC233HGS_NAME, dev_name(sd->dev));

	ret = v4l2_async_register_subdev_sensor(sd);
	if (ret)
		goto err_clean_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);
	return 0;

err_clean_entity:
	media_entity_cleanup(&sd->entity);
err_power_off:
	__sc233hgs_power_off(sc);
err_free_handler:
	v4l2_ctrl_handler_free(&sc->ctrl_handler);
err_put_endpoint:
	fwnode_handle_put(sc->endpoint);
err_destroy_mutex:
	mutex_destroy(&sc->mutex);
	return ret;
}

static void sc233hgs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sc233hgs *sc = container_of(sd, struct sc233hgs, subdev);

	v4l2_async_unregister_subdev(&sc->subdev);
	media_entity_cleanup(&sc->subdev.entity);
	v4l2_ctrl_handler_free(&sc->ctrl_handler);
	fwnode_handle_put(sc->endpoint);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__sc233hgs_power_off(sc);
	pm_runtime_set_suspended(&client->dev);
	mutex_destroy(&sc->mutex);
}

static const struct dev_pm_ops sc233hgs_pm_ops = {
	SET_RUNTIME_PM_OPS(sc233hgs_runtime_suspend,
			   sc233hgs_runtime_resume, NULL)
};

static const struct of_device_id sc233hgs_of_match[] = {
	{ .compatible = "sc,sc233hgs" },
	{},
};
MODULE_DEVICE_TABLE(of, sc233hgs_of_match);

static const struct i2c_device_id sc233hgs_match_id[] = {
	{ SC233HGS_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, sc233hgs_match_id);

static struct i2c_driver sc233hgs_i2c_driver = {
	.driver = {
		.name = SC233HGS_NAME,
		.pm = &sc233hgs_pm_ops,
		.of_match_table = sc233hgs_of_match,
	},
	.probe = sc233hgs_probe,
	.remove = sc233hgs_remove,
	.id_table = sc233hgs_match_id,
};

module_i2c_driver(sc233hgs_i2c_driver);

MODULE_DESCRIPTION("sc sc233hgs sensor driver (reconstructed)");
MODULE_LICENSE("GPL");
