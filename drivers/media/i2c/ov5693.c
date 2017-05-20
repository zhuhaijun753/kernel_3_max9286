
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "ov5693_mode_tbls.h"

#define MAX9286_DEFAULT_MODE		MAX9286_MODE_5120X720

#define MAX9286_DEFAULT_WIDTH		1280
#define MAX9286_DEFAULT_HEIGHT		720
#define MAX9286_DEFAULT_DATAFMT		V4L2_MBUS_FMT_YUYV8_2X8
#define MAX9286_DEFAULT_CLK_FREQ	24000000

/*****************************************************************/
/*************************max9286*********************************/
#define ADDR_MAX9286			0x48
#define ADDR_MAX9271			0x40
#define MAX_SENSOR_NUM			4
#define ADDR_MAX9271_ALL		(ADDR_MAX9271 + MAX_SENSOR_NUM + 1)


#define SENSOR_OV10635
#define ADDR_OV_SENSOR     		0x30


static inline int ov10635_read_reg(struct i2c_client *client, int index, unsigned short reg, unsigned char * val)
{
	unsigned char u8_buf[2] = {0};
	unsigned int buf_len = 2;
	int retry, timeout = 10;
	unsigned char u8_val = 0;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;

	client->addr = ADDR_OV_SENSOR + index;
	for (retry = 0; retry < timeout; retry ++) {
		if (i2c_master_send(client, u8_buf, buf_len) < 0) {		
			msleep(5);
			continue;
		}

		if (i2c_master_recv(client, &u8_val, 1) != 1) {	
			msleep(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {
		return -1;
	}

	*val = u8_val;

	return u8_val;
}

static inline int ov10635_write_reg(struct i2c_client *client,int index, unsigned short reg, unsigned char val)
{
	unsigned char u8_buf[3] = {0};
	unsigned int buf_len = 3;
	int retry, timeout = 10;

	u8_buf[0] = (reg >> 8) & 0xFF;
	u8_buf[1] = reg & 0xFF;
	u8_buf[2] = val;

	client->addr = ADDR_OV_SENSOR + index;
	for (retry = 0; retry < timeout; retry ++) {
		if (i2c_master_send(client, u8_buf, buf_len) < 0) {			
			msleep(5);
			continue;
		}
		break;
	}

	if (retry >= timeout) {		
		return -1;
	}

	return 0;
}

static int ov10635_check_device(struct i2c_client *client, int index)
{
	return 0;
}

// OV10635_YUV8 setting
static int ov10635_initialize(struct i2c_client *client, int index)
{
	return 0;
}


static int max9286_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	s32 ret;

	client->addr = ADDR_MAX9286;
	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {		
		return -1;
	}
	return 0;
}

static inline int max9286_read_reg(struct i2c_client *client, u8 reg)
{
	int val;

	client->addr = ADDR_MAX9286;
	val = i2c_smbus_read_byte_data(client, reg);
	if (val < 0) {		
		return -1;
	}
	return val;
}

static int max9271_write_reg(struct i2c_client *client, int index, u8 reg, u8 val)
{
	s32 ret;
	int retry, timeout = 10;

	client->addr = ADDR_MAX9271 + index;
	for (retry = 0; retry < timeout; retry ++) {
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (val < 0)
			msleep(5);
		else
			break;
	}

	if (retry >= timeout) {
		return -1;
	}

	return 0;
}

static int max9286_dump(struct i2c_client *client)
{
	int ret, i;
	client->addr = ADDR_MAX9286;	

	printk(KERN_INFO "Printing MAX9286, i2caddr: 0x%x\n", client->addr);
	printk(KERN_INFO "   ");
	for(i = 0; i < 0x10; i++)
	{
		printk("%4x ",i);
	}
	
	for(i = 0; i <= 0x71; i++)
	{
		if((i%16) == 0)
		{
			printk("\n%x: ",i/16);
		}
		ret = i2c_smbus_read_byte_data(client, i);
		printk("0x%02x ", ret);
	}
	printk(KERN_INFO"\n");

	return 0;
}


static int max9286_hardware_init(struct i2c_client *client)
{
	int i;
	u8 reg, sensor_addr = 0;
	unsigned int g_sensor_num = 0;
	unsigned char g_sensor_is_there = 0;
	//int ret = 0;

	//Disable CSI Output,CSI-2 outputs on virtual channel 0,Set virtual channel according to the link number,disable CSI-2 output
	max9286_write_reg(client, 0x15, 0x13);	

	//Enable PRBS test
	max9286_write_reg(client, 0x0E, 0x5F);
	msleep(10);

	//Enable Custom Reverse Channel & First Pulse Length
	max9286_write_reg(client, 0x3F, 0x4F);
	msleep(10);

	//Reverse Channel Amplitude to mid level and transition time
	max9286_write_reg(client, 0x3B, 0x1E);
	msleep(10);
	
	//Enable MAX9271 Configuration Link
	max9271_write_reg(client, 0, 0x04, 0x43);
	msleep(10);

	//Increase serializer reverse channel input thresholds


#ifdef SENSOR_OV10635	
//	max9271_write_reg(client, 0, 0x08, 0x09);
	max9271_write_reg(client, 0, 0x08, 0x01);
#endif

#ifdef SENSOR_AP0101
	//Invert VSYNC
	max9271_write_reg(client, 0, 0x08, 0x81);
#endif

	msleep(10);

	//Reverse Channel Amplitude level
	max9286_write_reg(client, 0x3B, 0x19);
	msleep(5);

	max9286_write_reg(client, 0x12, 0xF3);	//YUV422,8bit,Double Data Rate, 4 data lane
	msleep(5);


	//Frame Sync
#ifdef SENSOR_OV10635
	//Automatic Mode
//	max9286_write_reg(client, 0x01, 0xE2);	//单路模式，使用外同步
	max9286_write_reg(client, 0x01, 0x02);	//多路模式，内同步，0x01[7:6]=0b00,
	msleep(2);
	max9286_write_reg(client, 0x02, 0x00);	//
	msleep(2);
	max9286_write_reg(client, 0x63, 0x00);
	msleep(2);
	max9286_write_reg(client, 0x64, 0x00);
	msleep(2);
	max9286_write_reg(client, 0x05, 0x19);	//

//	max9286_write_reg(client, 0x06, 0x00);
//	msleep(2);
//	max9286_write_reg(client, 0x07, 0x00);
//	msleep(2);
//	max9286_write_reg(client, 0x08, 0x26);
//	msleep(2);
//	max9286_write_reg(client, 0x69, 0x30);
#endif


	msleep(100);

	// Detect link
	g_sensor_num = 0;
	reg = max9286_read_reg(client, 0x49);	//检测link情况,应该检测到link2,
	g_sensor_is_there = ((reg >> 4) & 0xF) | (reg & 0xF);
	if (g_sensor_is_there & (0x1 << 0))
		g_sensor_num += 1;
	if (g_sensor_is_there & (0x1 << 1))
		g_sensor_num += 1;
	if (g_sensor_is_there & (0x1 << 2))
		g_sensor_num += 1;
	if (g_sensor_is_there & (0x1 << 3))
		g_sensor_num += 1;
	pr_info("max9286_mipi: sensor number = %d.\n", g_sensor_num);	//得到有效链路（接上去的摄像头）个数

	if (g_sensor_num == 0) {
		pr_err("%s: no camera connected.\n", __func__);
	//	return -1;
	}
	
	max9286_write_reg(client, 0x0E, 0x50);	//Disable PRBS test
	msleep(2);
	max9286_write_reg(client, 0x0D, 0x03);	//

	// Set link order in MIPI CSI-2 output
	reg = 0xE4;  //Default setting
	if (g_sensor_num == 1) {			//只连接1个摄像头
		switch (g_sensor_is_there) {
			case 0x8:		//link3
				reg = 0x27;
				break;
			case 0x4:		//link2
				reg = 0xC6;
				break;
			case 0x2:		//link1
				reg = 0xE1;
				break;
			case 0x1:		//link0
			default:
				reg = 0xE4;
				break;
		}
	} else if (g_sensor_num == 2) {		//连接了2个摄像头
		switch (g_sensor_is_there) {
			case 0xC:
				reg = 0x4E;
				break;
			case 0xA:
				reg = 0x72;
				break;
			case 0x9:
				reg = 0x78;
				break;
			case 0x6:
				reg = 0xD2;
				break;
			case 0x5:
				reg = 0xD8;
				break;
			case 0x3:
			default:
				reg = 0xE4;
				break;
		}
	} else if (g_sensor_num == 3) {		//连接了3个摄像头
		switch (g_sensor_is_there) {
			case 0xE:
				reg = 0x93;
				break;
			case 0xD:
				reg = 0x9C;
				break;
			case 0xB:
				reg = 0xB4;
				break;
			case 0x7:
			default:
				reg = 0xE4;
				break;
		}
	}

	max9286_write_reg(client, 0x0B, reg);	//0x0B寄存器为link out order
	
	
	reg = 0xE0 | g_sensor_is_there;
	max9286_write_reg(client, 0x00, reg);	//enable all links,auto select

	//Set up links

	sensor_addr = ADDR_OV_SENSOR;	//ADDR_OV_SENSOR = 0x30

	printk("sensor_addr = ADDR_OV_SENSOR;\n");

	reg = 0;
	
	for (i=1; i<=MAX_SENSOR_NUM; i++) {		//MAX_SENSOR_NUM=4
		if (((0x1 << (i-1)) & g_sensor_is_there) == 0) {
			continue;
		}
		
		//Enable Link control channel
		reg |= (0x11 << (i-1));
		max9286_write_reg(client, 0x0A, reg);

		//Set MAX9271 new address for link 0
		max9271_write_reg(client, 0, 0x00, (ADDR_MAX9271+ i) << 1);
		msleep(10);			

		//Set MAX9271: Double Mode, PCLK latched on Rising Edge, HS/VS encoding
		//max9271_write_reg(client, i, 0x07, 0x94);
		max9271_write_reg(client, i, 0x07, 0x84);
		msleep(10);	
	//	max9271_write_reg(client, i, 0x02, 0x1F);
	//	msleep(5);		
	//	max9271_write_reg(client, i, 0x03, 0x00);
	//	msleep(5);		
	//	max9271_write_reg(client, i, 0x05, 0x01);
	//	msleep(5);		
	//	max9271_write_reg(client, i, 0x06, 0xA0);
	//	msleep(5);			
	

		max9271_write_reg(client, i, 0x01, ADDR_MAX9286 << 1);
		msleep(5);	
		max9271_write_reg(client, i, 0x09, (sensor_addr + i) << 1);
		msleep(5);
		max9271_write_reg(client, i, 0x0A, sensor_addr << 1);
		msleep(5);
		max9271_write_reg(client, i, 0x0B, ADDR_MAX9271_ALL << 1);
		msleep(5);
		max9271_write_reg(client, i, 0x0C, (ADDR_MAX9271 + i) << 1);
		msleep(5);
	}
	max9286_write_reg(client, 0x0A, reg);
	msleep(5);
	max9286_write_reg(client, 0x0A, reg);
	msleep(5);
	//Disable Local Auto I2C ACK
	max9286_write_reg(client, 0x34, 0x36);
	msleep(5);
	max9286_write_reg(client, 0x1A, 0x60);	//此处采用默认值，用张建军的0x74不能出图像

	//Initialize Camera Sensor
#ifdef SENSOR_OV10635
	printk("g_sensor_is_there = %x\n", g_sensor_is_there);

	if (g_sensor_is_there & (0x1 << 0)) {
		printk("the 0 camera\n");
		if (ov10635_check_device(client, 1) == 0)
			ov10635_initialize(client, 0);
	} else if (g_sensor_is_there & (0x1 << 1)) {
		printk("the 1 camera\n");
		if (ov10635_check_device(client, 2) == 0)
			ov10635_initialize(client, 0);
	} else if (g_sensor_is_there & (0x1 << 2)) {
		printk("the 2 camera\n");
		if (ov10635_check_device(client, 3) == 0)
			ov10635_initialize(client, 0);
	} else if (g_sensor_is_there & (0x1 << 3)) {
		printk("the 3 camera\n");
		if (ov10635_check_device(client, 4) == 0)
			ov10635_initialize(client, 0);
	}
#endif

	//Enable Local Auto I2C ACK
	max9286_write_reg(client, 0x34, 0xB6);
	
	//MAX9271: Enable Serial Links and Disable Configuration Link
	max9271_write_reg(client, ADDR_MAX9271_ALL - ADDR_MAX9271, 0x04, 0x83);
	msleep(100);  //Wait for more than 2 frame time

	//Enable CSI output, set virtual channel according to the link number
	max9286_write_reg(client, 0x15, 0x0B);
	msleep(10);
	
	max9286_dump(client);
	
	return 0;
}


/***************************************************************************************/
/***************************************************************************************/
/***************************************************************************************/
/***************************************************************************************/

struct max9286_priv {
	struct camera_common_power_rail		power;
	struct v4l2_ctrl_handler		ctrl_handler;
	struct i2c_client			*i2c_client;
	struct v4l2_subdev			*subdev;
	struct media_pad			pad;
	struct camera_common_data		*s_data;
	struct camera_common_pdata		*pdata;
//	struct v4l2_mbus_framefmt		mf;
	u32 					mbus_fmt_code;
	int					numctrls;
	struct v4l2_ctrl			*ctrls[];
};

static int test_mode;
module_param(test_mode, int, 0644);

static int max9286_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max9286_priv *priv = container_of(ctrl->handler, struct max9286_priv, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EEPROM_DATA:
		
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int max9286_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct max9286_priv *priv = container_of(ctrl->handler, struct max9286_priv, ctrl_handler);
	int err = 0;

	if (priv->power.state == SWITCH_OFF)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:		
		break;
	case V4L2_CID_FRAME_LENGTH:		
		break;
	case V4L2_CID_COARSE_TIME:		
		break;
	case V4L2_CID_COARSE_TIME_SHORT:		
		break;
	case V4L2_CID_GROUP_HOLD:		
		break;
	case V4L2_CID_EEPROM_DATA:		
		break;
	case V4L2_CID_HDR_EN:
		break;
	default:		
		return -EINVAL;
	}

	return err;
}




#define MAX9286_MIN_GAIN 			0
#define MAX9286_MAX_GAIN 			0
#define MAX9286_DEFAULT_GAIN 			0
#define MAX9286_MIN_FRAME_LENGTH 		0
#define MAX9286_MAX_FRAME_LENGTH 		0x7FFF
#define MAX9286_DEFAULT_FRAME_LENGTH		0xA00
#define MAX9286_MIN_EXPOSURE_COARSE		0
#define MAX9286_MAX_EXPOSURE_COARSE		0
#define MAX9286_DEFAULT_EXPOSURE_COARSE		0

#define MAX9286_EEPROM_STR_SIZE			0
#define MAX9286_FUSE_ID_STR_SIZE		0
#define MAX9286_OTP_STR_SIZE			0


static const struct v4l2_ctrl_ops max9286_ctrl_ops = {
	.g_volatile_ctrl 	= max9286_g_volatile_ctrl,
	.s_ctrl			= max9286_s_ctrl,
};


static struct v4l2_ctrl_config ctrl_config_list[] = {
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = MAX9286_MIN_GAIN,
		.max = MAX9286_MAX_GAIN,
		.def = MAX9286_DEFAULT_GAIN,
		.step = 1,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_FRAME_LENGTH,
		.name = "Frame Length",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = MAX9286_MIN_FRAME_LENGTH,
		.max = MAX9286_MAX_FRAME_LENGTH,
		.def = MAX9286_DEFAULT_FRAME_LENGTH,
		.step = 1,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME,
		.name = "Coarse Time",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = MAX9286_MIN_EXPOSURE_COARSE,
		.max = MAX9286_MAX_EXPOSURE_COARSE,
		.def = MAX9286_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_COARSE_TIME_SHORT,
		.name = "Coarse Time Short",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = MAX9286_MIN_EXPOSURE_COARSE,
		.max = MAX9286_MAX_EXPOSURE_COARSE,
		.def = MAX9286_DEFAULT_EXPOSURE_COARSE,
		.step = 1,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_EEPROM_DATA,
		.name = "EEPROM Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.min = 0,
		.max = MAX9286_EEPROM_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = MAX9286_OTP_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &max9286_ctrl_ops,
		.id = V4L2_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = MAX9286_FUSE_ID_STR_SIZE,
		.step = 2,
	},
};


static int max9286_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}


static int max9286_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format)
{
	int ret;
	printk("max9286_set_fmt\n");
	printk("max9286_set_fmt    format->format.code=%x\n",  format->format.code);
	

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		ret = camera_common_try_fmt(sd, &format->format);
	}
	else {
		ret = camera_common_s_fmt(sd, &format->format);
	}
	return ret;
}

static int max9286_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh, struct v4l2_subdev_format *format)
{
	return camera_common_g_fmt(sd, &format->format);
}

static int max9286_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct i2c_client *client 			= v4l2_get_subdevdata(sd);
	struct camera_common_data *s_data 		= to_camera_common_data(client);
	struct max9286_priv *priv 			= (struct max9286_priv *)s_data->priv;
	struct camera_common_power_rail *pw		= &priv->power;

	*status = pw->state == SWITCH_ON;
	return 0;
}


static struct v4l2_subdev_video_ops max9286_subdev_video_ops = {
	.s_stream		= max9286_s_stream,
	.s_mbus_fmt		= camera_common_s_fmt,
	.g_mbus_fmt		= camera_common_g_fmt,
	.try_mbus_fmt		= camera_common_try_fmt,
	.enum_mbus_fmt		= camera_common_enum_fmt,
	.g_mbus_config		= camera_common_g_mbus_config,
	.g_input_status 	= max9286_g_input_status,
	.enum_framesizes	= camera_common_enum_framesizes,
	.enum_frameintervals	= camera_common_enum_frameintervals,
};

static struct v4l2_subdev_core_ops max9286_subdev_core_ops = {
	.s_power		= camera_common_s_power,	
};


static struct v4l2_subdev_pad_ops max9286_subdev_pad_ops = {
	.enum_mbus_code 	= camera_common_enum_mbus_code,
	.set_fmt 		= max9286_set_fmt,
	.get_fmt 		= max9286_get_fmt,
};


static struct v4l2_subdev_ops max9286_subdev_ops = {
	.core	= &max9286_subdev_core_ops,
	.video	= &max9286_subdev_video_ops,
	.pad	= &max9286_subdev_pad_ops,
};

static struct of_device_id max9286_of_match[] = {
	{ .compatible = "nvidia,ov5693", },
	{ },
};


MODULE_DEVICE_TABLE(of, max9286_of_match);

static struct camera_common_pdata *max9286_parse_dt(struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	
	int err;

	if (!node)
		return NULL;

	match = of_match_device(max9286_of_match, &client->dev);
	if (!match) {
		dev_err(&client->dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(&client->dev, sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	err = camera_common_parse_clocks(client, board_priv_pdata);
	if (err) {
		dev_err(&client->dev, "Failed to find clocks\n");
		goto error;
	}


	return board_priv_pdata;

error:
	devm_kfree(&client->dev, board_priv_pdata);
	return NULL;
}


static int max9286_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	
	dev_dbg(&client->dev, "%s:\n", __func__);
	return 0;
}

static const struct v4l2_subdev_internal_ops max9286_subdev_internal_ops = {
	.open = max9286_open,
};

static const struct media_entity_operations max9286_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};



static int max9286_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct max9286_priv *priv = (struct max9286_priv *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power on\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_on(pw);
		if (err)
			pr_err("%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	pw->state = SWITCH_ON;
	return 0;
}


static int max9286_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct max9286_priv *priv = (struct max9286_priv *)s_data->priv;
	struct camera_common_power_rail *pw = &priv->power;

	dev_dbg(&priv->i2c_client->dev, "%s: power off\n", __func__);

	if (priv->pdata && priv->pdata->power_on) {
		err = priv->pdata->power_off(pw);
		if (!err)
			pw->state = SWITCH_OFF;
		else
			pr_err("%s failed.\n", __func__);
		return err;
	}	

	return 0;
}


static struct camera_common_sensor_ops max9286_common_ops = {
	.power_on = max9286_power_on,
	.power_off = max9286_power_off,
};


static int max9286_ctrls_init(struct max9286_priv *priv, bool eeprom_ctrl)
{
	struct i2c_client *client = priv->i2c_client;	
	struct v4l2_ctrl *ctrl;
	int numctrls;
	int err;
	int i;

	dev_dbg(&client->dev, "%s++\n", __func__);

	numctrls = ARRAY_SIZE(ctrl_config_list);
	v4l2_ctrl_handler_init(&priv->ctrl_handler, numctrls);

	for (i = 0; i < numctrls; i++) {	

		ctrl = v4l2_ctrl_new_custom(&priv->ctrl_handler,&ctrl_config_list[i], NULL);
		if (ctrl == NULL) {
			dev_err(&client->dev, "Failed to init %s ctrl\n",ctrl_config_list[i].name);
			continue;
		}

		if (ctrl_config_list[i].type == V4L2_CTRL_TYPE_STRING &&
			ctrl_config_list[i].flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->string = devm_kzalloc(&client->dev,ctrl_config_list[i].max + 1, GFP_KERNEL);
			if (!ctrl->string)
				return -ENOMEM;
		}
		priv->ctrls[i] = ctrl;
	}

	priv->numctrls = numctrls;
	priv->subdev->ctrl_handler = &priv->ctrl_handler;
	if (priv->ctrl_handler.error) {
		dev_err(&client->dev, "Error %d adding controls\n",	priv->ctrl_handler.error);
		err = priv->ctrl_handler.error;
		goto error;
	}

	err = v4l2_ctrl_handler_setup(&priv->ctrl_handler);
	if (err) {
		dev_err(&client->dev,"Error %d setting default controls\n", err);
		goto error;
	}


	return 0;

error:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	return err;
}


static int max9286_power_get(struct max9286_priv *priv)
{
	struct camera_common_power_rail *pw = &priv->power;
	const char *mclk_name;
	const char *parentclk_name;
	struct clk *parent;
	int err = 0;

	mclk_name = priv->pdata->mclk_name ? priv->pdata->mclk_name : "cam_mclk1";
	pw->mclk = devm_clk_get(&priv->i2c_client->dev, mclk_name);
	if (IS_ERR(pw->mclk)) {
		dev_err(&priv->i2c_client->dev,"unable to get clock %s\n", mclk_name);
		return PTR_ERR(pw->mclk);
	}

	parentclk_name = priv->pdata->parentclk_name;
	if (parentclk_name) {
		parent = devm_clk_get(&priv->i2c_client->dev, parentclk_name);
		if (IS_ERR(parent))
			dev_err(&priv->i2c_client->dev,"unable to get parent clcok %s",	parentclk_name);
		else
			clk_set_parent(pw->mclk, parent);
	}

	pw->state = SWITCH_OFF;
	return err;
}



static int max9286_power_put(struct max9286_priv *priv)
{
	struct camera_common_power_rail *pw = &priv->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->avdd))
		regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		regulator_put(pw->iovdd);

	pw->avdd = NULL;
	pw->iovdd = NULL;

	
	return 0;
}



static int max9286_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct camera_common_data *common_data;
	struct max9286_priv *priv;

	struct device_node *node = client->dev.of_node;	
	char debugfs_name[10];
	int err;
	int tmp_addr 	= 0x00;
	
	mdelay(5000);
	pr_info("tusimple [max9286]: probing v4l2 sensor.\n");

	tmp_addr = client->addr;
	printk("client->addr is 0x%x\n", client->addr);
	mdelay(5000);	
	
	if (!IS_ENABLED(CONFIG_OF) || !node)
		return -EINVAL;
	
	pr_info("tusimple_01\n");
	
	common_data = devm_kzalloc(&client->dev, sizeof(struct camera_common_data), GFP_KERNEL);
	pr_info("tusimple_02\n");
	if (!common_data)
		return -ENOMEM;

	priv = devm_kzalloc(&client->dev, sizeof(struct max9286_priv) + sizeof(struct v4l2_ctrl *) * ARRAY_SIZE(ctrl_config_list), GFP_KERNEL);
	pr_info("tusimple_03\n");
	if (!priv)
		return -ENOMEM;

	priv->pdata = max9286_parse_dt(client);

	{
		printk("priv->pdata->mclk_name is %s\n", priv->pdata->mclk_name);
		printk("priv->pdata->parentclk_name is %s\n", priv->pdata->parentclk_name);
		printk("priv->pdata->pwdn_gpio is %d\n", priv->pdata->pwdn_gpio);
		printk("priv->pdata->reset_gpio is %d\n", priv->pdata->reset_gpio);
		printk("priv->pdata->af_gpio is %d\n", priv->pdata->af_gpio);
		printk("priv->pdata->use_cam_gpio is %d\n", priv->pdata->use_cam_gpio);
		printk("priv->pdata->phas_eeprom is %d\n", priv->pdata->has_eeprom);		
	}

	pr_info("tusimple_04\n");
	if (!priv->pdata) {
		dev_err(&client->dev, "unable to get platform data\n");
		return -EFAULT;
	}
	pr_info("tusimple_05\n");
	
	common_data->ops		= &max9286_common_ops;
	common_data->ctrl_handler	= &priv->ctrl_handler;
	common_data->i2c_client		= client;
	common_data->frmfmt		= max9286_frmfmt;
	common_data->colorfmt		= camera_common_find_datafmt(MAX9286_DEFAULT_DATAFMT);
	printk("tusimple common_data->colorfmt->code=%x\n",common_data->colorfmt->code);
	common_data->power		= &priv->power;
	common_data->ctrls		= priv->ctrls;
	common_data->priv		= (void *)priv;
	common_data->numctrls		= ARRAY_SIZE(ctrl_config_list);
	common_data->numfmts		= ARRAY_SIZE(max9286_frmfmt);
	common_data->def_mode		= MAX9286_DEFAULT_MODE;
	common_data->def_width		= MAX9286_DEFAULT_WIDTH;
	common_data->def_height		= MAX9286_DEFAULT_HEIGHT;
	common_data->fmt_width		= common_data->def_width;
	common_data->fmt_height		= common_data->def_height;
	common_data->def_clk_freq	= MAX9286_DEFAULT_CLK_FREQ;

	priv->i2c_client 		= client;
	priv->s_data			= common_data;
	priv->subdev			= &common_data->subdev;
	priv->subdev->dev		= &client->dev;
	priv->s_data->dev		= &client->dev;


	err = max9286_power_get(priv);
	if (err)
		return err;


	err = camera_common_parse_ports(client, common_data);	//这里面进行mipi csi lane的解析
	if (err) {
		dev_err(&client->dev, "Failed to find port info\n");
		return err;
	}

	camera_common_create_debugfs(common_data, debugfs_name);		
	
	v4l2_i2c_subdev_init(priv->subdev, client, &max9286_subdev_ops);

	printk("v4l2_i2c_subdev_init()       \n");

	
printk("tusimple------ common_data->colorfmt->code=%x\n",common_data->colorfmt->code);

	err = max9286_ctrls_init(priv, !err);
	if (err)
		return err;

	priv->subdev->internal_ops = &max9286_subdev_internal_ops;

	priv->subdev->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags 	  = MEDIA_PAD_FL_SOURCE;
	priv->subdev->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	priv->subdev->entity.ops  = &max9286_media_ops;

	err = media_entity_init(&priv->subdev->entity, 1, &priv->pad, 0);
	if (err < 0) {
		dev_err(&client->dev, "unable to init media entity\n");
		return err;
	}
#endif

	err = v4l2_async_register_subdev(priv->subdev);
	printk("v4l2_async_register_subdev()       \n");
	if (err)
		return err;


	printk("start write *******************************************************************************************\n");

	//max9286_hardware_init(client);

	printk("end write *********************************************************************************************\n");


	mdelay(10000);

	printk("tusimple------ priv->s_data->colorfmt->code=%x\n",priv->s_data->colorfmt->code);
	dev_dbg(&client->dev, "Detected max9286 IC!\n");
	
	return 0;
}



static int max9286_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(client);
	struct max9286_priv *priv = (struct max9286_priv *)s_data->priv;

	v4l2_async_unregister_subdev(priv->subdev);
	
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev->entity);
#endif

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	max9286_power_put(priv);
	camera_common_remove_debugfs(s_data);

	return 0;
}

static const struct i2c_device_id max9286_id[] = {
	{ "ov5693", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, max9286_id);

static struct i2c_driver max9286_i2c_driver = {
	.driver = {
		.name = "ov5693",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(max9286_of_match),
	},
	.probe = max9286_probe,
	.remove = max9286_remove,
	.id_table = max9286_id,
};

module_i2c_driver(max9286_i2c_driver);

MODULE_DESCRIPTION("Camera driver for max9286+max9271+OV490+OV10640");
MODULE_AUTHOR("haijun.zhu@tusimple.com");
MODULE_LICENSE("GPL v2");

