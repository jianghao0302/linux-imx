#include <linux/acpi.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/types.h>

#define AP3216C_REGMAP_NAME	"ap3216c_regmap"
#define AP3216C_DRV_NAME	"ap3216c"

#define AP3216C_REG_ADDR    		0x1e	/** AP3216C 器件地址  */

/* AP3316C寄存器 */
#define AP3216C_REG_SYS_CONF		0x00	/** 配置寄存器       */
#define AP3216C_REG_INT_STATUS		0x01	/** 中断状态寄存器   */
#define AP3216C_REG_INT_CLEAR		0x02	/** 中断清除寄存器   */
#define AP3216C_REG_IR_DATA_LOW		0x0a	/** IR数据低字节     */
#define AP3216C_REG_IR_DATA_HIGH		0x0b	/** IR数据高字节     */
#define AP3216C_REG_ALS_DATA_LOW		0x0c	/** ALS数据低字节    */
#define AP3216C_REG_ALS_DATA_HIGH		0x0d	/** ALS数据高字节    */
#define AP3216C_REG_PS_DATA_LOW			0x0e	/** PS数据低字节     */
#define AP3216C_REG_PS_DATA_HIGH		0x0f	/** PS数据高字节     */

/** ALS Register Table */
#define AP3216C_REG_ALS_CONF				0x10	/** ALS配置寄存器 */
#define AP3216C_REG_ALS_CALIBRATION 		0x19	/** ALS window loss calibration */
#define AP3216C_REG_ALS_LOW_THRESHOLD_7_0	0x1a	/** ALS低阈值寄存器 */
#define AP3216C_REG_ALS_LOW_THRESHOLD_15_8	0x1b	/** ALS低阈值寄存器 */
#define AP3216C_REG_ALS_HIGH_THRESHOLD_7_0	0x1c	/** ALS高阈值寄存器 */
#define AP3216C_REG_ALS_HIGH_THRESHOLD_15_8	0x1d	/** ALS高阈值寄存器 */

/** PS Register Table */
#define AP3216C_REG_PS_CONF					0x20	/** PS配置寄存器 */
#define AP3216C_REG_PS_LED_DRIVER 			0x21	/** LED配置寄存器 */
#define AP3216C_REG_PS_INT_FORM				0x22	/** Interrupt algorithms style select of PS */
#define AP3216C_REG_PS_MEAN_TIME			0x23	/** PS average time selector */
#define AP3216C_REG_PS_LED_WAITING_TIME		0x24	/** Control PS LED waiting time */
#define AP3216C_REG_PS_CALIBRATION_L		0x28	/** Offset value to eliminate cross talk */
#define AP3216C_REG_PS_CALIBRATION_H		0x29	/** Offset value to eliminate cross talk */
#define AP3216C_REG_PS_LOW_THRESHOLD_2_0	0x2a	/** Lower byte of PS low threshold */
#define AP3216C_REG_PS_LOW_THRESHOLD_10_3	0x2b	/** Higher byte of PS low threshold */
#define AP3216C_REG_PS_HIGH_THRESHOLD_2_0	0x2c	/** Lower byte of PS high threshold */
#define AP3216C_REG_PS_HIGH_THRESHOLD_10_3	0x2d	/** Higher byte of PS high threshold */

struct ap3216c_data {
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct mutex lock;

	/* regmap fields */
	struct regmap *regmap;
	struct regmap_field *reg_int;
    struct regmap_field *reg_func;

	/* state */
	int ir_int;
	int als_int;
	int pxs_int;

	/* gain values */
    int ir_gain;
	int als_gain;
	int pxs_gain;

	/* integration time value in us */
	int als_adc_int_us;

	/* gesture buffer */
	u8 buffer[4]; /* 4 8-bit channels */
};

/* 默认寄存器值 */
static const struct reg_default ap3216c_reg_defaults[] = {
    { AP3216C_REG_SYS_CONF, 0x03 }, /* 默认开机配置 */
	{ AP3216C_REG_ALS_CONF, 0x00},
	{ AP3216C_REG_PS_CONF, 0x00},
	{ AP3216C_REG_PS_LED_DRIVER, 0x00},
};

/* 易变寄存器：实时数据和状态 */
static const struct regmap_range ap3216c_volatile_ranges[] = {
    regmap_reg_range(AP3216C_REG_INT_STATUS, AP3216C_REG_INT_CLEAR), /* Interrupt Status */
    regmap_reg_range(AP3216C_REG_IR_DATA_LOW, AP3216C_REG_IR_DATA_HIGH), /* IR 数据寄存器 */
    regmap_reg_range(AP3216C_REG_ALS_DATA_LOW, AP3216C_REG_ALS_DATA_HIGH),	/* ALS 数据寄存器 */
    regmap_reg_range(AP3216C_REG_PS_DATA_LOW, AP3216C_REG_PS_DATA_HIGH),	/* PS 数据寄存器 */
};

static const struct regmap_access_table ap3216c_volatile_table = {
    .yes_ranges   = ap3216c_volatile_ranges,
    .n_yes_ranges = ARRAY_SIZE(ap3216c_volatile_ranges),
};

/* precious 寄存器：AP3216C 没有 RAM 区域，可留空 */
static const struct regmap_range ap3216c_precious_ranges[] = { };

static const struct regmap_access_table ap3216c_precious_table = {
    .yes_ranges   = ap3216c_precious_ranges,
    .n_yes_ranges = ARRAY_SIZE(ap3216c_precious_ranges),
};

/* 可读寄存器范围 */
static const struct regmap_range ap3216c_readable_ranges[] = {
    regmap_reg_range(AP3216C_REG_SYS_CONF, AP3216C_REG_INT_CLEAR), /* System config + INT */
    regmap_reg_range(AP3216C_REG_IR_DATA_LOW, AP3216C_REG_IR_DATA_HIGH), /* IR 数据寄存器 */
    regmap_reg_range(AP3216C_REG_ALS_DATA_LOW, AP3216C_REG_ALS_DATA_HIGH), /* ALS 数据寄存器 */
    regmap_reg_range(AP3216C_REG_PS_DATA_LOW, AP3216C_REG_PS_DATA_HIGH), /* PS 数据寄存器 */
    regmap_reg_range(AP3216C_REG_ALS_CONF, AP3216C_REG_ALS_HIGH_THRESHOLD_15_8), /* ALS 配置和阈值 */
    regmap_reg_range(AP3216C_REG_PS_CONF, AP3216C_REG_PS_HIGH_THRESHOLD_10_3), /* PS 配置和阈值 */
};

static const struct regmap_access_table ap3216c_readable_table = {
    .yes_ranges   = ap3216c_readable_ranges,
    .n_yes_ranges = ARRAY_SIZE(ap3216c_readable_ranges),
};

/* 可写寄存器范围 */
static const struct regmap_range ap3216c_writeable_ranges[] = {
    regmap_reg_range(0x00, 0x02), /* System config + INT Clear */
    regmap_reg_range(0x10, 0x1D), /* ALS 配置和阈值 */
    regmap_reg_range(0x20, 0x2D), /* PS 配置和阈值 */
};

static const struct regmap_access_table ap3216c_writeable_table = {
    .yes_ranges   = ap3216c_writeable_ranges,
    .n_yes_ranges = ARRAY_SIZE(ap3216c_writeable_ranges),
};

/* regmap_config 示例 */
static const struct regmap_config ap3216c_regmap_cfg = {
	.name = AP3216C_REGMAP_NAME,
    .reg_bits        = 8,
    .val_bits        = 8,
    .max_register    = 0x2d,

    .cache_type      = REGCACHE_RBTREE,
	.max_register = AP3216C_REG_PS_HIGH_THRESHOLD_10_3,

    .reg_defaults    = ap3216c_reg_defaults,
    .num_reg_defaults= ARRAY_SIZE(ap3216c_reg_defaults),

    .volatile_table  = &ap3216c_volatile_table,
    .precious_table  = &ap3216c_precious_table,
    .rd_table  = &ap3216c_readable_table,
    .wr_table = &ap3216c_writeable_table,
};

static const struct reg_field ap3216c_reg_field_int_enable =
				REG_FIELD(AP3216C_REG_INT_STATUS, 0, 1);

static const struct reg_field ap3216c_reg_field_func_enable =
				REG_FIELD(AP3216C_REG_SYS_CONF, 0, 2);

/* 
 * AP3216C 的扫描元素，1 路 ALS(环境关)，1 路 PS(距离传感器)，1 路 IR
 */
enum ap3216c_scan_index {
	AP3216C_INDEX_ALS,
	AP3216C_INDEX_PS,
	AP3216C_INDEX_IR,
};

/** ap3216c通道，1路ALS(环境关)，1路PS(距离传感器)，1路IR */
static const struct iio_chan_spec ap3216c_channels[] = {
    /* ALS 通道 */
	{
		.type = IIO_INTENSITY,
		.modified = 1,
        .channel2 = IIO_MOD_LIGHT_BOTH,
        .address = AP3216C_REG_ALS_DATA_LOW,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
        .scan_index = AP3216C_INDEX_ALS,
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
    /** PS 通道 */
	{
		.type = IIO_PROXIMITY,
		.address = AP3216C_REG_PS_DATA_LOW,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = AP3216C_INDEX_PS,
		.scan_type = {
			.sign = 'u',
			.realbits = 10,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
	/* IR通道 */
	{
		.type = IIO_INTENSITY,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_IR,
		.address = AP3216C_REG_IR_DATA_LOW,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.scan_index = AP3216C_INDEX_IR,
		.scan_type = {
			.sign = 'u',
			.realbits = 10,
			.storagebits = 16,
			.endianness = IIO_LE,
		},
	},
};

/* 
 * ap3216c环境光传感器分辨率,扩大1000000倍,
 * 量程依次为0～20661，0～5162，0～1291，0～323。单位：lux
 */
static const int ap3216c_als_scale[] = {315000, 78800, 19700, 4900};

/*
 * 扫描掩码，两种情况，全启动 0X111，或者都不启动 0X0
 */
static const unsigned long ap3216c_scan_masks[] = {
	BIT(AP3216C_INDEX_ALS)
	| BIT(AP3216C_INDEX_PS)
	| BIT(AP3216C_INDEX_IR),
	0,
};

/*
 * @description	: 读取ap3216c指定寄存器值，读取一个寄存器
 * @param - dev:  ap3216c设备
 * @param - reg:  要读取的寄存器
 * @return 	  :   读取到的寄存器值
 */
static unsigned char ap3216c_read_reg(struct ap3216c_data *data, u8 reg)
{
	u8 ret;
	unsigned int buf;

	ret = regmap_read(data->regmap, reg, &buf);
	return (u8)buf;
}

/*
 * @description	: 向ap3216c指定寄存器写入指定的值，写一个寄存器
 * @param - dev:  ap3216c设备
 * @param - reg:  要写的寄存器
 * @param - data: 要写入的值
 * @return   :    无
 */
static void ap3216c_write_reg(struct ap3216c_data *data, u8 reg, u8 buf)
{
	regmap_write(data->regmap, reg, buf);
}


/*
  * @description  	: 读取AP3216C传感器数
  * @param - dev	: ap3216c设备 
  * @param - reg  	: 要读取的通道寄存器首地址。
  * @param - chann2 : 需要读取的通道，比如ALS，IR。
  * @param - val  	: 保存读取到的值。
  * @return			: 0，成功；其他值，错误
  */
static int ap3216c_read_alsir_data(struct ap3216c_data *data, 
									int reg,
									int chann2, 
									int *val)
{
	int ret = 0;
	unsigned char buf[2];

	switch (chann2) {
	case IIO_MOD_LIGHT_BOTH:	/* 读取ALS数据 */
		ret = regmap_bulk_read(data->regmap, reg, buf, 2);
		*val = ((int)buf[1] << 8) | buf[0];   
		break;
	case IIO_MOD_LIGHT_IR:		/* 读取IR数据 */
		ret = regmap_bulk_read(data->regmap, reg, buf, 2);
		*val = ((int)buf[1] << 2) | (buf[0] & 0X03); 
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret) {
		return -EINVAL;
	}
		
	return IIO_VAL_INT;
}

/*
  * @description     	: 读函数，当读取sysfs中的文件的时候最终此函数会执行，此函数
  * 					：里面会从传感器里面读取各种数据，然后上传给应用。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - val   		: 读取的值，如果是小数值的话，val是整数部分。
  * @param - val2   	: 读取的值，如果是小数值的话，val2是小数部分。
  * @return				: 0，成功；其他值，错误
  */
static int ap3216c_read_raw(struct iio_dev *indio_dev,
			   				struct iio_chan_spec const *chan,
			   				int *val, 
							int *val2, 
							long mask)
{
	int ret = 0;
	unsigned char buf[2];
	unsigned char regdata = 0;
	struct ap3216c_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:			/* 读取ICM20608加速度计、陀螺仪、温度传感器原始值 */
		mutex_lock(&data->lock);	/* 上锁 			*/
		switch (chan->type) {
		case IIO_INTENSITY:
			ret = ap3216c_read_alsir_data(data, chan->address, chan->channel2, val); /* 读取ALS */
			break;				/* 值为val */
		case IIO_PROXIMITY:
			ret = regmap_bulk_read(data->regmap, chan->address, buf, 2);
			*val = ((int)(buf[1] & 0X3F) << 4) | (buf[0] & 0X0F);  
			ret = IIO_VAL_INT; 	/* 值为val */
			break;
		default:
			ret = -EINVAL;
			break;
		}
		mutex_unlock(&data->lock);	/* 释放锁 			*/
		return ret;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_INTENSITY:			/* ALS量程 */
			mutex_lock(&data->lock);
			regdata = (ap3216c_read_reg(data, AP3216C_REG_ALS_CONF) & 0X30) >> 4;
			*val  = 0;
			*val2 = ap3216c_als_scale[regdata];
			mutex_unlock(&data->lock);
			return IIO_VAL_INT_PLUS_MICRO;	/* 值为val+val2/1000000 */
		default:
			return -EINVAL;
		}
		return ret;
		
	default:
		return -EINVAL;
	}
	return ret;
}

/*
  * @description  	: 设置AP3216C的ALS量程(分辨率)
  * @param - dev	: ap3216c设备
  * @param - val   	: 量程(分辨率值)。
  * @param - chann2 : 需要设置的通道。
  * @return			: 0，成功；其他值，错误
  */
static int ap3216c_write_als_scale(struct ap3216c_data *data, int chann2, int val)
{
	int ret = 0, i;	
	u8 d;

	switch (chann2) {
	case IIO_MOD_LIGHT_BOTH:	/* 设置ALS分辨率 */
		for (i = 0; i < ARRAY_SIZE(ap3216c_als_scale); ++i) {
			if (ap3216c_als_scale[i] == val) {
				d = (i << 4);
				ret = regmap_write(data->regmap, AP3216C_REG_ALS_CONF, d);
			}
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}
		
	return ret;
}

/**  @description     	: 写函数，当向sysfs中的文件写数据的时候最终此函数会执行，一般在此函数
  * 					：里面设置传感器，比如量程等。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - val   		: 应用程序写入的值，如果是小数值的话，val是整数部分。
  * @param - val2   	: 应用程序写入的值，如果是小数值的话，val2是小数部分。
  * @return				: 0，成功；其他值，错误
  */
static int ap3216c_write_raw(struct iio_dev *indio_dev, 
							struct iio_chan_spec const *chan, 
							int val, 
							int val2, 
							long mask)
{
	int ret = 0;
	struct ap3216c_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:	/* 设置ALS量程 */
		switch (chan->type) {
		case IIO_INTENSITY:		/* 设置ALS量程 */
			mutex_lock(&data->lock);
			ret = ap3216c_write_als_scale(data, chan->channel2, val2);
			mutex_unlock(&data->lock);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*
  * @description     	: 用户空间写数据格式，比如我们在用户空间操作sysfs来设置传感器的分辨率，
  * 					：如果分辨率带小数，那么这个小数传递到内核空间应该扩大多少倍，此函数就是
  *						: 用来设置这个的。
  * @param - indio_dev	: iio_dev
  * @param - chan   	: 通道
  * @param - mask   	: 掩码
  * @return				: 0，成功；其他值，错误
  */
static int ap3216c_write_raw_get_fmt(struct iio_dev *indio_dev, 
									struct iio_chan_spec const *chan, 
									long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_INTENSITY:		/* 用户空间写的陀螺仪分辨率数据要乘以1000000 */
			return IIO_VAL_INT_PLUS_MICRO;
		default:				
			return IIO_VAL_INT_PLUS_MICRO;
		}
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static const struct iio_info ap3216c_info = {
	.read_raw = ap3216c_read_raw,
	.write_raw = ap3216c_write_raw,
	.write_raw_get_fmt = &ap3216c_write_raw_get_fmt,	/* 用户空间写数据格式 */
};

static int ap3216c_chip_init(struct ap3216c_data *data)
{
	/* 初始化AP3216C */
	regmap_field_write(data->reg_func, 0x04);		/* 复位AP3216C 			*/
	mdelay(50);										/* AP3216C 复位最少10ms 	*/
	regmap_field_write(data->reg_func, 0X03);		/* 开启 ALS、PS+IR 		*/
	regmap_write(data->regmap, AP3216C_REG_ALS_CONF, 0X00);			/* ALS 单次转换触发，量程为 0～20661 lux */
	regmap_write(data->regmap, AP3216C_REG_PS_LED_DRIVER, 0X13);	/* IR LED 1脉冲，驱动电流 100% */
	return 0;
}

static int ap3216c_probe(struct i2c_client *client)
{
	struct ap3216c_data *data;
	struct iio_dev *indio_dev;
	int ret;

    /** iio info init */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;
    
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &ap3216c_info;
	indio_dev->name = AP3216C_DRV_NAME;
	indio_dev->channels = ap3216c_channels;
	indio_dev->num_channels = ARRAY_SIZE(ap3216c_channels);
	indio_dev->available_scan_masks = ap3216c_scan_masks;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/** 获取 ap3216c_data 结构体地址  */
	data = iio_priv(indio_dev);
	data->client = client;
	data->indio_dev = indio_dev;
	i2c_set_clientdata(client, indio_dev);

	/** 初始化 IIC 接口的 regmap */
	data->regmap = devm_regmap_init_i2c(client, &ap3216c_regmap_cfg);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "regmap initialization failed.\n");
		return PTR_ERR(data->regmap);
	}

	/* allocate regmap fields for bit-field access */
	data->reg_int = devm_regmap_field_alloc(&client->dev, data->regmap,
								 ap3216c_reg_field_int_enable);
	if (IS_ERR(data->reg_int)) {
		dev_err(&client->dev, "failed to allocate reg_int field\n");
		return PTR_ERR(data->reg_int);
	}

	data->reg_func = devm_regmap_field_alloc(&client->dev, data->regmap,
								 ap3216c_reg_field_func_enable);
	if (IS_ERR(data->reg_func)) {
		dev_err(&client->dev, "failed to allocate reg_func field\n");
		return PTR_ERR(data->reg_func);
	}

	mutex_init(&data->lock);

	/* 注册iio_dev */
	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_iio_register;

	ret = ap3216c_chip_init(data);
	if (ret)
		goto err_iio_register;

	return 0;

err_iio_register:
	iio_device_unregister(indio_dev);
	return ret;
}

static void ap3216c_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ap3216c_data *data = iio_priv(indio_dev);
	iio_device_unregister(indio_dev);
}

static const struct i2c_device_id ap3216c_id[] = {
	{ "liteon,ap3216c", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ap3216c_id);

static const struct of_device_id ap3216c_of_match[] = {
	{ .compatible = "liteon,ap3216c" },
	{ }
};
MODULE_DEVICE_TABLE(of, ap3216c_of_match);

static struct i2c_driver ap3216c_driver = {
	.driver = {
		.name	= AP3216C_DRV_NAME,
		.of_match_table = ap3216c_of_match,
	},
	.probe		= ap3216c_probe,
	.remove		= ap3216c_remove,
	.id_table	= ap3216c_id,
};
module_i2c_driver(ap3216c_driver);

MODULE_AUTHOR("Your Name <you@example.com>");
MODULE_DESCRIPTION("AP3216C IIO driver");
MODULE_LICENSE("GPL");