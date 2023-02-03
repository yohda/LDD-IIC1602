#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/kernel.h>

/* mdelay()  */
#include <linux/delay.h>
#include <linux/timer.h>

/* strlen() */
#include <linux/string.h>

/* errno */
#include <linux/errno.h>

#define IIC1602_NAME "iic1602"             

/* low 4-bits */
#define P0_RS 			(0x01)
#define P1_RW 			(0x02)
#define P2_E 			(0x04)
#define P3_BACKLIGHT 	(0x08)

/* IIC1602 Instruction set */
#define IIC_INS_CLEAR_DISPLAY 				(0x01)
#define IIC_INS_RETURN_HOME					(0x02)
#define IIC_INS_ENTRY_MODE_SET				(0x04)

#define IIC_INS_DISPLAY_ON_OFF				(0x08)
	#define DISPLAY_ON						((0x04)|(IIC_INS_DISPLAY_ON_OFF))					
	#define CURSOR_ON						((0x02)|(IIC_INS_DISPLAY_ON_OFF))
	#define BLINK_CUSOUR_ON					((0x01)|(IIC_INS_DISPLAY_ON_OFF))

#define IIC_INS_CURSOR_OR_DISPLAY_SHIFT		(0x10) 
	#define CURSOR_LEFT						((0x00)|(IIC_INS_CURSOR_OR_DISPLAY_SHIFT))
	#define CURSOR_RIGHT					((0x04)|(IIC_INS_CURSOR_OR_DISPLAY_SHIFT))
	#define DISPLAY_LEFT					((0x08)|(IIC_INS_CURSOR_OR_DISPLAY_SHIFT))
	#define DISPLAY_RIGHT					((0x0C)|(IIC_INS_CURSOR_OR_DISPLAY_SHIFT))

#define IIC_INS_FUNCTION_SET 				(0x20)
	#define BIT4							(IIC_INS_FUNCTION_SET)
	#define	BIT8							((IIC_INS_FUNCTION_SET)|0x01)
	#define LINE1 							(IIC_INS_FUNCTION_SET)
	#define LINE2 							((IIC_INS_FUNCTION_SET)|0x08)

#define IIC_INS_SET_CGRAM_ADDR				(0x40)
#define IIC_INS_SET_DDRAM_ADDR				(0x80)
#define IIC_INS_BUSY_ADDR					(0x02)

/* IIC1602 Data set */ 
#define IIC_DATA_WRITE					(0x01)
#define IIC_DATA_READ					(0x03)

/* IIC1602 Row & Col */
#define IIC_MIN_ROW						(0x00)
#define IIC_MAX_ROW 					(0x0F)
#define IIC_MIN_COL						(0x00)
#define IIC_MAX_COL 					(0x01)
#define IIC_1LINE_ADDR					(0x00)
#define IIC_2LINE_ADDR					(0x40)

/* IIC1602 Direction Shift */
#define LEFT  							(0x00)
#define RIGHT 							(0x01)

#ifdef ST_FUNCTION_SET
	#define ST_DB_4(x)					((x)&((0x00)<<4))			/**/
	#define ST_DL_8(x)					((x)|((0x01)<<4)) 			/**/
	#define ST_DIS_1(x)					((x)&((0x00)<<4))			/**/
	#define ST_DIS_2(x)					((x)|((0x01)<<4))			/**/ 
	#define ST_DIS_5_8(x)				((x)&((0x00)<<4))
	#define ST_DIS_5_11(x)				((x)|((0x01)<<4))
#endif

int debug = 1;

struct iic1602_reg_info {
	unsigned char val;
	unsigned char addr;	
};

struct iic1602_info {
	struct i2c_client *client;
	struct icc1602_reg *regs;
	bool is_backlight_on;
};

static struct i2c_client  *yohda_i2c_client = NULL;  

static int iic1602_write_8bits(const unsigned char data)
{
	i2c_smbus_write_byte(yohda_i2c_client, data); 
	ndelay(50); // Tsa(40ns)
	
	i2c_smbus_write_byte(yohda_i2c_client, P2_E);
	ndelay(150); // PWeh(230ns) - Tdsw(80ns) = 150ns
	
	i2c_smbus_write_byte(yohda_i2c_client, data|P2_E); 
	ndelay(80);
	
	i2c_smbus_write_byte(yohda_i2c_client, (data & ~P2_E)); 
	ndelay(270);

	return 0;
}

static int iic1602_write_4bits(const unsigned char data)
{
	i2c_smbus_write_byte(yohda_i2c_client, data); 
	ndelay(50); // Tsa(40ns)
	
	i2c_smbus_write_byte(yohda_i2c_client, P2_E);
	ndelay(150); // PWeh(230ns) - Tdsw(80ns) = 150ns
	
	i2c_smbus_write_byte(yohda_i2c_client, data|P2_E); 
	ndelay(80);
	
	i2c_smbus_write_byte(yohda_i2c_client, (data & ~P2_E)); 
	ndelay(270);

	return 0;
}

static void iic1602_write_4bits_instruction(const unsigned char instruction)
{
	iic1602_write_4bits((instruction & 0xF0) | P3_BACKLIGHT);
	iic1602_write_4bits(((instruction << 4) & 0xF0) | P3_BACKLIGHT);
}

static void iic1602_write_4bits_data(const unsigned char data)
{
	iic1602_write_4bits((data & 0xF0) | P3_BACKLIGHT | P0_RS);
	iic1602_write_4bits(((data << 4) & 0xF0) | P3_BACKLIGHT | P0_RS);
}

static int iic1602_write_string(const char *string)
{
	int len = strlen(string);
		
	if(len < IIC_MIN_ROW || len > IIC_MAX_ROW)
	{
		pr_err("[YOHDA] length of arguemnt:%d\n", len);
		return -EINVAL;
	}		

	while(*string)
	{
		iic1602_write_4bits_data(*string);
		string++;
	}

	return 0;
}

static int iic1602_set_position(const int col, const int row)
{
	int data = IIC_INS_SET_DDRAM_ADDR;
	if(col > IIC_MAX_COL || col < IIC_MIN_COL)
	{
		pr_err("[YOHDA] The first argument is invalid range. col:%d\n", col);
		return -EINVAL;
	}

	if(row > IIC_MAX_ROW || row < IIC_MIN_ROW) 
	{
		pr_err("[YOHDA] The second argument is invalid range. row:%d\n", row);
		return -EINVAL;
	}
	
	data |= col == IIC_MIN_COL ? row : (row + IIC_2LINE_ADDR);
	iic1602_write_4bits_instruction(data);

	return 0;	
}

static int iic1602_reset(void)
{
	
	return 0;
}

static void iic1602_clear_display(void)
{
	iic1602_write_4bits_instruction(IIC_INS_CLEAR_DISPLAY);
	udelay(500); // There is need to modify the delay value. Because, i can`t find out the condition about delay. If i don`t write this down, characters is displayed with strange characters.
}

static int iic1602_backlight_on(const bool on)
{
	int err;
	bool on_off = !!on;
	
	err = i2c_smbus_write_byte(yohda_i2c_client, on_off ? P3_BACKLIGHT : 0x00);	
	if(err < 0)
	{
		pr_err("Failed to control the backlight. err:%d\n", err);
		return err;
	}	

	return 0;
}

static int iic1602_display_on(const bool on)
{
	bool on_off = !!on;
	unsigned int ins = (on_off ? DISPLAY_ON : IIC_INS_DISPLAY_ON_OFF);
	
	if(debug)
		pr_info("iic1602 display on/off instruction:0x%x\n", ins);
	iic1602_write_4bits_instruction(ins);

	return 0;
}

static int iic1602_cursor_on(const bool on)
{
	bool on_off = !!on;
/* 커서가 켜질때 될 때, 전제는 반드시 display가 켜서 있어야 한다. 그래서 0x04를 추가한 것이다. */	
	unsigned int ins = DISPLAY_ON|(on_off ? CURSOR_ON : 0x00);

	if(debug)
		pr_info("iic1602 cursor on/off instruction:0x%x\n", ins);
	iic1602_write_4bits_instruction(ins);

	return 0;
}

static int iic1602_blink_cursor_on(const bool on)
{
	bool on_off = !!on;
/* 커서가 블링크 될 때, 전제는 반드시 display가 켜서 있어야 의미가 있다. 그래서 0x04를 추가한 것이다. */	
	unsigned int ins = DISPLAY_ON|(on_off ? BLINK_CUSOUR_ON : 0x00);

	if(debug)
		pr_info("iic1602 blink cursor on/off instruction:0x%x\n", ins);
	iic1602_write_4bits_instruction(ins);

	return 0;
}

static int iic1602_display_shift(const bool on, const bool dir)
{
	bool on_off = !!on;
	unsigned char ins = (dir == LEFT ? DISPLAY_LEFT : DISPLAY_RIGHT);
	
	if(debug)
		pr_info("iic1602 display shift instruction:0x%x\n", ins);
	iic1602_write_4bits_instruction(ins);
	return 0;
}

static int iic1602_cursor_shift(const bool on, const bool dir)
{
	bool on_off = !!on;
	unsigned char ins = (dir == LEFT ? CURSOR_LEFT : CURSOR_RIGHT);
	
	if(debug)
		pr_info("iic1602 cursor shift instruction:0x%x\n", ins);
	iic1602_write_4bits_instruction(ins);

	return 0;
}

static int iic1602_read_4bits(void)
{
	const unsigned char read_condition = P1_RW|P3_BACKLIGHT;
	int status = 0;

	//pr_info("read:0x%x\n", read_condition);	
	i2c_smbus_write_byte(yohda_i2c_client, read_condition);
	ndelay(50); // Tsa(Min 40ns)
	
	i2c_smbus_write_byte(yohda_i2c_client, read_condition|P2_E);
	ndelay(150); // Tddr(Max 160ns)
	status = i2c_smbus_read_byte(yohda_i2c_client); 
	ndelay(80); // PWeh(Min 230ns)
	
	i2c_smbus_write_byte(yohda_i2c_client, read_condition|(~P2_E)); 
	ndelay(270);

	return status;
}

static int iic1602_read(unsigned char *buf)
{
	int status, i;
	
	for(i = 1; i > -1; i--)
	{
		status = iic1602_read_4bits();
		if(status < 0)
		{
			pr_err("[YOHDA] Failed to read by i2c. status:%d\n", status);
			return status;
		}
			
		if(debug)
			pr_info("i:%d, status:0x%x\n", i, status);

		*buf |= (i == 1 ? 0xF0 : 0x0F) & (unsigned char)(status << (i*4));
		if(debug)
			pr_info("[YOHDA] read buf:0x%x\n", *buf);
	}

	return 0;	
}

static int iic1602_init(void)
{
	/* 8-bit initialize */
	iic1602_write_8bits(0x30|P3_BACKLIGHT);
	mdelay(5);
	iic1602_write_8bits(0x30|P3_BACKLIGHT);
	udelay(100);
	iic1602_write_8bits(0x30|P3_BACKLIGHT);
	udelay(100);
	iic1602_write_8bits(0x20|P3_BACKLIGHT);
	
	/* Function Set */
	iic1602_write_4bits_instruction(BIT4|LINE2);
	udelay(53);

	/* Display On/Off */  
	iic1602_display_on(FALSE);
	udelay(53);

	/* Clear Display */
	iic1602_clear_display();
	mdelay(3);

	/* Entry Mode set(0x03) I/D:0x02, S:0x01 */
	iic1602_write_4bits_instruction(0x06);

	mdelay(200);
	
	iic1602_display_on(TRUE);
	pr_info("YOHDA ST7760U initialized");
	
	return 0;
}

static int iic1602_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
	struct iic1602_info *iic1602;	
    int err = -1;
	pr_info("YOHDA Probe started!!!\n");
	unsigned char u = 0x00;

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
	{
		pr_err("not supported:%d\n", I2C_FUNC_SMBUS_BYTE_DATA);
		return -EIO;
	}

	iic1602 = devm_kzalloc(&client->dev, sizeof(struct iic1602_info), GFP_KERNEL);
	if(!iic1602)
	{
		pr_err("Couldn`t allocate the memory.\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, iic1602);

	iic1602->client = client;

	yohda_i2c_client = client;
	iic1602_init();

	iic1602_write_string("0");
	//msleep(1000);
	iic1602_read(&u);
	u = 0;	

	iic1602_write_string("1");
	msleep(1000);
	iic1602_read(&u);
	u = 0;	

	iic1602_write_string("2");
	//msleep(1000);
	iic1602_read(&u);
	u = 0;	

	iic1602_write_string("3");
	msleep(1000);
	iic1602_read(&u);
	pr_info("read:0x%x\n", u);
	u = 0;	

	iic1602_write_string("4");
	//msleep(1000);
	iic1602_read(&u);
	pr_info("read:0x%x\n", u);
	u = 0;	

	iic1602_write_string("5");
	msleep(1000);
	iic1602_read(&u);
	pr_info("read:0x%x\n", u);
	u = 0;	

	iic1602_write_string("6");
	//msleep(1000);
	iic1602_read(&u);
	pr_info("read:0x%x\n", u);
	u = 0;	

	iic1602_write_string("7");
	msleep(1000);
	iic1602_read(&u);
	pr_info("read:0x%x\n", u);
	u = 0;	

	iic1602_write_string("8");
	//msleep(1000);
	iic1602_read(&u);
	pr_info("read:0x%x\n", u);
	u = 0;	

	pr_info("YOHDA probe done!!!\n");
	return 0;
}

static int iic1602_remove(struct i2c_client *client)
{   
    pr_info("YOHDA Removed!!!\n");
    return 0;
}

static const struct of_device_id iic1602_dt_ids[] = {
	{ .compatible = "clcd,iic1602" },
	{}
};
MODULE_DEVICE_TABLE(of, iic1602_dt_ids);

static struct i2c_driver iic1602_driver = {
        .driver = {
            .name   = IIC1602_NAME ,
            .owner  = THIS_MODULE,
			.of_match_table = iic1602_dt_ids,
        },
        .probe          = iic1602_probe,
        .remove         = iic1602_remove,
};

static int __init yohda_iic1602_init(void)
{
    return i2c_add_driver(&iic1602_driver);}

static void __exit yohda_iic1602_exit(void)
{
    i2c_del_driver(&iic1602_driver);
}

module_init(yohda_iic1602_init);
module_exit(yohda_iic1602_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yohan Yoon <dbsdy1235@gmail.com>");
MODULE_DESCRIPTION("IIC1602 Device Driver");
