/* drivers/regulator/rk29-pwm-regulator.c
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*******************************************************************/
/*	  COPYRIGHT (C)  ROCK-CHIPS FUZHOU . ALL RIGHTS RESERVED.			  */
/*******************************************************************
FILE		:	    	rk29-pwm-regulator.c
DESC		:	rk29 pwm regulator driver
AUTHOR		:	hxy
DATE		:	2010-12-20
NOTES		:
$LOG: GPIO.C,V $
REVISION 0.01
********************************************************************/


#include <linux/bug.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/rk29-pwm-regulator.h>
#include <mach/iomux.h>
#include <linux/gpio.h>


#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif


#define	PWM_VCORE_120		40
#define PWM_VCORE_125		32
#define	PWM_VCORE_130		21
#define	PWM_VCORE_135		10
#define	PWM_VCORE_140		0

#define	PWM1_freq_factor		1	////1MHZ/PWM1_freq_factor

#if defined(CONFIG_ARCH_RK2928)
#define pwm_write_reg(id, addr, val)        __raw_writel(val, addr+(RK2928_PWM_BASE+id*0x10))
#define pwm_read_reg(id, addr)              __raw_readl(addr+(RK2928_PWM_BASE+id*0x10))
#else
#define pwm_write_reg(id, addr, val)        		__raw_writel(val, addr+(RK29_PWM_BASE+id*0x10)) 
#define pwm_read_reg(id, addr)              		__raw_readl(addr+(RK29_PWM_BASE+id*0x10))
#endif

const static int pwm_voltage_map[] = {
	950, 975, 1000, 1025, 1050, 1075, 1100, 1125, 1150, 1175, 1200, 1225, 1250, 1275, 1300, 1325, 1350, 1375, 1400
};

static struct clk *pwm_clk;

static int pwm_set_rate(struct pwm_platform_data *pdata,int nHz,u32 rate)
{
	u32 divh,divTotal;
	int id = pdata->pwm_id;
	unsigned long clkrate;
	printk("STEPHEN *****************************5! pwm_clk:%d \n",pwm_clk);
	pwm_clk = clk_get(NULL, "pwm01");
	printk("STEPHEN *****************************6! pwm_clk:%d \n",pwm_clk);
	/* 获取时钟频率（RK2926上为1485000000） */
	clkrate = clk_get_rate(pwm_clk);

	if ( id >3 || id <0 )
		return -1;

	if(rate == 0)
	{
		// iomux pwm2 to gpio2_a[3]
		rk29_mux_api_set(pdata->pwm_iomux_name, pdata->pwm_iomux_gpio);
		// set gpio to low level
		gpio_set_value(pdata->pwm_gpio,GPIO_LOW);
	}
	else if (rate <= 100)
	{
		// iomux pwm2
		rk29_mux_api_set(pdata->pwm_iomux_name, pdata->pwm_iomux_pwm);

		clk_enable(pwm_clk);
		pwm_write_reg(id,PWM_REG_CTRL, PWM_DIV|PWM_RESET);
		/* 计算APB预分频后（RK2926上，分配系数nHz为1000）一周期的时钟数 */
		divh = clkrate / nHz*PWM1_freq_factor;
		/* 根据PWM分频因子计算分频后的一周期时钟数，分频系数见下图 */
		divh = divh >> (1+(PWM_DIV>>9));
		pwm_write_reg(id,PWM_REG_LRC,(divh == 0)?1:divh);

		divTotal =pwm_read_reg(id,PWM_REG_LRC);
		divh = divTotal*rate/100;
		pwm_write_reg(id, PWM_REG_HRC, divh?divh:1);
		pwm_write_reg(id,PWM_REG_CNTR,0);
		//pwm_write_reg(id, PWM_REG_CTRL,pwm_read_reg(id,PWM_REG_CTRL)|PWM_DIV|PWM_ENABLE|PWM_TimeEN);
		pwm_write_reg(id, PWM_REG_CTRL,PWM_DIV|PWM_ENABLE|PWM_TimeEN);
		printk("STEPHEN *****************************4! PWM_REG_LRC:%d,PWM_REG_HRC:%d,PWM_DIV:%d \n",divTotal,divh,PWM_DIV);

		
	}
	else
	{
		return -1;
	}

	usleep_range(10*1000, 10*1000);


	return (0);
}

static int pwm_regulator_list_voltage(struct regulator_dev *dev,unsigned int index)
{
	DBG("Enter %s, index =%d\n",__FUNCTION__,index);
	if (index < sizeof(pwm_voltage_map)/sizeof(int))
		return pwm_voltage_map[index];
	else
		return -1;
}

static int pwm_regulator_is_enabled(struct regulator_dev *dev)
{
	DBG("Enter %s\n",__FUNCTION__);
	return 0;
}

static int pwm_regulator_enable(struct regulator_dev *dev)
{
	DBG("Enter %s\n",__FUNCTION__);
	return 0;
}

static int pwm_regulator_disable(struct regulator_dev *dev)
{
	DBG("Enter %s\n",__FUNCTION__);
	return 0;
}

static int pwm_regulator_get_voltage(struct regulator_dev *dev)
{
	struct pwm_platform_data *pdata = rdev_get_drvdata(dev);

	DBG("Enter %s\n",__FUNCTION__);  

	return (pdata->pwm_voltage*1000);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
static int pwm_regulator_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV, unsigned *selector)
#else
static int pwm_regulator_set_voltage(struct regulator_dev *dev,
		int min_uV, int max_uV)
#endif
{	   
	struct pwm_platform_data *pdata = rdev_get_drvdata(dev);

	const int *voltage_map = pwm_voltage_map;

	int min_mV = min_uV /1000, max_mA = max_uV / 1000;

	u32 size = sizeof(pwm_voltage_map)/sizeof(int), i, vol,pwm_value;

	DBG("%s:  min_uV = %d, max_uV = %d\n",__FUNCTION__, min_uV,max_uV);
	
	#ifdef	CONFIG_INCAR_PNLVCOM_ADJUST ///stephenchan 20130426  for PWM1 for lcd vcom adjust
	/*
	PWM FREQ=1000*1000/PWM1_freq_factor
	DUTY=PWM_REG_HRC/PWM_REG_LRC
	IN  this configuration PWM_REG_LRC=74
	{
	PWM_REG_LRC = pwm_clk / 1000*1000;
	PWM_REG_LRC = PWM_REG_LRC >> (1+(PWM_DIV>>9));
	}
	PWM_REG_HRC=PWM_REG_LRC*pwm_value/100
	pwm_value is percent value
			pwm_value=1;		//VCOM=2.85V
			pwm_value=50;		//VCOM=3.70V
			pwm_value=99;		//VCOM=4.55V
			1%=0.02			
	*/
	#if	defined(CONFIG_LCD_VCOM2V95)  
			pwm_value=1;		//VCOM=2.95V
	#elif	defined(CONFIG_LCD_XY81207X768G_VCOM3V2)  
			pwm_value=15;		//VCOM=3.19V
	#elif defined(CONFIG_LCD_JDF50P_VCOM3V3)    
			pwm_value=22;		//VCOM=3.32V
	#elif defined(CONFIG_LCD_XY8800X600_VCOM3V4)       
			pwm_value=29;		//VCOM=3.42V
	#elif defined(CONFIG_LCD_HC750P_VCOM3V5)     	
			pwm_value=35;		//VCOM=3.51V
	#elif defined(CONFIG_LCD_QC750P_VCOM3V6)     	
			pwm_value=40;		//VCOM=3.62V
	#elif defined(CONFIG_LCD_TM950P_VCOM3V7)           
			pwm_value=45;		//VCOM=3.72V
	#elif defined(CONFIG_LCD_750P_VCOM3V8)           
			pwm_value=50;		//VCOM=3.81V
	#elif defined(CONFIG_LCD_750P_VCOM3V9)    	
			pwm_value=56;		//VCOM=3.9V
	#elif defined(CONFIG_LCD_750P_VCOM4V0)    	
			pwm_value=61;		//VCOM=4.0V
	#elif defined(CONFIG_LCD_750P_VCOM4V1)    	
			pwm_value=67;		//VCOM=4.09V
	#elif defined(CONFIG_LCD_CPT50P_VCOM4V2)    	
			pwm_value=73;		//VCOM=4.21V
	#elif defined(CONFIG_LCD_50P_VCOM4V3)    	
			pwm_value=81;		//VCOM=4.49V			
	#elif defined(CONFIG_LCD_50P_VCOM4V4)    	
			pwm_value=86;		//VCOM=4.49V			
	#elif defined(CONFIG_LCD_50P_VCOM4V5)    	
			pwm_value=90;		//VCOM=4.49V			
	#elif defined(CONFIG_LCD_50P_VCOM4V69)    	
			pwm_value=99;		//VCOM=4.69V			
	#else
			pwm_value=99;		//VCOM=4.69V
	#endif
	
	#endif
	
	if (pwm_set_rate(pdata,1000*1000,pwm_value)!=0)
		return -1;


	printk("STEPHEN *****************************3! pdata:%d,pwm_value:%d,vol:%d,min_mV:%d,max_mA:%d  \n",pdata,pwm_value,vol,min_mV,max_mA);


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38))
	*selector = i;
#endif

	return 0;

}

static struct regulator_ops pwm_voltage_ops = {
	.list_voltage	= pwm_regulator_list_voltage,
	.set_voltage	=pwm_regulator_set_voltage,
	.get_voltage	= pwm_regulator_get_voltage,
	.enable		= pwm_regulator_enable,
	.disable	= pwm_regulator_disable,
	.is_enabled	= pwm_regulator_is_enabled,
};

static struct regulator_desc pwm_regulator= {
	.name = "pwm-regulator",
	.ops = &pwm_voltage_ops,
	.type = REGULATOR_VOLTAGE,
};

static int __devinit pwm_regulator_probe(struct platform_device *pdev)
{

	struct pwm_platform_data *pdata = pdev->dev.platform_data;
	struct regulator_dev *rdev;
	int ret ;

	printk("STEPHEN *****************************1 pwm_regulator: driver initialized\n");

	if (!pdata)
		return -ENODEV;

	#ifdef	CONFIG_INCAR_PNLVCOM_ADJUST ///stephenchan 20130508  for P2B1 as AVDD control
		#ifdef	CONFIG_INCAR_PNLAVDD_OUTPUT_HIGH
		if(gpio_request(RK2928_PIN2_PB1,NULL) != 0){
			gpio_free(RK2928_PIN2_PB1);
			printk("RK2928_PIN2_PB1  gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(RK2928_PIN2_PB1, GPIO_LOW);
		gpio_set_value(RK2928_PIN2_PB1,GPIO_HIGH);
		printk("RK2928_PIN2_PB1  stephen output HIGH ***************************************\n");
		#else
		if(gpio_request(RK2928_PIN2_PB1,NULL) != 0){
			gpio_free(RK2928_PIN2_PB1);
			printk("RK2928_PIN2_PB1  gpio_request error\n");
			return -EIO;
		}
		gpio_direction_output(RK2928_PIN2_PB1, GPIO_LOW);
		gpio_set_value(RK2928_PIN2_PB1,GPIO_LOW);
		printk("RK2928_PIN2_PB1  stephen output LOW ***************************************\n");
		#endif
	#endif

	if (!pdata->pwm_voltage)
		pdata->pwm_voltage = 1200;	// default 1.2v

	rdev = regulator_register(&pwm_regulator, &pdev->dev,
			pdata->init_data, pdata);
	if (IS_ERR(rdev)) {
		dev_dbg(&pdev->dev, "couldn't register regulator\n");
		return PTR_ERR(rdev);
	}

		printk("STEPHEN *****************************2 pdata->pwm_gpio:%d \n",pdata->pwm_gpio);

	//ret = gpio_request(pdata->pwm_gpio,"pwm");

	//if (ret) {
	//	dev_err(&pdev->dev,"failed to request pwm gpio\n");
	//	goto err_gpio;
	//}


	pwm_clk = clk_get(NULL, "pwm");
	clk_enable(pwm_clk);


	platform_set_drvdata(pdev, rdev);
	printk(KERN_INFO "pwm_regulator: driver initialized\n");

	//stephen add for initial pwm output
	unsigned selector = 0;
	pwm_regulator_set_voltage(rdev, 1000000, 1400000, &selector);



	return 0;


err_gpio:
	gpio_free(pdata->pwm_gpio);


	return ret;

}
static int __devexit pwm_regulator_remove(struct platform_device *pdev)
{
	struct pwm_platform_data *pdata = pdev->dev.platform_data;
	struct regulator_dev *rdev = platform_get_drvdata(pdev);

	regulator_unregister(rdev);
	gpio_free(pdata->pwm_gpio);

	return 0;
}



static struct platform_driver pwm_regulator_driver = {
	.driver = {
		.name = "pwm-voltage-regulator",
	},
	.remove = __devexit_p(pwm_regulator_remove),
};


static int __init pwm_regulator_module_init(void)
{
	return platform_driver_probe(&pwm_regulator_driver, pwm_regulator_probe);
}

static void __exit pwm_regulator_module_exit(void)
{
	platform_driver_unregister(&pwm_regulator_driver);
}

subsys_initcall(pwm_regulator_module_init);

module_exit(pwm_regulator_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("hxy <hxy@rock-chips.com>");
MODULE_DESCRIPTION("k29 pwm change driver");

