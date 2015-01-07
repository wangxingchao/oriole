#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <mach/regs-pwm.h>
#include <mach/system.h>
#include <mach/lcdif.h>
#include <mach/regulator.h>
#include <linux/string.h>

struct easy283_pwm_data {
	char    pwm_name[16];
	struct backlight_device * bd;
	struct easy283_pwm_platform_data *pdata;
        int current_intensity;
	int pwm_priod;
        int saved_intensity;
};

static struct clk *pwm_clk;

static int init_pwm(int ch,int period)
{

        struct clk *pwm_clk;
	int ret = 0;
#define	PWM_CH_NUMS	7
	if (ch > PWM_CH_NUMS) return -1;
 
        pwm_clk = clk_get(NULL, "pwm");
        if (IS_ERR(pwm_clk)) {
                ret = PTR_ERR(pwm_clk);
                return ret;
        }
        clk_enable(pwm_clk);
//      printk("%s,%d:set the back light on \n",__func__,__LINE__);
        mxs_reset_block(REGS_PWM_BASE, 1);

	printk("init_pwm ch %d\r\n",ch);
        __raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
                     BF_PWM_ACTIVEn_ACTIVE(0),
                     REGS_PWM_BASE + HW_PWM_ACTIVEn(ch));
        __raw_writel(BF_PWM_PERIODn_CDIV(5) |   /* divide by 64 */
                     BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
                     BF_PWM_PERIODn_ACTIVE_STATE(3) |   /* high */
                     BF_PWM_PERIODn_PERIOD(period-1),
                     REGS_PWM_BASE + HW_PWM_PERIODn(ch));
        __raw_writel(1<<ch, REGS_PWM_BASE + HW_PWM_CTRL_SET);

        return 0;
}

static void free_pwm(int ch)
{

	int pwm_period;

	if (ch > PWM_CH_NUMS) return ;
	pwm_period = __raw_readl(REGS_PWM_BASE + HW_PWM_PERIODn(ch)) & 0xffff;	
//      printk("%s,%d:set the back light on \n",__func__,__LINE__);
        __raw_writel(BF_PWM_ACTIVEn_INACTIVE(0) |
                     BF_PWM_ACTIVEn_ACTIVE(0),
                     REGS_PWM_BASE + HW_PWM_ACTIVEn(ch));
        __raw_writel(BF_PWM_PERIODn_CDIV(5) |   /* divide by 64 */
                     BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low */
                     BF_PWM_PERIODn_ACTIVE_STATE(3) |   /* high */
                     BF_PWM_PERIODn_PERIOD(pwm_period),
                     REGS_PWM_BASE + HW_PWM_PERIODn(ch));
        __raw_writel(1<<ch, REGS_PWM_BASE + HW_PWM_CTRL_CLR);

        clk_disable(pwm_clk);
        clk_put(pwm_clk);
}

static int set_pwm_intensity(int ch,
                            int intensity)
{
       
	unsigned short period;
	period = __raw_readl(REGS_PWM_BASE + HW_PWM_PERIODn(ch)) & 0xffff;	
 	if (ch > PWM_CH_NUMS) return -1;   
	if (intensity == (period+1)) {
		__raw_writel(BF_PWM_PERIODn_CDIV(5) |   /* divide by 64 */
                     BF_PWM_PERIODn_INACTIVE_STATE(3) | /* high  */
                     BF_PWM_PERIODn_ACTIVE_STATE(3) |   /* high */
                     BF_PWM_PERIODn_PERIOD(period),
                     REGS_PWM_BASE + HW_PWM_PERIODn(ch));
		return 0;
	}	
//        printk("period %d = %d,intensity=%d\r\n",ch,period+1,intensity);
	__raw_writel(BF_PWM_ACTIVEn_INACTIVE(intensity) |
                     BF_PWM_ACTIVEn_ACTIVE(0),
                     REGS_PWM_BASE + HW_PWM_ACTIVEn(ch));
       __raw_writel(BF_PWM_PERIODn_CDIV(5) |   /* divide by 64 */
                     BF_PWM_PERIODn_INACTIVE_STATE(2) | /* low  */
                     BF_PWM_PERIODn_ACTIVE_STATE(3) |   /* hight */
                     BF_PWM_PERIODn_PERIOD(period),
                     REGS_PWM_BASE + HW_PWM_PERIODn(ch));
 	return 0;
}


static int pwm_set_intensity(struct backlight_device *bd)
{
        struct platform_device *pdev = dev_get_drvdata(&bd->dev);
        struct easy283_pwm_data *data = platform_get_drvdata(pdev);
        struct easy283_pwm_platform_data *pdata = data->pdata;
        if (pdata) {
                int ret;

                ret =set_pwm_intensity(pdata->pwm_ch,bd->props.brightness);
                if (ret)
                        bd->props.brightness = data->current_intensity;
                else
                        data->current_intensity = bd->props.brightness;
                return ret;
        } else
                return -ENODEV;
}

static int pwm_get_intensity(struct backlight_device *bd)
{
        struct platform_device *pdev = dev_get_drvdata(&bd->dev);
        struct easy283_pwm_data *data = platform_get_drvdata(pdev);

        return data->current_intensity;
}

int pwm_check_fb (struct backlight_device *bl_dev, struct fb_info *fb)
{
//	printk("pwm_check_fb\r\n");
	return 0;
}

static struct backlight_ops bck_pwm_ops = {
        .get_brightness = pwm_get_intensity,
        .update_status  = pwm_set_intensity,
	.check_fb = pwm_check_fb,
};

static int __init pwm_probe(struct platform_device *pdev)
{
 	struct easy283_pwm_data *data;
        struct easy283_pwm_platform_data *pdata = pdev->dev.platform_data;

	int ret = 0;
        data = kzalloc(sizeof(*data), GFP_KERNEL);
        if (!data) {
                ret = -ENOMEM;
                goto out;
        }
	strlcat(data->pwm_name,pdev->name,strlen(pdev->name)+1);
	sprintf(data->pwm_name,"%s.%d",pdev->name, pdata->pwm_ch);
//	printk("%s\r\n",data->pwm_name);
        data->bd = backlight_device_register(data->pwm_name, &pdev->dev, pdev,
                                        &bck_pwm_ops, NULL);	        
	if (IS_ERR(data->bd)) {
                ret = PTR_ERR(data->bd);
                goto out_1;
        }
	data->pdata  = pdata;
       	data->bd->props.max_brightness = pdata->max_intensity;
        data->bd->props.brightness = pdata->default_intensity;
	printk("baclight_device %s register succcessed \r\n",data->pwm_name);
	get_device(&pdev->dev);
	init_pwm(pdata->pwm_ch,pdata->max_intensity);
	set_pwm_intensity(pdata->pwm_ch,pdata->default_intensity);
	platform_set_drvdata(pdev, data);
	return ret;
out_1:
	kfree(data);
out:	
	return ret;
}
static int pwm_remove(struct platform_device *pdev)
{
	struct easy283_pwm_platform_data  *pdata = pdev->dev.platform_data;
	struct easy283_pwm_data *data = platform_get_drvdata(pdev);
	struct backlight_device *bd = data->bd;
	
	bd->props.brightness = 0;
        data->current_intensity = bd->props.brightness;
	
	backlight_device_unregister(bd);
	platform_set_drvdata(pdev, NULL);
	free_pwm(pdata->pwm_ch);
	put_device(&pdev->dev);
	kfree(data);
	return 0;
}

int pwm_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

int pwm_resume(struct platform_device *dev)
{
	return 0;
}

static struct platform_driver easy283_pwm_driver = {
        .probe          = pwm_probe,
        .remove         = __devexit_p(pwm_remove),
        .suspend        = pwm_suspend,
        .resume         = pwm_resume,
        .driver         = {
                .name   = "easy283-pwm",
                .owner  = THIS_MODULE,
        },
};


static int __init easy283_pwm_init(void)
{
        return platform_driver_register(&easy283_pwm_driver);
}

static void __exit easy283_pwm_exit(void)
{
        platform_driver_unregister(&easy283_pwm_driver);
}


module_init(easy283_pwm_init);
module_exit(easy283_pwm_exit);

MODULE_AUTHOR("ZLG qjw");
MODULE_DESCRIPTION("EasyARM-iMX283 PWM_4 PWM7  Driver");
MODULE_LICENSE("GPL");


