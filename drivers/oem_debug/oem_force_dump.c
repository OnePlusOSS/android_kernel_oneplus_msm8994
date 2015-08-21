/*
 * oem_force_dump.c
 *
 * drivers supporting debug functions for Oneplus device.
 *
 * hefaxi@filesystems, 2015/07/03.
 */
#include <linux/reboot.h>
#include <linux/input.h>

extern int oem_get_download_mode(void);

/*
 * press the voluemup key and volumedown key, then
 * long press volumeup key, and at the same time,
 * press twice power key,it will goto the force dump.
 */
void oem_check_force_dump_key(unsigned int code, int value)
{
    static enum { NONE, STEP1, STEP2, STEP3,STEP4,
                STEP5,STEP6,STEP7} state = NONE;

    if(!oem_get_download_mode())
        return ;

    //printk(KERN_INFO "%s code %d value %d state %d\n",__func__,code,value,state);
    switch(state){
    case NONE:
        if(code == KEY_VOLUMEUP && value){
            state = STEP1;
        }else{
            state = NONE;
        }
        break;
    case STEP1:
        if(code == KEY_VOLUMEUP && !value){
            state = STEP2;
        }else{
            state = NONE;
        }
        break;
    case STEP2:
        if(code == KEY_VOLUMEDOWN && value){
            state = STEP3;
        }else{
            state = NONE;
        }
        break;
    case STEP3:
        if(code == KEY_VOLUMEDOWN && !value){
            state = STEP4;
        }else{
            state = NONE;
        }
        break;
    case STEP4:
        if(code == KEY_VOLUMEUP && value){
            state = STEP5;
        }else{
            state = NONE;
        }
        break;
    case STEP5:
        if(code == KEY_POWER && value){
            state = STEP6;
        }else
            state = NONE;
    case STEP6:
        if(code == KEY_POWER && !value){
            state = STEP7;
        }else{
            state = NONE;
        }
    case STEP7:
        if(code == KEY_POWER && value){
            panic("Force Dump");
        }else{
            state = NONE;
        }
        break;
    }
}