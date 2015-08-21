/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_SPI_FPC1020_COMMON_H
#define LINUX_SPI_FPC1020_COMMON_H

#define DEBUG
#define CONFIG_INPUT_FPC1020_NAV

#define AS_HOME_KEY

#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/semaphore.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>

#include <linux/wakelock.h>

#if 0 //#ifndef CONFIG_USE_OF //oneplus changhua.li modify for MSM8994
#include <linux/spi/fpc1020.h>
#include <linux/spi/fpc1020_regs.h>
#else
#include "fpc1020.h"
#include "fpc1020_regs.h"
#endif

/* -------------------------------------------------------------------- */
/* fpc1020 driver constants						*/
/* -------------------------------------------------------------------- */
extern const bool target_little_endian;

#define FPC1020_DEV_NAME                        "fpc1020"
#define FPC1020_TOUCH_PAD_DEV_NAME              "fpc1020tp"

//#define FPC1020_MAJOR				230
#define FPC1020_MAJOR               -1

#define FPC1020_SPI_CLOCK_SPEED			(8 * 1000000U)

#define FPC1020_BUFFER_MAX_IMAGES		3


#define FPC_TEE_INTERRUPT_ONLY

#ifdef VENDOR_EDIT
//Lycan.Wang@Prd.BasicDrv, 2014-09-12 Add for Hw Config
#define FPC1020_PIXEL_ROWS                      80//160U

#define FPC1020_PIXEL_COLUMNS                   208//160U

#define FPC1020_FRAME_SIZE_MAX                  (FPC1020_PIXEL_COLUMNS * \
                                                 FPC1020_PIXEL_ROWS)

#define FPC1020_DEADPIXEL_THRESHOLD		11
#endif /* VENDOR_EDIT */
#define FPC1020_MAX_ADC_SETTINGS        (FPC1020_BUFFER_MAX_IMAGES + 1)

#define FPC1020_DEFAULT_IRQ_TIMEOUT_MS		(500 * HZ / 1000)

#define FPC1020_STATUS_REG_RESET_VALUE 0x1e

#define FPC1020_STATUS_REG_MODE_MASK ( \
		FPC_1020_STATUS_REG_BIT_MAIN_IDLE_CMD | \
		FPC_1020_STATUS_REG_BIT_SYNC_PWR_IDLE | \
		FPC_1020_STATUS_REG_BIT_PWR_DWN_OSC_HIN)

#define FPC1020_STATUS_REG_IN_DEEP_SLEEP_MODE	0

#define FPC1020_STATUS_REG_IN_SLEEP_MODE	0

#define FPC1020_STATUS_REG_IN_IDLE_MODE ( \
		FPC_1020_STATUS_REG_BIT_MAIN_IDLE_CMD | \
		FPC_1020_STATUS_REG_BIT_SYNC_PWR_IDLE | \
		FPC_1020_STATUS_REG_BIT_PWR_DWN_OSC_HIN)

#define FPC1020_SLEEP_RETRIES			5
#define FPC1020_SLEEP_RETRY_TIME_US		1000

#define FPC1020_RESET_RETRIES			2
#define FPC1020_RESET_LOW_US			1000
#define FPC1020_RESET_HIGH1_US			100
#define FPC1020_RESET_HIGH2_US			1250

#define FPC1020_CAPTURE_WAIT_FINGER_DELAY_MS 	20

#define NAV_IMAGE_WIDTH 128
#define NAV_IMAGE_HEIGHT 8
#define NAV_FRAME_SIZE NAV_IMAGE_WIDTH*NAV_IMAGE_HEIGHT

#define FPC1020_WAKEUP_DETECT_ZONE_COUNT	2
#define FPC1020_WAKEUP_DETECT_ROWS		8
#define FPC1020_WAKEUP_DETECT_COLS		8

#define FPC1020_PXL_BIAS_CTRL			0x0F00

/* -------------------------------------------------------------------- */
/* fpc1020 data types							*/
/* -------------------------------------------------------------------- */
typedef enum {
	FPC_1020_STATUS_REG_BIT_IRQ			= 1 << 0,
	FPC_1020_STATUS_REG_BIT_MAIN_IDLE_CMD		= 1 << 1,
	FPC_1020_STATUS_REG_BIT_SYNC_PWR_IDLE		= 1 << 2,
	FPC_1020_STATUS_REG_BIT_PWR_DWN_OSC_HIN		= 1 << 3,
	FPC_1020_STATUS_REG_BIT_FIFO_EMPTY		= 1 << 4,
	FPC_1020_STATUS_REG_BIT_FIFO_FULL		= 1 << 5,
	FPC_1020_STATUS_REG_BIT_MISO_EDGRE_RISE_EN	= 1 << 6
} fpc1020_status_reg_t;

typedef enum {
	FPC1020_CMD_FINGER_PRESENT_QUERY	= 32,
	FPC1020_CMD_WAIT_FOR_FINGER_PRESENT	= 36,
	FPC1020_CMD_ACTIVATE_SLEEP_MODE		= 40,
	FPC1020_CMD_ACTIVATE_DEEP_SLEEP_MODE	= 44,
	FPC1020_CMD_ACTIVATE_IDLE_MODE		= 52,
	FPC1020_CMD_CAPTURE_IMAGE		= 192,
	FPC1020_CMD_READ_IMAGE			= 196,
	FPC1020_CMD_SOFT_RESET			= 248
} fpc1020_cmd_t;

typedef enum {
	FPC_1020_IRQ_REG_BIT_FINGER_DOWN   = 1 << 0,
	FPC_1020_IRQ_REG_BIT_ERROR         = 1 << 2,
	FPC_1020_IRQ_REG_BIT_FIFO_NEW_DATA = 1 << 5,
	FPC_1020_IRQ_REG_BIT_COMMAND_DONE  = 1 << 7,
	FPC_1020_IRQ_REG_BITS_REBOOT       = 0xff
} fpc1020_irq_reg_t;

typedef enum {
	FPC1020_CAPTURE_STATE_IDLE = 0,
	FPC1020_CAPTURE_STATE_STARTED,
	FPC1020_CAPTURE_STATE_PENDING,
	FPC1020_CAPTURE_STATE_WRITE_SETTINGS,
	FPC1020_CAPTURE_STATE_WAIT_FOR_FINGER_DOWN,
	FPC1020_CAPTURE_STATE_ACQUIRE,
	FPC1020_CAPTURE_STATE_FETCH,
	FPC1020_CAPTURE_STATE_WAIT_FOR_FINGER_UP,
	FPC1020_CAPTURE_STATE_COMPLETED,
	FPC1020_CAPTURE_STATE_FAILED,
} fpc1020_capture_state_t;

typedef struct fpc1020_worker_struct {
	struct task_struct *thread;
	struct semaphore sem_idle;
	wait_queue_head_t wq_wait_job;
	int req_mode;
	bool stop_request;
} fpc1020_worker_t;

typedef struct fpc1020_capture_struct {
	fpc1020_capture_mode_t	current_mode;
	fpc1020_capture_state_t	state;
	u32			read_offset;
	u32			available_bytes;
	wait_queue_head_t	wq_data_avail;
	int			last_error;
	bool			read_pending_eof;
	bool			deferred_finger_up;
} fpc1020_capture_task_t;

#ifdef CONFIG_INPUT_FPC1020_NAV
typedef struct fpc1020_nav_struct {
	bool enabled;

	/*image based navigation parameter*/
	u8 image_nav_row_start;
	u8 image_nav_row_count;
	u8 image_nav_col_start;
	u8 image_nav_col_groups;

	unsigned long time;
	int tap_status;
	u8 input_mode;
	int nav_sum_x;
	int nav_sum_y;
	
	u8 p_multiplier_x;
	u8 p_multiplier_y;
	u8 p_sensitivity_key;
	u8 p_sensitivity_ptr;
	u8 multiplier_key_accel;
	u8 multiplier_ptr_accel;
	u8 threshold_key_accel;
	u8 threshold_ptr_accel;
	u8 threshold_ptr_start;
	u8 duration_ptr_clear;
	u8 nav_finger_up_threshold;
#ifdef VENDOR_EDIT
//Lycan.Wang@Prd.BasicDrv, 2014-09-29 Add for move parameter
	int move_time_threshold;
	int move_distance_threshold;
#endif /* VENDOR_EDIT */
} fpc1020_nav_task_t;
#endif

typedef struct fpc1020_setup {
	u8 adc_gain[FPC1020_MAX_ADC_SETTINGS];
	u8 adc_shift[FPC1020_MAX_ADC_SETTINGS];
	u16 pxl_ctrl[FPC1020_MAX_ADC_SETTINGS];
	u8 capture_settings_mux;
	u8 capture_count;
	fpc1020_capture_mode_t capture_mode;
	u8 capture_row_start;	/* Row 0-191        */
	u8 capture_row_count;	/* Rows <= 192      */
	u8 capture_col_start;	/* ADC group 0-23   */
	u8 capture_col_groups;	/* ADC groups, 1-24 */
	u8 capture_finger_up_threshold;
	u8 capture_finger_down_threshold;
	u8 finger_detect_threshold;
	u8 wakeup_detect_rows[FPC1020_WAKEUP_DETECT_ZONE_COUNT];
	u8 wakeup_detect_cols[FPC1020_WAKEUP_DETECT_ZONE_COUNT];
} fpc1020_setup_t;

typedef struct fpc1020_diag {
	const char *chip_id;	/* RO */
	u8  selftest;		/* RO */
	u16 spi_register;	/* RW */
	u8  spi_regsize;	/* RO */
	u8  spi_data;		/* RW */
	u16 last_capture_time;	/* RO*/
	u16 finger_present_status;	/* RO*/
} fpc1020_diag_t;

typedef struct fpc1020_chip_info {
	fpc1020_chip_t         type;
	u8                     revision;
	u8                     pixel_rows;
	u8                     pixel_columns;
	u8                     adc_group_size;
}fpc1020_chip_info_t;

typedef struct {
	struct spi_device      *spi;
	struct class           *class;
	struct device          *device;
	struct cdev            cdev;
	dev_t                  devno;
	fpc1020_chip_info_t    chip;
#ifndef VENDOR_EDIT
//Lycan.Wang@Prd.BasicDrv, 2014-09-12 Remove for Hw Config
	u32                    cs_gpio;
#endif /* VENDOR_EDIT */
	u32                    reset_gpio;
	u32                    irq_gpio;
#ifdef VENDOR_EDIT
	//Lycan.Wang@Prd.BasicDrv, 2014-09-12 Add for Hw Config
	u32                    vdden_gpio;  //ranfei
#endif /* VENDOR_EDIT */

	#ifdef VENDOR_EDIT //changhua add for reconize DT CT module
	int vendor_gpio;
	int fp2050_gpio;
	bool  to_power;
	#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
    #endif
	#endif

	int                    irq;
	wait_queue_head_t      wq_irq_return;
	bool                   interrupt_done;
	struct semaphore       mutex;
	u8                     *huge_buffer;
	size_t                 huge_buffer_size;
	fpc1020_worker_t       worker;
	fpc1020_capture_task_t capture;
	fpc1020_setup_t        setup;
	fpc1020_diag_t         diag;
	bool                   soft_reset_enabled;
#ifndef VENDOR_EDIT
	//Lycan.Wang@Prd.BasicDrv, 2014-09-12 Remove for Hw Config
	struct regulator       *vcc_spi;
#endif /* VENDOR_EDIT */
    struct regulator       *vdd_ana;
	struct regulator       *vdd_io;
	bool                   power_enabled;
	int                    vddtx_mv;
	bool                   txout_boost;

#ifdef CONFIG_INPUT_FPC1020_NAV
	struct input_dev	*input_dev;
	struct input_dev	*touch_pad_dev;
	fpc1020_nav_task_t	nav;
	u8* prev_img_buf;
	u8* cur_img_buf;
#ifdef VENDOR_EDIT
	//Lycan.Wang@Prd.BasicDrv, 2014-09-29 Add for navigation move event
	unsigned long 		touch_time;
	int 				move_distance;	
	unsigned int 		moving_key;
#endif /* VENDOR_EDIT */

#ifdef FPC_TEE_INTERRUPT_ONLY
	wait_queue_head_t 		g_irq_event;
	bool                    tee_interrupt_done;
	bool 					tee_int_enable;
	bool                    wait_abort;
	bool                    wake_irq_state;

	struct wake_lock    wakelock;
	struct wake_lock    irq_wakelock;
#endif

#endif
} fpc1020_data_t;

typedef struct {
	fpc1020_reg_t reg;
	bool          write;
	u16           reg_size;
	u8            *dataptr;
} fpc1020_reg_access_t;


/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
extern size_t fpc1020_calc_huge_buffer_minsize(fpc1020_data_t *fpc1020);

extern int fpc1020_manage_huge_buffer(fpc1020_data_t *fpc1020,
					size_t new_size);

extern int fpc1020_setup_defaults(fpc1020_data_t *fpc1020);

extern int fpc1020_gpio_reset(fpc1020_data_t *fpc1020);

extern int fpc1020_spi_reset(fpc1020_data_t *fpc1020);

extern int fpc1020_reset(fpc1020_data_t *fpc1020);

extern int fpc1020_check_hw_id(fpc1020_data_t *fpc1020);

extern const char *fpc1020_hw_id_text(fpc1020_data_t *fpc1020);

extern int fpc1020_write_sensor_setup(fpc1020_data_t *fpc1020);

extern int fpc1020_wait_for_irq(fpc1020_data_t *fpc1020, int timeout);

extern int fpc1020_read_irq(fpc1020_data_t *fpc1020, bool clear_irq);

extern int fpc1020_read_status_reg(fpc1020_data_t *fpc1020);

extern int fpc1020_reg_access(fpc1020_data_t *fpc1020,
			      fpc1020_reg_access_t *reg_data);

extern int fpc1020_cmd(fpc1020_data_t *fpc1020, fpc1020_cmd_t cmd,
			u8 wait_irq_mask);

extern int fpc1020_wait_finger_present(fpc1020_data_t *fpc1020);
#ifdef AS_HOME_KEY
extern int fpc1020_wait_finger_present_timeout(fpc1020_data_t *fpc1020);
#endif
extern int fpc1020_get_finger_present_status(fpc1020_data_t *fpc1020);

extern int fpc1020_check_finger_present_raw(fpc1020_data_t *fpc1020);

extern int fpc1020_check_finger_present_sum(fpc1020_data_t *fpc1020);

extern int fpc1020_wake_up(fpc1020_data_t *fpc1020);

extern int fpc1020_sleep(fpc1020_data_t *fpc1020, bool deep_sleep);

extern int fpc1020_fetch_image(fpc1020_data_t *fpc1020,
				u8 *buffer,
				int offset,
				size_t image_size_bytes,
				size_t buff_size);

extern bool fpc1020_check_in_range_u64(u64 val, u64 min, u64 max);

extern int fpc1020_calc_finger_detect_threshold_min(fpc1020_data_t *fpc1020);

extern int fpc1020_set_finger_detect_threshold(fpc1020_data_t *fpc1020,
						int measured_val);

#define FPC1020_MK_REG_READ_BYTES(__dst, __reg, __count, __ptr) {	\
	(__dst).reg      = FPC1020_REG_TO_ACTUAL((__reg));		\
	(__dst).reg_size = (__count);					\
	(__dst).write    = false;					\
	(__dst).dataptr  = (__ptr); }

#define FPC1020_MK_REG_READ(__dst, __reg, __ptr) {			\
	(__dst).reg      = FPC1020_REG_TO_ACTUAL((__reg));		\
	(__dst).reg_size = FPC1020_REG_SIZE((__reg));			\
	(__dst).write    = false;					\
	(__dst).dataptr  = (u8 *)(__ptr); }

#define FPC1020_MK_REG_WRITE_BYTES(__dst, __reg, __count, __ptr) {	\
	(__dst).reg      = FPC1020_REG_TO_ACTUAL((__reg));		\
	(__dst).reg_size = (__count);					\
	(__dst).write    = true;					\
	(__dst).dataptr  = (__ptr); }

#define FPC1020_MK_REG_WRITE(__dst, __reg, __ptr) {			\
	(__dst).reg      = FPC1020_REG_TO_ACTUAL((__reg));		\
	(__dst).reg_size = FPC1020_REG_SIZE((__reg));			\
	(__dst).write    = true;					\
	(__dst).dataptr  = (u8 *)(__ptr); }

#define FPC1020_FINGER_DETECT_ZONE_MASK		0x0FFFU

#endif /* LINUX_SPI_FPC1020_COMMON_H */

