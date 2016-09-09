/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _ASM_ARCH_MSM_RESTART_H_
#define _ASM_ARCH_MSM_RESTART_H_

#define RESTART_NORMAL 0x0
#define RESTART_DLOAD  0x1

void msm_set_restart_mode(int mode);
extern int pmic_reset_irq;

#ifdef VENDOR_EDIT

#define uint32 uint32_t
#define uint64 uint64_t

struct boot_shared_imem_cookie_type
{
  /* First 8 bytes are two dload magic numbers */
  uint32 dload_magic_1;
  uint32 dload_magic_2;

  /* Magic number which indicates boot shared imem has been initialized
     and the content is valid.*/
  uint32 shared_imem_magic;

  /* Magic number for UEFI ram dump, if this cookie is set along with dload magic numbers,
     we don't enter dload mode but continue to boot. This cookie should only be set by UEFI*/
  uint32 uefi_ram_dump_magic;

  /* Pointer that points to etb ram dump buffer, should only be set by HLOS */
  uint32 etb_buf_addr;

  /* Region where HLOS would write the l2 cache dump buffer start address
     but currently unused                                                   */
  uint32 l2_cache_dump_buff_addr;

  uint32 ddr_training_cookie;

  /* Cookie that will be used to sync with RPM */
  uint32 rpm_sync_cookie;

  /* Abnormal reset cookie used by UEFI */
  uint32 abnormal_reset_occurred;

  /* Reset Status Register */
  uint32 reset_status_register;

  /* Kmsg buffer start address*/
  uint32 kmsg_address_start;

  /* Kmsg buffer size */
  uint32 kmsg_address_size;

  /* Device Info Start Address */
  uint32 device_info_addr;

  /* Device Info Size */
  uint32 device_info_size;

  /* Please add new cookie here, do NOT modify or rearrange the existing cookies*/
};
#endif /*VENDOR_EDIT*/

#endif

