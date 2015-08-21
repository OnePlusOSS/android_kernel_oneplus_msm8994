/* FPC1020 Touch sensor driver
 *
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#ifndef LINUX_SPI_FPC1020_INPUT_H
#define LINUX_SPI_FPC1020_INPUT_H

extern int /*__devinit*/ fpc1020_input_init(fpc1020_data_t *fpc1020);

extern void /*__devexit*/ fpc1020_input_destroy(fpc1020_data_t *fpc1020);

extern int fpc1020_input_enable(fpc1020_data_t *fpc1020, bool enabled);

extern int fpc1020_input_task(fpc1020_data_t *fpc1020);

#ifdef VENDOR_EDIT
//Lycan.Wang@Prd.BasicDrv, 2014-09-12 Add for report touch down and up
extern void fpc1020_report_finger_down(fpc1020_data_t *fpc1020);

extern void fpc1020_report_finger_up(fpc1020_data_t *fpc1020);
#endif /* VENDOR_EDIT */

#endif /* LINUX_SPI_FPC1020_NAV_H */

