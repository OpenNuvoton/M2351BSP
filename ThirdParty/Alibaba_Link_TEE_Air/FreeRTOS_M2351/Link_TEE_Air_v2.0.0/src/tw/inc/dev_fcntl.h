/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 *
 */

#ifndef _DEV_FCNTL_H_
#define _DEV_FCNTL_H_

#include "tee_types.h"

/*
 * return >0:success -1:error
 */
long dev_open(const int8_t *name, int32_t flags);
/*
 * return 0:success -1:error
 */
int32_t dev_close(long fd);
/*
 * return 0:success -1:error
 */
int32_t dev_ioctl(long fd, int32_t request, void *arg);
/*
 * return >=0:success -1:error
 */
ssize_t dev_read(long fd, void *buf, size_t count);
ssize_t dev_write(long fd, const void *buf, size_t count);

#endif /* _DEV_FCNTL_H_ */
