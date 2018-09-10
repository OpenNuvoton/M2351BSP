/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_OSA_MEM_H_
#define _TEE_OSA_MEM_H_

void *tee_osa_malloc(size_t size);
void *tee_osa_malloc_align(size_t size, size_t align);
void tee_osa_free(void *addr);

#endif /* _TEE_OSA_MEM_H_ */
