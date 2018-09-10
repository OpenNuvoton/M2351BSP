/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_TOS_H_
#define _TEE_TOS_H_

#include "tee_types.h"

#define TEE_SUCCESS                         (0x00000000)
#define TEE_ERROR_GENERIC                   (0xFFFF0000)
#define TEE_ERROR_ACCESS_DENIED             (0xFFFF0001)
#define TEE_ERROR_CANCEL                    (0xFFFF0002)
#define TEE_ERROR_ACCESS_CONFLICT           (0xFFFF0003)
#define TEE_ERROR_EXCESS_DATA               (0xFFFF0004)
#define TEE_ERROR_BAD_FORMAT                (0xFFFF0005)
#define TEE_ERROR_BAD_PARAMETERS            (0xFFFF0006)
#define TEE_ERROR_BAD_STATE                 (0xFFFF0007)
#define TEE_ERROR_ITEM_NOT_FOUND            (0xFFFF0008)
#define TEE_ERROR_NOT_IMPLEMENTED           (0xFFFF0009)
#define TEE_ERROR_NOT_SUPPORTED             (0xFFFF000A)
#define TEE_ERROR_NO_DATA                   (0xFFFF000B)
#define TEE_ERROR_OUT_OF_MEMORY             (0xFFFF000C)
#define TEE_ERROR_BUSY                      (0xFFFF000D)
#define TEE_ERROR_COMMUNICATION             (0xFFFF000E)
#define TEE_ERROR_SECURITY                  (0xFFFF000F)
#define TEE_ERROR_SHORT_BUFFER              (0xFFFF0010)
#define TEE_PENDING                         (0xFFFF2000)
#define TEE_ERROR_TIMEOUT                   (0xFFFF3001)
#define TEE_ERROR_OVERFLOW                  (0xFFFF300F)
#define TEE_ERROR_TARGET_DEAD               (0xFFFF3024)
#define TEE_ERROR_STORAGE_NO_SPACE          (0xFFFF3041)
#define TEE_ERROR_MAC_INVALID               (0xFFFF3071)
#define TEE_ERROR_SIGNATURE_INVALID         (0xFFFF3072)
#define TEE_ERROR_TIME_NOT_SET              (0xFFFF5000)
#define TEE_ERROR_TIME_NEEDS_RESET          (0xFFFF5001)

typedef uint32_t TEE_Result;

typedef struct _tee_uuid_t
{
    uint32_t timeLow;
    uint16_t timeMid;
    uint16_t timeHiAndVersion;
    uint8_t clockSeqAndNode[8];
} tee_uuid_t;

typedef struct
{
    uint32_t    login;
    tee_uuid_t  uuid;
} TEE_Identity;

typedef struct _tee_val_t
{
    uint32_t val_a;
    uint32_t val_b;
} tee_val_t;

typedef struct _tee_shm_t
{
    union
    {
        uintptr_t buf;
        uint64_t  rsvd;
    };
    uint32_t  size;
} tee_shm_t;

typedef union _tee_param_t
{
    tee_val_t val;
    tee_shm_t shm;
} tee_param_t;

typedef struct _tee_op_t
{
    uint32_t     param_types;
    tee_param_t  params[4];
} tee_op_t;

#define TEE_NONE                    (0x0)
#define TEE_VALUE                   (0x1)
#define TEE_PTR                     (0x2)

#define TEE_PARAM_TYPES(t0, t1, t2, t3) \
        ((t0) | ((t1) << 4) | ((t2) << 8) | ((t3) << 12))
#define TEE_PARAM_TYPE_GET(t, i) (((t) >> (i*4)) & 0xF)

typedef void *tee_ss_t;

typedef tee_stat_t (*ta_create_entry_point_t)(void) ;
typedef void (*ta_destroy_entry_point_t)(void);
typedef tee_stat_t (*ta_open_ss_entry_point)(
        tee_ss_t *ss);
typedef void (*ta_close_ss_entry_point)(tee_ss_t ss);
typedef tee_stat_t (*ta_inv_cmd_entry_point)(
        tee_ss_t ss, uint32_t cmd_id,
        uint32_t param_types, tee_param_t params[4]);

#if CONFIG_API_POOL

void *tee_malloc(size_t size);
/* not support now */
void *tee_malloc_align(size_t size, size_t align);
void tee_free(void *addr);

#endif

#if CONFIG_API_STRING

void *tee_memcpy(void *dest, const void *src, size_t count);
void *tee_memmove(void *dest, const void *src, size_t count);
void *tee_memset(void *s, int c, size_t n);
int tee_memcmp(const void *s1, const void *s2, size_t n);
int8_t *tee_strcpy(int8_t *dest, const int8_t *src);
int8_t *tee_strncpy(int8_t *dest, const int8_t *src, size_t count);
int8_t *tee_strcat(int8_t *dest, const int8_t *src);
int8_t *tee_strncat(int8_t *dest, const int8_t *src, size_t count);
int tee_strcmp(const int8_t *s1, const int8_t *s2);
int tee_strncmp(const int8_t *s1, const int8_t *s2, size_t n);
int tee_strcasecmp(const int8_t *s1, const int8_t *s2);
size_t tee_strlen(const int8_t * s);
size_t tee_strnlen(const int8_t * s, size_t count);

#endif

#if CONFIG_API_PRINTF

#include <stdarg.h>
int32_t tee_print(const int8_t *format, ...);

#endif /* CONFIG_API_PRINTF */

/* not support now */
#if CONFIG_SUPPORT_MULTI_THREAD

#define TEE_TIME_INFINITE   (-1)
typedef void * tee_sem_t;
typedef void * tee_mutex_t;
tee_sem_t tee_create_sem(uint32_t init_val);
void tee_destroy_sem(tee_sem_t sem);
tee_stat_t tee_wait_for_sem_timeout(tee_sem_t sem, int32_t msec);
void tee_release_sem(tee_mutex_t sem);
tee_mutex_t tee_create_mutex(void);
void tee_destroy_mutex(tee_mutex_t mutex);
tee_stat_t tee_wait_for_mutex_timeout(tee_mutex_t mutex, int32_t msec);
void tee_release_mutex(tee_mutex_t mutex);

#endif

/* not support now */
#if CONFIG_API_CB_MEM

void *tee_cb_malloc(size_t size);
void tee_cb_free(void *ptr);

#endif

/* not support now */
#if CONFIG_API_CB_FILE

typedef void * tee_file_iter_t;

/*
 * return >0:success -1:error
 */
int32_t tee_open(const int8_t *name, int32_t flags);
/*
 * return 0:success -1:error
 */
int32_t tee_close(int32_t fd);
int32_t tee_delete(const int8_t *name);
int32_t tee_read(int32_t fd, void *buf, size_t size);
int32_t tee_write(int32_t fd, const void *buf, size_t size);
/*
 * return 0:success -1:error
 */
int32_t tee_ioctl(int32_t fd, ulong_t request, ...);
int32_t tee_seek(int32_t fd, off_t offset, int32_t whence);
int32_t tee_truncate(int32_t fd, off_t length); 

tee_file_iter_t tee_create_file_iter(void);
void tee_destroy_file_iter(tee_file_iter_t iter);
int32_t tee_iter_file(tee_file_iter_t iter, int8_t *name);

#endif

typedef void (*int_handler)(void);
void register_int_handler(uint32_t vector, int_handler handler);
void unregister_int_handler(uint32_t vector);

void uart_print_uint8(unsigned char data);
void uart_print_uint32(unsigned int data);
void uart_print_string(const char *data);

#if CONFIG_API_TIME

void tee_udelay(uint32_t us);

#endif

#endif /* _TEE_TOS_H_ */
