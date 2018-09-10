/**
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef _TEE_OSA_STR_H_
#define _TEE_OSA_STR_H_

extern int osa_strcmp(const char *s1, const char *s2);
extern int osa_strncmp(const char *s1, const char *s2, size_t n);
extern size_t osa_strlen(const char * s);
extern char *osa_strcpy(char *dest, const char *src);
extern char *osa_strncpy(char *dest, const char *src, size_t count);
extern void *osa_memmove(void *dest, const void *src, size_t count);
extern void *osa_memcpy(void *dest, const void *src, size_t count);
extern void *osa_memset(void *s, int c, size_t n);
extern int osa_memcmp(const void *s1, const void *s2, size_t n);

static inline int32_t tee_osa_strcmp(
        const int8_t *str1, const int8_t *str2)
{
    return osa_strcmp((char const *)str1, (char const *)str2);
}

static inline int32_t tee_osa_strncmp(
        const int8_t *str1, const int8_t *str2, uint32_t size)
{
    return osa_strncmp((char const *)str1, (char const *)str2, size);
}

static inline uint32_t tee_osa_strlen(const int8_t *s)
{
    return osa_strlen((char const *)s);
}

static inline int8_t *tee_osa_strcpy(int8_t *dst, const int8_t *src)
{
    return (int8_t *)osa_strcpy((char *)dst, (char const *)src);
}

static inline int8_t *tee_osa_strncpy(int8_t *dst, const int8_t *src, uint32_t size)
{
    return (int8_t *)osa_strncpy((char *)dst, (char const *)src, size);
}

static inline void *tee_osa_memcpy(void *dst, const void *src, uint32_t n)
{
    return osa_memcpy(dst, src, n);
}

static inline void *tee_osa_memmove(void *dst, const void *src, uint32_t n)
{
    return osa_memmove(dst, src, n);
}

static inline void *tee_osa_memset(void *s, int32_t c, uint32_t n)
{
    return osa_memset(s, c, n);
}

static inline int32_t tee_osa_memcmp(const void *s1, const void *s2, uint32_t n)
{
    return osa_memcmp(s1, s2, n);
}

#if 0
static inline int32_t tee_osa_vsprintf(
        int8_t *str, const int8_t *fmt, va_list ap)
{
    //return vsprintf((char *)str, (const char *)fmt, ap);
    return 0;
}

static inline int32_t tee_osa_vsnprintf(
        int8_t *str, size_t size, const int8_t *fmt, va_list ap)
{
    //return vsnprintf((char *)str, size, (const char *)fmt, ap);
    return 0;
}
#endif

#endif /* _TEE_OSA_STR_H_ */


