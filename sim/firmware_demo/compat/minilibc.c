/*
 * minilibc.c - the few C library pieces this demo needs, so it also builds
 * with a bare clang/zig toolchain that has no newlib. A CubeIDE build uses
 * newlib instead and does not need this file.
 */
#include <stddef.h>

void __libc_init_array(void) {}

size_t strlen(const char *s) { const char *p = s; while (*p) p++; return (size_t)(p - s); }

void *memset(void *d, int c, size_t n) { unsigned char *p = d; while (n--) *p++ = (unsigned char)c; return d; }

void *memcpy(void *d, const void *s, size_t n)
{
    unsigned char *a = d; const unsigned char *b = s;
    while (n--) *a++ = *b++;
    return d;
}
