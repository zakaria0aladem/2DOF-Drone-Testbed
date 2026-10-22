/* Minimal <string.h> for the zig/clang build (no newlib). CubeIDE uses newlib's. */
#ifndef COMPAT_STRING_H
#define COMPAT_STRING_H
#include <stddef.h>
size_t strlen(const char *s);
void *memset(void *d, int c, size_t n);
void *memcpy(void *d, const void *s, size_t n);
#endif
