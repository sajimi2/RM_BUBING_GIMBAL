/* GCC/newlib 运行时适配：保留 8 KB C 堆边界，printf 沿用 USART6 输出。
 * FreeRTOS 堆独立管理；这里不占用或替换 pvPortMalloc 的内存。
 */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

extern unsigned char __heap_start__, __heap_end__;
extern int __io_putchar(int ch);

void *_sbrk(ptrdiff_t increment)
{
    static unsigned char *heap;
    if (heap == NULL) heap = &__heap_start__;
    const uintptr_t current = (uintptr_t)heap;
    const uintptr_t lower = (uintptr_t)&__heap_start__;
    const uintptr_t upper = (uintptr_t)&__heap_end__;
    if ((increment >= 0 && (uintptr_t)increment > upper - current) ||
        (increment < 0 && (uintptr_t)(-(increment + 1)) + 1U > current - lower))
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    void *previous = heap;
    heap += increment;
    return previous;
}

int _write(int file, char *data, int length)
{
    if (file != 1 && file != 2) { errno = EBADF; return -1; }
    for (int i = 0; i < length; ++i) __io_putchar((unsigned char)data[i]);
    return length;
}

int _read(int file, char *data, int length)
{
    (void)file; (void)data; (void)length;
    errno = ENOSYS;
    return -1;
}
int _close(int file) { (void)file; errno = EBADF; return -1; }
int _fstat(int file, struct stat *st) { (void)file; st->st_mode = S_IFCHR; return 0; }
int _isatty(int file) { return file == 0 || file == 1 || file == 2; }
int _lseek(int file, int offset, int whence)
{ (void)file; (void)offset; (void)whence; errno = ESPIPE; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int signal) { (void)pid; (void)signal; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; for (;;) {} }
