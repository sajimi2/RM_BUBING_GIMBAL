/* GCC/newlib 适配：C 库堆与 FreeRTOS 堆分开，printf 沿用 USART6 输出。 */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vofa_debug.h"

extern char __HeapBase, __HeapLimit;

void *_sbrk(ptrdiff_t increment)
{
    static uintptr_t next;
    if (next == 0) next = (uintptr_t)&__HeapBase;
    /* 限制在链接器预留的 8 KB 内，分配失败时不能侵入主栈。 */
    if ((increment >= 0 && (uintptr_t)increment > (uintptr_t)&__HeapLimit - next) ||
        (increment < 0 && (uintptr_t)(-(increment + 1)) + 1 > next - (uintptr_t)&__HeapBase))
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    void *previous = (void *)next;
    next += increment;
    return previous;
}

/* newlib malloc 可能被多个任务调用；挂起调度不阻断外设中断。禁止 ISR 调用 malloc。 */
struct _reent;
void __malloc_lock(struct _reent *reent)
{
    (void)reent;
    vTaskSuspendAll();
}
void __malloc_unlock(struct _reent *reent)
{
    (void)reent;
    (void)xTaskResumeAll();
}

int _write(int file, char *data, int length)
{
    if (file != 1 && file != 2) { errno = EBADF; return -1; }
    if (length < 0) { errno = EINVAL; return -1; }
    /* USART6 被 VOFA 独占时，拒绝混入 printf 文本；不等待正在运行的 DMA。 */
    if (VOFA_UART_INDEX == 6) { errno = EBUSY; return -1; }
    if (huart6.gState == HAL_UART_STATE_RESET) { errno = EIO; return -1; }
    int sent = 0;
    while (sent < length)
    {
        uint16_t count = (length - sent > UINT16_MAX) ? UINT16_MAX : (uint16_t)(length - sent);
        if (HAL_UART_Transmit(&huart6, (uint8_t *)data + sent, count, HAL_MAX_DELAY) != HAL_OK)
        {
            errno = EIO;
            return sent ? sent : -1;
        }
        sent += count;
    }
    return sent;
}

int _read(int file, char *data, int length)
{ (void)file; (void)data; (void)length; errno = ENOSYS; return -1; }
int _close(int file) { (void)file; errno = EBADF; return -1; }
int _lseek(int file, int offset, int whence)
{ (void)file; (void)offset; (void)whence; errno = ESPIPE; return -1; }
int _fstat(int file, struct stat *st)
{
    if (file < 0 || file > 2) { errno = EBADF; return -1; }
    st->st_mode = S_IFCHR;
    return 0;
}
int _isatty(int file) { return file >= 0 && file <= 2; }
int _getpid(void) { return 1; }
int _kill(int pid, int signal)
{ (void)pid; (void)signal; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; __disable_irq(); for (;;) {} }
void _init(void) {}
void _fini(void) {}
