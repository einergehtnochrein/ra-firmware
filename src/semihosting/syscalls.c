/**************************************************************************

	Translation layer from The GNU C Library system calls to TRACE32 terminal protocol

	other source files:
		t32term.h/t32term.c : provide the TRACE32 terminal protocol
	
	15/10/12		initial release
		
**************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <errno.h>
#include <unistd.h>
#include <string.h> 

#include "t32term.h"

#if SEMIHOSTING

#undef errno
extern int errno;

int _open (const char * path, int flags, int mode);
int _open (const char * path, int flags, int mode)
{
    (void) mode;

    int t32_mode = 0;
    int write = 0;
    // 0x0==READ, 0x2==READ/WRITE, 0x4,0x6=READ/WRITE/TRUNC, 0x8,0xC=READ/WRITE/APPEND
    // | 0x1 == BINARY
    switch (flags & (O_RDONLY|O_WRONLY|O_RDWR)) {
		case O_RDONLY: t32_mode |= T32_TERM_O_RDONLY; break;
		case O_WRONLY: t32_mode |= T32_TERM_O_WRONLY; write = 1; break;
		case O_RDWR: t32_mode |= T32_TERM_O_RDWR; write = 1; break;
	}
	if (write)
		switch (flags & (O_TRUNC|O_APPEND)) {
			case O_TRUNC: t32_mode |= T32_TERM_O_CREATE_TRUNC; break;
			case O_APPEND: t32_mode |= T32_TERM_O_CREATE_APPEND; break;
		}	
	//if (flags & O_BINARY)
		t32_mode |= T32_TERM_O_BINARY;
	return T32_Term_OpenFile(path, t32_mode);
}


int _close (int __fd);
int _close (int __fd)
{
	return T32_Term_CloseFile(__fd);
}


int _fstat (int __fd, struct stat *__sbuf);
int _fstat (int __fd, struct stat *__sbuf)
{
	if (__fd == STDOUT_FILENO || __fd == STDIN_FILENO) {
		memset (__sbuf, 0, sizeof (* __sbuf));
		__sbuf->st_mode = S_IFCHR;
	}
	memset (__sbuf, 0, sizeof (* __sbuf));
	__sbuf->st_mode = S_IFCHR | S_IFREG;
	return 0;
}



pid_t _getpid (void);
pid_t _getpid (void)
{
    return 1;
}



int _isatty (int __fildes);
int _isatty (int __fildes)
{
    if (__fildes == STDOUT_FILENO || __fildes == STDIN_FILENO) {
        return 1; /* IS a terminal */
    }
    return 0;
}



int _kill (pid_t __pid, int __signal);
int _kill (pid_t __pid, int __signal)
{
    (void) __pid;
    (void) __signal;

    errno = EINVAL;
    return -1;
}



_off_t _lseek (int __fd, _off_t __offset, int __whence);
_off_t _lseek (int __fd, _off_t __offset, int __whence)
{
	__offset &= ~(0x3 << 30);
	if (__whence == SEEK_END) {
		__offset |= (0x3 << 30);
	}
	else if (__whence == SEEK_SET) {
		__offset |= 0x0;
	}
	else {
		__offset |= (0x2 << 30);
	}
	return T32_Term_SeekFile(__fd, __offset);
}



int _read (int __fd, void *__buf, size_t __nbyte);
int _read (int __fd, void *__buf, size_t __nbyte)
{
	char* pCharBuffer = (char*)__buf;
	
	if (__fd == STDIN_FILENO) {
		for (size_t i=0; i<__nbyte; i++) {
			*pCharBuffer = (char)T32_Term_Getchar();
			if (*pCharBuffer=='\r' || *pCharBuffer=='\n' || *pCharBuffer==0x0) {
				*pCharBuffer = '\n';
				return (i+1);
			}
			pCharBuffer++;
		}
	}
	else {
		return T32_Term_ReadFile(__fd, pCharBuffer, __nbyte);
	}
	return 0;
}



caddr_t _sbrk (ptrdiff_t __incr);
caddr_t _sbrk (ptrdiff_t __incr)
{
	extern int __heap_start; /* Defined by the linker */
	static caddr_t* heap_end = NULL;
	caddr_t prev_heap_end;

	if (heap_end == NULL) {
		heap_end = (caddr_t*)&__heap_start;
	}
	prev_heap_end = (caddr_t)heap_end;
	heap_end += __incr;
	return prev_heap_end;
}



int _write (int __fd, const char *__buf, size_t __nbyte);
int _write (int __fd, const char *__buf, size_t __nbyte)
{
	char* pCharBuffer = (char*)__buf;
	if (__fd == STDOUT_FILENO) {
		T32_Term_PutBlock((const unsigned char *)__buf, __nbyte);
	}
	else {
		__nbyte = T32_Term_WriteFile(__fd, pCharBuffer, __nbyte);
	}
	return __nbyte;
}

#else
pid_t _getpid (void);
pid_t _getpid (void)
{
    return 1;
}

int _kill (pid_t __pid, int __signal);
int _kill (pid_t __pid, int __signal)
{
    (void) __pid;
    (void) __signal;

    errno = EINVAL;
    return -1;
}
#endif

