/**************************************************************************

	TRACE32 proprietary semihosting interface

	origin file:
	  files/demo/etc/terminal/t32term/t32term.h
	
	possible defines:
	  T32_TERM_BLOCKED         : Transfer larger data blocks by BUFFER* or DCC* method and allow TERM.GATE mode
	  T32_TERM_METHOD_MEMORY   : must be set when t32term_memory.c is used
	  T32_TERM_MEMORY_BLOCKED_SIZE :
	                           Size of the buffer used by terminal when T32_TERM_BLOCKED is definded should not be larger than 15K.
							   When T32_TERM_MEMORY_BLOCKED_SIZE is not set the function T32_Term_Memory_SetBuffer should be called
							   to define the communication buffers or T32_Term_*BufferSize, T32_Term_Memory_*Buffer must be set by the debugger
	  T32_TERM_NOCHECKSUM      : Disable the calculation of check sum for slow targets.
	  T32_TERM_WRITEQUEUE      : Disable acknownledge from host for T32_Term_WriteFile to speed up write sequences.
	  T32_TERM_TYPEAHEADBUFFER : Speed up single character reading
								  
	other sources files:
		t32term_memory.c   : provide access by (dual port) access memory methods
	                          BUFFERE,BUFFERC,BUFFERS (T32_TERM_BLOCKED) or
							  SINGLEE,SINGLEC,SINGLES
	
	15/10/12		initial release
		
**************************************************************************/

#ifdef __cplusplus
extern "C" { 
#endif

#include "lpclib.h"

extern void T32_Term_PutBlock(const unsigned char * data, int block_size);
extern void T32_Term_GetBlock(unsigned char * data, int size);
extern int T32_Term_Puts(const char * buffer);
extern int T32_Term_Getchar(void);
extern void T32_Term_Putchar(int ch);

#if defined(T32_TERM_BLOCKED)

#	define T32_TERM_O_OPEN_EXISTING 0x00
#	define T32_TERM_O_CREATE_TRUNC  0x18
#	define T32_TERM_O_CREATE_APPEND 0x08	
#	define T32_TERM_O_RDONLY        0x00
#	define T32_TERM_O_WRONLY        0x01
#	define T32_TERM_O_RDWR          0x02	
#	define T32_TERM_O_ASCII         0x00
#	define T32_TERM_O_BINARY        0x20
	extern int T32_Term_OpenFile(const char * fname, int flags);
#	define T32_TERM_OA_R 0
#	define T32_TERM_OA_W 4
#	define T32_TERM_OA_A 8
#	define T32_TERM_OA_B 1
#	define T32_TERM_OA_PLUS 2
	extern int T32_Term_OpenFileANSI(const char * fname, int mode);
	extern int T32_Term_CloseFile(int handle);
	extern int T32_Term_ReadFile(int handle, char * buffer, int len);
	extern int T32_Term_WriteFile(int handle, const char * buffer, int len);
	extern int T32_Term_SeekFile(int handle, long pos);
	extern int T32_Term_TellFile(int handle);
	extern int T32_Term_TellSizeFile(int handle);
	extern int T32_Term_GetTempName(char * name);
	extern int T32_Term_Remove(const char * fname);	
	extern int T32_Term_Rename(const char * fname, const char * fnamenew);
	extern int T32_Term_OsExecute(const char * fname, int mode);
	extern long T32_Term_GetTime(void);
	extern long T32_Term_GetClock(void);
	extern int T32_Term_GetCmdline(char * cmdline);
	extern int T32_Term_GetTempNameID(char * name, int id);
	extern int T32_Term_Getline(char * buffer, int len);
	
	extern void T32_Term_Memory_SetBuffer(unsigned char* Tar2HostBuffer, int Tar2HostBufferSize, unsigned char* Host2TarBuffer, int Host2TostBufferSize);
#endif
	
#ifdef __cplusplus
}
#endif
