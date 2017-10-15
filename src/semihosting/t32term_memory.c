/**************************************************************************

	TRACE32 Semihosting interface for methods BUFFER*,SINGLE* method

	origin file:
	  files/demo/etc/terminal/t32term/t32term_memory.c

	interface and possible defines in t32term.h
	
	15/10/12		initial release
		
**************************************************************************/

#include "t32term.h"

//exported functions and variables

#ifdef T32_TERM_BLOCKED
  extern int T32_Term_Tar2HostBufferSize;
  extern int T32_Term_Host2TarBufferSize;
# if defined(T32_TERM_MEMORY_BLOCKED_SIZE)
#  if (T32_TERM_MEMORY_BLOCKED_SIZE < 0x100)
#   undef T32_TERM_MEMORY_BLOCKED_SIZE
#   define T32_TERM_MEMORY_BLOCKED_SIZE 0x100
#  endif
#  if (T32_TERM_MEMORY_BLOCKED_SIZE > 0x3C00)
#   undef T32_TERM_MEMORY_BLOCKED_SIZE
#   define T32_TERM_MEMORY_BLOCKED_SIZE 0x3C00
#  endif
	static unsigned long Term_Memory_Tar2HostBuffer[(T32_TERM_MEMORY_BLOCKED_SIZE)/sizeof(long)];
	static unsigned long Term_Memory_Host2TarBuffer[(T32_TERM_MEMORY_BLOCKED_SIZE)/sizeof(long)];
	static unsigned char* T32_Term_Memory_Tar2HostBuffer = (unsigned char*) Term_Memory_Tar2HostBuffer;
	static unsigned char* T32_Term_Memory_Host2TarBuffer = (unsigned char*) Term_Memory_Host2TarBuffer;
	int T32_Term_Tar2HostBufferSize = T32_TERM_MEMORY_BLOCKED_SIZE;
	int T32_Term_Host2TarBufferSize = T32_TERM_MEMORY_BLOCKED_SIZE;
# else //not T32_TERM_MEMORY_BLOCKED_SIZE
	int T32_Term_Tar2HostBufferSize = 0;
	int T32_Term_Host2TarBufferSize = 0;
	static unsigned char* T32_Term_Memory_Tar2HostBuffer = 0;
	static unsigned char* T32_Term_Memory_Host2TarBuffer = 0;
	//need to call T32_Term_Memory_SetBuffer or set variables directly by practice
# endif //T32_TERM_MEMORY_BLOCKED_SIZE
#else //not T32_TERM_BLOCKED
	static unsigned char Term_Memory_Tar2HostBuffer[1];
	static unsigned char Term_Memory_Host2TarBuffer[1];
	static unsigned char* T32_Term_Memory_Tar2HostBuffer = (unsigned char*) Term_Memory_Tar2HostBuffer;
	static unsigned char* T32_Term_Memory_Host2TarBuffer = (unsigned char*) Term_Memory_Host2TarBuffer;
#endif

#ifdef T32_TERM_BLOCKED
void T32_Term_Memory_SetBuffer(unsigned char* Tar2HostBuffer, int Tar2HostBufferSize, unsigned char* Host2TarBuffer, int Host2TostBufferSize) {
	T32_Term_Memory_Tar2HostBuffer = Tar2HostBuffer;
	T32_Term_Tar2HostBufferSize = Tar2HostBufferSize;
	T32_Term_Memory_Host2TarBuffer = Host2TarBuffer;	
	T32_Term_Host2TarBufferSize = Host2TostBufferSize;
}
#else
void T32_Term_Memory_SetBuffer(unsigned char* Tar2HostBuffer, int Tar2HostBufferSize, unsigned char* Host2TarBuffer, int Host2TostBufferSize) {
	T32_Term_Memory_Tar2HostBuffer = Tar2HostBuffer;
	T32_Term_Memory_Host2TarBuffer = Host2TarBuffer;	
}
#endif

#ifdef T32_TERM_BLOCKED
void T32_Term_PutBlock(const unsigned char * data, int block_size) {
	volatile unsigned char * pout = (unsigned char *) T32_Term_Memory_Tar2HostBuffer;
	static int package_size = 0; /* used to collect multiple blocks of one package*/
	static int package_write_index = 0;

	if (!data) {
		/* preload package_size and start to collect block */
		package_size = block_size;
		package_write_index = 0;
		if (!data)
			return; /* expect data in the next calls */
	}
			
	while (block_size > 0) {
		int len, j;
		if (!package_size)
			package_size = block_size;
		if (package_write_index == 0)
			while (*pout) ;	/* wait for ready */
		len = block_size;
		if (len + package_write_index > (T32_Term_Tar2HostBufferSize-(int)sizeof(long)))
			len = (T32_Term_Tar2HostBufferSize-sizeof(long)) - package_write_index;
		for (j = 0 ; j < len; j++)
			pout[sizeof(long) + package_write_index + j] = data[j];
		package_write_index += len;
		if (package_write_index >= package_size || package_write_index >= (T32_Term_Tar2HostBufferSize-(int)sizeof(long))) {
			/* finish package */
			int ctrl_len = package_write_index;
			if (ctrl_len > 0xFC) {
				pout[3] = (ctrl_len >> 16) & 0xFF;
				pout[2] = (ctrl_len >> 8) & 0xFF;
				pout[1] = ctrl_len & 0xFF;				
				ctrl_len = 0xFF; /* escape for extended len, 0xFE, 0xFD are still not in use */
			}
			pout[0] = ctrl_len; /* signal host to retrieve package */
			package_size -= package_write_index;
			package_write_index = 0;
		}
		data += len;
		block_size -= len;
	}
}	
#else
void T32_Term_PutBlock(const unsigned char * data, int block_size) {
	volatile unsigned char * pout = (unsigned char *) T32_Term_Memory_Tar2HostBuffer;
	if (!data)
		return;
	while (block_size > 0) {
		while (*pout) ;	/* wait for ready */
		*pout = *data;
		data++;
		block_size--;
	}
}
#endif /* T32_TERM_BLOCKED */



#ifdef T32_TERM_BLOCKED
void T32_Term_GetBlock(unsigned char * data, int size) {
	volatile unsigned char * pin = (unsigned char *) T32_Term_Memory_Host2TarBuffer;
	static int bufsize = 0;
	static int bufindex;
	int len, j;

	while (size > 0) {
		if (!bufsize) {		
			int bufsize_ctrl = 0;
			while (!pin[0]) ;	/* wait for ready */
			bufsize = bufsize_ctrl = pin[0];
			if (bufsize_ctrl == 0xFF)
				bufsize = (int) pin[1] | (int) pin[2] << 8 | (int) pin[3] << 16;
			bufindex = 0;
		}
		
		len = bufsize - bufindex;
		if ( len > size)
			len = size;

		for ( j = 0 ; j < len ; j++)
			data[j] = pin[sizeof(long) + bufindex + j];

		bufindex += len;
		if (bufindex >= bufsize) {
			pin[0] = 0;	/* ready for next */
			bufsize = 0;
		}
		data += len;
		size -= len;
	}
}
#else
void T32_Term_GetBlock(unsigned char * data, int size) {
	volatile unsigned char * pin = (unsigned char *) T32_Term_Memory_Host2TarBuffer; 
	while (size > 0) {
		while (!*pin) ;	/* wait for ready */
		*data = *pin;
		*pin = 0;
		data++;
		size--;
	}
}	
#endif
