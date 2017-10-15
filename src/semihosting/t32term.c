/**************************************************************************

	TRACE32 Semihosting implementation for methods DCC*,BUFFER*,SINGLE*

	origin file:
	  files/demo/etc/terminal/t32term/t32term.c

	
	interface and possible defines in t32term.h	
	
	15/10/12		initial release
		
**************************************************************************/

#include <string.h>

#include "t32term.h"

extern int T32_Term_Tar2HostBufferSize;
extern int T32_Term_Host2TarBufferSize;

//exported functions
extern int T32_Term_PutPacket(const unsigned char * data, int len);
extern int T32_Term_GetPacket(unsigned char * data);

#ifndef NULL
#define NULL 0
#endif

int T32_Term_Errno;

#ifdef T32_TERM_TYPEAHEADBUFFER
static unsigned char T32_Term_TypeAheadBuffer[32];
static int T32_Term_TypeAheadNumber = 0;
#endif

int T32_Term_Getchar(void) {
    unsigned char   imsg[4];
#ifdef T32_TERM_TYPEAHEADBUFFER
    if (T32_Term_TypeAheadNumber) {
		int             i, ch;
		ch = T32_Term_TypeAheadBuffer[0];
		T32_Term_TypeAheadNumber--;
		for (i = 0; i < T32_Term_TypeAheadNumber; i++)
			T32_Term_TypeAheadBuffer[i] = T32_Term_TypeAheadBuffer[i + 1];
		return ch;
    }
#endif
	T32_Term_GetBlock(imsg, 1);
	return imsg[0];
}

int T32_Term_Puts(const char * buffer)
{
	T32_Term_PutBlock((const unsigned char *) buffer, strlen(buffer));
	return 0; /* no error */
}

void T32_Term_Putchar(int ch)
{
    unsigned char   msg[4];

    msg[0] = ch;
    T32_Term_PutBlock(msg, 1);
}


#ifdef T32_TERM_BLOCKED

static int T32_Term_PutPacketEx(const unsigned char * data, int len, const unsigned char * payload, int payload_len) {
	unsigned char sum = 0x55;
	unsigned char header_start[5];
	unsigned char *header = header_start;
	unsigned char checksum[2];
	int total_len = len + payload_len;
	*(header++) = 2;
	if (total_len > 252) {
		header[0] = 0xff;
		header[1] = total_len & 0xff;
		header[2] = (total_len >> 8) & 0xff;
		header[3] = (total_len >> 16) & 0xff;
#if !defined(T32_TERM_NOCHECKSUM)
			sum = header[0] + header[1] + header[2] + header[3];
#endif			
		header += 4;
	} else {
		*(header++) = total_len;
#if !defined(T32_TERM_NOCHECKSUM)
			sum = total_len & 0xff;
#endif			
	}
	
#if !defined(T32_TERM_NOCHECKSUM)
	{
		int i;
		for (i = 0 ; i < len ; i++)
			sum += data[i];
		for (i = 0 ; i < payload_len ; i++)
			sum += payload[i];
	}
#endif	
	checksum[0] = sum;
	checksum[1] = 3;		
	T32_Term_PutBlock(NULL, total_len + (int) (header - header_start) + 2);
	T32_Term_PutBlock(header_start, (int) (header - header_start) );
	T32_Term_PutBlock(data, len);
	if (payload)
		T32_Term_PutBlock(payload, payload_len);
	T32_Term_PutBlock(checksum, 2);
	return 0; /* no error */
}

int T32_Term_PutPacket(const unsigned char * data, int len)
{
	return T32_Term_PutPacketEx(data, len, NULL, 0);
}

static int T32_Term_GetPacketEx(unsigned char * data, int data_len, unsigned char * payload)
{
	int len;
	int payload_len = 0;
	unsigned char sum;
	unsigned char header[5];
	unsigned char checksum[2];

#ifdef T32_TERM_TYPEAHEADBUFFER	
    while (1) {
		T32_Term_GetBlock(header, 1);
		if (header[0] == 2)
			break;
		if (T32_Term_TypeAheadNumber >= sizeof(T32_Term_TypeAheadBuffer))
			return -1;
		T32_Term_TypeAheadBuffer[T32_Term_TypeAheadNumber++] = header[0];
    }
	T32_Term_GetBlock( header + 1, 1);
#else
	T32_Term_GetBlock( header, 2 );
#endif	
	if (header[0] != 2)
		return -1;
	len = header[1];
	sum = len;
	if (len == 0xff) {
		T32_Term_GetBlock( header + 2, 3 );
		len = (int) header[2] + ((int) header[3] << 8) + ((int) header[4] << 16);
		sum += header[2] + header[3] + header[4];
	}
	
	if (!data_len)
		data_len = len;

	T32_Term_GetBlock(data, data_len);
	if (payload) {
		payload_len = len - data_len;
		T32_Term_GetBlock(payload, payload_len);
	}	
	T32_Term_GetBlock(checksum, 2);
	if (checksum[1] != 3)
		return -1;

#if !defined(T32_TERM_NOCHECKSUM)
	{
		int i;
		for ( i = 0 ; i < data_len ; i++)
			sum += data[i];
		for ( i = 0 ; i < payload_len ; i++)
			sum += payload[i];
		if (sum != checksum[0])
			return -1;
	}
#endif	

	T32_Term_Errno = data[0];

	if (payload)
		return payload_len;
	else
		return len;
}

int T32_Term_GetPacket(unsigned char * data)
{
	return T32_Term_GetPacketEx(data, 0, NULL);	
}

int T32_Term_OpenFileANSI(const char * fname, int mode) {
	unsigned char msg[256];

	msg[2] = 0x1;
	msg[3] = mode;
	msg[4] = 0;
	strcpy((char *)(msg+5),fname);
	T32_Term_PutPacket( (const unsigned char*) msg + 2, strlen(fname)+3 );

	if (T32_Term_GetPacket( msg ) == -1)
		return -1;
	if (msg[0])
		return -1;
	return msg[1];
}

int T32_Term_OpenFile(const char * fname, int flags)
{
	unsigned char msg[256];

	msg[2] = 0x29;
	msg[3] = flags;
	msg[4] = 0;
	strcpy((char *)(msg+5),fname);
	T32_Term_PutPacket( (const unsigned char*) msg + 2, strlen(fname)+3 );

	if (T32_Term_GetPacket( msg ) == -1)
		return -1;
	if (msg[0])
		return -1;
	return msg[1];
}


int T32_Term_CloseFile(int handle)
{
	unsigned char msg[256];

	msg[2] = 0x02;
	msg[3] = handle;
	T32_Term_PutPacket( msg+2, 2 );

	if (T32_Term_GetPacket( msg ) == -1)
		return -1;
	if (msg[0])
		return -1;
	return 0;
}


int T32_Term_ReadFile(int handle, char * buffer, int len)
{
	int blen, rlen;
	unsigned char msg[6];

	rlen = 0;
	while (len > 0) {
		blen = len;
		if (blen > ((T32_Term_Host2TarBufferSize)-16))
			blen = ((T32_Term_Host2TarBufferSize)-16);

		msg[0] = 0x03;
		msg[1] = handle;
		if (blen > 240) {
			msg[2] = 0xff;
			msg[3] = blen & 0xFF;
			msg[4] = (blen >> 8) & 0xFF;
			msg[5] = (blen >> 16) & 0xFF;
			T32_Term_PutPacket( msg, 6 );
		} else {
			msg[2] = blen;
			T32_Term_PutPacket( msg, 3 );
		}

		if ((blen = T32_Term_GetPacketEx( msg, 1, (unsigned char*) buffer)) == -1)
			return -1;
		if (msg[0])
			return -1;
		if (blen == 0)
			break;
		buffer += blen;
		len -= blen;
		rlen += blen;
	}
	
	return rlen;
}


int T32_Term_WriteFile(int handle, const char * buffer, int len) {
	int blen, wlen;
	unsigned char msg[5];
	wlen = 0;
	while (len > 0) {
		blen = len;
		if (blen > ((T32_Term_Tar2HostBufferSize)-16) )
			blen = ((T32_Term_Tar2HostBufferSize)-16);

#if !defined(T32_TERM_WRITEQUEUE)
		msg[0] = 0x04;
#else
		msg[0] = 0x39;
#endif		
		msg[1] = handle;
		T32_Term_PutPacketEx(msg, 2, (const unsigned char *) buffer, blen);

#if !defined(T32_TERM_WRITEQUEUE)
			if (T32_Term_GetPacket(msg) == -1)
				return -1;
			if (msg[0])
				return -1;
			blen = msg[1];
			if (blen == 0xff)
				blen = (int) msg[2] + ((int) msg[3] << 8) + ((int) msg[4] << 16);
			if (blen == 0)
				break;
#endif
		buffer += blen;
		len -= blen;
		wlen += blen;
	}
	return wlen;
}


int T32_Term_SeekFile(int handle, long pos)
{
	unsigned char msg[256];

	msg[2] = 0x05;
	msg[3] = handle;
	msg[4] = (unsigned char) (pos);
	msg[5] = (unsigned char) (pos>>8);
	msg[6] = (unsigned char) (pos>>16);
	msg[7] = (unsigned char) (pos>>24);

	T32_Term_PutPacket( msg+2, 6 );

	if (T32_Term_GetPacket( msg ) == -1)
		return -1;
	if (msg[0])
		return -1;

	return ((long)msg[1])|(((long)msg[2])<<8)|(((long)msg[3])<<16)|(((long)msg[4])<<24);
}


int T32_Term_TellFile(int handle)
{
	unsigned char msg[256];

	msg[2] = 0x06;
	msg[3] = handle;

	T32_Term_PutPacket( msg+2, 2 );

	if (T32_Term_GetPacket( msg ) == -1)
		return -1;
	if (msg[0])
		return -1;

	return ((long)msg[1])|(((long)msg[2])<<8)|(((long)msg[3])<<16)|(((long)msg[4])<<24);
}


int T32_Term_TellSizeFile(int handle)
{
	unsigned char msg[256];

	msg[2] = 0x07;
	msg[3] = handle;

	T32_Term_PutPacket( msg+2, 2 );

	if (T32_Term_GetPacket( msg ) == -1)
		return -1;
	if (msg[0])
		return -1;

	return ((long)msg[1])|(((long)msg[2])<<8)|(((long)msg[3])<<16)|(((long)msg[4])<<24);
}

int T32_Term_GetTempName(char * name)
{
	int i, blen;
	unsigned char msg[256];

	msg[2] = 0x08;
	
	T32_Term_PutPacket( msg+2, 1 );

	if ((blen = T32_Term_GetPacket( msg )) == -1)
		return -1;
	if (msg[0])
		return -1;

	blen--;
	
	for ( i = 0 ; i < blen ; i++ )
		name[i] = msg[i+1];
	name[i] = '\0';

	return 1;
}

int T32_Term_Remove(const char * fname)
{
    unsigned char   msg[256];

    msg[2] = 0x11;
    strcpy((char *) msg + 3, fname);
    T32_Term_PutPacket(msg + 2, strlen(fname) + 1);

    if (T32_Term_GetPacket(msg) == -1)
	return -1;
    if (msg[0])
	return -1;
    return 0;
}

int T32_Term_Rename(const char * fname, const char * fnamenew)
{
    int             len, lennew;
    unsigned char   msg[256];

    len = strlen(fname);
    lennew = strlen(fnamenew);

    msg[2] = 0x12;
    msg[3] = len;
    strcpy((char *) msg + 4, fname);
    strcpy((char *) msg + 4 + len, fnamenew);
    T32_Term_PutPacket(msg + 2, 2 + len + lennew);

    if (T32_Term_GetPacket(msg) == -1)
	return -1;
    if (msg[0])
	return -1;
    return 0;
}

int T32_Term_OsExecute(const char * fname, int mode)
{
    unsigned char   msg[256];

    msg[2] = 0x24;
    msg[3] = mode;
    strcpy((char *) msg + 4, fname);
    T32_Term_PutPacket(msg + 2, strlen(fname) + 2);

    if (T32_Term_GetPacket(msg) == -1)
	return -1;
    if (msg[0])
	return -1;
    return 0;
}

long T32_Term_GetTime(void)
{
    unsigned char   msg[256];

    msg[2] = 0x22;

    T32_Term_PutPacket(msg + 2, 1);

    if (T32_Term_GetPacket(msg) == -1)
	return -1;
    if (msg[0])
	return -1;

    return ((long) msg[1]) | (((long) msg[2]) << 8) | (((long) msg[3]) << 16) | (((long) msg[4]) << 24);
}


long T32_Term_GetClock(void)
{
    unsigned char   msg[256];

    msg[2] = 0x23;

    T32_Term_PutPacket(msg + 2, 1);

    if (T32_Term_GetPacket(msg) == -1)
	return -1;
    if (msg[0])
	return -1;

    return ((long) msg[1]) | (((long) msg[2]) << 8) | (((long) msg[3]) << 16) | (((long) msg[4]) << 24);
}

int T32_Term_GetCmdline(char * cmdline)
{
    int             i, blen;
    unsigned char   msg[256];

    msg[2] = 0x28;

    T32_Term_PutPacket(msg + 2, 1);

    if ((blen = T32_Term_GetPacket(msg)) == -1)
	return -1;
    if (msg[0])
	return -1;

    blen--;

    for (i = 0; i < blen; i++)
	cmdline[i] = msg[i + 1];
    cmdline[i] = '\0';

    return blen;
}

int T32_Term_GetTempNameID(char * name, int id)
{
    int             i, blen;
    unsigned char   msg[256];

    msg[2] = 0x26;
    msg[3] = id;

    T32_Term_PutPacket(msg + 2, 2);

    if ((blen = T32_Term_GetPacket(msg)) == -1)
	return -1;
    if (msg[0])
	return -1;

    blen--;

    for (i = 0; i < blen; i++)
	name[i] = msg[i + 1];
    name[i] = '\0';

    return 0;
}

int T32_Term_Getline(char * buffer, int len)
{
    int             i, blen;
    unsigned char   msg[256];

    if (len <= 0 || len >= 256)
	return -1;

    msg[2] = 0x27;
    msg[3] = len;
    T32_Term_PutPacket(msg + 2, 2);

    if ((blen = T32_Term_GetPacket(msg)) == -1)
	return -1;
    if (msg[0])
	return -1;

    blen--;

    for (i = 0; i < blen; i++)
	buffer[i] = msg[i + 1];

    return blen;
}

#endif

