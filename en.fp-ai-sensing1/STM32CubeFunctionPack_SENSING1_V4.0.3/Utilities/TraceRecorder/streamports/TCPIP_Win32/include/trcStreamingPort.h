/*******************************************************************************
 * Trace Recorder Library for Tracealyzer v3.2.0
 * Percepio AB, www.percepio.com
 *
 * trcStreamingPort.h
 *
 * The interface definitions for trace streaming ("stream ports").
 * This "stream port" sets up the recorder to use TCP/IP as streaming channel.
 * The example is for Windows sockets (Winsock), for use with Windows ports.
 *
 * Terms of Use
 * This file is part of the trace recorder library (RECORDER), which is the 
 * intellectual property of Percepio AB (PERCEPIO) and provided under a
 * license as follows.
 * The RECORDER may be used free of charge for the purpose of recording data
 * intended for analysis in PERCEPIO products. It may not be used or modified
 * for other purposes without explicit permission from PERCEPIO.
 * You may distribute the RECORDER in its original source code form, assuming
 * this text (terms of use, disclaimer, copyright notice) is unchanged. You are
 * allowed to distribute the RECORDER with minor modifications intended for
 * configuration or porting of the RECORDER, e.g., to allow using it on a 
 * specific processor, processor family or with a specific communication
 * interface. Any such modifications should be documented directly below
 * this comment block.  
 *
 * Disclaimer
 * The RECORDER is being delivered to you AS IS and PERCEPIO makes no warranty
 * as to its use or performance. PERCEPIO does not and cannot warrant the 
 * performance or results you may obtain by using the RECORDER or documentation.
 * PERCEPIO make no warranties, express or implied, as to noninfringement of
 * third party rights, merchantability, or fitness for any particular purpose.
 * In no event will PERCEPIO, its technology partners, or distributors be liable
 * to you for any consequential, incidental or special damages, including any
 * lost profits or lost savings, even if a representative of PERCEPIO has been
 * advised of the possibility of such damages, or for any claim by any third
 * party. Some jurisdictions do not allow the exclusion or limitation of
 * incidental, consequential or special damages, or the exclusion of implied
 * warranties or limitations on how long an implied warranty may last, so the
 * above limitations may not apply to you.
 *
 * Tabs are used for indent in this file (1 tab = 4 spaces)
 *
 * Copyright Percepio AB, 2017.
 * www.percepio.com
 ******************************************************************************/

#ifndef TRC_STREAMING_PORT_H
#define TRC_STREAMING_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int32_t readFromSocket(void* data, uint32_t size, int32_t *ptrBytesRead);
int32_t writeToSocket(void* data, uint32_t size, int32_t *ptrBytesWritten);

/* This port supports both direct write and buffered mode ...*/
#define TRC_STREAM_PORT_USE_INTERNAL_BUFFER 0

#define TRC_STREAM_PORT_READ_DATA(_ptrData, _size, _ptrBytesRead) readFromSocket(_ptrData, _size, _ptrBytesRead)

#if (TRC_STREAM_PORT_USE_INTERNAL_BUFFER == 1)    
	#define TRC_STREAM_PORT_WRITE_DATA(_ptrData, _size, _ptrBytesWritten) writeToSocket(_ptrData, _size, _ptrBytesWritten)
#else
	/* In the direct mode, _ptrBytesWritten is not used, so it is assumed that "all or nothing" is written. */
	#define TRC_STREAM_PORT_WRITE_DATA(_ptrData, _size, UNUSED) writeToSocket(_ptrData, _size, NULL)
#endif

#ifdef __cplusplus
}
#endif

#endif /* TRC_STREAMING_PORT_H */
