/** @file
  Include file for SoCal Base type definition

  Copyright (c) 2015, Altera Corporation. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or other
  materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.

**/

#ifndef __SOCAL_BASE_H__
#define __SOCAL_BASE_H__

#include <console_printf.h>
 
#define IN
#define OUT
#define OPTIONAL
#define CONST     const
#define STATIC    static
#define VOID      void

#define EFIAPI

#define ProgressPrint  INFO
#define InfoPrint      printf
#define MmioHexDumpEx
#define CopyMem  memcpy
#define SetMem  memset
#define AsciiStrCmp strcmp
#define MmioWrite32  mmio_write_32
#define MmioRead32  mmio_read_32
#define MmioOr32  mmio_setbits_32
#define MmioAnd32(Address, Mask)  mmio_clrbits_32(Address, ~Mask)
#define MmioAndThenOr32(address, clear, set) mmio_clrsetbits_32(address, ~clear, set)
#define SerialPortPrint INFO
#define MicroSecondDelay udelay
#define NanoSecondDelay(a)  udelay(1000*a)
#define ASSERT assert
#define BASE_CR(Record, TYPE, Field)  ((TYPE *) ((CHAR8 *) (Record) - (CHAR8 *) &(((TYPE *) 0)->Field)))
#define CR(Record, TYPE, Field, TestSignature)   BASE_CR (Record, TYPE, Field)
#define ASSERT_PLATFORM_INIT assert

typedef signed char           INT8;
///
/// Logical Boolean.  1-byte value containing 0 for FALSE or a 1 for TRUE.  Other
/// values are undefined.
///
typedef unsigned char         BOOLEAN;
///
/// 1-byte unsigned value.
///
typedef unsigned char         UINT8;
///
/// 1-byte Character.
///
typedef char                  CHAR8;
///
/// 2-byte signed value.
///
typedef short                 INT16;
///
/// 2-byte unsigned value.
///
typedef unsigned short        UINT16;
///
/// 2-byte Character.  Unless otherwise specified all strings are stored in the
/// UTF-16 encoding format as defined by Unicode 2.1 and ISO/IEC 10646 standards.
///
typedef unsigned short        CHAR16;
///
/// 4-byte signed value.
///
typedef int                   INT32;
///
/// 4-byte unsigned value.
///
typedef unsigned int          UINT32;
///
/// 8-byte signed value.
///
typedef long long               INT64;
///
/// 8-byte unsigned value.
///
typedef unsigned long long       UINT64;

///
/// Signed value of native width.  (4 bytes on supported 32-bit processor instructions,
/// 8 bytes on supported 64-bit processor instructions)
/// "long" type scales to the processor native size with EBC compiler
///
typedef long                  INTN;
///
/// The unsigned value of native width.  (4 bytes on supported 32-bit processor instructions;
/// 8 bytes on supported 64-bit processor instructions)
/// "long" type scales to the processor native size with the EBC compiler.
///
typedef unsigned long         UINTN;

typedef UINT64                    EFI_LBA;
typedef UINT64         EFI_PHYSICAL_ADDRESS;
#define TRUE  ((BOOLEAN)(1==1))

#define FALSE ((BOOLEAN)(0==1))
#define MAX_BIT     (1ULL << (sizeof (INTN) * 8 - 1)) 
typedef UINTN RETURN_STATUS;


/**
  Produces a RETURN_STATUS code with the highest bit set.

  @param  StatusCode    The status code value to convert into a warning code.
                        StatusCode must be in the range 0x00000000..0x7FFFFFFF.

  @return The value specified by StatusCode with the highest bit set.

**/
#define ENCODE_ERROR(StatusCode)     ((RETURN_STATUS)(MAX_BIT | (StatusCode)))

/**
  Produces a RETURN_STATUS code with the highest bit clear.

  @param  StatusCode    The status code value to convert into a warning code.
                        StatusCode must be in the range 0x00000000..0x7FFFFFFF.

  @return The value specified by StatusCode with the highest bit clear.

**/
#define ENCODE_WARNING(StatusCode)   ((RETURN_STATUS)(StatusCode))

/**
  Returns TRUE if a specified RETURN_STATUS code is an error code.

  This function returns TRUE if StatusCode has the high bit set.  Otherwise, FALSE is returned.

  @param  StatusCode    The status code value to evaluate.

  @retval TRUE          The high bit of StatusCode is set.
  @retval FALSE         The high bit of StatusCode is clear.

**/
#define RETURN_ERROR(StatusCode)     (((INTN)(RETURN_STATUS)(StatusCode)) < 0)

///
/// The operation completed successfully.
///
#define RETURN_SUCCESS               0

///
/// The image failed to load.
///
#define RETURN_LOAD_ERROR            ENCODE_ERROR (1)

///
/// The parameter was incorrect.
///
#define RETURN_INVALID_PARAMETER     ENCODE_ERROR (2)

///
/// The operation is not supported.
///
#define RETURN_UNSUPPORTED           ENCODE_ERROR (3)

///
/// The buffer was not the proper size for the request.
///
#define RETURN_BAD_BUFFER_SIZE       ENCODE_ERROR (4)

///
/// The buffer was not large enough to hold the requested data.
/// The required buffer size is returned in the appropriate
/// parameter when this error occurs.
///
#define RETURN_BUFFER_TOO_SMALL      ENCODE_ERROR (5)

///
/// There is no data pending upon return.
///
#define RETURN_NOT_READY             ENCODE_ERROR (6)

///
/// The physical device reported an error while attempting the
/// operation.
///
#define RETURN_DEVICE_ERROR          ENCODE_ERROR (7)

///
/// The device can not be written to.
///
#define RETURN_WRITE_PROTECTED       ENCODE_ERROR (8)

///
/// The resource has run out.
///
#define RETURN_OUT_OF_RESOURCES      ENCODE_ERROR (9)

///
/// An inconsistency was detected on the file system causing the
/// operation to fail.
///
#define RETURN_VOLUME_CORRUPTED      ENCODE_ERROR (10)

///
/// There is no more space on the file system.
///
#define RETURN_VOLUME_FULL           ENCODE_ERROR (11)

///
/// The device does not contain any medium to perform the
/// operation.
///
#define RETURN_NO_MEDIA              ENCODE_ERROR (12)

///
/// The medium in the device has changed since the last
/// access.
///
#define RETURN_MEDIA_CHANGED         ENCODE_ERROR (13)

///
/// The item was not found.
///
#define RETURN_NOT_FOUND             ENCODE_ERROR (14)

///
/// Access was denied.
///
#define RETURN_ACCESS_DENIED         ENCODE_ERROR (15)

///
/// The server was not found or did not respond to the request.
///
#define RETURN_NO_RESPONSE           ENCODE_ERROR (16)

///
/// A mapping to the device does not exist.
///
#define RETURN_NO_MAPPING            ENCODE_ERROR (17)

///
/// A timeout time expired.
///
#define RETURN_TIMEOUT               ENCODE_ERROR (18)

///
/// The protocol has not been started.
///
#define RETURN_NOT_STARTED           ENCODE_ERROR (19)

///
/// The protocol has already been started.
///
#define RETURN_ALREADY_STARTED       ENCODE_ERROR (20)

///
/// The operation was aborted.
///
#define RETURN_ABORTED               ENCODE_ERROR (21)

///
/// An ICMP error occurred during the network operation.
///
#define RETURN_ICMP_ERROR            ENCODE_ERROR (22)

///
/// A TFTP error occurred during the network operation.
///
#define RETURN_TFTP_ERROR            ENCODE_ERROR (23)

///
/// A protocol error occurred during the network operation.
///
#define RETURN_PROTOCOL_ERROR        ENCODE_ERROR (24)

///
/// A function encountered an internal version that was
/// incompatible with a version requested by the caller.
///
#define RETURN_INCOMPATIBLE_VERSION  ENCODE_ERROR (25)

///
/// The function was not performed due to a security violation.
///
#define RETURN_SECURITY_VIOLATION    ENCODE_ERROR (26)

///
/// A CRC error was detected.
///
#define RETURN_CRC_ERROR             ENCODE_ERROR (27)

///
/// The beginning or end of media was reached.
///
#define RETURN_END_OF_MEDIA          ENCODE_ERROR (28)

///
/// The end of the file was reached.
///
#define RETURN_END_OF_FILE           ENCODE_ERROR (31)

///
/// The language specified was invalid.
///
#define RETURN_INVALID_LANGUAGE      ENCODE_ERROR (32)

///
/// The security status of the data is unknown or compromised
/// and the data must be updated or replaced to restore a valid
/// security status.
///
#define RETURN_COMPROMISED_DATA      ENCODE_ERROR (33)

///
/// The string contained one or more characters that
/// the device could not render and were skipped.
///
#define RETURN_WARN_UNKNOWN_GLYPH    ENCODE_WARNING (1)

///
/// The handle was closed, but the file was not deleted.
///
#define RETURN_WARN_DELETE_FAILURE   ENCODE_WARNING (2)

///
/// The handle was closed, but the data to the file was not
/// flushed properly.
///
#define RETURN_WARN_WRITE_FAILURE    ENCODE_WARNING (3)

///
/// The resulting buffer was too small, and the data was
/// truncated to the buffer size.
///
#define RETURN_WARN_BUFFER_TOO_SMALL ENCODE_WARNING (4)

///
/// The data has not been updated within the timeframe set by
/// local policy for this type of data.
///
#define RETURN_WARN_STALE_DATA       ENCODE_WARNING (5)
typedef RETURN_STATUS             EFI_STATUS;

#define EFI_SUCCESS               RETURN_SUCCESS              
#define EFI_LOAD_ERROR            RETURN_LOAD_ERROR           
#define EFI_INVALID_PARAMETER     RETURN_INVALID_PARAMETER    
#define EFI_UNSUPPORTED           RETURN_UNSUPPORTED          
#define EFI_BAD_BUFFER_SIZE       RETURN_BAD_BUFFER_SIZE      
#define EFI_BUFFER_TOO_SMALL      RETURN_BUFFER_TOO_SMALL     
#define EFI_NOT_READY             RETURN_NOT_READY            
#define EFI_DEVICE_ERROR          RETURN_DEVICE_ERROR         
#define EFI_WRITE_PROTECTED       RETURN_WRITE_PROTECTED      
#define EFI_OUT_OF_RESOURCES      RETURN_OUT_OF_RESOURCES     
#define EFI_VOLUME_CORRUPTED      RETURN_VOLUME_CORRUPTED     
#define EFI_VOLUME_FULL           RETURN_VOLUME_FULL          
#define EFI_NO_MEDIA              RETURN_NO_MEDIA             
#define EFI_MEDIA_CHANGED         RETURN_MEDIA_CHANGED        
#define EFI_NOT_FOUND             RETURN_NOT_FOUND            
#define EFI_ACCESS_DENIED         RETURN_ACCESS_DENIED        
#define EFI_NO_RESPONSE           RETURN_NO_RESPONSE          
#define EFI_NO_MAPPING            RETURN_NO_MAPPING           
#define EFI_TIMEOUT               RETURN_TIMEOUT              
#define EFI_NOT_STARTED           RETURN_NOT_STARTED          
#define EFI_ALREADY_STARTED       RETURN_ALREADY_STARTED      
#define EFI_ABORTED               RETURN_ABORTED              
#define EFI_ICMP_ERROR            RETURN_ICMP_ERROR           
#define EFI_TFTP_ERROR            RETURN_TFTP_ERROR           
#define EFI_PROTOCOL_ERROR        RETURN_PROTOCOL_ERROR       
#define EFI_INCOMPATIBLE_VERSION  RETURN_INCOMPATIBLE_VERSION 
#define EFI_SECURITY_VIOLATION    RETURN_SECURITY_VIOLATION   
#define EFI_CRC_ERROR             RETURN_CRC_ERROR   
#define EFI_END_OF_MEDIA          RETURN_END_OF_MEDIA
#define EFI_END_OF_FILE           RETURN_END_OF_FILE
#define EFI_INVALID_LANGUAGE      RETURN_INVALID_LANGUAGE
#define EFI_COMPROMISED_DATA      RETURN_COMPROMISED_DATA
#define EFI_HTTP_ERROR            RETURN_HTTP_ERROR

#define EFI_WARN_UNKNOWN_GLYPH    RETURN_WARN_UNKNOWN_GLYPH   
#define EFI_WARN_DELETE_FAILURE   RETURN_WARN_DELETE_FAILURE  
#define EFI_WARN_WRITE_FAILURE    RETURN_WARN_WRITE_FAILURE   
#define EFI_WARN_BUFFER_TOO_SMALL RETURN_WARN_BUFFER_TOO_SMALL
#define EFI_WARN_STALE_DATA       RETURN_WARN_STALE_DATA
#define EFI_WARN_FILE_SYSTEM      RETURN_WARN_FILE_SYSTEM

#define EFIERR(_a)                ENCODE_ERROR(_a)

#define EFI_ERROR(A)              RETURN_ERROR(A)

#define EFI_NETWORK_UNREACHABLE   EFIERR(100)
#define EFI_HOST_UNREACHABLE      EFIERR(101) 
#define EFI_PROTOCOL_UNREACHABLE  EFIERR(102)
#define EFI_PORT_UNREACHABLE      EFIERR(103)

#define EFI_CONNECTION_FIN        EFIERR(104)
#define EFI_CONNECTION_RESET      EFIERR(105)
#define EFI_CONNECTION_REFUSED    EFIERR(106)


#define SIGNATURE_16(A, B)        ((A) | (B << 8))

#define SIGNATURE_32(A, B, C, D)  (SIGNATURE_16 (A, B) | (SIGNATURE_16 (C, D) << 16))

#define SIGNATURE_64(A, B, C, D, E, F, G, H) \
    (SIGNATURE_32 (A, B, C, D) | ((UINT64) (SIGNATURE_32 (E, F, G, H)) << 32))

#define  BIT0     0x00000001
#define  BIT1     0x00000002
#define  BIT2     0x00000004
#define  BIT3     0x00000008
#define  BIT4     0x00000010
#define  BIT5     0x00000020
#define  BIT6     0x00000040
#define  BIT7     0x00000080
#define  BIT8     0x00000100
#define  BIT9     0x00000200
#define  BIT10    0x00000400
#define  BIT11    0x00000800
#define  BIT12    0x00001000
#define  BIT13    0x00002000
#define  BIT14    0x00004000
#define  BIT15    0x00008000
#define  BIT16    0x00010000
#define  BIT17    0x00020000
#define  BIT18    0x00040000
#define  BIT19    0x00080000
#define  BIT20    0x00100000
#define  BIT21    0x00200000
#define  BIT22    0x00400000
#define  BIT23    0x00800000
#define  BIT24    0x01000000
#define  BIT25    0x02000000
#define  BIT26    0x04000000
#define  BIT27    0x08000000
#define  BIT28    0x10000000
#define  BIT29    0x20000000
#define  BIT30    0x40000000
#define  BIT31    0x80000000
#define  BIT32    0x0000000100000000ULL
#define  BIT33    0x0000000200000000ULL
#define  BIT34    0x0000000400000000ULL
#define  BIT35    0x0000000800000000ULL
#define  BIT36    0x0000001000000000ULL
#define  BIT37    0x0000002000000000ULL
#define  BIT38    0x0000004000000000ULL
#define  BIT39    0x0000008000000000ULL
#define  BIT40    0x0000010000000000ULL
#define  BIT41    0x0000020000000000ULL
#define  BIT42    0x0000040000000000ULL
#define  BIT43    0x0000080000000000ULL
#define  BIT44    0x0000100000000000ULL
#define  BIT45    0x0000200000000000ULL
#define  BIT46    0x0000400000000000ULL
#define  BIT47    0x0000800000000000ULL
#define  BIT48    0x0001000000000000ULL
#define  BIT49    0x0002000000000000ULL
#define  BIT50    0x0004000000000000ULL
#define  BIT51    0x0008000000000000ULL
#define  BIT52    0x0010000000000000ULL
#define  BIT53    0x0020000000000000ULL
#define  BIT54    0x0040000000000000ULL
#define  BIT55    0x0080000000000000ULL
#define  BIT56    0x0100000000000000ULL
#define  BIT57    0x0200000000000000ULL
#define  BIT58    0x0400000000000000ULL
#define  BIT59    0x0800000000000000ULL
#define  BIT60    0x1000000000000000ULL
#define  BIT61    0x2000000000000000ULL
#define  BIT62    0x4000000000000000ULL
#define  BIT63    0x8000000000000000ULL

#define 	MAX_UINT32   ((UINT32)0xFFFFFFFF)
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))

#define ABS(a)     (((a) < 0) ? (-(a)) : (a))
#endif

