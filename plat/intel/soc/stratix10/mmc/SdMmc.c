/** @file

  Copyright (c) 2016, Intel Corporation. All rights reserved.

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

//
// Include files
//
#include <assert.h>
#include <console.h>
#include <debug.h>
#include <mmio.h>
#include <string.h>
#include "Base.h"
#include "AlteraSdMmcMain.h"
#include "SdMmc.h"

//
// Functions
//

VOID
EFIAPI
InitSdMmc (
  VOID
  )
{
  EFI_STATUS          Status;
  InfoPrint ("Initializing SD/MMC controller\r\n");
  Status = AlteraSdMmcPeiInit ();
  ASSERT(!EFI_ERROR(Status));
  InfoPrint ("Done Initializing SD/MMC controller\r\n");
}


VOID
EFIAPI
HexDumpFile (
  IN  CHAR8*           TextFileFilename
  )
{
  FAT32_FILE           MyFile;
  EFI_STATUS           Status;

  InfoPrint("Open file by name\n");
  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);
  if (MyFile.Found == TRUE)
  {
    InfoPrint("File is found\n");
    while (MyFile.EndOfFile == FALSE)
    {
      Status = ReadFileNextData (&MyFile);
      if (EFI_ERROR(Status))
		break;
	  //SerialPortMmioHexDumpEx((UINTN)MyFile.FileData, MyFile.LastReadDataSize/4, MyFile.NextFilePos - MyFile.LastReadDataSize);
    }
  } else {
    SerialPortPrint ("File not found %s\r\n", TextFileFilename);
  }
}


EFI_STATUS
EFIAPI
LoadFileToMemory (
  IN  CHAR8*           TextFileFilename,
  IN  UINTN            DestinationMemoryBase,
  OUT UINT32*          pFileSize
  )
{
  EFI_STATUS           Status;
  FAT32_FILE           MyFile;
  UINT8*               Destination;
  UINT32                BytesReadCounter;
  UINT32               Percentage;

  Destination = (UINT8*) DestinationMemoryBase;
  BytesReadCounter = 0;
  Percentage = 0;

  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);
  if (MyFile.Found == TRUE)
  {
    ProgressPrint ("Copying %s from Flash to RAM at 0x%08x\r\n", TextFileFilename, (UINT32) DestinationMemoryBase);
    while (MyFile.EndOfFile == FALSE)
    {
      // Read a block of data
      Status = ReadFileNextData (&MyFile);
      if (EFI_ERROR(Status)) return EFI_DEVICE_ERROR;
      // Copy to memory
      CopyMem (Destination, MyFile.FileData, MyFile.LastReadDataSize);

      // Dump data
       //SerialPortMmioHexDumpEx((UINTN)Destination, MyFile.LastReadDataSize/4, MyFile.NextFilePos - MyFile.LastReadDataSize);

      // Point to new destination for next loop
      Destination += MyFile.LastReadDataSize;

      // Count progress
      BytesReadCounter += MyFile.LastReadDataSize;

      // Update once every 10%
      if (((BytesReadCounter * 100 / MyFile.FileSize) - Percentage) >= 10)
      {
        Percentage = (BytesReadCounter * 100 / MyFile.FileSize);
        InfoPrint ("\r%2d", Percentage);
      }
    }
    ProgressPrint ("\rDone.\r\n");
    // Return with filesize
    if (pFileSize != NULL)
      *pFileSize = MyFile.FileSize;
    return EFI_SUCCESS;
  } else {
    SerialPortPrint ("File not found %s\r\n", TextFileFilename);
    return EFI_NOT_FOUND;
  }
}

EFI_STATUS
EFIAPI
LoadBootImageFile (
  IN  CHAR8*           TextFileFilename,
  OUT UINTN           pLoadAddr
  )
{
  EFI_STATUS           Status;
  FAT32_FILE           MyFile;
  UINT8*               Destination;
  UINT32                BytesReadCounter;
  UINT32                Percentage;

  Destination = (UINT8*) pLoadAddr;
  BytesReadCounter = 0;
  Percentage = 0;

  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);

  if (MyFile.Found == TRUE)
  {
    while (MyFile.EndOfFile == FALSE)
    {
      // Read a block of data
      Status = ReadFileNextData (&MyFile);
      if (EFI_ERROR(Status)) return EFI_DEVICE_ERROR;

      // Copy to memory
      CopyMem (Destination, MyFile.FileData, MyFile.LastReadDataSize);
      // Point to new destination for next loop
      Destination += MyFile.LastReadDataSize;

      // Count progress
      BytesReadCounter += MyFile.LastReadDataSize;

      // Update once every 10%
      if (((BytesReadCounter * 100 / MyFile.FileSize) - Percentage) >= 10)
      {
        Percentage = (BytesReadCounter * 100 / MyFile.FileSize);
        InfoPrint ("\r%2d", Percentage);
      }
    }
    ProgressPrint ("\rDone.\r\n");
    return EFI_SUCCESS;
  } else {
    SerialPortPrint ("File not found %s\r\n", TextFileFilename);
    return EFI_NOT_FOUND;
  }
}


