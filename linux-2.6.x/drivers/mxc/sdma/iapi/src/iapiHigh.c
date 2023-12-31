/******************************************************************************
 *
 * Copyright 2005 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 * 
 ******************************************************************************
 *
 * File: iapiHigh.c
 *
 * $Id iapiHigh.c $
 *
 * Description:
 *  This library is written in C to guarantee functionality and integrity in
 * the usage of SDMA virtual DMA channels. This API (Application Programming
 * Interface)  allow SDMA channels' access in an OPEN, READ, WRITE, CLOSE
 * fashion.
 *  These are the HIGH level functions of the I.API.
 *
 *
 * /
 *
 * $Log iapiHigh.c $
 *
 *****************************************************************************/

/* ****************************************************************************
 * Include File Section
 *****************************************************************************/
#include <stdarg.h>
#include <string.h>

#include "epm.h"
#include "iapi.h"

/* ****************************************************************************
 * External Reference Section (for compatibility with already developed code)
 *****************************************************************************/


/* ****************************************************************************
 * Global Variable Section
 *****************************************************************************/

/* ****************************************************************************
 * Function Section
 *****************************************************************************/

/* ***************************************************************************/
/**Opens an SDMA channel to be used by the library.
 *
 * <b>Algorithm:</b>\n
 *
 * - Check if initialization is necessary.
 *   - Check that user initialized OS dependant functions.
 *   - Test validity of input parameters
 *   - Check whole channel control block data structure
 *   - Finish initializations (tables with default values)
 *   - Initialize channel 0 is dedicated to communications with SDMA
 * - Check channel control block definition
 *   - if the channel descriptor is not initialized, initialize it with
 * the default value
 * - If buffer descriptor already allocated, exit with iapi_errno filled
 * complete the lowest bits with the number of 'D' bits set
 * - Buffer Descriptors allocation
 * - Channel's configuration properties (mcu side only)
 * - read/write direction => enable/disable channel setting
 *
 * @param  *cd_p pointer to channel descriptor for the channnel to be opened. 
 *               Must be allocated.
 * @param  channelNumber channel to be opened
 *
 * @return
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : close failed, return negated value of iapi_errno
 */
int 
iapi_Open (channelDescriptor * cd_p, unsigned char channelNumber)
{
  channelControlBlock * ccb_p;
  channelControlBlock * local_ccb_p;
  channelDescriptor   * local_cd_p;
  bufferDescriptor    * bd_p;
  unsigned char index = 0;
  int result = IAPI_SUCCESS;
#ifdef MCU    
  volatile unsigned long * channelPriorityMatx;
#endif /* MCU */


  /*
   * 1. Check if initialization is necessary
   */
  if (cd_p == NULL){
    result = IAPI_ERR_CD_UNINITIALIZED | 
      IAPI_ERR_CH_AVAILABLE | channelNumber;
    iapi_errno = result;
    return -result;
  }
   
  /* Verify these functions every time*/
  if((iapi_GetChannel == NULL)||(iapi_ReleaseChannel == NULL))
  {
     result = IAPI_ERR_NO_OS_FN | channelNumber;
     iapi_errno = result;
     return -result;
  }
  
  /* Try to aquire channel*/
  if(iapi_GetChannel(channelNumber) != 0)
  {
     result = IAPI_ERR_CH_IN_USE | channelNumber;
     iapi_errno = result;
     return -result;
  }
  
  if (channelNumber == 0 && cd_p->ccb_ptr == NULL){
    /*  Verify that the user initialized all OS dependant functions required
     * by the library.
     */
    if((iapi_Malloc == NULL)||(iapi_Free == NULL)||(iapi_Virt2Phys == NULL)||
       (iapi_Phys2Virt == NULL)||(iapi_GotoSleep == NULL)||
       (iapi_WakeUp == NULL)||(iapi_InitSleep == NULL)||(iapi_memset == NULL)||
       (iapi_memcpy == NULL))
    {
      result = IAPI_ERR_NO_OS_FN | channelNumber;
      iapi_errno = result;
      iapi_ReleaseChannel(channelNumber);
      return -result;
    }
    /* Whole channel control block data structure */
    ccb_p = (channelControlBlock *)
      MALLOC(CH_NUM*sizeof(channelControlBlock));
    if (ccb_p == NULL){
      result = IAPI_ERR_CCB_ALLOC_FAILED |
               IAPI_ERR_CH_AVAILABLE | channelNumber;
      iapi_errno = result;
      iapi_ReleaseChannel(channelNumber);      
      return -result;
    }
    /* Zero-out the CCB structures array just allocated*/
    iapi_memset(ccb_p, 0x00, CH_NUM*sizeof(channelControlBlock));
    /* Save the address of the CCB structures array*/
    iapi_CCBHead = ccb_p;
 
    cd_p->ccb_ptr = (struct iapi_channelControlBlock *)ccb_p;
    ccb_p->channelDescriptor = cd_p;
#ifdef MCU
    /* finish initializations */
    iapi_InitChannelTables();
#endif /* MCU */
    /* Channel 0 is dedicated to communications with SDMA */
    cd_p->ownership = ((DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_EVT ) | 
                       (     OWN_CHANNEL << CH_OWNSHP_OFFSET_MCU ) |
                       (DONT_OWN_CHANNEL << CH_OWNSHP_OFFSET_DSP ));      
    cd_p->bufferDescNumber = 1;
  }
  
  /*
   * 2. Check channel control block
   */
  ccb_p = cd_p->ccb_ptr;
  if (ccb_p == NULL){
    result = IAPI_ERR_NO_CCB_DEFINED | IAPI_ERR_CH_AVAILABLE|channelNumber;
    iapi_errno = result;
    iapi_ReleaseChannel(channelNumber);      
    return -result;
  }

  /* Control block & Descriptor associated with the channel being worked on */
  local_ccb_p = &ccb_p[channelNumber];
  local_cd_p  =  ccb_p[channelNumber].channelDescriptor;

  /* If the channel is not initialized, initialize it with the default value */
  if (local_cd_p == NULL){
    result = iapi_AllocChannelDesc (&local_cd_p, channelNumber);
    if ( result!= IAPI_SUCCESS)
    {
       iapi_ReleaseChannel(channelNumber);      
       return result; //is allready negated from iapi_AllocChannelDesc
    }

    local_cd_p->ccb_ptr = (struct iapi_channelControlBlock *)local_ccb_p;
    local_ccb_p->channelDescriptor = local_cd_p;
  }
  
  /*
   * 3. If buffer descriptor already allocated, exit with iapi_errno filled
   */
  if ( local_ccb_p->baseBDptr != NULL ){
    bd_p = (bufferDescriptor *)iapi_Phys2Virt(local_ccb_p->baseBDptr);
    result = IAPI_ERR_BD_ALLOCATED;
    for (index=1 ; index< local_cd_p->bufferDescNumber ; index++){
      if ((bd_p->mode.status & BD_DONE) == BD_DONE){
        /* complete the lowest bits with the number of 'D' bits set */
        result++;
      }
      bd_p++;
    }
    iapi_errno = result;
    iapi_ReleaseChannel(channelNumber);      
    return -result;
  }

  /*
   * 4. Buffer Descriptors allocation
   */
  iapi_InitializeMemory(local_ccb_p);

#ifdef MCU    
  /*
   * 5. Channel's configuration properties (mcu side only)
   */
  iapi_ChannelConfig( channelNumber,
                      ( local_cd_p->ownership >> CH_OWNSHP_OFFSET_EVT ) & 1UL,
                      ( local_cd_p->ownership >> CH_OWNSHP_OFFSET_MCU ) & 1UL,
                      ( local_cd_p->ownership >> CH_OWNSHP_OFFSET_DSP ) & 1UL);
#endif /* MCU */
  
  /* Setting interrupt handling */
  iapi_ChangeCallbackISR(local_cd_p, local_cd_p->callbackISR_ptr);
  
  /* Call initialization fn for polling synch on this channel*/
  INIT_SLEEP(channelNumber);

  /* No user arg pointer yet*/
  userArgTable[cd_p->channelNumber]= NULL;
  
  /*
   * 6. read/write direction => enable/disable channel
   */
#ifdef MCU
  channelPriorityMatx = &SDMA_CHNPRI_0;
  channelPriorityMatx[channelNumber] = 1;
#endif /* MCU */

  local_ccb_p->status.openedInit = TRUE;
  iapi_ReleaseChannel(channelNumber);      
  return IAPI_SUCCESS;
}

/* ***************************************************************************/
/** Attempts to read nbyte from the data buffer descriptor associated with the
 * channel channelNumber, into the user's data buffer pointed to by buf.
 *
 * <b>Algorithm:</b>\n
 * - Check data structures are properly initialized:
 *   - Channel descriptor validity
 *   - Control block & Descriptor associated with the channel being worked on
 *   - Check initialization has been done for trusted channels
 *   - If transfer data size is used, check validity of combination transfer
 *     size/requested bytes
 * - Set the 'D' done bits on all buffer descriptors
 * - Starting of the channel
 * - Synchronization mechanism handling:
 *   - for callback: just exit function
 *   - for polling: call the synchronization function then read data from
 *     buffer until either nbyte parameter is reached or all buffer descriptors
 *     have been processed.
 *
 * <b>Notes:</b>\n
 *   1) Virtual DMA SDMA channels are unidirectional, an iapi_Read authorized
 *       on a channel means that we are expecting to receive from the SDMA. The
 *       meaning of an interrupt received from the SDMA is therefore that the
 *       data has been copied from the SDMA to the host's data buffers and is
 *       already passed on upper layers of the application.\n
 *
 * @param *cd_p chanenl descriptor for the channel to read from
 * @param *buf buffer to receive the data
 * @param nbyte number of bytes to read from channel
 *
 * @return 
 *       - number of bytes read
 *       - -iapi_errno : in case of failure return negated value of iapi_errno
 */
int
iapi_Read (channelDescriptor * cd_p, void * buf, unsigned short nbyte)
{
  int index = 0;
  int readBytes;
  int toRead;
  int result = IAPI_SUCCESS;
  unsigned int copyFinished;
  int bufsize;
  bufferDescriptor * bd_p;
  channelControlBlock * ccb_p;
  unsigned char * local_buf;
  unsigned char chNum;
  unsigned char div;

  iapi_errno = IAPI_ERR_NO_ERROR;

  /*
   * 1. Check data structures are properly initialized
   */
  /* Channel descriptor validity */
  if (cd_p == NULL){
    result = IAPI_ERR_CD_UNINITIALIZED;
    iapi_errno = result;
    return -result;
  }
  
  /* Channel control block validity */
  if (cd_p->ccb_ptr == NULL){
    result = IAPI_ERR_CCB_UNINITIALIZED;
    iapi_errno = result;
    return -result;
  }

  /* Control block & Descriptor associated with the channel being worked on */
  chNum = cd_p->channelNumber;
  ccb_p = cd_p->ccb_ptr;

  /* Try to aquire channel*/
  if(iapi_GetChannel(chNum) != 0)
  {
     result = IAPI_ERR_CH_IN_USE | chNum;
     iapi_errno = result;
     return -result;
  }

  /* Check if channel is already opened/initialized */
  if (ccb_p->status.openedInit == FALSE) {
    result = IAPI_ERR_CHANNEL_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
    iapi_errno = result;
    iapi_ReleaseChannel(chNum);
    return -result;
  }

  /* Buffer descriptor validity */
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
  if (bd_p == NULL ){
    result = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
    iapi_errno = result;
    iapi_ReleaseChannel(chNum);
    return -result;
  }


  /* Check initialization has been done for trusted channels */
  if (cd_p->trust == TRUE) {
    bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
       for(index=0 ; index < cd_p->bufferDescNumber ; index++){
        if ((bd_p->bufferAddr == NULL) || (bd_p->mode.count == 0)){
          result = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
          iapi_errno = result;
          iapi_ReleaseChannel(chNum);
          return -result;
        }
        bd_p++;
      }
  }
   
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
  /*If transfer data size is used, check that the required read length is  
   * divisible by transfer data size expressed in bytes
   */
   if(cd_p->useDataSize)
   {
      /*Check for divisibility only if data size different then 8bit*/
      if(cd_p->dataSize != TRANSFER_8BIT)
      {
         switch(cd_p->dataSize)
         {
            case TRANSFER_32BIT:
               div = 4;
               break;
            case TRANSFER_16BIT:
               div = 2;
               break;
            case TRANSFER_24BIT:
               div = 3;
               break;
            /*we should not get to default*/
            default:
               result = IAPI_ERR_INVALID_PARAMETER | chNum;
               iapi_errno = result;
               iapi_ReleaseChannel(chNum);
               return -result;
         }
         /*check the total number of bytes requested*/
         if((nbyte % div) != 0)
         {
            result = IAPI_ERR_INVALID_PARAMETER | chNum;
            iapi_errno = result;
            iapi_ReleaseChannel(chNum);
            return -result;
         }
         /*now check the length of every BD*/
         for(index=0 ; index < cd_p->bufferDescNumber ; index++)
         {
            if((bd_p->mode.count % div) != 0)
            {
               result = IAPI_ERR_INVALID_PARAMETER | chNum;
               iapi_errno = result;
               iapi_ReleaseChannel(chNum);
               return -result;
            }
			bd_p++;
         }
      }
   }

  /*
   * 2. Set the 'D' done bits on all buffer descriptors
   */
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
  for (index=0 ; index < cd_p->bufferDescNumber ; index++) {
    bd_p->mode.status |= BD_DONE;
    bd_p++;
  }

  /*
   * 3. Starting of the channel
   */
  iapi_lowStartChannel (chNum);
  ccb_p->status.execute = TRUE;
  readBytes = 0;
  
  /*
   * 4. Synchronization mechanism handling
   */
  if( cd_p->callbackSynch == DEFAULT_POLL){
    iapi_SynchChannel(chNum);
    
    bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
    toRead = nbyte;
    copyFinished = FALSE;
    local_buf = (unsigned char *)buf;
    
   /*
    * Check the 'RROR' bit on all buffer descriptors, set error number
    *    and return IAPI_FAILURE if set.
    */
   for (index=0 ; index < cd_p->bufferDescNumber ; index++) 
   {
      if(bd_p->mode.status & BD_RROR)
      {
          result = IAPI_ERR_RROR_BIT_READ | chNum;
          iapi_errno = result;
          iapi_ReleaseChannel(chNum);
          return -result;
      }
	  bd_p++;
   }
   
    
  /*
   * 5. Read loop
   */
   
    bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);    
    while (!copyFinished)
    {
      if (!(bd_p->mode.status & BD_DONE))
      {
        if (cd_p->trust == FALSE) {
          bufsize = cd_p->bufferSize;
        } else {
          bufsize = bd_p->mode.count;
        }
        /*if L bit is set, read only "count" bytes and exit the loop*/
        if(bd_p->mode.status & BD_LAST)
         {
            bufsize = bd_p->mode.count;
            copyFinished = TRUE;
         }
        if (toRead > bufsize)
        {
          if (cd_p->trust == FALSE)
          {
            iapi_memcpy(local_buf, iapi_Phys2Virt(bd_p->bufferAddr), bufsize);
            local_buf += bufsize; 
          }
          readBytes += bufsize;
          toRead -= bufsize;
          /*advance bd_p only if bit L is not set. The loop will exit anyway.*/
          if(!(bd_p->mode.status & BD_LAST))
          {
              if (bd_p->mode.status & BD_WRAP)
              {
                bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
              } 
              else if(((bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr) + 
                       (cd_p->bufferDescNumber - 1)*sizeof(bufferDescriptor)) != bd_p)
                     {
                        bd_p++;
                     }
                     else
                     {
                        /* finished here : end of buffer descriptors */
                        copyFinished = TRUE;
                     }
          }
        }
        else
        {
          if (cd_p->trust == FALSE)
          {
            iapi_memcpy(local_buf, iapi_Phys2Virt(bd_p->bufferAddr), toRead);
            local_buf += toRead;
          }
          readBytes += toRead;
          toRead = 0;
          /* finished successfully : readBytes = nbytes */
          copyFinished = TRUE;
        }
      }
      else
      {
        /* finished here : buffer not already done*/
        copyFinished = TRUE;
      }
    }
    iapi_ReleaseChannel(chNum);
  }
  
  /* 
   *If synchronization type is callback, the user of I.API must 
   *release the channel 
   */
  return readBytes;
}

/* ***************************************************************************/
/*Attempts to write nbyte from the buffer pointed to by buf to the channel
 * data buffers associated with the opened channel number channelNumber
 *
 * <b>Algorithm:</b>\n
 *
 * - Check data structures are properly initialized:
 *   - Channel descriptor validity
 *   - Channel control block validity
 *   - Buffer descriptor validity
 *   - If transfer data size is used, check validity of combination transfer
 *     size/requested bytes
 * - Write loop\n
 *  Write occurs in the buffer acceded form buffer descriptor and continues
 *  to the "next" buffer which can be:\n
 *    -# the last BD of the ring so re-start from beginning\n
 *    -# the last BD  of the BD array but no ring so finish\n
 *    -# (general case) the next BD in the BD array\n
 *  And copy continues until data fit in the current buffer or the nbyte
 *  parameter is reached.
 * - Starting of the channel
 *
 * <b>Notes:</b>\n
 *   1) Virtual DMA SDMA channels are unidirectionnal, an iapi_Write authorized
 *      on a channel means that we are expecting to send to the SDMA. The 
 *      meaning of an interrupt received from the SDMA is therfore that the 
 *      data has been delivered to the SDMA.
 *
 * @param *cd_p chanenl descriptor for the channel to write to
 * @param *buf buffer with data to be written
 * @param nbyte number of bytes to write to channel
 *
 * @return 
 *       - number of bytes written
 *       - -iapi_errno if failure
 */
int
iapi_Write (channelDescriptor * cd_p, void * buf, unsigned short nbyte)
{
  unsigned int writtenBytes = 0;
  unsigned int toWrite;
  int result = IAPI_SUCCESS;
  unsigned int copyFinished;
  unsigned int buffsize;
  unsigned int index = 0;
  bufferDescriptor * bd_p;
  channelControlBlock * ccb_p;
  unsigned char * local_buf;
  unsigned char chNum; 
  unsigned char div;

  iapi_errno = IAPI_ERR_NO_ERROR;

  /*
   * 1. Check data structures are properly initialized
   */
  /* Channel descriptor validity */
  if (cd_p == NULL){
    result = IAPI_ERR_CD_UNINITIALIZED;
    iapi_errno = result;
    return -result;
  }
  
  /* Channel control block validity */
  if (cd_p->ccb_ptr == NULL){
    result = IAPI_ERR_CCB_UNINITIALIZED;
    iapi_errno = result;
    return -result;
  }

  /* Control block & Descriptpor associated with the channel being worked on */
  chNum = cd_p->channelNumber;
  ccb_p = cd_p->ccb_ptr;

  /* Try to aquire channel*/
  if(iapi_GetChannel(chNum) != 0)
  {
     result = IAPI_ERR_CH_IN_USE | chNum;
     iapi_errno = result;
     return -result;
  }

  /* Buffer descriptor validity */
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
  if (bd_p == NULL ){
    result = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
    iapi_errno = result;
    iapi_ReleaseChannel(chNum);
    return -result;
  }

  /* Check initialization has been done for trusted channels */
  if (cd_p->trust == TRUE) {
    bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
    for(index=0 ; index < cd_p->bufferDescNumber ; index++){
      if ((bd_p->bufferAddr == NULL) || (bd_p->mode.count == 0)){
        result = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
        iapi_errno = result;
        iapi_ReleaseChannel(chNum);
        return -result;
      }
      bd_p++;
    }
  }
  
  
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
 /*If transfer data size is used, check that the required write length is  
   * divisible by transfer data size expressed in bytes
   */
   if(cd_p->useDataSize)
   {
      /*Check for divisibility only if data size different then 8bit*/
      if(cd_p->dataSize != TRANSFER_8BIT)
      {
         switch(cd_p->dataSize)
         {
            case TRANSFER_32BIT:
               div = 4;
               break;
            case TRANSFER_16BIT:
               div = 2;
               break;
            case TRANSFER_24BIT:
               div = 3;
               break;
            /*we should not get to default*/
            default:
               result = IAPI_ERR_INVALID_PARAMETER | chNum;
               iapi_errno = result;
               iapi_ReleaseChannel(chNum);
               return -result;
         }
         /*check the total number of bytes requested*/
         if((nbyte % div) != 0)
         {
            result = IAPI_ERR_INVALID_PARAMETER | chNum;
            iapi_errno = result;
            iapi_ReleaseChannel(chNum);
            return -result;
         }
         /*now check the length of every BD*/
         for(index=0 ; index < cd_p->bufferDescNumber ; index++)
         {
            if((bd_p->mode.count % div) != 0)
            {
               result = IAPI_ERR_INVALID_PARAMETER | chNum;
               iapi_errno = result;
               iapi_ReleaseChannel(chNum);
               return -result;
            }
		    bd_p++;
         }
      }
   }

  /*
   * 2. Write loop
   */

  local_buf = (unsigned char *)buf;
  toWrite = nbyte;
  copyFinished = FALSE;
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);    
  
  while (!copyFinished){
    
	/* variable buffsize contains the nb of bytes that the SDMA will transfer at each pass of the while loop*/

	/* in NON trusted mode, buffsize is copied from Channel descriptor bufferSize (same size for all transfers) */
	if (cd_p->trust == FALSE) {
      buffsize = cd_p->bufferSize;
    } 
	/* in TRUSTED mode, it's up to the user to specify the size of each buffer thru an IoCtl call */
	/* This IoCtl has directly modified the bd_p->mode.count     */
	/* therefore, buffersize is copied from the bd_p->mode.count */
	else {
      buffsize = bd_p->mode.count;
    }

    /* in any mode (trusted or non trusted), the transfer size must be overridden by  */
	/* "toWrite" when there is less remaining bytes to transfer than the current buffer size */
    if (toWrite < buffsize) {
	  buffsize = toWrite;
	}


    if (!(bd_p->mode.status & BD_DONE)){
        /* More data to write than a single buffer can contain */
        if (cd_p->trust == FALSE ){
          iapi_memcpy(iapi_Phys2Virt(bd_p->bufferAddr), local_buf, buffsize);
          local_buf += buffsize; 
        }
        
		/* update the BD count that will be used by the SDMA to transfer the proper nb of bytes */
        bd_p->mode.count = buffsize;

        bd_p->mode.status |= BD_DONE;
        writtenBytes += buffsize;
        toWrite -= buffsize;
        /* Prepares access to the "next" buffer */
        /* - case 1 - finished successfully : writtenBytes = nbytes */
        if (toWrite == 0) {
          copyFinished = TRUE;
		}
        /* - case 2 - Last BD and WRAP bit set so re-start from beginning */
        /*else if ((bd_p->mode.status & BD_WRAP)){
          bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
        }*/
        /* - case 3 - Last BD of the BD but nor ring*/
        else if (((bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr) + 
             (cd_p->bufferDescNumber - 1) *  sizeof(bufferDescriptor)) == bd_p){
          copyFinished = TRUE;
        }
        /* - case 4 - general : next BD in the BD array */
        else {
          bd_p++;
        }

    } else {
      /* finished here : buffer not already done */
      copyFinished = TRUE;
    }
  }

  ccb_p->currentBDptr = ccb_p->baseBDptr;

  /*
   * 3. Starting of the channel
   */
  iapi_lowStartChannel(chNum);
  ccb_p->status.execute = TRUE;
  
  if( cd_p->callbackSynch == DEFAULT_POLL)
  {
      iapi_SynchChannel(chNum);
      /*
       * Check the 'RROR' bit on all buffer descriptors, set error number
       *    and return IAPI_FAILURE if set.
       */
      bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
      for (index=0 ; index < cd_p->bufferDescNumber ; index++) 
      {
         if(bd_p->mode.status & BD_RROR)
         {
            result = IAPI_ERR_RROR_BIT_WRITE | chNum;
            iapi_errno = result;
            iapi_ReleaseChannel(chNum);
            return -result;
         }
		 bd_p++;
      }
      iapi_ReleaseChannel(chNum);
  }

  /* 
   *If synchronization type is callback, the user of I.API must 
   *release the channel 
   */
  return writtenBytes;
}

/* ***************************************************************************/
/**Terminates a channel. 
 *
 * <b>Algorithm:</b>\n
 *   - Check input parameters ans data structures
 *   - Check that all buffes have been processed (test all 'D' bits)
 *   - Stop the channel execution
 *   - Free alocated memory structures
 *   - Re-instantiate default interrupt handling
 *
 * @param *cd_p chanenl descriptor for the channel to close
 *
 * @return 
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : close failed
 */
int
iapi_Close (channelDescriptor * cd_p)
{
  int index = 0;
  int result = IAPI_SUCCESS;
  unsigned char chNum;
  bufferDescriptor * bd_p;
  channelControlBlock * ccb_p;

  /*
   * 1. Check input parameters ans data structures
   */
  if (cd_p != NULL){
    if (cd_p->ccb_ptr != NULL) {
      chNum = cd_p->channelNumber;
      ccb_p = cd_p->ccb_ptr;
    } else {
      result = IAPI_ERR_NO_CCB_DEFINED | IAPI_ERR_CH_AVAILABLE;
      iapi_errno = result;
      return -result;
    }
  } else {
      result = IAPI_ERR_CD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE;
      iapi_errno = result;
      return -result;
  }
  /* Try to aquire channel*/
  if(iapi_GetChannel(chNum) != 0)
  {
     result = IAPI_ERR_CH_IN_USE | chNum;
     iapi_errno = result;
     return -result;
  }

  /*
   * 2. Check that all buffes have been processed (test all 'D' bits),
   * only if the forceClose bit in channel descriptor is set to FALSE
   */
  bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
  if (bd_p == NULL){ 
    result = IAPI_ERR_BD_UNINITIALIZED | IAPI_ERR_CH_AVAILABLE | chNum;
    iapi_errno = result;
    return -result;
  }
  if(cd_p->forceClose == FALSE)
  {
     for (index=cd_p->bufferDescNumber ; index>0 ; index--){
       if (bd_p->mode.status & BD_DONE){
         result = IAPI_ERR_CLOSE | IAPI_ERR_CH_AVAILABLE | chNum;
         iapi_errno = result;
         iapi_ReleaseChannel(chNum);
         return -result;
       }
       bd_p++;
     }
  }
  /*if the closing is forced,mark channel unused,set BD ownership to processor*/
  else
  {
      ccb_p->status.execute = FALSE;
      for (index=cd_p->bufferDescNumber ; index>0 ; index--)
      {
         bd_p->mode.status &= ~BD_DONE;
         bd_p++;
      }
  }
  
  /*
   * 3. Stop the channel execution
   */
  iapi_lowStopChannel(chNum);

  /* 
   * 4. Free alocated memory structures
   */
  if (cd_p->trust == FALSE ){
    bd_p = (bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr);
    for (index=cd_p->bufferDescNumber ; index>0 ; index--){
      FREE (iapi_Phys2Virt(bd_p->bufferAddr));
      bd_p++;
    }
  }

  /*
   * 5. Re-instantiate default interrupt handling
   */
  iapi_DetachCallbackISR (cd_p);
  FREE ((bufferDescriptor *)iapi_Phys2Virt(ccb_p->baseBDptr));
  FREE (cd_p);
  ccb_p->baseBDptr = NULL;
  ccb_p->currentBDptr = NULL;
  ccb_p->channelDescriptor = NULL;
  ccb_p->status.openedInit = FALSE;

  iapi_ReleaseChannel(chNum);

  return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**The request argument selects the control function to be performed.
 *
 * <b>Algorithm:</b>\n
 *
 * - Check data structures are properly initialized:
 *   - Channel descriptor validity
 *   - Channel control block validity
 * - The ctlRequest parameter contains in the lower 16 bits the control code of
 *   the change to be performed, and in the upper 16 bits, the BD to be
 *   modified if the change affects a BD od the channel.
 * - Selection of the parameter to change and appropriate sanity checks:
 *   - Channel Descriptor: changes the pointer to the channel descriptor 
 * structure, the pointer to the new channel descriptor is given in the third
 * argument call
 *   - Buffer Descriptor Number: changes the number of buffer descriptor for the
 * channel
 *   - Buffer size: changes the size of the data buffers pointed to by the 
 * buffer descriptor; note that all buffer descriptors are assumed to have the 
 * same size for a given buffer descripotr chain
 *   - Blocking policy: changes the blocking policy for the read and write calls
 *   - Ownership: changes direction: turnaround
 *   - Synchronization method: changes the callback type, default or user. The*
 * callback function table is set accordingly
 *   - Trust property: trust can only be changed through ChangeChannelDesc first
 * request, this guarantees the close/open sequence for the channel
 *   - Callback Interrupt service routine pointer: changes the callback function
 * pointer, when this method is used, to replace it with a new one
 *   - Channel control block pointer: not available
 *   - Priority: changes the channel priority directly in SDMA register
 *   - Watermark level: changes the value of the peripheral watermark level that
 * passed to the script. The new value is passed in the third parameter call.
 *   - Wrap bit: changes to set to 1 the Wrap bit of the last buffer descriptor
 *
 * @param *cd_p channel descriptor for the channel to modify
 * @param ctlRequest request control code and, if tha case, number of BD to be 
 *                   changed
 * @param param parameter for the modification
 *
 * @return 
 *     - IAPI_SUCCESS : OK
 *     - -iapi_errno : operation failed
 */
int
iapi_IoCtl (channelDescriptor * cd_p, unsigned long ctlRequest, 
            unsigned long param)
{
  int retvalue;
  int result = IAPI_SUCCESS;
  unsigned char chNum;
  unsigned long clean_ctlRequest; /* lower 16 bits of the ctlRequest*/
  unsigned long bd_num; /* upper 16 bits of the ctlRequest*/

  /*
   * 1. Check data structures are properly initialized
   */
  /* Channel descriptor validity */
  if (cd_p == NULL){
    result = IAPI_ERR_CD_UNINITIALIZED;
    iapi_errno = result;
    return -result;
  }
  
  /* Channel control block validity */
  if (cd_p->ccb_ptr == NULL){
    result = IAPI_ERR_CCB_UNINITIALIZED;
    iapi_errno = result;
    return -result;
  }

  /* Control block & Descriptor associated with the channel being worked on */
  chNum = cd_p->channelNumber;

  /* Remove, if exists, BD number specified in upper bits of ctlRequest*/
  clean_ctlRequest = ctlRequest & (~BD_NUM_MASK);
  
  /* Extract, if exists, BD number specified in upper bits of ctlRequest*/
  bd_num = (ctlRequest & BD_NUM_MASK) >> BD_NUM_OFFSET;
  
  /* Check that the bd_num is valid*/
  if(bd_num > cd_p->bufferDescNumber)
  {
     result = IAPI_ERR_INVALID_PARAMETER | chNum;
     iapi_errno = result;
     return -result;
  }

  /*All checks OK, try to aquire channel*/
  if(iapi_GetChannel(chNum) != 0)
  {
     result = IAPI_ERR_CH_IN_USE | chNum;
     iapi_errno = result;
     return -result;
  }

  /*
   * 2. Selection of the parameter to change and appropriate sanity checks
   */
  switch (clean_ctlRequest){
    
    /*
     * Channel Descriptor
     * --- Changes the pointer to the channel descriptor structure: the pointer
     * to the new channel descriptor is given in the third argument call.
     */    
  case IAPI_CHANGE_CHANDESC:
    if ((void *) param == NULL) {
      result = IAPI_ERR_INVALID_PARAMETER;
      iapi_errno = result;
      iapi_ReleaseChannel(chNum);
      return -result;
    } else {
      channelDescriptor * chParam = (channelDescriptor *)param;
      if (chParam->channelNumber != chNum){
        /* Release ch so it can be aquired by the Close fn*/
        iapi_ReleaseChannel(chNum);
        result = iapi_Close(cd_p);
        if (result == IAPI_SUCCESS){
          FREE((void*)cd_p);
          iapi_AllocChannelDesc (&cd_p, chParam->channelNumber);
          iapi_memcpy((void*)cd_p, (void*)chParam, sizeof (channelDescriptor));
          /* Channel is released allready, so Open can get the channel*/
          result = iapi_Open(cd_p, chParam->channelNumber);
          if(result != IAPI_SUCCESS)
          {
            return result; /* error code already set in iapi_Open*/
          }
        } else {
          return result; /* error code already set in iapi_Close*/
        }
      } else {
        result = IAPI_ERR_CD_CHANGE | IAPI_ERR_CH_AVAILABLE | 
          cd_p->channelNumber;
        iapi_ReleaseChannel(chNum);
        iapi_errno = result;
        return -result;
      }
      return IAPI_SUCCESS;
    }
    
    /*
     * Buffer Descriptor Number
     * --- Changes the number of buffer descriptor for the channel.
     */
  case IAPI_CHANGE_BDNUM:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERDESCNUMBER, param); 
    iapi_ReleaseChannel(chNum);
    return result;
    
    /*
     * Buffer size
     * --- Changes the size of the data buffers pointed to by the buffer
     * descriptor; note that all buffer descriptors are assumed to have the
     * same size for a given buffer descripotr chain.
     */
  case IAPI_CHANGE_BUFFSIZE:
    result =  iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERSIZE, param);
    iapi_ReleaseChannel(chNum);
    return result;
        
    /*
     * Blocking policy
     * --- Changes the blocking policy for the read and write calls.
     */
  case IAPI_CHANGE_CHANBLOCK:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_BLOCKING, param);
    iapi_ReleaseChannel(chNum);
    return result;

    /*
     * Ownership
     * --- Changes direction: turnaround
     */
  case IAPI_CHANGE_OWNERSHIP:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_OWNERSHIP, param);
    iapi_ReleaseChannel(chNum);
    return result;

    /*
     * Synchronization method
     * --- Changes the callback type, default or user. The callback function
     * table is set accordingly.
     */
  case IAPI_CHANGE_SYNCH:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_CALLBACKSYNCH, param);
    iapi_ReleaseChannel(chNum);
    return result;

    /*
     * Trust property
     * --- trust can only be changed through ChangeChannelDesc first request,
     * this guarantees the close/open sequence for the channel.
     */
  case IAPI_CHANGE_TRUST:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_TRUST, param);
    iapi_ReleaseChannel(chNum);
    return result;

    /*
     * Callback Interrupt service routine pointer
     * --- Cahnges the callback function pointer, when this method is used, to
     * replace it with a new one.
     */
  case IAPI_CHANGE_CALLBACKFUNC:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_CALLBACKISR_PTR, param);
    iapi_ReleaseChannel(chNum);
    return result;

    /*
     * Channel control block pointer
     * --- NA
     */
  case IAPI_CHANGE_CHANCCB:
    result = iapi_ChangeChannelDesc(cd_p, IAPI_CCB_PTR, param);
    iapi_ReleaseChannel(chNum);
    return result;

#ifdef MCU
    /*
     * Priority
     * --- Changes the channel priority directly in SDMA register
     */ 
  case IAPI_CHANGE_PRIORITY:
    {
      volatile unsigned long * ChannelPriorities = &SDMA_CHNPRI_0;
      if(param < MAX_CH_PRIORITY)
      {
         ChannelPriorities[ cd_p->channelNumber ] = param;
      }
      else
      {
         iapi_ReleaseChannel(chNum);
         return IAPI_FAILURE;
      }
    }
    break;
#endif /* MCU */

    /*
     * Wrap
     * --- Set to 1 the wrap bit of the last buffer descriptor of the array.
     * it provides the possibility to have a circular buffer structure.
     */ 
  case IAPI_CHANGE_BDWRAP:
    {
      result =  iapi_ChangeChannelDesc(cd_p,IAPI_BDWRAP , param);
      iapi_ReleaseChannel(chNum);
      return result;
    }
    
    /*
     * Watermark
     * --- Changes the value of the peripheral watermark level that triggers
     * a DMA request. It impacts context of the channel, therefore channel 0
     * must be started to update the context with this new value.
     */ 
  case IAPI_CHANGE_WATERMARK:
    {
      result = iapi_ChangeChannelDesc(cd_p,IAPI_WML , param);
      iapi_ReleaseChannel(chNum);
      return result;
    }
    /*
     * INTR
     * --- Set the INTR bit on specified BD or on all BD's if SET_BIT_ALL
     * is passed as parameter.
     */ 
  case IAPI_CHANGE_SET_BDINTR:
    {
       bufferDescriptor * bde_p;
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.status |= BD_INTR;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
                 param;
         bde_p->mode.status |= BD_INTR;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
    /*
     * INTR
     * --- Unset the INTR bit on specified BD or on all BD's if SET_BIT_ALL
     * is passed as parameter.
     */ 
  case IAPI_CHANGE_UNSET_BDINTR:
    {
       bufferDescriptor * bde_p;
       
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.status &= ~BD_INTR;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
                 param;
            bde_p->mode.status &= ~BD_INTR;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
/*
     * EventMask1
     * --- Changes the value of the eventMask1
     */ 
  case IAPI_CHANGE_EVTMASK1:
    {
       cd_p->eventMask1 = param;
    }
    break;
    /*
     * EventMask2
     * --- Changes the value of the eventMask2
     */ 
  case IAPI_CHANGE_EVTMASK2:
    {
       cd_p->eventMask2 = param;
    }
    break;
    /*
     * Peripheral Address
     * --- Changes the value of the peripheralAddr
     */ 
  case IAPI_CHANGE_PERIPHADDR:
    {
       cd_p->peripheralAddr = param;
    }
    break;
    /*
     * Cont
     * --- Set the CONT bit on specified BD on all BD's if SET_BIT_ALL
     * is passed as parameter.
     */ 
  case IAPI_CHANGE_SET_BDCONT:
    {
       bufferDescriptor * bde_p;
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.status |= BD_CONT;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
                 param;
         bde_p->mode.status |= BD_CONT;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
    /*
     * Cont
     * --- Unset the CONT bit on specified BD or on all BD's if SET_BIT_ALL
     * is passed as parameter.
     */ 
  case IAPI_CHANGE_UNSET_BDCONT:
    {
       bufferDescriptor * bde_p;
       
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.status &= ~BD_CONT;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
                 param;
         bde_p->mode.status &= ~BD_CONT;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }

    /*
     * EXTD
     * --- Set the EXTD bit on specified BD or on all BD's if SET_BIT_ALL
     * is passed as parameter.
     */ 
  case IAPI_CHANGE_SET_BDEXTD:
    {
       bufferDescriptor * bde_p;
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.status |= BD_EXTD;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
         param;
         bde_p->mode.status |= BD_EXTD;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
    /*
     * EXTD
     * --- Unset the EXTD bit on specified BD or on all BD's if SET_BIT_ALL
     * is passed as parameter.
     */ 
  case IAPI_CHANGE_UNSET_BDEXTD:
    {
       bufferDescriptor * bde_p;
       
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.status &= ~BD_EXTD;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
         param;
         bde_p->mode.status &= ~BD_EXTD;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
    /*
     * TRANSFER SIZE to be used for this channel
     * --- Set the transfer size used indicator and code for transfer size in 
     * the CD
     */ 
case IAPI_CHANGE_SET_TRANSFER_CD:
   {
      bufferDescriptor * bde_p;
      int j = 0;
      retvalue = IAPI_SUCCESS;
      if((param == TRANSFER_8BIT) || (param == TRANSFER_16BIT) ||
         (param == TRANSFER_24BIT) || (param == TRANSFER_32BIT))
      {
         cd_p->useDataSize = TRUE;
         cd_p->dataSize = param;
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
#ifdef CONFIG_MOT_WFN458
             bde_p->mode.command = ((bde_p->mode.command & CHANGE_ENDIANNESS) | param);
#else
             bde_p->mode.command = param;
#endif
            bde_p++;
         }
      }
      else
      {
         retvalue = IAPI_FAILURE;
      }
      iapi_ReleaseChannel(chNum);
      return retvalue;
   }


    /*
     * USER_ARG
     * --- Set the user selectable pointer to be received by the callback 
     * function, if IRQ synch is used
     */ 
  case IAPI_CHANGE_USER_ARG:
    {
      userArgTable[cd_p->channelNumber]= (void*)param;  
      iapi_ReleaseChannel(chNum);
      return IAPI_SUCCESS;
    }
    /*
     * FORCE_CLOSE
     * --- Set the forceClose bit in channelDescriptor to value passed in param.
     * If this bit is TRUE, the channel in closed even if some BD are still 
     * owned by the SDMA.
     */ 
  case IAPI_CHANGE_FORCE_CLOSE:
    {
       retvalue = IAPI_SUCCESS;
       if((param == TRUE) || (param == FALSE))
       {
          cd_p->forceClose = param;
       }
       else
       {
           iapi_errno = IAPI_ERR_INVALID_PARAMETER | cd_p->channelNumber;
           retvalue = -iapi_errno;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
    /*
     * TRANSFER type
     * --- Set the last 2 bits in the command field of the BD to specify the 
     * transfer type 8, 16, 24, or 32 bits on all BD's, allready set in the CD
     */ 
  case IAPI_CHANGE_SET_TRANSFER:
    {
       bufferDescriptor * bde_p;
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if((param == TRANSFER_8BIT)||(param == TRANSFER_16BIT)||
          (param == TRANSFER_24BIT)||(param == TRANSFER_32BIT))
       {
         bde_p = cd_p->ccb_ptr->baseBDptr;
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.command = param;
            bde_p++;
         }
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
    /*
     * BUFFER address
     * --- Change buffer address in BD specified in the upper 16 bits of the
     * ctlRequest.
     */ 
  case IAPI_CHANGE_SET_BUFFERADDR:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
	  
      /* DO NOT translate address to physical */
      bde_p->bufferAddr = (void*)param;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * BUFFER address
     * --- Get the buffer address from the BD specified in the upper 16 bits of the
     * ctlRequest.
     */ 
  case IAPI_CHANGE_GET_BUFFERADDR:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Translate to virtual*/
      *((unsigned long*)param) = (unsigned long)bde_p->bufferAddr;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * EXTENDED BUFFER address
     * --- Change extended buffer address in BD specified in the upper 16 bits 
     * of the ctlRequest.
     */ 
  case IAPI_CHANGE_SET_EXTDBUFFERADDR:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      
      /* DO NOT translate address to physical. The user might want something else 
       *here
       */
      bde_p->extBufferAddr = (void*)param;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * EXTENDED BUFFER address
     * --- Get extended buffer address from the BD specified in the upper 16 bits 
     * of the ctlRequest.
     */ 
  case IAPI_CHANGE_GET_EXTDBUFFERADDR:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      
      /* DO NOT translate address to vitual - user knows what is here. 
       */
      *((unsigned long*)param) = (unsigned long)bde_p->extBufferAddr;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * COMMAND field
     * --- Change command field in BD specified in the upper 16 bits of the
     * ctlRequest.
     */ 
  case IAPI_CHANGE_SET_COMMAND:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Update command field*/
      bde_p->mode.command = param;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * COMMAND field
     * --- Get the command field from the BD specified in the upper 16 bits 
     * of the ctlRequest.
     */ 
  case IAPI_CHANGE_GET_COMMAND:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Get the command field*/
      *((unsigned long*)param) = bde_p->mode.command;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * COUNT field
     * --- Change count field in BD specified in the upper 16 bits of the
     * ctlRequest.
     */ 
  case IAPI_CHANGE_SET_COUNT:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Update count field*/
      bde_p->mode.count = param;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * COUNT field
     * --- Get the count field of the BD specified in the upper 16 bits of the
     * ctlRequest.
     */ 
  case IAPI_CHANGE_GET_COUNT:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Update count field*/
      *((unsigned long*)param) = bde_p->mode.count;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * STATUS field
     * --- Change status field in BD specified in the upper 16 bits of the
     * ctlRequest.
     */ 
  case IAPI_CHANGE_SET_STATUS:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Update status field*/
      bde_p->mode.status = param;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }
    /*
     * STATUS field
     * --- Get the status field of the BD specified in the upper 16 bits 
     * of the ctlRequest.
     */ 
  case IAPI_CHANGE_GET_STATUS:
    {
      bufferDescriptor * bde_p;
      retvalue = IAPI_SUCCESS;

      /* Get pointer to the BD structure to change*/
      bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
      bde_p+= bd_num;
      /* Update status field*/
      *((unsigned long*)param) = bde_p->mode.status;
      
      iapi_ReleaseChannel(chNum);
      return retvalue;
    }

#ifdef MCU
    /*
     * Endianness
     * --- Set the ENDIANNESS indicator in the command filed of the specified BD 
     * or on all BD's if SET_BIT_ALL is passed as parameter.
     */ 
  case IAPI_CHANGE_SET_ENDIANNESS:
    {
       bufferDescriptor * bde_p;
       int j = 0;
       
       retvalue = IAPI_SUCCESS;
       if(param == SET_BIT_ALL)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
         for(j = 0; j < cd_p->bufferDescNumber; j++)
         {
            bde_p->mode.command = CHANGE_ENDIANNESS;
            bde_p++;
         }
       }
       else if(param < cd_p->bufferDescNumber)
       {
         bde_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr) + 
                 param;
         bde_p->mode.command = CHANGE_ENDIANNESS;
       }
       else
       {
          retvalue = IAPI_FAILURE;
       }
       iapi_ReleaseChannel(chNum);
       return retvalue;
    }
#endif

  default:
    retvalue = IAPI_ERR_CD_CHANGE_UNKNOWN | IAPI_ERR_CH_AVAILABLE | chNum; 
    iapi_errno = retvalue;
    iapi_ReleaseChannel(chNum);
    return -retvalue;
  }
    

  iapi_ReleaseChannel(chNum);
  return IAPI_SUCCESS;
}
/* ***************************************************************************/
/**Initialization of the SDMA - opening of channel 0, download RAM image.
 *
 * <b>Algorithm:</b>\n
 *     - open channel 0
 *     - if ram_image pointer passed is not NULL, download RAM image to SDMA
 *
 * @param 
 *     - cd_p channel descriptor pointer for channel 0
 *     - ram_image pointer to RAM image to download, or NULL if this operation
 *                 is not required
 *     - code_size size of the RAM image, in bytes 
 *     - start_addr start address for the RAM image
 *
 * @return 
 *     - IAPI_SUCCESS if all operations were successful
 *     - negated I.API error code if any operation failed
 */
#ifdef MCU
int 
iapi_Init(channelDescriptor * cd_p, configs_data * config_p, unsigned short* ram_image, 
          unsigned short code_size, void * start_addr)
{
#ifdef CONFIG_MOT_WFN476
	unsigned int i;
#endif
#endif
#ifdef DSP
int 
iapi_Init(channelDescriptor * cd_p)
{
#endif

int retvalue = IAPI_SUCCESS;  /* Variable to store the results from I.API calls */

   /* Check initialization not allredy done*/
   if(iapi_CCBHead != NULL)
   {
      retvalue = IAPI_ERR_NOT_ALLOWED;
      iapi_errno = retvalue;
      return -retvalue;
   }
   /* Be sure SDMA has not started yet */
#ifdef MCU
    SDMA_H_C0PTR = NULL;
#endif
#ifdef DSP
    SDMA_D_C0PTR = NULL;
#endif

   /*Try to open channel 0*/
   retvalue = iapi_Open(cd_p, 0);
   if(retvalue != IAPI_SUCCESS)
   {
      return retvalue;
   }
      
#ifdef MCU
   /* Set Command Channel (Channel Zero) */
   SDMA_CHN0ADDR = 0x4050;
   
   /* Set bits of CONFIG register*/
   SDMA_H_CONFIG = (config_p->dspdma << 12) | (config_p->rtdobs << 11) |
                    (config_p->acr << 4) | (config_p->csm);
   
   /* Send the address for the host channel table to the SDMA*/
   SDMA_H_C0PTR = (channelControlBlock *)iapi_Virt2Phys(iapi_CCBHead);

#ifdef CONFIG_MOT_WFN476
   /* Clean context #0 */
   SDMA_ONCE_ENB = 0x1;
   SDMA_ONCE_CMD = 0x5;

   SDMA_ONCE_INSTR = 0x0808;	/* ldi r0, 0x08 */
   SDMA_ONCE_CMD = 0x2;
   SDMA_ONCE_INSTR = 0x0011;	/* revblo r0 */
   SDMA_ONCE_CMD = 0x2;
   SDMA_ONCE_INSTR = 0x0900;	/* ldi r1, 0x00 */
   SDMA_ONCE_CMD = 0x2;
   for (i = 0; i < 32; i++)
   {
       SDMA_ONCE_INSTR = 0x5900 + (i << 3); /* st r1,(r0,i) */
       SDMA_ONCE_CMD = 0x2;
   }

   SDMA_ONCE_CMD = 0x3;
   SDMA_ONCE_ENB = 0x0;	/* disable ONCE because of clock impact */
#endif

   /* If required, download the RAM image for SDMA*/
   if(ram_image != NULL)
   {
      retvalue = iapi_SetScript(cd_p, (void*)ram_image, code_size,  
                                start_addr);   
   }
#endif
#ifdef DSP
   /* Send the address for the host channel table to the SDMA*/
   SDMA_D_C0PTR = (unsigned long)iapi_Virt2Phys(iapi_CCBHead);
#endif

   return retvalue;
}

/* ***************************************************************************/
/**High layer interface for starting a channel
 *
 * <b>Algorithm:</b>\n
 *    - call low layer function for starting a channel
 *
 * @return 
 *     - IAPI_SUCCESS
 */
int 
iapi_StartChannel(unsigned char channel)
{
   iapi_lowStartChannel(channel);
   return IAPI_SUCCESS;
}
/* ***************************************************************************/
/**High layer interface for stopping a channel
 *
 * <b>Algorithm:</b>\n
 *    - call low layer function for stopping a channel
 *
 * @return 
 *     - IAPI_SUCCESS
 */
int 
iapi_StopChannel(unsigned char channel)
{
   iapi_lowStopChannel(channel);
   return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for synchronising a channel
 *
 * <b>Algorithm:</b>\n
 *    - call low layer function for stopping a channel
 *
 * @return 
 *     - IAPI_SUCCESS
 */
int iapi_SynchChannel(unsigned char channel)
{
   iapi_lowSynchChannel(channel);
   return IAPI_SUCCESS;
}

#ifdef MCU
/* ***************************************************************************/
/**High layer interface for getting program memory data from SDMA
 *
 * <b>Algorithm:</b>\n
 *    - call corresponding low layer function
 *
 * @return 
 *     - IAPI_SUCCESS
 */
int
iapi_GetScript(channelDescriptor * cd_p, void * buf, unsigned short size, 
                    void * address)
{
   iapi_lowGetScript(cd_p, buf, size, address);
   return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for getting data memory from SDMA
 *
 * <b>Algorithm:</b>\n
 *    - call corresponding low layer function
 *
 * @return 
 *     - IAPI_SUCCESS
 */                    
int
iapi_GetContext(channelDescriptor * cd_p, void * buf,
                     unsigned char channel)
{
   iapi_lowGetContext(cd_p, buf, channel);
   return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for set program memory data to SDMA - e.g. scripts
 *
 * <b>Algorithm:</b>\n
 *    - call corresponding low layer function
 *
 * @return 
 *     - IAPI_SUCCESS
 */                    
int
iapi_SetScript(channelDescriptor * cd_p, void * buf, unsigned short nbyte, 
                     void * destAddr)
{
   iapi_lowSetScript(cd_p, buf, nbyte, destAddr);   
   return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface for set data memory to SDMA - e.g. contexts.
 *
 * <b>Algorithm:</b>\n
 *    - call corresponding low layer function
 *
 * @return 
 *     - IAPI_SUCCESS
 */                    
int 
iapi_SetContext(channelDescriptor * cd_p, void * buf, 
                     unsigned char channel)
{
   iapi_lowSetContext(cd_p, buf, channel);
   return IAPI_SUCCESS;
}

/* ***************************************************************************/
/**High layer interface used to associate specified channel with a script.
 *
 * <b>Algorithm:</b>\n
 *    - call corresponding low layer function
 *
 * @return 
 *     - IAPI_SUCCESS
 */                    
int
iapi_AssignScript(channelDescriptor * cd_p, script_data * data_p)
{
   /* VERIFY THAT THE CHANNEL IT IS OPENED !!!!*/
   return iapi_lowAssignScript(cd_p, data_p);
}

/* ***************************************************************************/
/**High layer interface used to associate specified channel with a script.
 *
 * <b>Algorithm:</b>\n
 *    - call corresponding low layer function
 *
 * @return 
 *     - IAPI_SUCCESS
 */                    
int
iapi_SetChannelEventMapping(unsigned char event, unsigned long channel_map)
{
   return iapi_lowSetChannelEventMapping(event, channel_map);
}
#endif



#ifdef DSP
#define SDMA_DI  SDMA_D_INTR
void IRQ_Handler();
#pragma interrupt IRQ_Handler
#endif

#ifdef MCU
#define SDMA_DI  SDMA_H_INTR
#endif

#ifndef IRQ_KEYWORD
#define IRQ_KEYWORD
#endif /* IRQ_KEYWORD */

/* ***************************************************************************/
/**
 *@brief  Find the first set bit in data parameter.
 *       
 *       Find the first set bit in unsigned integer parameter data. Data is scanned
 *  from MSB to LSB, searching for the set bit. The value returned is the
 *  offset from the most significant bit of data. If bit 31 is set, the value
 *  returned is zero. If no bits are set, a value of 32 is returned. This is compliant 
 *  with the MCore FF1 instruction.
 *
 *        
 *
 * @param  
 *   - data: variable to check
 *
 * @return 
 *   - the offset of the most significant bit set from the MSB
 */
unsigned int
quartz_FF1( unsigned int data )
{
   register unsigned int result = 0;
   while ( (result <= 31 ) && !( data & 0x80000000U) )
   {
      data <<= 1U;
      result++;
   }
   
   return result;
}

IRQ_KEYWORD
void 
IRQ_Handler(void)
{
   unsigned int intrReg;/* interrupt register mask for clearing the interrupt bit */
   unsigned char chNum; /* SDMA channel number generating the a IRQ*/

   /* Disable interrupts */
   iapi_DisableInterrupts();
   /*
    * Clear interrupt in SDMA DI register => ACK to the SDMA the IT request.
    * Get each interrupt number, clear them one after the other.
    */
   if(SDMA_DI != 0)
   {
     chNum = (CH_NUM - 1 - quartz_FF1(SDMA_DI));
   }
   else
   {
     chNum = 32;
   }
   intrReg = 1 << chNum;
   while (intrReg != 0)
   {
      SDMA_DI &= intrReg;
      iapi_SDMAIntr |= intrReg;
      iapi_WakeUp(chNum);
      if (callbackIsrTable[chNum] != NULL)
      {
         /* release channel before callback, so IoCtl's are available*/
         iapi_ReleaseChannel(chNum);
         callbackIsrTable[chNum](iapi_CCBHead[chNum].channelDescriptor,
                                 userArgTable[chNum]);       
      }
   	   
      chNum = (CH_NUM - 1 - quartz_FF1(SDMA_DI));
      intrReg = 1 << chNum;
   }
   
   /* Enable interrupts */
   iapi_EnableInterrupts();
}

/* ***************************************************************************/
/**
 *@brief  Perform a memory copy operation, in the memory of the same processor
 *       
 *       Size bytes are copied from the src address to dest address. It is used
 *    the channel pointed by cd_p, which must be configured prior to this call:
 *    opened, associated with the script to perform the operation - DSP_2_DSP,
 *    or MCU_2_MCU - and have the synchronization option set.
 *
 *        
 *
 * @param  
 *   - cd_p: channel configured to perform DSP_2_DSP or MCU_2_MCU transfers
 *   - dest: destination memory address
 *   - src : source memory address
 *   - size: number of bytes to copy from src to dest
 *
 * @return 
 *   - the offset of the most significant bit set from the MSB
 */

int iapi_MemCopy(channelDescriptor * cd_p, void* dest, void* src, unsigned long size)
{
   int result = IAPI_SUCCESS;
   bufferDescriptor * bd_p;
   
   /* Channel descriptor validity */
   if (cd_p == NULL)
   {
      result = IAPI_ERR_CD_UNINITIALIZED;
      iapi_errno = result;
      return -result;
   }
   
   /* Check and set correct parameter */
   if(cd_p->trust != TRUE)
   {
      result = iapi_ChangeChannelDesc(cd_p, IAPI_TRUST, TRUE);
   }
   
   if(cd_p->bufferDescNumber != 1)
   {
      result = iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERDESCNUMBER, 1); 
      if(result != IAPI_SUCCESS)
      {
         return result;
      }
   }
   
   if(cd_p->bufferSize != size)
   {
      result = iapi_ChangeChannelDesc(cd_p, IAPI_BUFFERSIZE, size); 
      if(result != IAPI_SUCCESS)
      {
         return result;
      }
   }
   /* Set addresses*/
   bd_p = (bufferDescriptor *)iapi_Phys2Virt(cd_p->ccb_ptr->baseBDptr);
   bd_p->bufferAddr = iapi_Virt2Phys(src);
   bd_p->extBufferAddr = iapi_Virt2Phys(dest);
   
   /* Set mode*/
   bd_p->mode.count = size;
   bd_p->mode.command = 0x00;
   bd_p->mode.status = BD_INTR|BD_EXTD|BD_DONE|BD_WRAP;   

   /*Decide if we sleep or not*/
   if(cd_p->callbackSynch == DEFAULT_POLL)
   {
      iapi_StartChannel(cd_p->channelNumber);
      /* Call synchronization routine*/
      iapi_SynchChannel(cd_p->channelNumber);
   }
   else
   {
      /* Just start the channel*/
      iapi_StartChannel(cd_p->channelNumber);
   }
      
   return result;
}

/* ***************************************************************************/
/**Return the channel number from the channel descriptor
 *
 * @param cd_p  pointer to channel descriptor to obtain the channel number
 *
 * @return
 *     - the channel number
 *
 */ 
int iapi_GetChannelNumber(channelDescriptor * cd_p)
{
   return cd_p->channelNumber;
}

/* ***************************************************************************/
/**Return the error bit from the current BD of the channel
 *
 *
 * @param cd_p  pointer to channel descriptor
 *
 * @return
 *     - 0 if no error detected
 *     - BD_RROR | DATA_ERROR if error detected
 *
 */ 
unsigned long iapi_GetError(channelDescriptor * cd_p)
{
   return ((cd_p->ccb_ptr->currentBDptr->mode.status & BD_RROR) | 
           (*(unsigned long*)&cd_p->ccb_ptr->status & DATA_ERROR));
}

/* ***************************************************************************/
/**Return the count from the current BD of the channel
 *
 *
 * @param cd_p  pointer to channel descriptor
 *
 * @return
 *     - count field of the current BD for the channel
 *
 */ 
int iapi_GetCount(channelDescriptor * cd_p)
{
   return (cd_p->ccb_ptr->currentBDptr->mode.count);
}

/* ***************************************************************************/
/**Return the sum of counts for all the BD's owned by the processor for 
 * the channel specified by the received parameter.
 *
 *
 * @param cd_p  pointer to channel descriptor
 *
 * @return
 *     - sum of count fields
 *
 */ 
int iapi_GetCountAll(channelDescriptor * cd_p)
{
   int retval = 0;
   int i = 0;
   bufferDescriptor* bd_p;
   
   bd_p = cd_p->ccb_ptr->baseBDptr;
   
   while((i < cd_p->bufferDescNumber) && ((bd_p->mode.status & BD_DONE) == 0))
   {
      retval += bd_p->mode.count;
      i++;
      bd_p++; 
   }
   return retval;
}
