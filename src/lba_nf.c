/**************************************************************************//**
 * Copyright 2016 by Silicon Laboratories Inc. All rights reserved.
 *
 * http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
 *****************************************************************************/
#include <stdio.h>
#include <memory.h>
#include "lba_nf.h"


uint16_t regionMap[NUM_BLKS_RGN]; //!< logical to physical block map array
uint8_t blockMap[NUM_SECS_BLK]; //!< sector map array of current block. 
uint8_t previousBlockMap[NUM_SECS_BLK]; //!< sector map array of previous block

uint8_t sectorBuf[SECTOR_SIZE]; //!< sector buffer used by merge block function

struct blockInfo{
  uint8_t currSec;  //!< current free sector for writing
  uint16_t currBlk; //!< current block number
  uint16_t prevBlk; //!< The previous block number
  uint8_t *blkMap;  //!< The pointer to sector map of current block 
  uint8_t *prevBlkMap; //!< The pointer sector map of previous block
};

struct regionInfo{
  uint16_t currBlk; //!< current active block in a region
  uint16_t currRlb; //!< current relative logical block number in a region
  uint16_t *rgnMap; //!< The pointer to logical to physical block map array
  struct blockInfo blkInfo; //!< block information structure
};

struct driveInfo{
   uint16_t numRgns; //!< Numbers of region a driver
   uint16_t numBlks; //!< Numbers of blocks a driver
   uint32_t numSecs; //!< Numbers of sectors a driver
   uint16_t currRgn; //!< Current active region number of a driver
   struct regionInfo rgnInfo; //!< Region information structure
};


struct driveInfo drvInfo; //!< Driver information structure

// Internal function 
static void nf_writePage(uint32_t addr, uint8_t *buf);
static void nf_readPage(uint32_t addr, uint8_t *buf);
static void nf_eraseBlock(uint32_t addr);
static void nf_writeBytes(uint32_t addr, uint8_t *buf, uint16_t size);
static void nf_readBytes(uint32_t addr, uint8_t *buf, uint16_t size);
static void discoverLsa2psaTable(struct driveInfo *drv);
static void discoverLba2pbaTable(struct driveInfo *drv);
static uint8_t findFreeSector(uint8_t rls, struct driveInfo *drv);
static void mergeTwoBlocks(struct driveInfo *drv, uint16_t blk);
static void lba2pba(uint16_t logBlk, struct driveInfo* drv);
static uint16_t findFreeBlock(struct driveInfo *drv);
static void buildBlkInfo(uint16_t blk0, uint16_t blk1, struct blockInfo *blkInfo);
static void eraseBlock(uint16_t blk);
static void resetBlockInfo(struct blockInfo *blkInfo);
static void driverFlush(struct driveInfo *drv);
static void writeRlb(uint16_t phyBlk, uint16_t rlb, struct driveInfo *drv);

void nfTest()
{
  uint8_t tmp[SECTOR_SIZE];
  memset(tmp, 0x5a, SECTOR_SIZE);
  nf_writePage(0x200, tmp);
  nf_readPage(0x400, tmp);
  nf_readPage(0x200, tmp);

  nf_writePage(NF_PAGE_SIZE, tmp);
  nf_eraseBlock(0x400);
  nf_readPage(0x200, tmp);
  nf_readPage(NF_PAGE_SIZE, tmp);
}

void createNfImage()
{
  unsigned int imgSize = NF_SIZE;
  int i;
  unsigned char buf[NF_PAGE_SIZE];
  FILE * outf;

  outf = fopen("nf_img.bin", "wb+");
  memset(buf, 0xff, NF_PAGE_SIZE);
  for (i = 0; i < NF_NUM_PAGES; i++) 
  {
    fwrite(buf, NF_PAGE_SIZE, 1, outf);
  }
  fclose(outf);
}

void writeTest(uint32_t sec) 
{
  uint16_t tmp[SECTOR_SIZE];
  int32_t i;
  for (i = 0; i < (SECTOR_SIZE / 2); i++) 
  {
    tmp[i] = (uint16_t)sec;
  }
  writeSector(sec, tmp);
}

void readTest(uint32_t sec)
{
  uint16_t tmp[SECTOR_SIZE];

  readSector(sec, tmp);
  if (tmp[0] != (uint16_t)sec) 
  {
    while(1);
  }
}

uint8_t testBuf[128];

void writeSectorTest(uint32_t sec, uint32_t size)
{
  int32_t i;
  uint32_t nfSec, nfSecNum;
  nfSec = sec * 4;
  nfSecNum = size / 128;
  for (i = 0; i < nfSecNum; i++)
  {
	  memset(testBuf, i % 31 , 128);
	  writeSector(nfSec + i, testBuf);
  }
}

void readSectorTest(uint32_t sec, uint32_t size)
{
  int32_t i;
  uint32_t nfSec, nfSecNum;
  nfSec = sec * 4;
  nfSecNum = size / 128;
  for (i = 0; i < nfSecNum; i++)
  {
    readSector(nfSec+i, testBuf);
    if(testBuf[5] != (uint8_t)(i%31))
    {
		testBuf[5] = 0x5a;
    }
  }
}

void formatTest(void)
{
  writeSectorTest(0, 0x200);
  readSectorTest(0, 0x200);
  writeSectorTest(0x3d07, 0x200);
  readSectorTest(0x3d07, 0x200);
  writeSectorTest(0x08, 0x1800);
  readSectorTest(0x08, 0x1800);
  writeSectorTest(0x14, 0x1800);
  readSectorTest(0x14, 0x1800);
  writeSectorTest(0x20, 0x4000);
  readSectorTest(0x20, 0x4000);
  writeSectorTest(0, 0x1000);
  readSectorTest(0, 0x1000);
  writeSectorTest(0, 0x2000);
  readSectorTest(0, 0x2000);
  writeSectorTest(0, 0x200);
  readSectorTest(0, 0x200);
  while(1);
}

void driveTest()
{
  int32_t i;
  uint8_t tmp[SECTOR_SIZE];
  //nfTest();
  driveInit();
#if 1
  createNfImage(); // Create img file to simulate Nor Flash
#endif
  formatTest();
#if 0  
  for (i = 0; i < NUM_SECS_BLK; i++) 
  {
    writeTest(i);
  }
  writeTest(4);
  writeTest(7);
  writeTest(306);
  for (i = 0; i < NUM_SECS_BLK; i++) 
  {
    readTest(i);
  }
  readTest(306);
#endif  
  while(1);
}             

/***************************************************************************//**
 * @brief      Initialize all drive related variables
 ******************************************************************************/
void driveInit()
{
  int16_t i;

  drvInfo.numRgns = NUM_REGIONS;
  drvInfo.currRgn = NF_INIT_VALUE;
  drvInfo.numBlks = NUM_BLKS_RGN * NUM_REGIONS;
  drvInfo.numSecs = NUM_SECS_BLK * NUM_BLKS_RGN * NUM_REGIONS;

  drvInfo.rgnInfo.currBlk = NF_INIT_VALUE;
  drvInfo.rgnInfo.currRlb = NF_INIT_VALUE;
  drvInfo.rgnInfo.rgnMap = regionMap;
  for (i = 0; i < NUM_BLKS_RGN; i++) 
  {
    drvInfo.rgnInfo.rgnMap[i] = NF_INIT_VALUE;
  }

  drvInfo.rgnInfo.blkInfo.blkMap = blockMap;
  drvInfo.rgnInfo.blkInfo.prevBlkMap = previousBlockMap;
  resetBlockInfo(&drvInfo.rgnInfo.blkInfo);
}

/***************************************************************************//**
 * Write a sector data into driver
 ******************************************************************************/
int8_t writeSector(uint32_t sec, uint8_t *buf)
{
  uint32_t addr, phySec;
  uint8_t rls, rps;
  uint16_t logBlk;

  if (sec >= drvInfo.numSecs)
  {
    return -1;
  }

  // the region currBlk will be updated with physical block number
  logBlk = (uint16_t)(sec / NUM_SECS_BLK);
  lba2pba(logBlk, &drvInfo);

  // Find free sector for relative sector index.  
  rls = sec % NUM_SECS_BLK;
  rps = findFreeSector(rls, &drvInfo);

  phySec = (uint32_t)drvInfo.rgnInfo.currBlk * NUM_PHY_SECS_BLK + rps;

  addr = phySec * SECTOR_SIZE;
  nf_writePage(addr, buf);
  return 0;
}

/***************************************************************************//**
 * Read a sector data from driver
 ******************************************************************************/
int8_t readSector(uint32_t sec, uint8_t *buf)
{
  // relative logical sector(rls) and relative physical sector(rps)
  uint8_t rls, rps;
  uint16_t logBlk, phyBlk;
  uint32_t addr, phySec;

  if (sec >= drvInfo.numSecs)
  {
    return -1;
  }

  // the region currBlk will be updated with physical block number
  logBlk = (uint16_t)(sec / NUM_SECS_BLK);
  lba2pba(logBlk, &drvInfo);

  rls = sec % NUM_SECS_BLK;
  rps = drvInfo.rgnInfo.blkInfo.blkMap[rls];

  if (rps == (uint8_t)NF_INIT_VALUE) 
  {
    rps = drvInfo.rgnInfo.blkInfo.prevBlkMap[rls];
    // The target block number is prev block number
    if (rps != (uint8_t)NF_INIT_VALUE) 
    {
      phyBlk = drvInfo.rgnInfo.blkInfo.prevBlk;
    } // The target block number is new finding block number 
    else 
    {
      rps = findFreeSector(rls, &drvInfo);
      phyBlk = drvInfo.rgnInfo.currBlk;
    }
  }
  else 
  {
    phyBlk = drvInfo.rgnInfo.currBlk;
  }

  phySec = (uint32_t)phyBlk * NUM_PHY_SECS_BLK + rps;
  addr = phySec * SECTOR_SIZE;
  nf_readPage(addr, buf);
  return 0;
}

/***************************************************************************//**
 * @brief      Nor flash page write fucntion
 *
 * @param[in]  addr  The write address
 * @param      buf   Pointer to data buffer to be wrote
 ******************************************************************************/
static void nf_writePage(uint32_t addr, uint8_t *buf)
{
  nf_writeBytes(addr, buf, SECTOR_SIZE);
}

/***************************************************************************//**
 * @brief      Nor flash page read function
 *
 * @param[in]  addr  The write address
 * @param      buf   Pointer to data buffer to be read
 ******************************************************************************/
static void nf_readPage(uint32_t addr, uint8_t *buf)
{
  nf_readBytes(addr, buf, SECTOR_SIZE);
}

/***************************************************************************//**
 * @brief      Nor flash block erase function
 *
 * @param[in]  addr  The erase address
 ******************************************************************************/
static void nf_eraseBlock(uint32_t addr)
{
  uint8_t tmp[NF_PAGE_SIZE];

  memset(tmp, 0xFF, NF_PAGE_SIZE);
  addr = addr & ~(NF_PAGE_SIZE - 1);
  nf_writeBytes(addr, tmp, NF_PAGE_SIZE);
}

/***************************************************************************//**
 * @brief      Nor flash bytes write function
 *
 * @param[in]  addr  The write address
 * @param      buf   Pointer to data buffer to be wrote
 * @param[in]  size  The bytes of data
 ******************************************************************************/
static void nf_writeBytes(uint32_t addr, uint8_t *buf, uint16_t size)
{
  FILE * outf;

  outf = fopen("nf_img.bin", "rb+");
  fseek(outf, addr, SEEK_SET);
  fwrite(buf, size, 1, outf);
  fclose(outf);
}

/***************************************************************************//**
 * @brief      Nor flash bytes read fucntion
 *
 * @param[in]  addr  The read address
 * @param      buf   Pointer to data buffer to be read
 * @param[in]  size  The bytes of data
 ******************************************************************************/
static void nf_readBytes(uint32_t addr, uint8_t *buf, uint16_t size)
{
  FILE * inf;

  inf = fopen("nf_img.bin", "rb+");
  fseek(inf, addr, SEEK_SET);
  fread(buf, size, 1, inf);
  fclose(inf);
}

/***************************************************************************//**
 * @brief      Read sector index information from information sector and updated
 *             to map array. It returns when found first NF_INIT_VALUE
 *
 * @param[in]  addr  The start address of sector index.
 * @param      map   The map array, the index is relative logical sector number,
 *                   the value is physical sector number.
 *
 * @return     the current sector index which value is NF_INIT_VALUE
 ******************************************************************************/
static uint8_t readSector_map(uint32_t addr, uint8_t *map)
{
  uint8_t i, tmp;

  for (i = 0; i < NUM_SECS_BLK; i++) 
  {
    nf_readBytes(addr + i, &tmp, 1);
    if (tmp < NUM_SECS_BLK) 
    {
      map[tmp] = i;
    }
    if (tmp == (uint8_t)NF_INIT_VALUE) 
    {
      break;
    }
  }
  return i;
}

/***************************************************************************//**
 * @brief      Build block information with two given blocks. and update block
 *             information with sector mapping information.
 *
 * @param[in]  blk0     The first block number
 * @param[in]  blk1     The second block number
 * @param      blkInfo  The block information structure which will be fully 
 *                      updated by reading two blocks sector mapping information.
 ******************************************************************************/
static void buildBlkInfo(uint16_t blk0, uint16_t blk1, struct blockInfo *blkInfo)
{
  uint8_t tmp;
  uint32_t addr;

  resetBlockInfo(blkInfo);

  // If last byte of sector index  
  addr = (blk0 + 1) * PHY_BLK_SIZE - SECTOR_SIZE + SEC_IDX_OFFSET 
        + NUM_SECS_BLK - 1;
  nf_readBytes(addr, &tmp, 1);

  // it is NF_INIT_VALUE. it should be current block
  if (tmp == (uint8_t)NF_INIT_VALUE) 
  {
    blkInfo->prevBlk = blk1;
    blkInfo->currBlk = blk0;
  } // otherwise, it should be previous block
  else 
  {
    blkInfo->prevBlk = blk0;
    blkInfo->currBlk = blk1;
  }

  addr = (blkInfo->prevBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE + SEC_IDX_OFFSET;
  readSector_map(addr, blkInfo->prevBlkMap);

  addr = (blkInfo->currBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE + SEC_IDX_OFFSET;
  blkInfo->currSec = readSector_map(addr, blkInfo->blkMap);
}

/***************************************************************************//**
 * @brief      Discover the sector mapping in region current block. Updated 
 *             block information accordingly 
 *
 * @param      drv   Pointer to the driver information structure
 ******************************************************************************/
void discoverLsa2psaTable(struct driveInfo *drv)
{
  struct blockInfo *blkInfo = &drv->rgnInfo.blkInfo;
  uint32_t addr;

  resetBlockInfo(blkInfo);
  blkInfo->currBlk = drv->rgnInfo.currBlk;
  addr = (blkInfo->currBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE + SEC_IDX_OFFSET;
  blkInfo->currSec = readSector_map(addr, blkInfo->blkMap);
}

/***************************************************************************//**
 * @brief      Find free sector for given the relative logical sector.
 *
 * @param[in]  rls   The relative logical sector number
 * @param      drv   The pointer to driver information structure. 
 *
 * @return     relative physical sector number (rps)
 * 
 * Detailed description: First, it checks current block. If current block is
 * full, and if there is previous block, then find a new free block and combine
 * two blocks into the new free blocks. If the new block is full after merging,
 * then find new block and update block information structure. Get relative 
 * physical sector number with currSec, update Nor flash info sector sector index.
 * And update block info structure blkMap and currSec. 
 ******************************************************************************/
static uint8_t findFreeSector(uint8_t rls, struct driveInfo *drv)
{
  struct blockInfo *blkInfo = &drv->rgnInfo.blkInfo;
  int8_t i;
  uint8_t rps;
  uint16_t phyBlk;
  uint32_t addr;

  phyBlk = blkInfo->currBlk;

  // There is free sector in current block 
  if (blkInfo->currSec >= NUM_SECS_BLK) 
  {
    // Two blocks has same logical block address, merge is needed 
    if (blkInfo->prevBlk != (uint16_t)NF_INIT_VALUE) 
    {
      phyBlk = findFreeBlock(drv);
      mergeTwoBlocks(drv, phyBlk);
    }
    // Find a new free block, copy mapping arrary 
    if (blkInfo->currSec >= NUM_SECS_BLK) 
    {
      phyBlk = findFreeBlock(drv);
      writeRlb(phyBlk, drv->rgnInfo.currRlb, drv);
      blkInfo->currSec = 0;
      blkInfo->prevBlk = blkInfo->currBlk;
      blkInfo->currBlk = phyBlk;
      for (i = 0; i < NUM_SECS_BLK; i++) 
      {
        blkInfo->prevBlkMap[i] = blkInfo->blkMap[i];
        blkInfo->blkMap[i] = (uint8_t)NF_INIT_VALUE;
      }
    }
  }
  rps = blkInfo->currSec++;
  // Update sector index table 
  addr = (phyBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE + SEC_IDX_OFFSET + rps;
  nf_writeBytes(addr, &rls, 1);
  blkInfo->blkMap[rls] = rps;
  return rps;
}

/***************************************************************************//**
 * @brief      Erase Nor flash and update erase counter in info sector.  
 *
 * @param[in]  blk   The block number to be erased. 
 ******************************************************************************/
static void eraseBlock(uint16_t blk)
{
  uint32_t des,cnt;

  des = (blk + 1) * PHY_BLK_SIZE - SECTOR_SIZE + ERASE_CNT_OFFSET;
  nf_readBytes(des, (uint8_t *)&cnt, 4);
  cnt++;
  nf_eraseBlock(des);
  nf_writeBytes(des, (uint8_t *)&cnt, 4);
}

/***************************************************************************//**
 * @brief      Simply initialize block information structure members. 
 *
 * @param      blkInfo  The pointer to block information structure. 
 ******************************************************************************/
static void resetBlockInfo(struct blockInfo *blkInfo)
{
  uint8_t i;
  blkInfo->currSec = NF_INIT_VALUE;
  blkInfo->currBlk = NF_INIT_VALUE;
  blkInfo->prevBlk = NF_INIT_VALUE;
  for (i = 0; i < NUM_SECS_BLK; i++) 
  {
    blkInfo->blkMap[i] = NF_INIT_VALUE;
    blkInfo->prevBlkMap[i] = NF_INIT_VALUE;
  }
}

/***************************************************************************//**
 * @brief      Merge two blocks to new block.
 *
 * @param      drv   The pointer to driver information structure 
 * @param[in]  blk   The destination block number. 
 * 
 * Detailed description: Using relative sector index, first check current block 
 * map contains physical sector index or not, if not then check previous block 
 * map. If there is a mapping relation then copy the source sector into 
 * destination block, and update information sector accordingly. Once all sectors
 * have been copied into new block, erase previous and current block and build 
 * sector map and update logical number in the new block. 
 * 
 ******************************************************************************/
static void mergeTwoBlocks(struct driveInfo *drv, uint16_t blk)
{
  struct blockInfo *blkInfo = &drv->rgnInfo.blkInfo;
  int8_t i;
  uint8_t cnt = 0;
  uint16_t rlb;
  uint32_t src, des, info;

  des = blk * PHY_BLK_SIZE;
  info = (blk + 1) * PHY_BLK_SIZE - SECTOR_SIZE + SEC_IDX_OFFSET;
  for (i = 0; i < NUM_SECS_BLK; i++) 
  { 
    if (blkInfo->blkMap[i] != (uint8_t)NF_INIT_VALUE) 
    {
      src = blkInfo->currBlk * PHY_BLK_SIZE + blkInfo->blkMap[i] * SECTOR_SIZE;
    }
    else if (blkInfo->prevBlkMap[i] != (uint8_t)NF_INIT_VALUE) 
    {
      src = blkInfo->prevBlk * PHY_BLK_SIZE + blkInfo->prevBlkMap[i] 
          * SECTOR_SIZE;
    }
    else 
    {
      continue;
    }
    nf_readPage(src, sectorBuf);
    nf_writePage(des, sectorBuf);
    // Update info sector with relative sector index
    nf_writeBytes(info++, &i, 1);
    des += SECTOR_SIZE;
  }
  // Read relative logical block number 
  src = (blkInfo->currBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE;
  nf_readBytes(src, (uint8_t *)&rlb, 2);

  eraseBlock(blkInfo->currBlk);
  eraseBlock(blkInfo->prevBlk);

  writeRlb(blk, rlb, drv);
  // The block mapping table need to be created with new block.
  discoverLsa2psaTable(drv);
}


/***************************************************************************//**
 * @brief      If the block number change, combine two blocks or discover sector
 * map in the new block. 
 *
 * @param      drv   The pointer to driver information structure. 
 ******************************************************************************/
static void driverFlush(struct driveInfo *drv)
{
  struct blockInfo *blkInfo = &drv->rgnInfo.blkInfo;
  uint16_t phyBlk, tmp;

  if (drv->rgnInfo.currBlk != blkInfo->currBlk) 
  {
    // There are two blocks mapping to same logical block need to be merged 
    if (blkInfo->prevBlk != (uint16_t)NF_INIT_VALUE) 
    {
      phyBlk = findFreeBlock(drv);
      /* Save current block of region, since merge block will change the value*/
      tmp = drv->rgnInfo.currBlk;
      mergeTwoBlocks(drv, phyBlk);
      drv->rgnInfo.currBlk = tmp;
    }
    discoverLsa2psaTable(drv);
  }
}

/***************************************************************************//**
 * @brief      Logical block address(lba) convert to physical block address(pba)
 *
 * @param[in]  logBlk  The logical block number
 * @param      drv     The pointer to the driver information structure
 * 
 * Detailed description: If the region number is difference, discover the logical
 * block to physical block map at first. if there is no physical block corresponds
 * to the logical block number, then find a new block. and update region current
 * block number with physical block number. 
 ******************************************************************************/
static void lba2pba(uint16_t logBlk, struct driveInfo *drv)
{
  uint16_t rgn, phyBlk;

  rgn = logBlk / NUM_BLKS_RGN;
  // Get relative logical block number within a region 
  drv->rgnInfo.currRlb = logBlk % NUM_BLKS_RGN;

  if (rgn != drv->currRgn) 
  {
    drv->currRgn = rgn;
    discoverLba2pbaTable(drv);
  }
  phyBlk = drv->rgnInfo.rgnMap[drv->rgnInfo.currRlb];

  drv->rgnInfo.currBlk = phyBlk;
  driverFlush(drv);

  if (phyBlk == (uint16_t)NF_INIT_VALUE) 
  {
    phyBlk = findFreeBlock(drv);
    writeRlb(phyBlk, drv->rgnInfo.currRlb, drv);
    discoverLsa2psaTable(drv);
  }
}

/***************************************************************************//**
 * @brief      Discover logical to physical block map within region
 *
 * @param      drv   The pointer to driver information structure
 * 
 * Detailed description: Read every block's info sector within region, to build
 * mapping array. If there are two or more physical blocks contain same logical 
 * block number, combine them into new block. 
 * 
 ******************************************************************************/
static void discoverLba2pbaTable(struct driveInfo *drv)
{
  struct regionInfo *rgnInfo = &drv->rgnInfo;
  int16_t i;
  uint16_t tmp, startBlk, phyBlk;
  uint32_t addr;

  startBlk = drv->currRgn * NUM_PHY_BLKS_RGN;
  // the relative logical number stored in first byte of info sector
  addr = (startBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE;

  for (i = 0; i < NUM_BLKS_RGN; i++) 
  {
    rgnInfo->rgnMap[i] = NF_INIT_VALUE;
  }

  for (i = startBlk; i < (startBlk + NUM_PHY_BLKS_RGN); i++) 
  {
    nf_readBytes(addr, (uint8_t *)&tmp, 2);
    addr += PHY_BLK_SIZE;
    if (tmp < NUM_BLKS_RGN) 
    {
      if (rgnInfo->rgnMap[tmp] != (uint16_t)NF_INIT_VALUE) 
      {
        buildBlkInfo(rgnInfo->rgnMap[tmp], i, &drv->rgnInfo.blkInfo);
        phyBlk = findFreeBlock(drv);
        mergeTwoBlocks(drv, phyBlk);
      }
      else 
      {
        rgnInfo->rgnMap[tmp] = i;
      }
    }
  }
}

/***************************************************************************//**
 * @brief      Write relative logical block number into info sector
 *
 * @param[in]  phyBlk  The target physical block number
 * @param[in]  rlb     The relative logical block number
 * @param      drv     The pointer to driver information structure. 
 ******************************************************************************/
static void writeRlb(uint16_t phyBlk, uint16_t rlb, struct driveInfo *drv)
{
  uint32_t addr;

  addr = (phyBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE;
  nf_writeBytes(addr, &rlb, 2);
  drv->rgnInfo.rgnMap[rlb] = phyBlk;
  drv->rgnInfo.currBlk = phyBlk;
}

/***************************************************************************//**
 * @brief      Find a free block within a region
 *
 * @param      drv   The pointer to driver information structure
 *
 * @return     The new physical block number
 * 
 * Detailed description: Read info sector within a region, if the logical number 
 * is NF_INIT_VALUE, then read the erase counter and take the minimum erase 
 * counter block as the new block. 
 * 
 ******************************************************************************/
static uint16_t findFreeBlock(struct driveInfo *drv)
{
  int16_t i;
  uint16_t startBlk, tmp, phyBlk;
  uint32_t addr, cnt, cntMin = NF_INIT_VALUE;

  startBlk = drv->currRgn * NUM_PHY_BLKS_RGN;
  // the relative logical number stored in first byte of info sector
  addr = (startBlk + 1) * PHY_BLK_SIZE - SECTOR_SIZE;

  phyBlk = startBlk;

  for (i = startBlk; i < (startBlk + NUM_PHY_BLKS_RGN); i++) 
  {
    // find a block without logical block assignment 
    nf_readBytes(addr, (uint8_t *)&tmp, 2);
    if (tmp == (uint16_t)NF_INIT_VALUE) 
    {
      // Find the minimum erase count block 
      nf_readBytes(addr + ERASE_CNT_OFFSET, (uint8_t *)&cnt, 4);
      // Update erase counter if its value is 0xFFFFFFFF
      if (cnt == (uint32_t)NF_INIT_VALUE) 
      {
        cnt = 0;
        nf_writeBytes(addr + ERASE_CNT_OFFSET, (uint8_t *)&cnt, 4);
      }
      if (cnt < cntMin) 
      {
        cntMin = cnt;
        phyBlk = i;
        if (cnt == 0) 
        {
          break;
        }
      }
    }
    addr += PHY_BLK_SIZE;
  }

//  drv->rgnInfo.currBlk = phyBlk;

  return phyBlk;
}
