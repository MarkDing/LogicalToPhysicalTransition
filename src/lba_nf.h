#ifndef __LBA_NF_H__
#define __LBA_NF_H__

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long uint32_t;

typedef signed char int8_t;
typedef short int16_t;
typedef long int32_t;

/* MX25L6433F, 3V, 64Mb*/
#define NF_PAGE_SIZE 4096
#define NF_NUM_PAGES 2048
#define NF_SIZE (NF_PAGE_SIZE * NF_NUM_PAGES)

#define NF_INIT_VALUE (-1)

/******************************************************************************
  128 bytes per sector, 
  32 sectors per block(page), 1 sector for relative sector index 
  128 blocks per region, 2 blocks for inter transfer usage.
  16 regions per Nor flash.
******************************************************************************/
#define SECTOR_SIZE 128
#define NUM_PHY_SECS_BLK (NF_PAGE_SIZE / SECTOR_SIZE)
#define NUM_PHY_BLKS_RGN 128
// #define NUM_PHY_BLKS_RGN 32

#define NUM_SECS_BLK (NUM_PHY_SECS_BLK - 1)
/* 
 * Reserve two blocks, one for two blocks map to same logical block, the other one 
 * is for new block in merge block procedure. 
 */
#define NUM_BLKS_RGN (NUM_PHY_BLKS_RGN - 2)
#define NUM_SECS_RGN (NUM_BLKS_RGN * NUM_SECS_BLK)

#define PHY_BLK_SIZE (SECTOR_SIZE * NUM_PHY_SECS_BLK)
#define LOG_BLK_SIZE (NUM_SECS_BLK * SECTOR_SIZE)

#define NUM_REGIONS  16
// #define NUM_REGIONS  1

/******************************************************************************
  Info sector structure:
  Logical block number: 2 bytes, it is relative block number in a region
  Erase count: 4 bytes
  Relative sector index: 1 byte each
******************************************************************************/
#define LBA_OFFSET 0
#define ERASE_CNT_OFFSET (LBA_OFFSET + 2)
#define SEC_IDX_OFFSET (ERASE_CNT_OFFSET + 4)

/******************************************************************************
  Function propertype
******************************************************************************/


/***************************************************************************//**
 * @brief     Initialize all drive related variables
 ******************************************************************************/
void driveInit();

/***************************************************************************//**
 * @brief      Drive test function. 
 ******************************************************************************/
void driveTest();

/***************************************************************************//**
 * @brief      Write a sector data into driver
 *
 * @param[in]  sec   Logical sector number 
 * @param      buf   Data buffer to be wrote
 *
 * @return     0: success, -1: false
 ******************************************************************************/
int8_t writeSector(uint32_t sec, uint8_t *buf);

/***************************************************************************//**
 * @brief      Read a sector data from driver
 *
 * @param[in]  sec   Logical sector number
 * @param      buf   Pointer to buffer of data to be read
 *
 * @return     return 0: success, -1: false
 ******************************************************************************/
int8_t readSector(uint32_t sec, uint8_t *buf);

#endif // __LBA_NF_H__