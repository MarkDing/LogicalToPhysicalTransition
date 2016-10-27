Nor Flash Logical To Physical Transition
========================
## 1. Introduction  
One specific requirement on MCS8051 USB mass storage device, the storage media is a 64 Mb Nor Flash, page size is 4096 bytes. The flash working method is that it can be read by byte unit. However, it cannot write without page erase. If we erase every page before write, the write performance is extremely low. And the life time for frequently write page will be much shorter.   

Consider above disadvantage, we need to implement logical to physical address mapping method. When overwrite the existing page, the data can be written into new free physical page without erasing existing one, just update logical to physical address mapping array. That helps greatly on read/write performance. And at the meaning time , the wear leveling technology can be implemented here to average the write/erase times for each page.  

The logical to physical transition is popular in NAND flash for mass storage device. Compare with NAND flash, the Nor Flash advantage is that it doesn't need ECC correction, no bad block management. It is much easier to maintain the address transition and wear leveling.  

## 2. Design Method  
Hardware:  
* MCS8051 MCU-EFM8UB2, 16 KB flash memory, 256 bytes standard 8051 RAM, 1024 bytes on-chip XRAM.  
* Nor Flash MX25L6433F, 64Mb size, erase unit(page) is 4K bytes.  

This is resource constrain system. we cannot make all logical to physical mapping in the small size XRAM. What we should do is that divide whole flash into several regions. Each region keeps a logical block to physical block mapping address. The block size should be same as erase unit size, which is 4K bytes. The 4K bytes is still much bigger than MCU XRAM, we divide the block into several sectors. The trade off result is 128 bytes per sector.  

Detail information about the memory layout.  

1. There are 16 regions, 512K bytes per region.  
2. There are 512K / 4K = 128 blocks per region, including 126 data blocks and 2 exchange blocks for data exchange buffer usage.   
3. There are 4096 / 128 = 32 sectors per block, including 31 data sector and 1 information sector.   
4. The information sector structure  
a. Relative logical block number (0-125) within a region, 2 bytes size.  
  The logical to physical mapping array is for a region, when try to read/write another region, we need to rebuild the logical to physical map for the new region.   
b. Erase counter which record how many times the block has been erased, 4 bytes size.  
  When write data into block, the find free block function will pick the lowest erase times block which helps average the write/erase times, that is basic idea of the wear leveling technology.  
c. Relative sector index logical value(0-30), 1 byte for each sector, total 31 bytes.  
  When first write logical sector 4, it update offset 0 to 4. and then write logical sector 3, and update offset 1 to 3. And then write logical sector 4 again, then update offset 2 to 4. The latest and valid logical sector 4 data are in the physical sector 2 within the block.  


![Memory Layout][mem-layout]  
__Figure 1 Memory Layout__  

## 3. Detailed List of Functions
This section discusses the detailed function declarations in the Nor FLash logical to physical transition firmware example.  

### 3.1 Write Sector
Write a sector data into flash driver.  
__Syntax__  
int8_t writeSector(uint32_t sec, uint8_t *buf)  
__Prameters__  
sec: Absolute sector number  
buf: Pointer to sector data buffer  
__Return Value__  
 0:  Success  
-1:  Failure  
__Description__  
The function first check the sector number within the boundary. And then calculate the block number. Call logical block to physical block transition function to get the physical block number. And then find free sector within this physical block. Calculate physical address and write sector data into it.  

![Write Sector][writeSector]  
__Figure 2 Write Sector__  


### 3.2 Read Sector
Read a sector data from flash driver.  
__Syntax__  
int8_t readSector(uint32_t sec, uint8_t *buf)  
__Prameters__  
sec: Absolute sector number  
buf: Pointer to sector data buffer  
__Return Value__  
 0:  Success  
-1:  Failure  
__Description__  
The function first checks the sector number within the boundary. And then calculate the block number. Call logical block to physical block transition function to get the physical block number. If the sector is already mapped, get physical sector number from mapping array, if not then find free sector within this physical block. Calculate physical address and read data from the physical sector.  

![Read Sector][readSector]  
__Figure 3 Read Sector__  


### 3.3 Logical to Physical Block Transition  
Logical block address(lba) convert to physical block address(pba).  
__Syntax__  
static void lba2pba(uint16_t logBlk, struct driverInfo *drv)  
__Prameters__  
logBlk: Logical block number  
drv: Pointer to the driver information structure  
__Return Value__  
None  
__Description__  
The function first checks region number. If the region number is different, that means the current region mapping is not in the ram. Call discover logical block to physical block function to build the mapping array. Get physical block number from the map. If there is no physical block corresponds to the logical block number, then find a new block. Build up sector mapping within the founded physical block. and update region current block number with physical block number.  


![LBA2PBA][lba2pba]  
__Figure 4 LBA2PBA__  



### 3.4 Discover Logical to Physical Block map  
Discover logical to physical block map within region.  
__Syntax__  
static void discoverLba2pbaTable(struct driverInfo *drv)  
__Prameters__  
drv: Pointer to the driver information structure  
__Return Value__  
None  
__Description__  
The function read every physical block's info sector to get logical block number within a region, and build a logical to physical block mapping array. If there are two or more physical blocks map to same logical block number, combine them into new block.  


![Discover LBA2PBA Table][discoverLba2pbaTable]  
__Figure 5 Discover LBA2PBA Table__  



### 3.5 Driver Flush  
Flush last block info into flash and build sector map for current block.  
__Syntax__  
static void driverFlush(struct driverInfo *drv)  
__Prameters__  
drv: Pointer to the driver information structure  
__Return Value__  
None  
__Description__  
This function check if the block number is changed. If previous block has only one block map to the logical block, then simple discover sector mapping array in current block. If the previous blocks has two physical blocks map to same logical block, then we need to merge them into a new block. and then discover sector mapping array in current.  

![Driver Flush][driverFlush]  
__Figure 6 Driver Flush__  



### 3.6 Build Block Information  
Build block information with two given blocks. And update block information with sector mapping information.  
__Syntax__  
static void buildBlkInfo(uint16_t blk0, uint16_t blk1, struct blockInfo *blkInfo)  
__Prameters__  
blk0: Physical block number for first block  
blk1: Physical block number for second block  
blkInfo: Pointer to the block information structure  
__Return Value__  
None  
__Description__  
This function determine which block is current block by checking last sector index value in information sector. If the last sector index has valid value that means the block is full, and set it as previous block, the other block set as current block. Discover previous block sector index mapping array. And then discover current block sector index mapping array. Update structure blkInfo members with all information we get.  

![Build Block Info][buildBlkInfo]  
__Figure 7 Build Block Info__  



### 3.7 Find Free Block  
Find a free block within a region.  
__Syntax__  
static uint16_t findFreeBlock(struct driverInfo *drv)  
__Prameters__  
drv: Pointer to the driver information structure  
__Return Value__  
The new physical block number  
__Description__  
This function search every block info sector within a region. Find a free block with minimum erase times. That is wear leveling.   

![Find Free Block][findFreeBlock]  
__Figure 8 Find Free Block__  



### 3.8 Find Free Sector  
Find free sector for given the relative logical sector.  
__Syntax__  
static uint8_t findFreeSector(uint8_t rls, struct driverInfo *drv)  
__Prameters__  
rls: Relative logic sector number.  
drv: Pointer to the driver information structure  
__Return Value__  
The new physical block number  
__Description__  
This function checks current block. If current block is full, and if there is previous block, then find a new free block and combine two blocks into the new free blocks. If the new block is full after merging, then find new block and update block information structure. Get relative physical sector number with currSec, update Nor flash info sector sector index. And update block info structure blkMap and currSec.  


![Find Free Sector][findFreeSector]  
__Figure 9 Find Free Sector__  



### 3.9 Merge Two Blocks  
Merge two blocks to new block.  
__Syntax__  
static void mergeTwoBlocks(struct driverInfo *drv, uint16_t blk)  
__Prameters__  
drv: Pointer to the driver information structure  
blk: Destination block number.  
__Return Value__  
None  
__Description__  
This function gets valid sector data and copy them into the destinate block. It check if the logical sector number is mapped in current block, if no then check if in previous block. Copy the source data to destinate address. Repeat above steps until all contents has been copied into destinate block. Erase previous and current block. Build sector index map and update logical number into info sector of the new block.  
  
    
![Merge Two Blocks][mergeTwoBlocks]  
__Figure 10 Merge Two Blocks__  



## 4. Source Code  
There is Virtual Studio 2013 project included for logical to physical transition simulation. Using a file to act as Nor Flash. File read/write in low level Nor flash read/write function. It can be easily porting to any other platform.  






[buildBlkInfo]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/buildBlkInfo.png
[discoverLba2pbaTable]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/discoverLba2pbaTable.png
[driverFlush]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/driverFlush.png
[findFreeBlock]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/findFreeBlock.png
[findFreeSector]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/findFreeSector.png
[lba2pba]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/lba2pba.png
[mem-layout]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/mem-layout.png
[mem-layout]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/mem-layout.png
[mergeTwoBlocks]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/mergeTwoBlocks.png
[readSector]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/readSector.png
[writeSector]:https://raw.github.com/MarkDing/LogicalToPhysicalTransition/master/images/writeSector.png

