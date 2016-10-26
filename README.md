Nor Flash Logical To Physical Transition
========================
## 1. Introduction
One specific requirement on MCS8051 USB mass storage device, the storage media is a 64 Mb Nor Flash, page size is 4096 bytes. The flash working method is that it can be read by byte unit. However, it cannot write without page erase. If we erase every page before write, the write performance is extremely low. And the life time for frequently write page will be much shorter. 

Consider above disadvantage, we need to implement logical to physical address mapping method. When overwrite the existing page, the data can be written into new free physical page without erasing existing one, just update logical to physical address mapping array. That helps greatly on read/write performance. And at the meaning time , the wear leveling technology can be implemented here to average the write/erase times for each page.

The logical to physical transition is popular in NAND flash for mass storage device. Compare with NAND flash, the Nor Flash advantage is that it doesn't need ECC correction, no bad block management. It is much easier to maintain the address transition and wear leveling. 

## 2. Design Method
The hardware:
* MCS8051 MCU-EFM8UB2, 16 KB flash memory, 256 bytes standard 8051 RAM, 1024 bytes on-chip XRAM.
* Nor Flash MX25L6433F, 64Mb size, erase unit(page) is 4K bytes. 

This is resource constrain system. we cannot make all logical to physical mapping in the small size XRAM. What we should do is that divide whole flash into several regions. Each region keeps a logical block to physical block mapping address. The block size should be same as erase unit size, which is 4K bytes. The 4K bytes is still much bigger than MCU XRAM, we divide the block into several sectors. The trade off result is 128 bytes per sector. 

Detail information about the memory layout. 

1. There are 16 regions, 512K bytes per region.

2. There are 512K / 4K = 128 blocks per region, including 126 data blocks and 2 exchange blocks for data exchange buffer usage. 

3. There are 4096 / 128 = 32 sectors per block, including 31 data sector and 1 information sector. 

4. The information sector structure

* Relative logical block number (0-125) within a region, 2 bytes size.

  The logical to physical mapping array is for a region, when try to read/write another region, we need to rebuild the logical to physical map for the new region. 

* Erase counter which record how many times the block has been erased, 4 bytes size.  

  When write data into block, the find free block function will pick the lowest erase times block which helps average the write/erase times, that is basic idea of the wear leveling technology.  

* Relative sector index logical value(0-30), 1 byte for each sector, total 31 bytes.

  When first write logical sector 4, it update offset 0 to 4. and then write logical sector 3, and update offset 1 to 3. And then write logical sector 4 again, then update offset 2 to 4. The latest and valid logical sector 4 data are in the physical sector 2 within the block. 

![Memory Layout][mem-layout]

__Figure 1 Memory Layout__


## 3. Coding Implementation

## 4. Source Code


![Build Block Info][buildBlkInfo]

__Figure 1 Build Block Info__

![Discover LBA2PBA Table][discoverLba2pbaTable]

__Figure 2 Discover LBA2PBA Table__

![Driver Flush][driverFlush]

__Figure 3 Driver Flush__

![Find Free Block][findFreeBlock]

__Figure 4 Find Free Block__

![Find Free Sector][findFreeSector]

__Figure 5 Find Free Sector__

![LBA2PBA][lba2pba]

__Figure 6 LBA2PBA__



![Merge Two Blocks][mergeTwoBlocks]

__Figure 8 Merge Two Blocks__

![Read Sector][readSector]

__Figure 9 Read Sector__

![Write Sector][writeSector]

__Figure 10 Write Sector__


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

