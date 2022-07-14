#include <stdint.h>
#include "stm32f1xx.h"

#define BIT0 (1 << 0)
#define BIT1 (1 << 1)
#define BIT2 (1 << 2)
#define BIT3 (1 << 3)
#define BIT4 (1 << 4)
#define BIT5 (1 << 5)
#define BIT6 (1 << 6)


// #define FLASH_KEY1 0x45670123
// #define FLASH_KEY2 0xCDEF89AB

#define FLASH_SIZE_DATA_REGISTER     0x1FFFF7E0U

#define REGISTERED_NUMBER_FLASH_SECTORS       (*((uint16_t *) FLASH_SIZE_DATA_REGISTER))

#if !defined(USING_FLASH_SECTOR_NUMBER)
  #define USING_FLASH_SECTOR_NUMBER           (REGISTERED_NUMBER_FLASH_SECTORS - 1)
#endif

#define START_FLASH_ADDRESS                   0x8000000

#define START_FLASH_STORAGE_ADDRESS           (START_FLASH_ADDRESS + USING_FLASH_SECTOR_NUMBER * 1024) 





uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);
int  writeSector(const volatile void *flash_ptr, const void *data,uint16_t data_size);
int eraseSector(const volatile void *flash_ptr);
void readSector(const volatile void *flash_ptr, void *data, uint16_t _size);
void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);


/* 
page 0 0x8000000 
page 1 0x8000400 
page 2 0x8000800 
page 3 0x8000c00 
page 4 0x8001000 
page 5 0x8001400 
page 6 0x8001800 
page 7 0x8001c00 
page 8 0x8002000 
page 9 0x8002400 
page 10 0x8002800 
page 11 0x8002c00 
page 12 0x8003000 
page 13 0x8003400 
page 14 0x8003800 
page 15 0x8003c00 
page 16 0x8004000 
page 17 0x8004400 
page 18 0x8004800 
page 19 0x8004c00 
page 20 0x8005000 
page 21 0x8005400 
page 22 0x8005800 
page 23 0x8005c00 
page 24 0x8006000 
page 25 0x8006400 
page 26 0x8006800 
page 27 0x8006c00 
page 28 0x8007000 
page 29 0x8007400 
page 30 0x8007800 
page 31 0x8007c00 
page 32 0x8008000 
page 33 0x8008400 
page 34 0x8008800 
page 35 0x8008c00 
page 36 0x8009000 
page 37 0x8009400 
page 38 0x8009800 
page 39 0x8009c00 
page 40 0x800a000 
page 41 0x800a400 
page 42 0x800a800 
page 43 0x800ac00 
page 44 0x800b000 
page 45 0x800b400 
page 46 0x800b800 
page 47 0x800bc00 
page 48 0x800c000 
page 49 0x800c400 
page 50 0x800c800 
page 51 0x800cc00 
page 52 0x800d000 
page 53 0x800d400 
page 54 0x800d800 
page 55 0x800dc00 
page 56 0x800e000 
page 57 0x800e400 
page 58 0x800e800 
page 59 0x800ec00 
page 60 0x800f000 
page 61 0x800f400 
page 62 0x800f800 
page 63 0x800fc00 
page 64 0x8010000 
page 65 0x8010400 
page 66 0x8010800 
page 67 0x8010c00 
page 68 0x8011000 
page 69 0x8011400 
page 70 0x8011800 
page 71 0x8011c00 
page 72 0x8012000 
page 73 0x8012400 
page 74 0x8012800 
page 75 0x8012c00 
page 76 0x8013000 
page 77 0x8013400 
page 78 0x8013800 
page 79 0x8013c00 
page 80 0x8014000 
page 81 0x8014400 
page 82 0x8014800 
page 83 0x8014c00 
page 84 0x8015000 
page 85 0x8015400 
page 86 0x8015800 
page 87 0x8015c00 
page 88 0x8016000 
page 89 0x8016400 
page 90 0x8016800 
page 91 0x8016c00 
page 92 0x8017000 
page 93 0x8017400 
page 94 0x8017800 
page 95 0x8017c00 
page 96 0x8018000 
page 97 0x8018400 
page 98 0x8018800 
page 99 0x8018c00 
page 100 0x8019000 
page 101 0x8019400 
page 102 0x8019800 
page 103 0x8019c00 
page 104 0x801a000 
page 105 0x801a400 
page 106 0x801a800 
page 107 0x801ac00 
page 108 0x801b000 
page 109 0x801b400 
page 110 0x801b800 
page 111 0x801bc00 
page 112 0x801c000 
page 113 0x801c400 
page 114 0x801c800 
page 115 0x801cc00 
page 116 0x801d000 
page 117 0x801d400 
page 118 0x801d800 
page 119 0x801dc00 
page 120 0x801e000 
page 121 0x801e400 
page 122 0x801e800 
page 123 0x801ec00 
page 124 0x801f000 
page 125 0x801f400 
page 126 0x801f800 
page 127 0x801fc00 


 */