// low level routines for programming flash memory.  Interrupts should be disabled when doing any of this.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32f1xx.h"
#include "util.h"

#include "nvm.h"
#define FLASH_PAGE_SIZE 1024

//const uint32_t flash_size = 1024;

/* STM32F103 have 128 PAGES (Page 0 to Page 127) of 1 KB each. This makes up 128 KB Flash Memory
 * Some STM32F103C8 have 64 KB FLASH Memory, so I guess they have Page 0 to Page 63 only.
 */

/* FLASH_PAGE_SIZE should be able to get the size of the Page according to the controller */
static uint32_t GetPage(uint32_t Address)
{
  for (int indx = 0; indx < 128; indx++)
  {
    if ((Address < (0x08000000 + (FLASH_PAGE_SIZE * (indx + 1)))) && (Address >= (0x08000000 + FLASH_PAGE_SIZE * indx)))
    {
      return (0x08000000 + FLASH_PAGE_SIZE * indx);
    }
  }

  return 0;
}

uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{

  uint32_t Error;
  int sofar = 0;
  uint32_t address = 0U;
  unsigned page_count;
  const unsigned int_per_page = FLASH_PAGE_SIZE / sizeof(int);
  int page[int_per_page];
  memset(page, 0, sizeof(page));

  /* Erase the user Flash area*/

  uint32_t StartPage = GetPage(StartPageAddress);
  uint32_t EndPageAdress = StartPageAddress + numberofwords * 4;
  uint32_t EndPage = GetPage(EndPageAdress);
  uint32_t NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;
  printf("Start page: %#010x  EndPage: %#010x next free page %#010x  number of Pages: %d\n", StartPage, EndPage, EndPage+FLASH_PAGE_SIZE,NbPages);
  //  /* Fill EraseInit structure*/
  //  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  //  EraseInitStruct.PageAddress = StartPage;
  //  EraseInitStruct.NbPages     = ((EndPage - StartPage)/FLASH_PAGE_SIZE) +1;

  //  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
  //  {
  //    /*Error occurred while page erase.*/
  //   return HAL_FLASH_GetError ();
  //  }

  /* Program the user Flash area word by word*/

  for (address = StartPage;
       address < (StartPage + (NbPages)*FLASH_PAGE_SIZE);
       address += FLASH_PAGE_SIZE)
  {
    eraseSector(address);
  }
 

  //fill
  sofar = 0;
  page_count = 0;
  StartPageAddress = StartPage;
  while (sofar <= (numberofwords))
  {
    
    if (page_count != int_per_page && sofar <(numberofwords))
    {
       // printf("ind: %d  data: %d \n",sofar, Data[sofar]);
      page[page_count++] =  Data[sofar];
    }
    if(page_count == int_per_page){
       if (writeSector(StartPageAddress, page, sizeof(page)) == 0)
      {
        StartPageAddress += FLASH_PAGE_SIZE; // use StartPageAddress += 2 for half word and 8 for double word
        page_count = 0;
      }}
     if(sofar == numberofwords){
        printf("xxxxind: %d  data: %d \n",sofar, Data[sofar]);
       page[page_count] =  Data[sofar];
       if (writeSector(StartPageAddress, page, sizeof(page)) == 0)
      {
       
         printf("done writing to flash: %ld  \n",StartPageAddress);

      }
    
      
      
    }
   
    sofar++;
  }

  /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/

  return 0;
}

int writeSector(const volatile void *flash_ptr, const void *data, uint16_t data_size)
{
  uint16_t *AddressPtr;
  uint16_t *valuePtr;

  AddressPtr = (uint16_t *)flash_ptr;
  valuePtr = (uint16_t *)data;

  // we're using 16 bit words here while flash_size in bytes
  uint32_t size = data_size / 2;

  while (size)
  {
    // unlock the flash
    // Key 1 : 0x45670123
    // Key 2 : 0xCDEF89AB
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
    FLASH->CR &= ~(1 << 1); // ensure PER is low
    FLASH->CR |= (1 << 0);  // set the PG bit
    *(AddressPtr) = *(valuePtr);

    while (FLASH->SR & (1 << 0))
      ; // wait while busy

    if (FLASH->SR & (1 << 2))
    {
      printf("%s \n", "Flash not erased");
      return 1;
    }

    if (FLASH->SR & (1 << 4))
    {
      printf("%s \n", "Write protect error");
      return 2;
    }

    AddressPtr++;
    valuePtr++;
    size--;
  }
  return 0;
}
int eraseSector(const volatile void *flash_ptr)
{

  // Key 1 : 0x45670123
  // Key 2 : 0xCDEF89AB
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
  FLASH->CR &= ~(1 << 0); // Ensure PG bit is low
  FLASH->CR |= (1 << 1);  // set the PER bit
  FLASH->AR = (uint32_t)flash_ptr;
  FLASH->CR |= (1 << 6); // set the start bit

  while (FLASH->SR & (1 << 0))
    ; // wait while busy
  return 0;
}
void readSector(const volatile void *flash_ptr, void *data, uint16_t _size)
{
  uint16_t *AddressPtr;
  uint16_t *valuePtr;

  AddressPtr = (uint16_t *)flash_ptr;
  valuePtr = (uint16_t *)data;

  // we're using 16 bit words here while flash_size in bytes
  uint32_t size = _size / 2;
  while (size)
  {
    *((uint16_t *)valuePtr) = *((uint16_t *)AddressPtr);

    valuePtr++;
    AddressPtr++;
    size--;
  }
}
void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
  while (1)
  {

    *RxBuf = *(__IO uint32_t *)StartPageAddress;
    StartPageAddress += 4;
    RxBuf++;
    if (!(numberofwords--))
      break;
  }
}