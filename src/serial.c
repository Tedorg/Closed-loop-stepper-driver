#include <stdint.h>
#include "stm32f1xx.h"
#include "util.h"
#include "serial.h"

#define MAXBUFFER 64
typedef struct tagComBuffer
{
  unsigned char Buffer[MAXBUFFER];
  unsigned Head, Tail;
  unsigned Count;
  unsigned Ready;
} ComBuffer;

ComBuffer ComRXBuffer, ComTXBuffer;
int PutBuf(ComBuffer *Buf, unsigned char Data);
unsigned char GetBuf(ComBuffer *Buf);
unsigned GetBufCount(ComBuffer *Buf);
int ReadCom(int Max, unsigned char *Buffer);
int WriteCom(int Count, char *Buffer);

void usart_tx(void);
void usart_rx(void);
unsigned ComOpen;
unsigned ComError;
unsigned ComBusy;

int ReadCom(int Max, unsigned char *Buffer)
{
  // Read up to Max bytes from the communications buffer
  // into Buffer.  Return number of bytes read
  unsigned i;
  if (!ComOpen)
    return (-1);
  i = 0;
  while ((i < Max - 1) && (GetBufCount(&ComRXBuffer)))
    Buffer[i++] = GetBuf(&ComRXBuffer);
  if (i > 0)
  {
    Buffer[i] = 0;

    return (i);
  }
  else
  {
    return (0);
  }
};

// Redirect standard output to the serial port
int _write(int file, char *ptr, int len)
{

  //
  //eputs(ptr);

  //printf("test: %c\n",s);
  //WriteCom(len,ptr);

  USART2->CR1 |= BIT3;

  for (int i = 0; i < len; i++)
  {

    while (!(USART2->SR & USART_SR_TXE))
      ;
    USART2->DR = *ptr++;
  }

  return len;
}

int WriteCom(int Count, char *Buffer)
{
  // Writes Count bytes from Buffer into the the communications TX buffer
  // returns -1 if the port is not open (configured)
  // returns -2 if the message is too big to be sent
  // If the transmitter is idle it will initiate interrupts by
  // writing the first character to the hardware transmit buffer
  unsigned i, BufLen;
  if (!ComOpen)
    return (-1);
  BufLen = GetBufCount(&ComTXBuffer);
  if ((MAXBUFFER - BufLen) < Count)
    return (-2);
  for (i = 0; i < Count; i++)
    PutBuf(&ComTXBuffer, Buffer[i]);

  if ((USART2->CR1 & BIT3) == 0)
  { // transmitter was idle, turn it on and force out first character
    USART2->CR1 |= BIT3;
    USART2->DR = GetBuf(&ComTXBuffer);
  }
  return 0;
};

void init_uart()
{

  ComRXBuffer.Head = ComRXBuffer.Tail = ComRXBuffer.Count = 0;
  ComTXBuffer.Head = ComTXBuffer.Tail = ComTXBuffer.Count = 0;
   ComRXBuffer.Ready =  ComTXBuffer.Ready = 0;
  ComOpen = 1;
  ComError = 0;
   // Enable clock for Port A, alternate functions and USART2
    SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN + RCC_APB2ENR_AFIOEN);
    SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

    // PA2 (TxD) shall use the alternate function with push-pull
    MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF2 + GPIO_CRL_MODE2, GPIO_CRL_CNF2_1 + GPIO_CRL_MODE2_1);

    // Enable transmitter, receiver and receive-interrupt of USART2
    USART2->CR1 = USART_CR1_UE + USART_CR1_TE + USART_CR1_RE + USART_CR1_RXNEIE;

    // Set baudrate, assuming that USART2 is clocked with 
    // the same frequency as the CPU core (no prescalers).
   // USART2->BRR = (SystemCoreClock / 9600);
    
    // With > 36 MHz system clock, the USART2 receives usually half of it:
    USART2->BRR = (SystemCoreClock /  115200);

    // Enable interrupt in NVIC
    NVIC_EnableIRQ(USART2_IRQn);

}

// called after each received character
void USART2_IRQHandler()
{
  // check which interrupt happened.
  if (USART2->SR & BIT6) // is it a TXE interrupt?
    usart_tx();
  if (USART2->SR & BIT5) // is it an RXNE interrupt?
    usart_rx();
}

void usart_rx(void)
{
  // printf("in: %d \n", USART2->DR);
  // Handles serial comms reception
  // simply puts the data into the buffer and sets the ComError flag
  // if the buffer is fullup
  if (PutBuf(&ComRXBuffer, USART2->DR))
    ComError = 1; // if PutBuf returns a non-zero value then there is an error
}
void usart_tx(void)
{
 
  // Handles serial comms transmission
  // When the transmitter is idle, this is called and the next byte
  // is sent (if there is one)
  if (GetBufCount(&ComTXBuffer))
    USART2->DR = GetBuf(&ComTXBuffer);
  else
  {
    // No more data, disable the transmitter
    USART2->CR1 &= ~BIT3;
    if (USART2->SR & BIT6)
      // Clear the TC interrupt flag
      USART2->SR &= ~BIT6;
  }
}
int PutBuf(ComBuffer *Buf, unsigned char Data)
{
  if ((Buf->Head == Buf->Tail) && (Buf->Count != 0))
    return (1); /* OverFlow */
  if(Data == NEWLINE)  ComRXBuffer.Ready = 1;
  disable_interrupts();
  Buf->Buffer[Buf->Head++] = Data;
  Buf->Count++;

  // printf("%d\n",Buf->Count);
  if (Buf->Head == MAXBUFFER)
    Buf->Head = 0;
  enable_interrupts();
  return (0);
};
unsigned char GetBuf(ComBuffer *Buf)
{
  unsigned char Data;
  if (Buf->Count == 0)
    return (0);
  disable_interrupts();
  Data = Buf->Buffer[Buf->Tail++];
  if (Buf->Tail == MAXBUFFER)
    Buf->Tail = 0;
  Buf->Count--;
  enable_interrupts();
  return (Data);
};
unsigned int GetBufCount(ComBuffer *Buf)
{
  return Buf->Count;
};

int rxAvailable()
{
  return  ComRXBuffer.Ready;
}

int eputs(char *s)
{

  // only writes to the comms port at the moment
  if (!ComOpen)
    return -1;
  while (*s)
  {
    //printf("test: %c\n",s);
    WriteCom(1, s++);
  }
  return 0;
}
int egets(char *s, int Max)
{
  // read from the comms port until end of string
  // or newline is encountered.  Buffer is terminated with null
  // returns number of characters read on success
  // returns 0 or -1 if error occurs
  // Warning: This is a blocking call.

  int Len, bufferCount;
  char c;

  if (!ComOpen)
    return -1;
  Len = 0;
  c = 0;
  bufferCount = GetBufCount(&ComRXBuffer);

  while ((Len < Max - 1) && (c != NEWLINE) && (bufferCount > 0))
  {
    //bufferCount =GetBufCount(&ComRXBuffer);
    // wait for a character

    c = GetBuf(&ComRXBuffer);
    s[Len++] = c;
  }

  if (Len > 0)
  {
    s[Len] = 0;
  }
   ComRXBuffer.Ready = 0;

  return Len;
}

// void init_uart(){
//     SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);

//     // PA2 (TxD) shall use the alternate function with push-pull
//     MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF2 + GPIO_CRL_MODE2, GPIO_CRL_CNF2_1 + GPIO_CRL_MODE2_1);

//     // Enable transmitter, receiver and receive-interrupt of USART2
//     USART2->CR1 = USART_CR1_UE + USART_CR1_TE + USART_CR1_RE + USART_CR1_RXNEIE;

//     // Set baudrate, assuming that USART2 is clocked with
//     // the same frequency as the CPU core (no prescalers).
//     //USART2->BRR = (0x1D4C);

//     // With > 36 MHz system clock, the USART2 receives usually half of it:
//     USART2->BRR = (SystemCoreClock /16/9600); //74880 baudrate

//     // Enable interrupt in NVIC
//     NVIC_EnableIRQ(USART2_IRQn);

//     printf("%ld\n",GPIOA->IDR & (0x1));
//     printf("%s\n","ready");

// }

// // Redirect standard output to the serial port
// int _write(int file, char *ptr, int len)
// {
//     for (int i=0; i<len; i++)
//     {
//         while(!(USART2->SR & USART_SR_TXE));
//         USART2->DR = *ptr++;
//     }
//     return len;
// }

// // called after each received character
// void USART2_IRQHandler()
// {
//     char received=USART2->DR;

//     // send echo back
//     while(!(USART2->SR & USART_SR_TXE));
//     USART2->DR = received;
// }