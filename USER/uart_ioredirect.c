#include <stdio.h>
#include "stm32f10x.h"
#include "user_usart.h"

int fputc(int ch, FILE *f){
	/* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(IO_REDIR_USART_PORT, (uint8_t) ch);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(IO_REDIR_USART_PORT, USART_FLAG_TXE) == RESET) ;
  return ch;
}

int fgetc(FILE *f){
	int c = 0;
	while (USART_GetFlagStatus(IO_REDIR_USART_PORT, USART_FLAG_RXNE) == RESET) ;
	c = (int) USART_ReceiveData(IO_REDIR_USART_PORT);
	//fputc(c, f);
	return c;
}
