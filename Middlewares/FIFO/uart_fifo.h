

#ifndef __UART_FIFO_H__
#define __UART_FIFO_H__

#include "usart.h"
#include <stdint.h>
#include <stdbool.h>
#include <cmsis_os.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum eUART_FIFO_MODE {
    UART_FIFO_DMA,
    UART_FIFO_IT,
} UART_FIFO_MODE_t;

typedef struct sUART_RX_FIFO{
  UART_HandleTypeDef *huart;
  uint8_t *buff;
  size_t head;
  size_t tail;
  size_t buff_len;
  size_t max_used_len;
  UART_FIFO_MODE_t mode;
	size_t error_cnt;
  struct sUART_RX_FIFO *next_ptr;
	TaskHandle_t task_waitfor_idle;
	osMutexId mutex_rx;
} UART_RX_FIFO_t;

int UART_RX_FIFO_open(UART_RX_FIFO_t *fifo, UART_HandleTypeDef *huart, size_t buff_len, UART_FIFO_MODE_t mode);

int UART_RX_FIFO_close(UART_RX_FIFO_t *fifo);

size_t UART_RX_FIFO_getlen(UART_RX_FIFO_t *fifo);

bool UART_RX_FIFO_getc(UART_RX_FIFO_t *fifo, uint8_t *c);

size_t UART_RX_FIFO_read(UART_RX_FIFO_t *fifo, uint8_t *data, size_t len);

void UART_RX_FIFO_flush(UART_RX_FIFO_t *fifo);

typedef struct sUART_TX_FIFO{
  UART_HandleTypeDef *huart;
  uint8_t *buff[2];//Ë«Í¨µÀ
  size_t head;
  size_t buff_len;
  size_t max_used_len;
  UART_FIFO_MODE_t mode;
  size_t sel;
  bool sending;
  struct sUART_TX_FIFO *next_ptr;
	osMutexId mutex_tx;
} UART_TX_FIFO_t;

int UART_TX_FIFO_open(UART_TX_FIFO_t *fifo, UART_HandleTypeDef *huart, size_t buff_len, UART_FIFO_MODE_t mode);

bool UART_TX_FIFO_write(UART_TX_FIFO_t *fifo, uint8_t *buff, size_t maxlen);



//void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);


#ifdef __cplusplus
}
#endif


#endif /* __UART_FIFO_H__ */
