
#include "uart_fifo.h"
#include <string.h>

static UART_RX_FIFO_t *rx_fifo_head = NULL;
static UART_TX_FIFO_t *tx_fifo_head = NULL;

void RegRxIdleIRQ(UART_RX_FIFO_t *fifo)
{
	if (rx_fifo_head)
	{
		UART_RX_FIFO_t *head = rx_fifo_head;
		while (head->next_ptr)//一个一个往下走，直到最后一个
		{
		  head = head->next_ptr;
		}
	  head->next_ptr = fifo;
	}
	else
	{
	  rx_fifo_head = fifo;
	}
}

void RegTxCpltIRQ(UART_TX_FIFO_t *fifo)
{
	if (tx_fifo_head)
	{
		UART_TX_FIFO_t *head = tx_fifo_head;
		while (head->next_ptr)
		{
		  head = head->next_ptr;
		}
	  head->next_ptr = fifo;
	}
	else
	{
	  tx_fifo_head = fifo;
	}
}

int UART_RX_FIFO_open(UART_RX_FIFO_t *fifo, UART_HandleTypeDef *huart, size_t buff_len, UART_FIFO_MODE_t mode)
{
	fifo->huart = huart;
	fifo->buff_len = buff_len;
	fifo->max_used_len = 0u;
	fifo->error_cnt = 0u;
	fifo->task_waitfor_idle = NULL;
	fifo->buff = (uint8_t *)pvPortMalloc(buff_len);
	if (!fifo->buff)//检查是否分配成功
	{
	  return -1;
	}
	fifo->head = 0u;
	fifo->tail = 0u;
	fifo->next_ptr = NULL;
	osMutexDef(rx);
	fifo->mutex_rx = osMutexCreate(osMutex(rx));

	RegRxIdleIRQ(fifo);
  __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);//enable idle int
	
	switch (mode)
	{
		case UART_FIFO_DMA:
			if (huart->hdmarx && huart->hdmarx->Init.Mode == DMA_CIRCULAR)
			{
				fifo->mode = UART_FIFO_DMA;
				HAL_UART_Receive_DMA(huart,fifo->buff, buff_len);
				return 0;
			}
			break;
		default:
	    fifo->mode = UART_FIFO_IT;
		  HAL_UART_Receive_IT(huart,fifo->buff, buff_len);
		  return 0;
	}

	return -1;
}

static inline size_t UART_RX_FIFO_gethead(UART_RX_FIFO_t *fifo)
{
	size_t head;

	switch (fifo->mode)
	{
		case UART_FIFO_DMA:
			head = fifo->buff_len -__HAL_DMA_GET_COUNTER(fifo->huart->hdmarx);
			break;
		default:
			head = fifo->head;
	}
  return head;
}

size_t UART_RX_FIFO_getlen(UART_RX_FIFO_t *fifo)
{
	size_t head;
	size_t used_len;
	
	head = UART_RX_FIFO_gethead(fifo);
	
	 if (head >= fifo->tail)
	 {
		 used_len = head - fifo->tail;
	 }
	 else
	 {
		 used_len = head+fifo->buff_len-fifo->tail;
	 }
	 if (used_len > fifo->max_used_len)
	 {
		 fifo->max_used_len = used_len;
	 }
   return used_len;
}


//中断进入，对缓存进行查找
void	BSP_UART_IRQHandler(UART_HandleTypeDef *huart)
{
	UART_RX_FIFO_t *fifo = rx_fifo_head;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	while (fifo)
	{
		if (fifo->huart == huart)
		{
      uint32_t tmp_flag = 0;
      tmp_flag =__HAL_UART_GET_FLAG(huart,UART_FLAG_IDLE); 
      if((tmp_flag != RESET))
      {
         __HAL_UART_CLEAR_IDLEFLAG(huart);
				switch (fifo->mode)
				{
					case UART_FIFO_IT:
						fifo->head = fifo->buff_len - huart->RxXferCount;
						break;
					default:
						break;
				}
				if (fifo->task_waitfor_idle)
				{
				  vTaskNotifyGiveFromISR(fifo->task_waitfor_idle, &xHigherPriorityTaskWoken );
					fifo->task_waitfor_idle = NULL;
					portYIELD_FROM_ISR( xHigherPriorityTaskWoken);	
				}
			}
			break;
		}
		else
		{
		  fifo = fifo->next_ptr;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_RX_FIFO_t *fifo = rx_fifo_head;
	while (fifo)
	{
		if (fifo->huart == huart)
		{
			 if (fifo->mode == UART_FIFO_IT)
			 {
				 fifo->head = 0u;
	       HAL_UART_Receive_IT(huart,fifo->buff, fifo->buff_len);
			 }
			break;
		}
		else
		{
		  fifo = fifo->next_ptr;
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->ErrorCode != HAL_UART_ERROR_NONE)
	{
		UART_RX_FIFO_t *fifo = rx_fifo_head;
		while (fifo)
		{
			if (fifo->huart == huart)
			{
				 switch (fifo->mode)
				 {
					 case UART_FIFO_DMA:
						 fifo->tail = 0u;
						 HAL_UART_Receive_DMA(huart,fifo->buff, fifo->buff_len);
						 break;
					 default:
						 fifo->tail = 0u;
						 fifo->head = 0u;
						 HAL_UART_Receive_IT(huart,fifo->buff, fifo->buff_len);
						 break;
				 }
				fifo->error_cnt++;
				break;
			}
			else
			{
				fifo = fifo->next_ptr;
			}
		}
	}
	
	{
		UART_TX_FIFO_t *fifo = tx_fifo_head;
		while (fifo)
		{
			if (fifo->huart == huart)
			{
				fifo->sending = false;
				break;
			}
			else
			{
				fifo = fifo->next_ptr;
			}
		}
	}
}

bool UART_RX_FIFO_getc(UART_RX_FIFO_t *fifo, uint8_t *c)
{
	size_t head;

	head = UART_RX_FIFO_gethead(fifo);

	if (fifo->tail == head)
	{
		return false;
	}
	
	*c = fifo->buff[fifo->tail++];
	if (fifo->tail >= fifo->buff_len)
	{
		fifo->tail = 0u;
	}
	
	return true;
}

size_t UART_RX_FIFO_read(UART_RX_FIFO_t *fifo, uint8_t *buff, size_t maxlen)
{
	size_t rslt = 0u;
	if (!osMutexWait(fifo->mutex_rx, 1))
	{
	  rslt = UART_RX_FIFO_getlen(fifo);
		if (rslt > maxlen)
		{
		  rslt = maxlen;
		}
		for (size_t i=0u;i<rslt;++i)
		{
		  buff[i] = fifo->buff[fifo->tail++];
			if (fifo->tail >= fifo->buff_len)
			{
				fifo->tail = 0u;
			}
		}
		osMutexRelease(fifo->mutex_rx);
	}
	return rslt;
}

void UART_RX_FIFO_flush(UART_RX_FIFO_t *fifo)
{
	size_t head;
	
	head = UART_RX_FIFO_gethead(fifo);
	fifo->tail = head;
}

int UART_TX_FIFO_open(UART_TX_FIFO_t *fifo, UART_HandleTypeDef *huart, size_t buff_len, UART_FIFO_MODE_t mode)
{
	fifo->huart = huart;
	fifo->buff_len = buff_len;
	fifo->mode = mode;
	fifo->head = 0u;
	fifo->max_used_len = 0u;
	fifo->buff[0] = (uint8_t *)pvPortMalloc(buff_len*2);
	if (!fifo->buff[0])
	{
	  return -1;
	}
	fifo->buff[1] = fifo->buff[0]+buff_len;
	fifo->sel = 0;
	fifo->sending = false;
	RegTxCpltIRQ(fifo);
	osMutexDef(tx);
	fifo->mutex_tx = osMutexCreate(osMutex(tx));
	return 0;
}

static void UART_TX_FIFO_sendstr(UART_TX_FIFO_t *fifo)
{
		if (fifo->head)
		{
	     fifo->sending = true; 
			 if (fifo->max_used_len < fifo->head)
			 {
			     fifo->max_used_len = fifo->head;
			 }
			 switch (fifo->mode)
			 {
				 case UART_FIFO_DMA:
			     HAL_UART_Transmit_DMA(fifo->huart,fifo->buff[fifo->sel],fifo->head);
				   break;
				 default:
			     HAL_UART_Transmit_IT(fifo->huart,fifo->buff[fifo->sel],fifo->head);
					 break;
			 }
			 fifo->head = 0u;
			 fifo->sel = fifo->sel?0:1;
		 }
		else
		{
	     fifo->sending = false; 
		}
}


bool UART_TX_FIFO_write(UART_TX_FIFO_t *fifo, uint8_t *data, size_t len)
{
	bool rslt = false;
	if (!osMutexWait(fifo->mutex_tx, 1))
	{
		if (fifo->head + len <= fifo->buff_len)
		{
			memcpy(fifo->buff[fifo->sel] + fifo->head,data,len);
			//给head加上尺寸
			fifo->head += len;
		 
			if(!fifo->sending)
			{
				UART_TX_FIFO_sendstr(fifo);
			}
		  rslt = true;
		}
		else
		{
			if (fifo->max_used_len < fifo->head + len) 
			{
				fifo->max_used_len = fifo->head + len;
			}
	  }
		osMutexRelease(fifo->mutex_tx);
	}
	return rslt;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UART_TX_FIFO_t *fifo = tx_fifo_head;
	while (fifo)
	{
		if (fifo->huart == huart)
		{
			UART_TX_FIFO_sendstr(fifo);
			break;
		}
		else
		{
		  fifo = fifo->next_ptr;
		}
	}
}


