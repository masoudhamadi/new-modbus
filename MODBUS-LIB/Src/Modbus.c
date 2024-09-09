/*
 * Modbus.c
 *  Modbus RTU Master and Slave library for STM32 CUBE with FreeRTOS
 *  Created on: May 5, 2020
 *      Author: Alejandro Mera
 *      Adapted from https://github.com/smarmengol/Modbus-Master-Slave-for-Arduino
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "main.h"
#include "Modbus.h"
#include "timers.h"
#include "semphr.h"




#if ENABLE_TCP == 1
#include "api.h"
#include "ip4_addr.h"
#include "netif.h"
#endif

#ifndef ENABLE_USART_DMA
#define ENABLE_USART_DMA 0
#endif

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)


modbusHandler_t *mHandlers[MAX_M_HANDLERS];

const osMessageQueueAttr_t QueueTelegram_attributes = {//Queue Modbus telegrams for master
       .name = "QueueModbusTelegram"
};


const osThreadAttr_t myTaskModbusA_attributes = {
    .name = "TaskModbusSlave",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
};

const osThreadAttr_t myTaskModbusA_attributesTCP = {
    .name = "TaskModbusSlave",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 4
};



//Task Modbus Master
const osThreadAttr_t myTaskModbusB_attributes = {//osThreadId_t myTaskModbusAHandle;
    .name = "TaskModbusMaster",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 128 * 4
};


const osThreadAttr_t myTaskModbusB_attributesTCP = {
    .name = "TaskModbusMaster",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 256 * 4
};
const osSemaphoreAttr_t ModBusSphr_attributes = {//Semaphore to access the Modbus Data
    .name = "ModBusSphr"
};


uint8_t numberHandlers = 0;


static void sendTxBuffer(modbusHandler_t *modH);
static int16_t getRxBuffer(modbusHandler_t *modH);
static uint8_t validateAnswer(modbusHandler_t *modH);
static void buildException( uint8_t u8exception, modbusHandler_t *modH );
static uint8_t validateRequest(modbusHandler_t * modH);
static uint16_t word(uint8_t H, uint8_t l);
static void get_FC1(modbusHandler_t *modH);
static void get_FC3(modbusHandler_t *modH);
static int8_t process_FC1(modbusHandler_t *modH );
static int8_t process_FC3(modbusHandler_t *modH );
static int8_t process_FC5( modbusHandler_t *modH);
static int8_t process_FC6(modbusHandler_t *modH );
static int8_t process_FC15(modbusHandler_t *modH );
static int8_t process_FC16(modbusHandler_t *modH);
static void vTimerCallbackT35(TimerHandle_t *pxTimer);
static void vTimerCallbackTimeout(TimerHandle_t *pxTimer);
//static int16_t getRxBuffer(modbusHandler_t *modH);
static int8_t SendQuery(modbusHandler_t *modH ,  modbus_t telegram);

#if ENABLE_TCP ==1

static bool TCPwaitConnData(modbusHandler_t *modH);
static void  TCPinitserver(modbusHandler_t *modH);
static mb_errot_t TCPconnectserver(modbusHandler_t * modH, modbus_t *telegram);
static mb_errot_t TCPgetRxBuffer(modbusHandler_t * modH);

#endif




/* Ring Buffer functions */
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val) {// This function must be called only after disabling USART RX interrupt or inside of the RX interrupt

	xRingBuffer->uxBuffer[xRingBuffer->u8end] = u8Val;
	xRingBuffer->u8end = (xRingBuffer->u8end + 1) % MAX_BUFFER;
	if (xRingBuffer->u8available == MAX_BUFFER)
	{
		xRingBuffer->overflow = true;
		xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
	}
	else
	{
		xRingBuffer->overflow = false;
		xRingBuffer->u8available++;
	}

}
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer){// This function must be called only after disabling USART RX interrupt
	return RingGetNBytes(xRingBuffer, buffer, xRingBuffer->u8available);
}
uint8_t RingGetNBytes (modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber){// This function must be called only after disabling USART RX interrupt
	uint8_t uCounter;
	if(xRingBuffer->u8available == 0  || uNumber == 0 ) return 0;
	if(uNumber > MAX_BUFFER) return 0;

	for(uCounter = 0; uCounter < uNumber && uCounter< xRingBuffer->u8available ; uCounter++)
	{
		buffer[uCounter] = xRingBuffer->uxBuffer[xRingBuffer->u8start];
		xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
	}
	xRingBuffer->u8available = xRingBuffer->u8available - uCounter;
	xRingBuffer->overflow = false;
	RingClear(xRingBuffer);

	return uCounter;
}

uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer){
return xRingBuffer->u8available;
}

void RingClear(modbusRingBuffer_t *xRingBuffer){
xRingBuffer->u8start = 0;
xRingBuffer->u8end = 0;
xRingBuffer->u8available = 0;
xRingBuffer->overflow = false;
}

/* End of Ring Buffer functions */



const unsigned char fctsupported[] =
{
    MB_FC_READ_COILS,
    MB_FC_READ_DISCRETE_INPUT,
    MB_FC_READ_REGISTERS,
    MB_FC_READ_INPUT_REGISTER,
    MB_FC_WRITE_COIL,
    MB_FC_WRITE_REGISTER,
    MB_FC_WRITE_MULTIPLE_COILS,
    MB_FC_WRITE_MULTIPLE_REGISTERS
};


/**
 * @brief
 * Initialization for a Master/Slave.
 * this function will check the configuration parameters
 * of the modbus handler
 *
 * @param modH   modbus handler
 */
void ModbusInit(modbusHandler_t *modH) {
    if (numberHandlers >= MAX_M_HANDLERS) {
        while (1); // Error: No more Modbus handlers supported
    }

    // Initialize the ring buffer
    RingClear(&modH->xBufferRX);

    // Create Modbus tasks based on the type
    if (modH->uModbusType == MB_SLAVE) {
        createModbusSlaveTask(modH);
    } else if (modH->uModbusType == MB_MASTER) {
        createModbusMasterTask(modH);
    } else {
        while (1); // Error: Modbus type not supported
    }

    // Check if the task was created successfully
    if (modH->myTaskModbusAHandle == NULL) {
        while (1); // Error: Creating Modbus task failed
    }

    // Create timers
    createTimers(modH);

    // Create semaphore
    modH->ModBusSphrHandle = osSemaphoreNew(1, 1, &ModBusSphr_attributes);
    if (modH->ModBusSphrHandle == NULL) {
        while (1); // Error: Creating semaphore failed
    }

    // Register the handler
    mHandlers[numberHandlers++] = modH;
}

void createModbusSlaveTask(modbusHandler_t *modH) {
#if ENABLE_TCP == 1
    if (modH->xTypeHW == TCP_HW) {
        modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusSlave, modH, &myTaskModbusA_attributesTCP);
    } else {
        modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusSlave, modH, &myTaskModbusA_attributes);
    }
#else
    modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusSlave, modH, &myTaskModbusA_attributes);
#endif
}

void createModbusMasterTask(modbusHandler_t *modH) {
#if ENABLE_TCP == 1
    if (modH->xTypeHW == TCP_HW) {
        modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusMaster, modH, &myTaskModbusB_attributesTCP);
    } else {
        modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusMaster, modH, &myTaskModbusB_attributes);
    }
#else
    modH->myTaskModbusAHandle = osThreadNew(StartTaskModbusMaster, modH, &myTaskModbusB_attributes);
#endif

    // Create message queue for telegrams
    modH->QueueTelegramHandle = osMessageQueueNew(MAX_TELEGRAMS, sizeof(modbus_t), &QueueTelegram_attributes);
    if (modH->QueueTelegramHandle == NULL) {
        while (1); // Error: Creating queue for telegrams failed
    }
}

void createTimers(modbusHandler_t *modH) {
    modH->xTimerTimeout = xTimerCreate("xTimerTimeout", modH->u16timeOut, pdFALSE,
                                        (void *)modH->xTimerTimeout, vTimerCallbackTimeout);
    if (modH->xTimerTimeout == NULL) {
        while (1); // Error: Creating timeout timer failed
    }

    modH->xTimerT35 = xTimerCreate("TimerT35", T35, pdFALSE,
                                    (void *)modH->xTimerT35, vTimerCallbackT35);
    if (modH->xTimerT35 == NULL) {
        while (1); // Error: Creating T35 timer failed
    }
}
void ModbusStart(modbusHandler_t *modH) {
    // Validate hardware type
    if (!isValidHardwareType(modH->xTypeHW)) {
        while (1); // ERROR: Invalid hardware type selected
    }

    // Check for USART_DMA requirement
    if (modH->xTypeHW == USART_HW_DMA && ENABLE_USART_DMA == 0) {
        while (1); // ERROR: USART_HW_DMA requires enabling in ModbusConfig.h
    }

    // Handle USART and USART_DMA initialization
    if (modH->xTypeHW == USART_HW || modH->xTypeHW == USART_HW_DMA) {
        configureTransceiver(modH);

        if (modH->uModbusType == MB_SLAVE && modH->u16regs == NULL) {
            while (1); // ERROR: Define the DATA pointer shared through Modbus
        }

        // Wait for UART port to be ready
        waitForUARTReady(modH->port);

#if ENABLE_USART_DMA == 1
        if (modH->xTypeHW == USART_HW_DMA) {
            initializeUSARTDMA(modH);
        } else {
            initializeUSARTInterrupt(modH);
        }
#else
        initializeUSARTInterrupt(modH);
#endif

        validateMasterSlaveID(modH);
    }

#if ENABLE_TCP == 1
    // TCP-specific initialization can go here
#endif

    // Reset counters
    resetModbusCounters(modH);
}

// Helper functions

bool isValidHardwareType(int type) {
    return (type == USART_HW || type == TCP_HW || type == USB_CDC_HW || type == USART_HW_DMA);
}

void configureTransceiver(modbusHandler_t *modH) {
    if (modH->EN_Port != NULL) {
        HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_RESET); // Set RS485 to transmit mode
    }
}

void waitForUARTReady(UART_HandleTypeDef *port) {
    while (HAL_UART_GetState(port) != HAL_UART_STATE_READY) {
        // Optionally add a timeout mechanism here
    }
}

void initializeUSARTDMA(modbusHandler_t *modH) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(modH->port, modH->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK) {
        while (1); // ERROR: Initialization code failed
    }
    __HAL_DMA_DISABLE_IT(modH->port->hdmarx, DMA_IT_HT); // Disable half-transfer interrupt
}

void initializeUSARTInterrupt(modbusHandler_t *modH) {
    if (HAL_UART_Receive_IT(modH->port, &modH->dataRX, 1) != HAL_OK) {
        while (1); // ERROR: Initialization code failed
    }
}

void validateMasterSlaveID(modbusHandler_t *modH) {
    if (modH->u8id != 0 && modH->uModbusType == MB_MASTER) {
        while (1); // ERROR: Master ID must be zero
    }

    if (modH->u8id == 0 && modH->uModbusType == MB_SLAVE) {
        while (1); // ERROR: Slave ID must be non-zero
    }
}

void resetModbusCounters(modbusHandler_t *modH) {
    modH->u8lastRec = modH->u8BufferSize = 0;
    modH->u16InCnt = modH->u16OutCnt = modH->u16errCnt = 0;
}

#if ENABLE_USB_CDC == 1
extern void MX_USB_DEVICE_Init(void);
void ModbusStartCDC(modbusHandler_t * modH)
{


    if (modH->uModbusType == MB_SLAVE &&  modH->u16regs == NULL )
    {
    	while(1); //ERROR define the DATA pointer shared through Modbus
    }

    modH->u8lastRec = modH->u8BufferSize = 0;
    modH->u16InCnt = modH->u16OutCnt = modH->u16errCnt = 0;
}
#endif


void vTimerCallbackT35(TimerHandle_t *pxTimer)
{
	//Notify that a stream has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{

		if( (TimerHandle_t *)mHandlers[i]->xTimerT35 ==  pxTimer ){
			if(mHandlers[i]->uModbusType == MB_MASTER)
			{
				xTimerStop(mHandlers[i]->xTimerTimeout,0);
			}
			xTaskNotify(mHandlers[i]->myTaskModbusAHandle, 0, eSetValueWithOverwrite);
		}

	}
}

void vTimerCallbackTimeout(TimerHandle_t *pxTimer)
{
	//Notify that a stream has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{

		if( (TimerHandle_t *)mHandlers[i]->xTimerTimeout ==  pxTimer ){
				xTaskNotify(mHandlers[i]->myTaskModbusAHandle, ERR_TIME_OUT, eSetValueWithOverwrite);
		}

	}

}


#if ENABLE_TCP ==1

bool TCPwaitConnData(modbusHandler_t *modH)
{
  struct netbuf *inbuf;
  err_t recv_err, accept_err;
  char* buf;
  uint16_t buflen;
  uint16_t uLength;
  bool xTCPvalid;
  xTCPvalid = false;
  tcpclients_t *clientconn;

  //select the next connection slot to work with using round-robin
  modH->newconnIndex++;
  if (modH->newconnIndex>NUMBERTCPCONN)
  {
	  modH->newconnIndex = 0;
  }
  clientconn = &modH->newconns[modH->newconnIndex];


  //NULL means there is a free connection slot, so we can accept an incoming client connection
  if (clientconn->conn == NULL){
      /* accept any incoming connection */
	  accept_err = netconn_accept(modH->conn, &clientconn->conn);
	  if(accept_err != ERR_OK)
	  {
		  // not valid incoming connection at this time
		  //ModbusCloseConn(clientconn->conn);
		  ModbusCloseConnNull(modH);
		  return xTCPvalid;
      }
	  else
	  {
		  clientconn->aging=0;
	  }

  }

  netconn_set_recvtimeout(clientconn->conn ,  modH->u16timeOut);
  recv_err = netconn_recv(clientconn->conn, &inbuf);

  if (recv_err == ERR_CLSD) //the connection was closed
  {
	  //Close and clean the connection
	  //ModbusCloseConn(clientconn->conn);
	  ModbusCloseConnNull(modH);

	  clientconn->aging = 0;
	  return xTCPvalid;

  }

  if (recv_err == ERR_TIMEOUT) //No new data
   {
 	  //continue the aging process
	  modH->newconns[modH->newconnIndex].aging++;

	  // if the connection is old enough and inactive close and clean it up
	  if (modH->newconns[modH->newconnIndex].aging >= TCPAGINGCYCLES)
	  {
		  //ModbusCloseConn(clientconn->conn);
		  ModbusCloseConnNull(modH);
		  clientconn->aging = 0;
	  }

 	  return xTCPvalid;

   }

  if (recv_err == ERR_OK)
  {
      if (netconn_err(clientconn->conn) == ERR_OK)
      {
    	  /* Read the data from the port, blocking if nothing yet there.
    	  We assume the request (the part we care about) is in one netbuf */
   	      netbuf_data(inbuf, (void**)&buf, &buflen);
		  if (buflen>11) // minimum frame size for modbus TCP
		  {
			  if(buf[2] == 0 || buf[3] == 0 ) //validate protocol ID
			  {
			  	  uLength = (buf[4]<<8 & 0xff00) | buf[5];
			  	  if(uLength< (MAX_BUFFER-2)  && (uLength + 6) <= buflen)
			   	  {
			          for(int i = 0; i < uLength; i++)
			          {
			        	  modH->u8Buffer[i] = buf[i+6];
			          }
			          modH->u16TransactionID = (buf[0]<<8 & 0xff00) | buf[1];
			          modH->u8BufferSize = uLength + 2; //add 2 dummy bytes for CRC
			          xTCPvalid = true; // we have data for the modbus slave

			      }
			  }

		  }
		  netbuf_delete(inbuf); // delete the buffer always
		  clientconn->aging = 0; //reset the aging counter
	   }
   }

  return xTCPvalid;

}


void  TCPinitserver(modbusHandler_t *modH)
{
      err_t err;

	  /* Create a new TCP connection handle */
	  if(modH-> xTypeHW == TCP_HW)
	  {
		  modH->conn = netconn_new(NETCONN_TCP);
		  if (modH->conn!= NULL)
		  {
		     /* Bind to port (502) Modbus with default IP address */
			 if(modH->uTcpPort == 0) modH->uTcpPort = 502; //if port not defined
		     err = netconn_bind(modH->conn, NULL, modH->uTcpPort);
		     if (err == ERR_OK)
		     {
		    	 /* Put the connection into LISTEN state */
		    	 netconn_listen(modH->conn);
		    	 netconn_set_recvtimeout(modH->conn, 1); // this is necessary to make it non blocking
		     }
		     else{
		    		  while(1)
		    		  {
		    			  // error binding the TCP Modbus port check your configuration
		    		  }
		    	  }
		  }
		  else{
			  while(1)
			  {
				  // error creating new connection check your configuration,
				  // this function must be called after the scheduler is started
			  }
		  }
	  }
}

#endif



void StartTaskModbusSlave(void *argument) {
    modbusHandler_t *modH = (modbusHandler_t *)argument;

#if ENABLE_TCP == 1
    if (modH->xTypeHW == TCP_HW) {
        TCPinitserver(modH); // Start the Modbus server slave
    }
#endif

    for (;;) {
        modH->i8lastError = 0;

#if ENABLE_USB_CDC == 1
        if (modH->xTypeHW == USB_CDC_HW) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Block until a Modbus Frame arrives
            if (modH->u8BufferSize == ERR_BUFF_OVERFLOW) {
                handleBufferOverflow(modH);
                continue;
            }
        }
#endif

#if ENABLE_TCP == 1
        if (modH->xTypeHW == TCP_HW) {
            if (!TCPwaitConnData(modH)) { // Wait for connection and receive data
                continue; // TCP package was not validated
            }
        }
#endif

        if (isUsartHardware(modH)) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Block until a Modbus Frame arrives
            if (getRxBuffer(modH) == ERR_BUFF_OVERFLOW) {
                handleBufferOverflow(modH);
                continue;
            }
        }

        if (modH->u8BufferSize < 7) {
            handleInvalidFrameSize(modH);
            continue;
        }

        if (!isValidSlaveId(modH)) {
            continue; // This request is not for us
        }

        uint8_t u8exception = validateRequest(modH);
        if (handleValidationException(u8exception, modH)) {
            continue; // Handle exception and continue
        }

        xSemaphoreTake(modH->ModBusSphrHandle, portMAX_DELAY); // Acquire semaphore

        // Process message based on function code
        modH->i8state = processFunctionCode(modH);

        xSemaphoreGive(modH->ModBusSphrHandle); // Release semaphore
    }
}

// Helper functions

bool isUsartHardware(modbusHandler_t *modH) {
    return (modH->xTypeHW == USART_HW || modH->xTypeHW == USART_HW_DMA);
}

void handleBufferOverflow(modbusHandler_t *modH) {
    modH->i8lastError = ERR_BUFF_OVERFLOW;
    modH->u16errCnt++;
}

void handleInvalidFrameSize(modbusHandler_t *modH) {
    modH->i8lastError = ERR_BAD_SIZE;
    modH->u16errCnt++;
}

bool isValidSlaveId(modbusHandler_t *modH) {
#if ENABLE_TCP == 0
    return (modH->u8Buffer[ID] == modH->u8id);
#else
    return (modH->xTypeHW != TCP_HW || modH->u8Buffer[ID] == modH->u8id);
#endif
}

bool handleValidationException(uint8_t u8exception, modbusHandler_t *modH) {
    if (u8exception > 0) {
        if (u8exception != ERR_TIME_OUT) {
            buildException(u8exception, modH);
            sendTxBuffer(modH);
        }
        modH->i8lastError = u8exception;
        return true; // Exception handled
    }
    modH->i8lastError = 0;
    return false; // No exception
}

int32_t processFunctionCode(modbusHandler_t *modH) {
    switch (modH->u8Buffer[FUNC]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUT:
            return process_FC1(modH);
        case MB_FC_READ_INPUT_REGISTER:
        case MB_FC_READ_REGISTERS:
            return process_FC3(modH);
        case MB_FC_WRITE_COIL:
            return process_FC5(modH);
        case MB_FC_WRITE_REGISTER:
            return process_FC6(modH);
        case MB_FC_WRITE_MULTIPLE_COILS:
            return process_FC15(modH);
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return process_FC16(modH);
        default:
            return -1; // Invalid function code
    }
}



void ModbusQuery(modbusHandler_t * modH, modbus_t telegram )
{
	//Add the telegram to the TX tail Queue of Modbus
	if (modH->uModbusType == MB_MASTER)
	{
	telegram.u32CurrentTask = (uint32_t *) osThreadGetId();
	xQueueSendToBack(modH->QueueTelegramHandle, &telegram, 0);
	}
	else{
		while(1);// error a slave cannot send queries as a master
	}
}



void ModbusQueryInject(modbusHandler_t * modH, modbus_t telegram )
{
	//Add the telegram to the TX head Queue of Modbus
	xQueueReset(modH->QueueTelegramHandle);
	telegram.u32CurrentTask = (uint32_t *) osThreadGetId();
	xQueueSendToFront(modH->QueueTelegramHandle, &telegram, 0);
}


#if ENABLE_TCP ==1
void ModbusCloseConn(struct netconn *conn)
{

	if(conn != NULL)
	{
		netconn_close(conn);
		netconn_delete(conn);
	}

}

void ModbusCloseConnNull(modbusHandler_t * modH)
{

	if(modH->newconns[modH->newconnIndex].conn  != NULL)
	{

		netconn_close(modH->newconns[modH->newconnIndex].conn);
		netconn_delete(modH->newconns[modH->newconnIndex].conn);
		modH->newconns[modH->newconnIndex].conn = NULL;
	}

}

#endif


/**
 * @brief
 * *** Only Modbus Master ***
 * Generate a query to an slave with a modbus_t telegram structure
 * The Master must be in COM_IDLE mode. After it, its state would be COM_WAITING.
 * This method has to be called only in loop() section.
 *
 * @see modbus_t
 * @param modH  modbus handler
 * @param modbus_t  modbus telegram structure (id, fct, ...)
 * @ingroup loop
 */
int8_t SendQuery(modbusHandler_t *modH ,  modbus_t telegram )
{


	uint8_t u8regsno, u8bytesno;
	uint8_t  error = 0;
	xSemaphoreTake(modH->ModBusSphrHandle , portMAX_DELAY); //before processing the message get the semaphore

	if (modH->u8id!=0) error = ERR_NOT_MASTER;
	if (modH->i8state != COM_IDLE) error = ERR_POLLING ;
	if ((telegram.u8id==0) || (telegram.u8id>247)) error = ERR_BAD_SLAVE_ID;


	if(error)
	{
		 modH->i8lastError = error;
		 xSemaphoreGive(modH->ModBusSphrHandle);
		 return error;
	}


	modH->u16regs = telegram.u16reg;

	// telegram header
	modH->u8Buffer[ ID ]         = telegram.u8id;
	modH->u8Buffer[ FUNC ]       = telegram.u8fct;
	modH->u8Buffer[ ADD_HI ]     = highByte(telegram.u16RegAdd );
	modH->u8Buffer[ ADD_LO ]     = lowByte( telegram.u16RegAdd );

	switch( telegram.u8fct )
	{
	case MB_FC_READ_COILS:
	case MB_FC_READ_DISCRETE_INPUT:
	case MB_FC_READ_REGISTERS:
	case MB_FC_READ_INPUT_REGISTER:
	    modH->u8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
	    modH->u8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
	    modH->u8BufferSize = 6;
	    break;
	case MB_FC_WRITE_COIL:
	    modH->u8Buffer[ NB_HI ]      = (( telegram.u16reg[0]> 0) ? 0xff : 0);
	    modH->u8Buffer[ NB_LO ]      = 0;
	    modH->u8BufferSize = 6;
	    break;
	case MB_FC_WRITE_REGISTER:
	    modH->u8Buffer[ NB_HI ]      = highByte( telegram.u16reg[0]);
	    modH->u8Buffer[ NB_LO ]      = lowByte( telegram.u16reg[0]);
	    modH->u8BufferSize = 6;
	    break;
	case MB_FC_WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
	    u8regsno = telegram.u16CoilsNo / 16;
	    u8bytesno = u8regsno * 2;
	    if ((telegram.u16CoilsNo % 16) != 0)
	    {
	        u8bytesno++;
	        u8regsno++;
	    }

	    modH->u8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
	    modH->u8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
	    modH->u8Buffer[ BYTE_CNT ]    = u8bytesno;
	    modH->u8BufferSize = 7;

	    for (uint16_t i = 0; i < u8bytesno; i++)
	    {
	        if(i%2)
	        {
	        	modH->u8Buffer[ modH->u8BufferSize ] = lowByte( telegram.u16reg[ i/2 ] );
	        }
	        else
	        {
	        	modH->u8Buffer[  modH->u8BufferSize ] = highByte( telegram.u16reg[ i/2 ] );

	        }
	        modH->u8BufferSize++;
	    }
	    break;

	case MB_FC_WRITE_MULTIPLE_REGISTERS:
	    modH->u8Buffer[ NB_HI ]      = highByte(telegram.u16CoilsNo );
	    modH->u8Buffer[ NB_LO ]      = lowByte( telegram.u16CoilsNo );
	    modH->u8Buffer[ BYTE_CNT ]    = (uint8_t) ( telegram.u16CoilsNo * 2 );
	    modH->u8BufferSize = 7;

	    for (uint16_t i=0; i< telegram.u16CoilsNo; i++)
	    {

	        modH->u8Buffer[  modH->u8BufferSize ] = highByte(  telegram.u16reg[ i ] );
	        modH->u8BufferSize++;
	        modH->u8Buffer[  modH->u8BufferSize ] = lowByte( telegram.u16reg[ i ] );
	        modH->u8BufferSize++;
	    }
	    break;
	}


	sendTxBuffer(modH);

	xSemaphoreGive(modH->ModBusSphrHandle);

	modH->i8state = COM_WAITING;
	modH->i8lastError = 0;
	return 0;


}

#if ENABLE_TCP == 1

static  mb_errot_t TCPconnectserver(modbusHandler_t * modH, modbus_t *telegram)
{


	err_t err;
	 tcpclients_t *clientconn;


	//select the current connection slot to work with
    clientconn = &modH->newconns[modH->newconnIndex];

	if(telegram->u8clientID >= NUMBERTCPCONN )
	{
		return ERR_BAD_TCP_ID;
	}

	// if the connection is null open a new connection
	if (clientconn->conn == NULL)
	{
		 clientconn->conn = netconn_new(NETCONN_TCP);
	     if (clientconn->conn  == NULL)
	     {
	   	     while(1)
	   	     {
	     	  // error creating new connection check your configuration and heap size
	     	 }
	     }


	     err = netconn_connect(clientconn->conn, (ip_addr_t *)&telegram->xIpAddress, telegram->u16Port);

	     if (err  != ERR_OK )
	     {

	    	   //ModbusCloseConn(clientconn->conn);
	    	   ModbusCloseConnNull(modH);

	           return ERR_TIME_OUT;
	      }
	}
	return ERR_OK;
}


static mb_errot_t TCPgetRxBuffer(modbusHandler_t * modH)
{

	struct netbuf *inbuf;
	err_t err = ERR_TIME_OUT;
	char* buf;
	uint16_t buflen;
	uint16_t uLength;

	 tcpclients_t *clientconn;
	 //select the current connection slot to work with
     clientconn = &modH->newconns[modH->newconnIndex];

	netconn_set_recvtimeout(clientconn->conn, modH->u16timeOut);
	err = netconn_recv(clientconn->conn, &inbuf);

	uLength = 0;

    if (err == ERR_OK)
    {
    	err = netconn_err(clientconn->conn) ;
    	if (err == ERR_OK)
    	{
    		/* Read the data from the port, blocking if nothing yet there.
   	  	  	We assume the request (the part we care about) is in one netbuf */
  	        err = netbuf_data(inbuf, (void**)&buf, &buflen);
  	        if(err == ERR_OK )
  	        {
                 if( (buflen>11  && (modH->uModbusType == MB_SLAVE  ))  ||
                	 (buflen>=10 && (modH->uModbusType == MB_MASTER ))  ) // minimum frame size for modbus TCP
                 {
        	         if(buf[2] == 0 || buf[3] == 0 ) //validate protocol ID
        	         {
        	  	          uLength = (buf[4]<<8 & 0xff00) | buf[5];
        	  	          if(uLength< (MAX_BUFFER-2)  && (uLength + 6) <= buflen)
        	  	          {
        	  	  	          for(int i = 0; i < uLength; i++)
        	  	  	          {
        	  	  	  	         modH->u8Buffer[i] = buf[i+6];
        	  	  	          }
        	  	  	          modH->u16TransactionID = (buf[0]<<8 & 0xff00) | buf[1];
        	  	  	          modH->u8BufferSize = uLength + 2; //include 2 dummy bytes for CRC
        	  	           }
        	          }
                  }
  	        } // netbuf_data
  	        netbuf_delete(inbuf); //delete the buffer always
    	}
    }

    //netconn_close(modH->newconn);
	//netconn_delete(modH->newconn);
	return err;
}

#endif

void StartTaskModbusMaster(void *argument)
{

  modbusHandler_t *modH =  (modbusHandler_t *)argument;
  uint32_t ulNotificationValue;
  modbus_t telegram;



  for(;;)
  {
	  /*Wait indefinitely for a telegram to send */
	  xQueueReceive(modH->QueueTelegramHandle, &telegram, portMAX_DELAY);


#if ENABLE_TCP ==1
     if(modH->xTypeHW == TCP_HW)
     {
    	  modH->newconnIndex = telegram.u8clientID;
    	  ulNotificationValue = TCPconnectserver( modH, &telegram);
 	      if(ulNotificationValue == ERR_OK)
	      {


 	    	  SendQuery(modH, telegram);
 		     /* Block until a Modbus Frame arrives or query timeouts*/
 		      ulNotificationValue = TCPgetRxBuffer(modH); // TCP receives the data and the notification simultaneously since it is synchronous

 		      if (ulNotificationValue != ERR_OK) //close the TCP connection
 		      {

 		    	 //ModbusCloseConn(modH->newconns[modH->newconnIndex].conn);
 		    	 ModbusCloseConnNull(modH);

 		      }
	      }
 	      else
 	      {
 	    	 //ModbusCloseConn(modH->newconns[modH->newconnIndex].conn);
 	    	 ModbusCloseConnNull(modH);
 	      }
     }
     else // send a query for USART and USB_CDC
     {
   	   SendQuery(modH, telegram);
       /* Block until a Modbus Frame arrives or query timeouts*/
   	   ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
     }
#else
     // This is the case for implementations with only USART support
     SendQuery(modH, telegram);
     /* Block indefinitely until a Modbus Frame arrives or query timeouts*/
     ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#endif

	  // notify the task the request timeout
      modH->i8lastError = 0;
      if(ulNotificationValue)
      {
    	  modH->i8state = COM_IDLE;
    	  modH->i8lastError = ERR_TIME_OUT;
    	  modH->u16errCnt++;
    	  xTaskNotify((TaskHandle_t)telegram.u32CurrentTask, modH->i8lastError, eSetValueWithOverwrite);
    	  continue;
      }


#if ENABLE_USB_CDC ==1 || ENABLE_TCP ==1

      if(modH->xTypeHW == USART_HW) //TCP and USB_CDC use different methods to get the buffer
      {
    	  getRxBuffer(modH);
      }

#else
      getRxBuffer(modH);
#endif



	  if ( modH->u8BufferSize < 6){

		  modH->i8state = COM_IDLE;
		  modH->i8lastError = ERR_BAD_SIZE;
		  modH->u16errCnt++;
		  xTaskNotify((TaskHandle_t)telegram.u32CurrentTask, modH->i8lastError, eSetValueWithOverwrite);
		  continue;
	  }

	  xTimerStop(modH->xTimerTimeout,0); // cancel timeout timer


	  // validate message: id, CRC, FCT, exception
	  int8_t u8exception = validateAnswer(modH);
	  if (u8exception != 0)
	  {
		 modH->i8state = COM_IDLE;
         modH->i8lastError = u8exception;
		 xTaskNotify((TaskHandle_t)telegram.u32CurrentTask, modH->i8lastError, eSetValueWithOverwrite);
	     continue;
	  }

	  modH->i8lastError = u8exception;

	  xSemaphoreTake(modH->ModBusSphrHandle , portMAX_DELAY); //before processing the message get the semaphore
	  // process answer
	  switch( modH->u8Buffer[ FUNC ] )
	  {
	  case MB_FC_READ_COILS:
	  case MB_FC_READ_DISCRETE_INPUT:
	      //call get_FC1 to transfer the incoming message to u16regs buffer
	      get_FC1(modH);
	      break;
	  case MB_FC_READ_INPUT_REGISTER:
	  case MB_FC_READ_REGISTERS :
	      // call get_FC3 to transfer the incoming message to u16regs buffer
	      get_FC3(modH);
	      break;
	  case MB_FC_WRITE_COIL:
	  case MB_FC_WRITE_REGISTER :
	  case MB_FC_WRITE_MULTIPLE_COILS:
	  case MB_FC_WRITE_MULTIPLE_REGISTERS :
	      // nothing to do
	      break;
	  default:
	      break;
	  }
	  modH->i8state = COM_IDLE;

	  if (modH->i8lastError ==0) // no error the error_OK, we need to use a different value than 0 to detect the timeout
	  {
		  xSemaphoreGive(modH->ModBusSphrHandle); //Release the semaphore
		  xTaskNotify((TaskHandle_t)telegram.u32CurrentTask, ERR_OK_QUERY, eSetValueWithOverwrite);
	  }


	  continue;
	 }

}

/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void get_FC1(modbusHandler_t *modH)
{
    uint8_t u8byte, i;
    u8byte = 3;
     for (i=0; i< modH->u8Buffer[2]; i++) {

        if(i%2)
        {
        	modH->u16regs[i/2]= word(modH->u8Buffer[i+u8byte], lowByte(modH->u16regs[i/2]));
        }
        else
        {

        	modH->u16regs[i/2]= word(highByte(modH->u16regs[i/2]), modH->u8Buffer[i+u8byte]);
        }

     }
}

/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void get_FC3(modbusHandler_t *modH){
    uint8_t u8byte, i;
    u8byte = 3;

    for (i=0; i< modH->u8Buffer[ 2 ] /2; i++)
    {
    	modH->u16regs[ i ] = word(modH->u8Buffer[ u8byte ], modH->u8Buffer[ u8byte +1 ]);
        u8byte += 2;
    }
}
uint8_t validateAnswer(modbusHandler_t modH) {
    // Check message CRC vs calculated CRC
    #if ENABLE_TCP == 1
        if (modH->xTypeHW != TCP_HW) { 
    #endif
            uint16_t u16MsgCRC = (modH->u8Buffer[modH->u8BufferSize - 2] << 8) | modH->u8Buffer[modH->u8BufferSize - 1]; // Combine CRC Low & High bytes
            if (calcCRC(modH->u8Buffer, modH->u8BufferSize - 2) != u16MsgCRC) { // Compare calculated CRC with message CRC
                modH->u16errCnt++;
                return ERR_BAD_CRC;
            }
    #if ENABLE_TCP == 1
        }
    #endif

    // Check Modbus Exception
    if ((modH->u8Buffer[FUNC] & 0x80) != 0) { // Check bit 7 of Function Code
        modH->u16errCnt++;
        return ERR_EXCEPTION;
    }

    // Check Function Code
    bool isSupported = false;
    for (uint8_t i = 0; i < sizeof(fctsupported); i++) {
        if (fctsupported[i] == modH->u8Buffer[FUNC]) { // Check Function Code against supported Function Codes
            isSupported = true;
            break;
        }
    }

    if (!isSupported) {
        modH->u16errCnt++;
        return EXC_FUNC_CODE;
    }

    return 0; // OK, no exception code thrown
}
int16_t getRxBuffer(modbusHandler_t *modH){//This method moves Serial buffer data to the Modbus u8Buffer.

    int16_t i16result;

    if(modH->xTypeHW == USART_HW)
    {
    	HAL_UART_AbortReceive_IT(modH->port); // disable interrupts to avoid race conditions on serial port
    }

	if (modH->xBufferRX.overflow)
    {
       	RingClear(&modH->xBufferRX); // clean up the overflowed buffer
       	i16result =  ERR_BUFF_OVERFLOW;
    }
	else
	{
		modH->u8BufferSize = RingGetAllBytes(&modH->xBufferRX, modH->u8Buffer);
		modH->u16InCnt++;
		i16result = modH->u8BufferSize;
	}

	if(modH->xTypeHW == USART_HW)
	{
		HAL_UART_Receive_IT(modH->port, &modH->dataRX, 1);
	}

    return i16result;
}
uint8_t validateRequest(modbusHandler_t *modH){//This method validates slave incoming messages
	// check message crc vs calculated crc

#if ENABLE_TCP ==1
	    uint16_t u16MsgCRC;
		    u16MsgCRC= ((modH->u8Buffer[modH->u8BufferSize - 2] << 8)
		   	         | modH->u8Buffer[modH->u8BufferSize - 1]); // combine the crc Low & High bytes

	    if (modH->xTypeHW != TCP_HW)
	    {
	    	if ( calcCRC( modH->u8Buffer,  modH->u8BufferSize-2 ) != u16MsgCRC )
	    	{
	    		modH->u16errCnt ++;
	    		return ERR_BAD_CRC;
	    		}
	    }
#else
	    uint16_t u16MsgCRC;
	    u16MsgCRC= ((modH->u8Buffer[modH->u8BufferSize - 2] << 8)
	    		   	         | modH->u8Buffer[modH->u8BufferSize - 1]); // combine the crc Low & High bytes


	    if ( calcCRC( modH->u8Buffer,  modH->u8BufferSize-2 ) != u16MsgCRC )
	    {
	       		modH->u16errCnt ++;
	       		return ERR_BAD_CRC;
	    }


#endif

	    // check fct code
	    bool isSupported = false;
	    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
	    {
	        if (fctsupported[i] == modH->u8Buffer[FUNC])
	        {
	            isSupported = 1;
	            break;
	        }
	    }
	    if (!isSupported)
	    {
	    	modH->u16errCnt ++;
	        return EXC_FUNC_CODE;
	    }

	    // check start address & nb range
	    uint16_t u16AdRegs = 0;
	    uint16_t u16NRegs = 0;

	    //uint8_t u8regs;
	    switch ( modH->u8Buffer[ FUNC ] )
	    {
	    case MB_FC_READ_COILS:
	    case MB_FC_READ_DISCRETE_INPUT:
	    case MB_FC_WRITE_MULTIPLE_COILS:
	    	u16AdRegs = word( modH->u8Buffer[ ADD_HI ], modH->u8Buffer[ ADD_LO ]) / 16;
	    	u16NRegs = word( modH->u8Buffer[ NB_HI ], modH->u8Buffer[ NB_LO ]) /16;
	    	if(word( modH->u8Buffer[ NB_HI ], modH->u8Buffer[ NB_LO ]) % 16) u16NRegs++; // check for incomplete words
	    	// verify address range
	    	if((u16AdRegs + u16NRegs) > modH->u16regsize) return EXC_ADDR_RANGE;

	    	//verify answer frame size in bytes

	    	u16NRegs = word( modH->u8Buffer[ NB_HI ], modH->u8Buffer[ NB_LO ]) / 8;
	    	if(word( modH->u8Buffer[ NB_HI ], modH->u8Buffer[ NB_LO ]) % 8) u16NRegs++;
	    	u16NRegs = u16NRegs + 5; // adding the header  and CRC ( Slave address + Function code  + number of data bytes to follow + 2-byte CRC )
	        if(u16NRegs > 256) return EXC_REGS_QUANT;

	        break;
	    case MB_FC_WRITE_COIL:
	    	u16AdRegs = word( modH->u8Buffer[ ADD_HI ], modH->u8Buffer[ ADD_LO ]) / 16;
	    	if(word( modH->u8Buffer[ ADD_HI ], modH->u8Buffer[ ADD_LO ]) % 16) u16AdRegs++;	// check for incomplete words
	        if (u16AdRegs > modH->u16regsize) return EXC_ADDR_RANGE;
	        break;
	    case MB_FC_WRITE_REGISTER :
	    	u16AdRegs = word( modH->u8Buffer[ ADD_HI ], modH->u8Buffer[ ADD_LO ]);
	        if (u16AdRegs > modH-> u16regsize) return EXC_ADDR_RANGE;
	        break;
	    case MB_FC_READ_REGISTERS :
	    case MB_FC_READ_INPUT_REGISTER :
	    case MB_FC_WRITE_MULTIPLE_REGISTERS :
	    	u16AdRegs = word( modH->u8Buffer[ ADD_HI ], modH->u8Buffer[ ADD_LO ]);
	        u16NRegs = word( modH->u8Buffer[ NB_HI ], modH->u8Buffer[ NB_LO ]);
	        if (( u16AdRegs + u16NRegs ) > modH->u16regsize) return EXC_ADDR_RANGE;

	        //verify answer frame size in bytes
	        u16NRegs = u16NRegs*2 + 5; // adding the header  and CRC
	        if ( u16NRegs > 256 ) return EXC_REGS_QUANT;
	        break;
	    }
	    return 0; // OK, no exception code thrown

}
uint16_t word(uint8_t H, uint8_t L){//This method creates a word from 2 bytes
	bytesFields W;
	W.u8[0] = L;
	W.u8[1] = H;

	return W.u16[0];
}
uint16_t calcCRC(uint8_t Buffer, uint8_t u8length) {
    uint16_t crc = 0xFFFF; // Initial CRC value

    for (uint8_t i = 0; i < u8length; i++) { 
        crc ^= Buffer[i]; // XOR with current data byte

        for (uint8_t j = 0; j < 8; j++) { // For each bit in the data byte
            if (crc & 0x0001) { // If the least significant bit of CRC is 1
                crc >>= 1; // Right shift CRC
                crc ^= 0xA001; // XOR with CRC-16 constant
            } else {
                crc >>= 1; // Right shift CRC
            }
        }
    }
    return crc; // Return the calculated CRC value
}
void buildException( uint8_t u8exception, modbusHandler_t *modH ){
    uint8_t u8func = modH->u8Buffer[ FUNC ];  // get the original FUNC code

    modH->u8Buffer[ ID ]      = modH->u8id;
    modH->u8Buffer[ FUNC ]    = u8func + 0x80;
    modH->u8Buffer[ 2 ]       = u8exception;
    modH->u8BufferSize         = EXCEPTION_SIZE;
}


#if ENABLE_USB_CDC == 1
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
#endif
static void sendTxBuffer(modbusHandler_t *modH) {//send the handeler buffer on selected port
    // Append CRC to message
#if ENABLE_TCP == 1
    if (modH->xTypeHW != TCP_HW) {
#endif
        uint16_t u16crc = calcCRC(modH->u8Buffer, modH->u8BufferSize);
        modH->u8Buffer[modH->u8BufferSize++] = u16crc >> 8;
        modH->u8Buffer[modH->u8BufferSize++] = u16crc & 0x00FF;
#if ENABLE_TCP == 1
    }
#endif

#if ENABLE_USB_CDC == 1 || ENABLE_TCP == 1
    if (modH->xTypeHW == USART_HW || modH->xTypeHW == USART_HW_DMA) {
#endif
        if (modH->EN_Port != NULL) {
            // Enable transmitter, disable receiver to avoid echo on RS485 transceivers
            HAL_HalfDuplex_EnableTransmitter(modH->port);
            HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_SET);
        }

#if ENABLE_USART_DMA == 1
        if (modH->xTypeHW == USART_HW) {
            // Transfer buffer to serial line IT
            HAL_UART_Transmit_IT(modH->port, modH->u8Buffer, modH->u8BufferSize);
        } else {
            // Transfer buffer to serial line DMA
            HAL_UART_Transmit_DMA(modH->port, modH->u8Buffer, modH->u8BufferSize);
        }
#else
        // Transfer buffer to serial line IT by default
        HAL_UART_Transmit_IT(modH->port, modH->u8Buffer, modH->u8BufferSize);
#endif

        ulTaskNotifyTake(pdTRUE, 250); // Wait notification from TXE interrupt

        // Block until the last byte is sent out of the shifting buffer in USART
#if defined(STM32H7) || defined(STM32F3) || defined(STM32L4)
        while ((modH->port->Instance->ISR & USART_ISR_TC) == 0);
#else
        while ((modH->port->Instance->SR & USART_SR_TC) == 0);
#endif

        if (modH->EN_Port != NULL) {
            // Return RS485 transceiver to receive mode
            HAL_GPIO_WritePin(modH->EN_Port, modH->EN_Pin, GPIO_PIN_RESET);
            HAL_HalfDuplex_EnableReceiver(modH->port);
        }

        // Set timeout for master query
        if (modH->uModbusType == MB_MASTER) {
            xTimerReset(modH->xTimerTimeout, 0);
        }

#if ENABLE_USB_CDC == 1
    } else if (modH->xTypeHW == USB_CDC_HW) {
        CDC_Transmit_FS(modH->u8Buffer, modH->u8BufferSize);

        // Set timeout for master query
        if (modH->uModbusType == MB_MASTER) {
            xTimerReset(modH->xTimerTimeout, 0);
        }
    }
#endif

#if ENABLE_TCP == 1
    else if (modH->xTypeHW == TCP_HW) {
        uint8_t u8MBAPheader[6];
        struct netvector xNetVectors[2];
        size_t uBytesWritten;

        u8MBAPheader[0] = highByte(modH->u16TransactionID);
        u8MBAPheader[1] = lowByte(modH->u16TransactionID);
        u8MBAPheader[2] = u8MBAPheader[3] = 0; // Protocol ID
        u8MBAPheader[4] = 0; // High byte data length always 0
        u8MBAPheader[5] = modH->u8BufferSize; // Data length

        xNetVectors[0].len = sizeof(u8MBAPheader);
        xNetVectors[0].ptr = u8MBAPheader;

        xNetVectors[1].len = modH->u8BufferSize;
        xNetVectors[1].ptr = modH->u8Buffer;

        netconn_set_sendtimeout(modH->newconns[modH->newconnIndex].conn, modH->u16timeOut);

        err_enum_t err = netconn_write_vectors_partly(modH->newconns[modH->newconnIndex].conn, xNetVectors, 2, NETCONN_COPY, &uBytesWritten);
        
        if (err != ERR_OK) {
            ModbusCloseConnNull(modH);
        }

        if (modH->uModbusType == MB_MASTER) {
            xTimerReset(modH->xTimerTimeout, 0);
        }
    }
#endif

    modH->u8BufferSize = 0; // Reset buffer size
    modH->u16OutCnt++;      // Increase message counter
}


int8_t process_FC1(modbusHandler_t *modH) {// Function to process function code 1 (Read Coils)
    uint16_t startCoil = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t coilCount = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    // Calculate the number of bytes needed in the response
    uint8_t byteCount = (coilCount + 7) / 8; // Ceiling division
    modH->u8Buffer[ADD_HI] = byteCount;
    modH->u8BufferSize = ADD_LO;

    // Initialize the output buffer
    for (uint8_t i = 0; i < byteCount; i++) {
        modH->u8Buffer[modH->u8BufferSize + i] = 0; // Clear the byte
    }

    // Read coils and populate the response buffer
    for (uint16_t currentCoil = 0; currentCoil < coilCount; currentCoil++) {
        uint16_t coilIndex = startCoil + currentCoil;
        uint16_t registerIndex = coilIndex / 16;
        uint8_t bitIndex = coilIndex % 16;

        // Set the appropriate bit in the response buffer
        if (bitRead(modH->u16regs[registerIndex], bitIndex)) {
            modH->u8Buffer[modH->u8BufferSize + (currentCoil / 8)] |= (1 << (currentCoil % 8));
        }
    }

    // Update buffer size
    modH->u8BufferSize += byteCount;

    // Send the outgoing message
    sendTxBuffer(modH);
    return modH->u8BufferSize + 2; // Return total size including Modbus header
}
int8_t process_FC3(modbusHandler_t *modH) {// Function to process function code 3 (Read Holding Registers)
    uint16_t startAddress = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t registerCount = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    // Set the byte count in the response
    modH->u8Buffer[2] = registerCount * 2; // Each register is 2 bytes
    modH->u8BufferSize = 3; // Start with the Modbus header size

    // Populate the response buffer with register values
    for (uint16_t i = 0; i < registerCount; i++) {
        uint16_t currentRegister = startAddress + i;
        modH->u8Buffer[modH->u8BufferSize++] = highByte(modH->u16regs[currentRegister]);
        modH->u8Buffer[modH->u8BufferSize++] = lowByte(modH->u16regs[currentRegister]);
    }

    // Calculate total response size including Modbus header
    int8_t totalResponseSize = modH->u8BufferSize + 2;

    // Send the outgoing message
    sendTxBuffer(modH);

    return totalResponseSize; // Return total size including Modbus header
}
int8_t process_FC5(modbusHandler_t *modH) {// Function to process function code 5 (Write Single Coil)
    uint16_t coilAddress = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t currentRegister = coilAddress / 16;
    uint8_t currentBit = coilAddress % 16;

    // Determine the value to write to the coil (ON or OFF)
    uint16_t coilValue = (modH->u8Buffer[NB_HI] == 0xFF) ? 1 : 0;

    // Write to the specified coil
    bitWrite(modH->u16regs[currentRegister], currentBit, coilValue);

    // Prepare the response to send back to the master
    modH->u8BufferSize = 6; // Modbus response size
    sendTxBuffer(modH);

    return modH->u8BufferSize + 2; // Return total size including Modbus header
}
int8_t process_FC6(modbusHandler_t *modH) {// Function to process function code 6 (Write Single word)
    // Extract address and value from the received buffer
    uint16_t u16add = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    uint16_t u16val = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    // Update the register with the new value
    modH->u16regs[u16add] = u16val;

    // Prepare response header
    modH->u8BufferSize = RESPONSE_SIZE;

    // Send the response back
    sendTxBuffer(modH);

    // Return the size of the response buffer
    return modH->u8BufferSize + 2; // Including CRC
}
int8_t process_FC15(modbusHandler_t *modH){// Function to process function code 15 (Write muliple Coil)
    uint8_t currentBit, frameByte = 7, bitsCount = 0;
    uint16_t startCoil, coilCount, currentRegister;
    
    // Get the start coil and the number of coils from the incoming message
    startCoil = word(modH->u8Buffer[ADD_HI], modH->u8Buffer[ADD_LO]);
    coilCount = word(modH->u8Buffer[NB_HI], modH->u8Buffer[NB_LO]);

    // Read each coil from the register map and update its value
    for (uint16_t currentCoil = 0; currentCoil < coilCount; currentCoil++) {
        uint16_t coilIndex = startCoil + currentCoil;
        currentRegister = coilIndex / 16;
        currentBit = (uint8_t)(coilIndex % 16);

        // Read the bit from the incoming message and write it to the register
        bool bitValue = bitRead(modH->u8Buffer[frameByte], bitsCount);
        bitWrite(modH->u16regs[currentRegister], currentBit, bitValue);

        // Increment bit count and manage byte overflow
        bitsCount++;
        if (bitsCount > 7) {
            bitsCount = 0;
            frameByte++;
        }
    }

    // Prepare the outgoing message (copying the first 6 bytes)
    modH->u8BufferSize = 6;
    uint8_t responseSize = modH->u8BufferSize + 2; // Include CRC
    sendTxBuffer(modH);

    return responseSize;
}
int8_t process_FC16(modbusHandler_t *modH) {// Function to process function code 16 (Write muliple words)
    // Extract starting address and number of registers from the buffer
    uint16_t startAddress = (modH->u8Buffer[ADD_HI] << 8) | modH->u8Buffer[ADD_LO];
    uint16_t registerCount = (modH->u8Buffer[NB_HI] << 8) | modH->u8Buffer[NB_LO];

    // Set up response header
    modH->u8Buffer[NB_HI] = 0;
    modH->u8Buffer[NB_LO] = (uint8_t)registerCount; // Response size will not exceed 256 bytes
    modH->u8BufferSize = RESPONSE_SIZE;

    // Write registers based on incoming data
    for (uint16_t i = 0; i < registerCount; i++) {
        uint16_t temp = word(
            modH->u8Buffer[BYTE_CNT + 1 + i * 2],
            modH->u8Buffer[BYTE_CNT + 2 + i * 2]
        );
        
        modH->u16regs[startAddress + i] = temp;
    }

    // Prepare the size for the response buffer
    int8_t responseSize = modH->u8BufferSize + 2; // Include CRC
    sendTxBuffer(modH);

    return responseSize;
}





