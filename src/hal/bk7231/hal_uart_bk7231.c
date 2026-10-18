#include "../hal_uart.h"
#include "../../new_pins.h"
#include "../../new_cfg.h"
#include "../../cmnds/cmd_public.h"
#include "../../cmnds/cmd_local.h"
#include "../../logging/logging.h"
#include "../../../beken378/func/user_driver/BkDriverUart.h"

// from uart_bk.c
extern void bk_send_byte(UINT8 uport, UINT8 data);

void test_ty_read_uart_data_to_buffer(int port, void* param)
{
	int rc = 0;
  int fbufindex = UART_GetBufIndexFromPort(port);

	while((rc = uart_read_byte(port)) != -1)
		UART_AppendByteToReceiveRingBufferEx(fbufindex,rc);
}

int bk_port_from_portindex(int auartindex) {
  // BK_UART_1 is defined to 0
  return (auartindex == UART_PORT_INDEX_1) ? BK_UART_2 : BK_UART_1; //just to avoid error if more uarts in future
}

// Manually drain the hardware UART RX FIFO into the ring buffer.
// Workaround for a BK7231N UART2 RX-interrupt reliability bug
// (openshwprojects/OpenBK7231T_App#1258): the callback registered via
// bk_uart_set_rx_callback does not reliably fire for UART2, but the
// bytes ARE present in hardware - uart_read_byte() still finds them
// when called directly. Call this from a quick/frequent tick instead
// of relying solely on the interrupt.
void HAL_UART_PollEx(int auartindex) {
  test_ty_read_uart_data_to_buffer(bk_port_from_portindex(auartindex), NULL);
}

void HAL_UART_SendByteEx(int auartindex, byte b)
{
	bk_send_byte(bk_port_from_portindex(auartindex), b);
}

int HAL_UART_InitEx(int auartindex, int baud, int parity, bool hwflowc, int txOverride, int rxOverride)
{
    bk_uart_config_t config;

    config.baud_rate = baud;
    config.data_width = 0x03;
    config.parity = parity;    //0:no parity,1:odd,2:even
    config.stop_bits = 0;   //0:1bit,1:2bit
    config.flow_control = hwflowc == false ? 0 : 3;   //FLOW_CTRL_DISABLED or FLOW_CTRL_RTS_CTS
    config.flags = 0;

    int g_chosenUART=bk_port_from_portindex(auartindex);
    bk_uart_initialize(g_chosenUART, &config, NULL);
    bk_uart_set_rx_callback(g_chosenUART, test_ty_read_uart_data_to_buffer, NULL);
	return 1;
}