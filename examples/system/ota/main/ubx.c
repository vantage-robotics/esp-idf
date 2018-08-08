/*
 * ubx.c
 *
 *  Created on: Aug 6, 2018
 *      Author: Xensr
 */
#include "ubx.h"

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include "esp_log.h"
#include <stddef.h>
#include <string.h>


static const int RX_BUF_SIZE = 1024;
static const int TX_BUF_SIZE = 128;
static const char *TAG = "ubx";

#define EX_UART_NUM UART_NUM_1
#define TXD_PIN (GPIO_NUM_13)
#define RXD_PIN (GPIO_NUM_14)

#define UBX_CONFIG_TIMEOUT 100
#define nullptr ((void*)0)

uint32_t pvtUpdates;
ubx_payload_rx_nav_pvt_t *currPvt;

bool			_configured;
ubx_ack_state_t		_ack_state;
bool			_got_posllh;
bool			_got_velned;
ubx_decode_state_t	_decode_state;
uint16_t		_rx_msg;
ubx_rxmsg_state_t	_rx_state;
uint16_t		_rx_payload_length;
uint16_t		_rx_payload_index;
uint8_t			_rx_ck_a;
uint8_t			_rx_ck_b;
uint16_t		_ack_waiting_msg;
ubx_buf_t		_buf;
uint32_t		_ubx_version;
bool			_use_nav_pvt;


/**
 * Parse the binary UBX packet
 */
int			parse_char(const uint8_t b);

/**
 * Start payload rx
 */
int			payload_rx_init(void);

/**
 * Add payload rx byte
 */
int			payload_rx_add(const uint8_t b);
int			payload_rx_add_mon_ver(const uint8_t b);

/**
 * Finish payload rx
 */
int			payload_rx_done(void);

/**
 * Reset the parse state machine for a fresh start
 */
void			decode_init(void);

/**
 * While parsing add every byte (except the sync bytes) to the checksum
 */
void			add_byte_to_checksum(const uint8_t);

/**
 * Send a message
 */
void			send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length);

/**
 * Configure message rate
 */
void			configure_message_rate(const uint16_t msg, const uint8_t rate);

/**
 * Calculate & add checksum for given buffer
 */
void			calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum);

/**
 * Wait for message acknowledge
 */
int			wait_for_ack(const uint16_t msg, const unsigned timeout, const bool report);


int configure_pam(ubx_payload_rx_nav_pvt_t *pvt)
{
  _configured = false;
  currPvt = pvt;
  /* try different baudrates */
  const uint32_t baudrates[] = {9600};

  uint32_t baud_i;

  for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++)
  {
	ESP_LOGI(TAG, "trying baud:%d",baudrates[baud_i]);
    //set_baudrate(_fd, baudrate);
    //GPS_Uart.begin(baudrates[baud_i], SERIAL_8N1, 4, 5); //RX, TX
    const uart_config_t uart_config = {
    		.baud_rate = baudrates[baud_i],
    		.data_bits = UART_DATA_8_BITS,
    		.parity = UART_PARITY_DISABLE,
    		.stop_bits = UART_STOP_BITS_1,
    		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    	};

    //uart_driver_delete(UART_NUM_1);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, TX_BUF_SIZE * 2, 0, NULL, 0);

    /* flush input and wait for at least 20 ms silence */
    decode_init();
    receive(200);
    decode_init();

    /* Send a CFG-PRT message to set the UBX protocol for in and out
     * and leave the baudrate as it is, we just want an ACK-ACK for this */
    memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
    _buf.payload_tx_cfg_prt.portID    = UBX_TX_CFG_PRT_PORTID;
    _buf.payload_tx_cfg_prt.mode    = UBX_TX_CFG_PRT_MODE;
    _buf.payload_tx_cfg_prt.baudRate  = baudrates[baud_i];
    _buf.payload_tx_cfg_prt.inProtoMask = UBX_TX_CFG_PRT_INPROTOMASK;
    _buf.payload_tx_cfg_prt.outProtoMask  = UBX_TX_CFG_PRT_OUTPROTOMASK;

    send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));

    if (wait_for_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false) < 0) {
      /* try next baudrate */
      continue;
    }

    ESP_LOGI(TAG, "baud got response");

    /* Send a CFG-PRT message again, this time change the baudrate */
    memset(&_buf.payload_tx_cfg_prt, 0, sizeof(_buf.payload_tx_cfg_prt));
    _buf.payload_tx_cfg_prt.portID    = UBX_TX_CFG_PRT_PORTID;
    _buf.payload_tx_cfg_prt.mode    = UBX_TX_CFG_PRT_MODE;
    _buf.payload_tx_cfg_prt.baudRate  = UBX_TX_CFG_PRT_BAUDRATE;
    _buf.payload_tx_cfg_prt.inProtoMask = UBX_TX_CFG_PRT_INPROTOMASK;
    _buf.payload_tx_cfg_prt.outProtoMask  = UBX_TX_CFG_PRT_OUTPROTOMASK;

    send_message(UBX_MSG_CFG_PRT, _buf.raw, sizeof(_buf.payload_tx_cfg_prt));

    /* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
    wait_for_ack(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);

    if (UBX_TX_CFG_PRT_BAUDRATE != baudrates[baud_i]) {
    	const uart_config_t uart_config = {
				.baud_rate = UBX_TX_CFG_PRT_BAUDRATE,
				.data_bits = UART_DATA_8_BITS,
				.parity = UART_PARITY_DISABLE,
				.stop_bits = UART_STOP_BITS_1,
				.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
			};
        uart_driver_delete(UART_NUM_1);
		uart_param_config(UART_NUM_1, &uart_config);
		uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
		// We won't use a buffer for sending data.
		uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    }

    /* at this point we have correct baudrate on both ends */
    break;
  }

  if (baud_i >= sizeof(baudrates) / sizeof(baudrates[0])) {
    return 1; // connection and/or baudrate detection failed
  }

  /* Send a CFG-RATE message to define update rate */
  memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
  _buf.payload_tx_cfg_rate.measRate = UBX_TX_CFG_RATE_MEASINTERVAL;
  _buf.payload_tx_cfg_rate.navRate  = UBX_TX_CFG_RATE_NAVRATE;
  _buf.payload_tx_cfg_rate.timeRef  = UBX_TX_CFG_RATE_TIMEREF;

  send_message(UBX_MSG_CFG_RATE, _buf.raw, sizeof(_buf.payload_tx_cfg_rate));

  if (wait_for_ack(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, true) < 0) {
    return 1;
  }

  /* send a NAV5 message to set the options for the internal filter */
  memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
  _buf.payload_tx_cfg_nav5.mask   = UBX_TX_CFG_NAV5_MASK;
  _buf.payload_tx_cfg_nav5.dynModel = UBX_TX_CFG_NAV5_DYNMODEL;
  _buf.payload_tx_cfg_nav5.fixMode  = UBX_TX_CFG_NAV5_FIXMODE;
  _buf.payload_tx_cfg_nav5.minElev  = UBX_TX_CFG_NAV5_MINELEV;

  send_message(UBX_MSG_CFG_NAV5, _buf.raw, sizeof(_buf.payload_tx_cfg_nav5));

  if (wait_for_ack(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, true) < 0) {
    return 1;
  }

#define UBX_CONFIGURE_SBAS
#ifdef UBX_CONFIGURE_SBAS
  /* send a SBAS message to set the SBAS options */
  memset(&_buf.payload_tx_cfg_sbas, 0, sizeof(_buf.payload_tx_cfg_sbas));
  _buf.payload_tx_cfg_sbas.mode   = UBX_TX_CFG_SBAS_MODE;
  _buf.payload_tx_cfg_sbas.usage = 0x07; // use for nav, diff corrections, integrity
  _buf.payload_tx_cfg_sbas.maxSBAS = 3; // dedicate 3 channels max

  send_message(UBX_MSG_CFG_SBAS, _buf.raw, sizeof(_buf.payload_tx_cfg_sbas));

  if (wait_for_ack(UBX_MSG_CFG_SBAS, UBX_CONFIG_TIMEOUT, true) < 0) {
    return 1;
  }
#endif

  /* configure message rates */
  /* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */

  /* try to set rate for NAV-PVT */
  /* (implemented for ubx7+ modules only, use NAV-SOL, NAV-POSLLH, NAV-VELNED and NAV-TIMEUTC for ubx6) */
  configure_message_rate(UBX_MSG_NAV_PVT, 1);
  if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
    _use_nav_pvt = false;
  } else {
    _use_nav_pvt = true;
  }

  if (!_use_nav_pvt) {
    configure_message_rate(UBX_MSG_NAV_TIMEUTC, 5);
    if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
      return 1;
    }

    configure_message_rate(UBX_MSG_NAV_POSLLH, 1);
    if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
      return 1;
    }

    configure_message_rate(UBX_MSG_NAV_SOL, 1);
    if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
      return 1;
    }

    configure_message_rate(UBX_MSG_NAV_VELNED, 1);
    if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
      return 1;
    }
  }

  //configure_message_rate(UBX_MSG_NAV_SVINFO, (_satellite_info != nullptr) ? 5 : 0);
  //if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
  //  return 1;
  //}

  configure_message_rate(UBX_MSG_MON_HW, 1);
  if (wait_for_ack(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
    return 1;
  }

  /* request module version information by sending an empty MON-VER message */
  send_message(UBX_MSG_MON_VER, nullptr, 0);

  _configured = true;
  return 0;
}

// 0 = decoding, 1 = message handled, 2 = sat info message handled
int parse_char(const uint8_t b)
{
  int ret = 0;

  switch (_decode_state) {

  /* Expecting Sync1 */
  case UBX_DECODE_SYNC1:
    if (b == UBX_SYNC1) { // Sync1 found --> expecting Sync2
      _decode_state = UBX_DECODE_SYNC2;
    }
    break;

  /* Expecting Sync2 */
  case UBX_DECODE_SYNC2:
    if (b == UBX_SYNC2) { // Sync2 found --> expecting Class
      _decode_state = UBX_DECODE_CLASS;

    } else {    // Sync1 not followed by Sync2: reset parser
      decode_init();
    }
    break;

  /* Expecting Class */
  case UBX_DECODE_CLASS:
    add_byte_to_checksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
    _rx_msg = b;
    _decode_state = UBX_DECODE_ID;
    break;

  /* Expecting ID */
  case UBX_DECODE_ID:
    add_byte_to_checksum(b);
    _rx_msg |= b << 8;
    _decode_state = UBX_DECODE_LENGTH1;
    break;

  /* Expecting first length byte */
  case UBX_DECODE_LENGTH1:
    add_byte_to_checksum(b);
    _rx_payload_length = b;
    _decode_state = UBX_DECODE_LENGTH2;
    break;

  /* Expecting second length byte */
  case UBX_DECODE_LENGTH2:
    add_byte_to_checksum(b);
    _rx_payload_length |= b << 8; // calculate payload size
    if (payload_rx_init() != 0) { // start payload reception
      // payload will not be handled, discard message
      decode_init();
    } else {
      _decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
    }
    break;

  /* Expecting payload */
  case UBX_DECODE_PAYLOAD:
    add_byte_to_checksum(b);
    switch (_rx_msg) {
    case UBX_MSG_NAV_SVINFO:
      //ret = payload_rx_add_nav_svinfo(b); // add a NAV-SVINFO payload byte
      break;
    case UBX_MSG_MON_VER:
      ret = payload_rx_add_mon_ver(b);  // add a MON-VER payload byte
      break;
    default:
      ret = payload_rx_add(b);    // add a payload byte
      break;
    }
    if (ret < 0) {
      // payload not handled, discard message
      decode_init();
    } else if (ret > 0) {
      // payload complete, expecting checksum
      _decode_state = UBX_DECODE_CHKSUM1;
    } else {
      // expecting more payload, stay in state UBX_DECODE_PAYLOAD
    }
    ret = 0;
    break;

  /* Expecting first checksum byte */
  case UBX_DECODE_CHKSUM1:
    if (_rx_ck_a != b) {
      decode_init();
    } else {
      _decode_state = UBX_DECODE_CHKSUM2;
    }
    break;

  /* Expecting second checksum byte */
  case UBX_DECODE_CHKSUM2:
    if (_rx_ck_b != b) {
    } else {
      ret = payload_rx_done();  // finish payload processing
    }
    decode_init();
    break;

  default:
    break;
  }

  return ret;
}


void decode_init(void)
{
  _decode_state = UBX_DECODE_SYNC1;
  _rx_ck_a = 0;
  _rx_ck_b = 0;
  _rx_payload_length = 0;
  _rx_payload_index = 0;
}


int wait_for_ack(const uint16_t msg, const unsigned timeout, const bool report)
{
  int ret = -1;

  _ack_state = UBX_ACK_WAITING;
  _ack_waiting_msg = msg; // memorize sent msg class&ID for ACK check

  receive(timeout);

  if (_ack_state == UBX_ACK_GOT_ACK)
  {
    ret = 0;  // ACK received ok
    ESP_LOGI(TAG, "ack");
  }
  else
  {
    ret = -1;
    ESP_LOGI(TAG, "nack");
  }

  _ack_state = UBX_ACK_IDLE;
  return ret;
}


int receive(const unsigned timeout)
{
  uint8_t buf[RX_BUF_SIZE];
  uint16_t bytesRead = 0;

  int handled = 0;

  bytesRead = uart_read_bytes(UART_NUM_1, buf, RX_BUF_SIZE, timeout / portTICK_PERIOD_MS);

  if (bytesRead)
  {
    ESP_LOGI(TAG, "bytesread:%d",bytesRead);
    for (int i = 0; i < bytesRead; i++)
    {
      handled |= parse_char(buf[i]);
    }
  }
  else
    return(-1);

  return(handled);
}


void configure_message_rate(const uint16_t msg, const uint8_t rate)
{
  ubx_payload_tx_cfg_msg_t cfg_msg; // don't use _buf (allow interleaved operation)

  cfg_msg.msg = msg;
  cfg_msg.rate  = rate;

  send_message(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
}


void send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
  ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
  ubx_checksum_t checksum = {0, 0};

  // Populate header
  header.msg  = msg;
  header.length = length;

  // Calculate checksum
  calc_checksum(((uint8_t*)&header) + 2, sizeof(header) - 2, &checksum);  // skip 2 sync bytes
  if (payload != nullptr)
    calc_checksum(payload, length, &checksum);

  // Send message
  //GPS_Uart.write((const uint8_t*)&header, sizeof(header));
  uart_write_bytes(UART_NUM_1, (const char*)&header, sizeof(header));
  if (payload != NULL)
  {
    uart_write_bytes(UART_NUM_1, (const char*)payload, length);
  }
  uart_write_bytes(UART_NUM_1, (const char*)&checksum, sizeof(checksum));
}

void calc_checksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
  for (uint16_t i = 0; i < length; i++) {
    checksum->ck_a = checksum->ck_a + buffer[i];
    checksum->ck_b = checksum->ck_b + checksum->ck_a;
  }
}

void add_byte_to_checksum(const uint8_t b)
{
  _rx_ck_a = _rx_ck_a + b;
  _rx_ck_b = _rx_ck_b + _rx_ck_a;
}

/**
 * Start payload rx
 */
int payload_rx_init(void)
{
  int ret = 0;

  _rx_state = UBX_RXMSG_HANDLE; // handle by default

  switch (_rx_msg) {
  case UBX_MSG_NAV_PVT:
    if (   (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)   /* u-blox 7 msg format */
        && (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8))  /* u-blox 8+ msg format */
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    else if (!_use_nav_pvt)
      _rx_state = UBX_RXMSG_DISABLE;  // disable if not using NAV-PVT
    break;

  case UBX_MSG_NAV_POSLLH:
    if (_rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t))
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    else if (_use_nav_pvt)
      _rx_state = UBX_RXMSG_DISABLE;  // disable if using NAV-PVT instead
    break;

  case UBX_MSG_NAV_SOL:
    if (_rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t))
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    else if (_use_nav_pvt)
      _rx_state = UBX_RXMSG_DISABLE;  // disable if using NAV-PVT instead
    break;

  case UBX_MSG_NAV_TIMEUTC:
    if (_rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t))
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    else if (_use_nav_pvt)
      _rx_state = UBX_RXMSG_DISABLE;  // disable if using NAV-PVT instead
    break;

/*
  case UBX_MSG_NAV_SVINFO:
    if (_satellite_info == nullptr)
      _rx_state = UBX_RXMSG_DISABLE;  // disable if sat info not requested
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    else
      memset(_satellite_info, 0, sizeof(*_satellite_info)); // initialize sat info
    break;
*/

  case UBX_MSG_NAV_VELNED:
    if (_rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t))
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    else if (_use_nav_pvt)
      _rx_state = UBX_RXMSG_DISABLE;  // disable if using NAV-PVT instead
    break;

  case UBX_MSG_MON_VER:
    break;    // unconditionally handle this message

  case UBX_MSG_MON_HW:
    if (   (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t)) /* u-blox 6 msg format */
        && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t)))  /* u-blox 7+ msg format */
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (!_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if not _configured
    break;

  case UBX_MSG_ACK_ACK:
    if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t))
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if _configured
    break;

  case UBX_MSG_ACK_NAK:
    if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t))
      _rx_state = UBX_RXMSG_ERROR_LENGTH;
    else if (_configured)
      _rx_state = UBX_RXMSG_IGNORE; // ignore if _configured
    break;

  default:
    _rx_state = UBX_RXMSG_DISABLE;  // disable all other messages
    break;
  }

  switch (_rx_state) {
  case UBX_RXMSG_HANDLE:  // handle message
  case UBX_RXMSG_IGNORE:  // ignore message but don't report error
    ret = 0;
    break;

  case UBX_RXMSG_DISABLE: // disable unexpected messages

    {
      //hrt_abstime t = hrt_absolute_time();

      //if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL) {
        /* don't attempt for every message to disable, some might not be disabled */
      //  _disable_cmd_last = t;
      //  configure_message_rate(_rx_msg, 0);
      //}
    }

    ret = -1; // return error, abort handling this message
    break;

  case UBX_RXMSG_ERROR_LENGTH:  // error: invalid length
    ret = -1; // return error, abort handling this message
    break;

  default:  // invalid message state
    ret = -1; // return error, abort handling this message
    break;
  }

  return ret;
}

int payload_rx_add_mon_ver(const uint8_t b)
{
  int ret = 0;

  if (_rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
    // Fill Part 1 buffer
    _buf.raw[_rx_payload_index] = b;
  } else {
    if (_rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
      // Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
      //_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
      //_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.hwVersion, _ubx_version);
    }
    // fill Part 2 buffer
    unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(ubx_payload_rx_mon_ver_part2_t);
    _buf.raw[buf_index] = b;
    if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
      // Part 2 complete: decode Part 2 buffer
    }
  }

  if (++_rx_payload_index >= _rx_payload_length) {
    ret = 1;  // payload received completely
  }

  return ret;
}

int payload_rx_add(const uint8_t b)
{
  int ret = 0;

  _buf.raw[_rx_payload_index] = b;

  if (++_rx_payload_index >= _rx_payload_length) {
    ret = 1;  // payload received completely
  }

  return ret;
}



int payload_rx_done(void)
{
  int ret = 0;

  // return if no message handled
  if (_rx_state != UBX_RXMSG_HANDLE) {
    return ret;
  }

  // handle message
  switch (_rx_msg) {

  case UBX_MSG_NAV_PVT:
    //Serial.println("Got NAV-PVT");

    *currPvt = _buf.payload_rx_nav_pvt;
    pvtUpdates++;
    //UBX_TRACE_RXMSG("Rx NAV-PVT\n");

    //_gps_position->fix_type    = _buf.payload_rx_nav_pvt.fixType;
    //if (_gps_position->fix_type >= 2)
      //_gps_position->vel_ne_valid = true;
    //if (_gps_position->fix_type >= 3)
      //_gps_position->vel_d_valid = true;


    //_gps_position->satellites_used  = _buf.payload_rx_nav_pvt.numSV;

    //_gps_position->lat    = _buf.payload_rx_nav_pvt.lat;
    //_gps_position->lon    = _buf.payload_rx_nav_pvt.lon;
    //_gps_position->alt    = _buf.payload_rx_nav_pvt.hMSL;

    //u-blox gives us falsely accurate eph, when velocity variance goes as high as 5 m/s
    //to facilitate switching out of pos control, we do our own eph here that takes into account speed variance
    //normal speed variance is about 0.2 m/s, at speed variance of 3, want eph to be above 10
    //eph = raw_eph + (speed_var-0.2) * 4
    //_gps_position->raw_eph    = (float)_buf.payload_rx_nav_pvt.hAcc * 1e-3f;
    //_gps_position->epv    = (float)_buf.payload_rx_nav_pvt.vAcc * 1e-3f;
    //_gps_position->s_variance_m_s = (float)_buf.payload_rx_nav_pvt.sAcc * 1e-3f;

    //_gps_position->vel_m_s    = (float)_buf.payload_rx_nav_pvt.gSpeed * 1e-3f;

    //_gps_position->vel_n_m_s  = (float)_buf.payload_rx_nav_pvt.velN * 1e-3f;
    //_gps_position->vel_e_m_s  = (float)_buf.payload_rx_nav_pvt.velE * 1e-3f;
    //_gps_position->vel_d_m_s  = (float)_buf.payload_rx_nav_pvt.velD * 1e-3f;

    //_gps_position->eph    = _gps_position->raw_eph;

    //_gps_position->cog_rad    = (float)_buf.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
    //_gps_position->c_variance_rad = (float)_buf.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;

    //Check if time and date fix flags are good
    if( (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
     && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
     && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED))
    {
      /* convert to unix timestamp */
      //struct tm timeinfo;
      //timeinfo.tm_year  = _buf.payload_rx_nav_pvt.year - 1900;
      //timeinfo.tm_mon   = _buf.payload_rx_nav_pvt.month - 1;
      //timeinfo.tm_mday  = _buf.payload_rx_nav_pvt.day;
      //timeinfo.tm_hour  = _buf.payload_rx_nav_pvt.hour;
      //timeinfo.tm_min   = _buf.payload_rx_nav_pvt.min;
      //timeinfo.tm_sec   = _buf.payload_rx_nav_pvt.sec;
      //time_t epoch = mktime(&timeinfo);

/*
      if (epoch > GPS_EPOCH_SECS) {
        // FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
        // and control its drift. Since we rely on the HRT for our monotonic
        // clock, updating it from time to time is safe.

        timespec ts;
        ts.tv_sec = epoch;
        ts.tv_nsec = _buf.payload_rx_nav_pvt.nano;
        if (clock_settime(CLOCK_REALTIME, &ts)) {
          //warn("failed setting clock");
        }

        _gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
        _gps_position->time_utc_usec += _buf.payload_rx_nav_timeutc.nano / 1000;
      } else {
        _gps_position->time_utc_usec = 0;
      }
      */
    }
    //_gps_position->day = _buf.payload_rx_nav_pvt.day;
    //_gps_position->month = _buf.payload_rx_nav_pvt.month;
    //_gps_position->year = _buf.payload_rx_nav_pvt.year;

    //_gps_position->timestamp_time   = hrt_absolute_time();
    //_gps_position->timestamp_velocity   = hrt_absolute_time();
    //_gps_position->timestamp_variance   = hrt_absolute_time();
    //_gps_position->timestamp_position = hrt_absolute_time();

    //_rate_count_vel++;
    //_rate_count_lat_lon++;

    _got_posllh = true;
    _got_velned = true;

    ret = 1;
    break;

  case UBX_MSG_NAV_POSLLH:
    //Serial.println("Got NAV-POSLLH");
    //UBX_TRACE_RXMSG("Rx NAV-POSLLH\n");

    //_gps_position->lat  = _buf.payload_rx_nav_posllh.lat;
    //_gps_position->lon  = _buf.payload_rx_nav_posllh.lon;
    //_gps_position->alt  = _buf.payload_rx_nav_posllh.hMSL;
    //_gps_position->eph  = (float)_buf.payload_rx_nav_posllh.hAcc * 1e-3f; // from mm to m
    //_gps_position->epv  = (float)_buf.payload_rx_nav_posllh.vAcc * 1e-3f; // from mm to m

    //_gps_position->timestamp_position = hrt_absolute_time();

    //_rate_count_lat_lon++;
    //_got_posllh = true;

    ret = 1;
    break;

  case UBX_MSG_NAV_SOL:
    //UBX_TRACE_RXMSG("Rx NAV-SOL\n");

    //_gps_position->fix_type   = _buf.payload_rx_nav_sol.gpsFix;
    //_gps_position->s_variance_m_s = (float)_buf.payload_rx_nav_sol.sAcc * 1e-2f;  // from cm to m
    //_gps_position->satellites_used  = _buf.payload_rx_nav_sol.numSV;

    //_gps_position->timestamp_variance = hrt_absolute_time();

    ret = 1;
    break;

  case UBX_MSG_NAV_TIMEUTC:
    //Serial.println("Got NAV-TIMEUTC");
    //UBX_TRACE_RXMSG("Rx NAV-TIMEUTC\n");

    //if(_buf.payload_rx_nav_timeutc.valid & UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC)
    //{
      // convert to unix timestamp
      //struct tm timeinfo;
      //timeinfo.tm_year  = _buf.payload_rx_nav_timeutc.year - 1900;
      //timeinfo.tm_mon   = _buf.payload_rx_nav_timeutc.month - 1;
      //timeinfo.tm_mday  = _buf.payload_rx_nav_timeutc.day;
      //timeinfo.tm_hour  = _buf.payload_rx_nav_timeutc.hour;
      //timeinfo.tm_min   = _buf.payload_rx_nav_timeutc.min;
      //timeinfo.tm_sec   = _buf.payload_rx_nav_timeutc.sec;
    //}

    ret = 1;
    break;

  case UBX_MSG_NAV_SVINFO:
    //UBX_TRACE_RXMSG("Rx NAV-SVINFO\n");

    // _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
    //_satellite_info->timestamp = hrt_absolute_time();

    ret = 2;
    break;

  case UBX_MSG_NAV_VELNED:
    //Serial.println("Got NAV-VELNED");
    //UBX_TRACE_RXMSG("Rx NAV-VELNED\n");

    //_gps_position->vel_m_s    = (float)_buf.payload_rx_nav_velned.speed * 1e-2f;
    //_gps_position->vel_n_m_s  = (float)_buf.payload_rx_nav_velned.velN * 1e-2f; /* NED NORTH velocity */
    //_gps_position->vel_e_m_s  = (float)_buf.payload_rx_nav_velned.velE * 1e-2f; /* NED EAST velocity */
    //_gps_position->vel_d_m_s  = (float)_buf.payload_rx_nav_velned.velD * 1e-2f; /* NED DOWN velocity */
    //_gps_position->cog_rad    = (float)_buf.payload_rx_nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f;
    //_gps_position->c_variance_rad = (float)_buf.payload_rx_nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
    //_gps_position->vel_ne_valid =  _gps_position->vel_d_valid = true;

    //_gps_position->timestamp_velocity = hrt_absolute_time();

    //_rate_count_vel++;
    //_got_velned = true;

    ret = 1;
    break;

  case UBX_MSG_MON_VER:
    //UBX_TRACE_RXMSG("Rx MON-VER\n");

    ret = 1;
    break;

  case UBX_MSG_MON_HW:
    //UBX_TRACE_RXMSG("Rx MON-HW\n");
    //Serial.println("Got MON-HW");

    switch (_rx_payload_length) {

    case sizeof(ubx_payload_rx_mon_hw_ubx6_t):  /* u-blox 6 msg format */
      //_gps_position->noise_per_ms   = _buf.payload_rx_mon_hw_ubx6.noisePerMS;
      //_gps_position->jamming_indicator  = _buf.payload_rx_mon_hw_ubx6.jamInd;

      ret = 1;
      break;

    case sizeof(ubx_payload_rx_mon_hw_ubx7_t):  /* u-blox 7+ msg format */
      //_gps_position->noise_per_ms   = _buf.payload_rx_mon_hw_ubx7.noisePerMS;
      //_gps_position->jamming_indicator  = _buf.payload_rx_mon_hw_ubx7.jamInd;

      ret = 1;
      break;

    default:    // unexpected payload size:
      ret = 0;  // don't handle message
      break;
    }
    break;

  case UBX_MSG_ACK_ACK:
    //UBX_TRACE_RXMSG("Rx ACK-ACK\n");

    if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
      _ack_state = UBX_ACK_GOT_ACK;
    }

    ret = 1;
    break;

  case UBX_MSG_ACK_NAK:
    //UBX_TRACE_RXMSG("Rx ACK-NAK\n");

    if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
      _ack_state = UBX_ACK_GOT_NAK;
    }

    ret = 1;
    break;

  default:
    break;
  }

  return ret;
}


