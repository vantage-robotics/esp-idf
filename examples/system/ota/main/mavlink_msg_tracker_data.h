// MESSAGE TRACKER_DATA PACKING

#define MAVLINK_MSG_ID_TRACKER_DATA 161

MAVPACKED(
typedef struct __mavlink_tracker_data_t {
 uint32_t iTOW; /*< GPS Time of Week [ms]*/
 int32_t lon; /*< Longitude [1e-7 deg]*/
 int32_t lat; /*< Latitude [1e-7 deg]*/
 float gpsAlt; /*< gps Altitude [m]*/
 float baroAlt; /*< barometer Altitude [m]*/
 int32_t velN; /*< NED north velocity [mm/s]*/
 int32_t velE; /*< NED east velocity [mm/s]*/
 int32_t velD; /*< NED down velocity [mm/s]*/
 uint32_t sAcc; /*< Speed accuracy estimate [mm/s]*/
 uint32_t eph; /*< Horizontal accuracy [mm]*/
 uint32_t epv; /*< Vertical accuracy [mm]*/
 uint8_t severity; /*< Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.*/
 uint8_t fixType; /*< GNSSfix type: 0=Nofix,1=DeadReckoningonly,2=2Dfix,3=3d-fix,4=GNSS+dead reckoning,5=time only fix.*/
 uint8_t numSV; /*< Number of SVs used in Nav Solution*/
}) mavlink_tracker_data_t;

#define MAVLINK_MSG_ID_TRACKER_DATA_LEN 47
#define MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN 47
#define MAVLINK_MSG_ID_161_LEN 47
#define MAVLINK_MSG_ID_161_MIN_LEN 47

#define MAVLINK_MSG_ID_TRACKER_DATA_CRC 83
#define MAVLINK_MSG_ID_161_CRC 83



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_TRACKER_DATA { \
	161, \
	"TRACKER_DATA", \
	14, \
	{  { "iTOW", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_tracker_data_t, iTOW) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tracker_data_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tracker_data_t, lat) }, \
         { "gpsAlt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tracker_data_t, gpsAlt) }, \
         { "baroAlt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tracker_data_t, baroAlt) }, \
         { "velN", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_tracker_data_t, velN) }, \
         { "velE", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_tracker_data_t, velE) }, \
         { "velD", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_tracker_data_t, velD) }, \
         { "sAcc", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_tracker_data_t, sAcc) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_tracker_data_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_tracker_data_t, epv) }, \
         { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_tracker_data_t, severity) }, \
         { "fixType", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_tracker_data_t, fixType) }, \
         { "numSV", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_tracker_data_t, numSV) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_TRACKER_DATA { \
	"TRACKER_DATA", \
	14, \
	{  { "iTOW", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_tracker_data_t, iTOW) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tracker_data_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tracker_data_t, lat) }, \
         { "gpsAlt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_tracker_data_t, gpsAlt) }, \
         { "baroAlt", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_tracker_data_t, baroAlt) }, \
         { "velN", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_tracker_data_t, velN) }, \
         { "velE", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_tracker_data_t, velE) }, \
         { "velD", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_tracker_data_t, velD) }, \
         { "sAcc", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_tracker_data_t, sAcc) }, \
         { "eph", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_tracker_data_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_tracker_data_t, epv) }, \
         { "severity", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_tracker_data_t, severity) }, \
         { "fixType", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_tracker_data_t, fixType) }, \
         { "numSV", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_tracker_data_t, numSV) }, \
         } \
}
#endif

/**
 * @brief Pack a tracker_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param iTOW GPS Time of Week [ms]
 * @param fixType GNSSfix type: 0=Nofix,1=DeadReckoningonly,2=2Dfix,3=3d-fix,4=GNSS+dead reckoning,5=time only fix.
 * @param numSV Number of SVs used in Nav Solution
 * @param lon Longitude [1e-7 deg]
 * @param lat Latitude [1e-7 deg]
 * @param gpsAlt gps Altitude [m]
 * @param baroAlt barometer Altitude [m]
 * @param velN NED north velocity [mm/s]
 * @param velE NED east velocity [mm/s]
 * @param velD NED down velocity [mm/s]
 * @param sAcc Speed accuracy estimate [mm/s]
 * @param eph Horizontal accuracy [mm]
 * @param epv Vertical accuracy [mm]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracker_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t severity, uint32_t iTOW, uint8_t fixType, uint8_t numSV, int32_t lon, int32_t lat, float gpsAlt, float baroAlt, int32_t velN, int32_t velE, int32_t velD, uint32_t sAcc, uint32_t eph, uint32_t epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKER_DATA_LEN];
	_mav_put_uint32_t(buf, 0, iTOW);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_float(buf, 12, gpsAlt);
	_mav_put_float(buf, 16, baroAlt);
	_mav_put_int32_t(buf, 20, velN);
	_mav_put_int32_t(buf, 24, velE);
	_mav_put_int32_t(buf, 28, velD);
	_mav_put_uint32_t(buf, 32, sAcc);
	_mav_put_uint32_t(buf, 36, eph);
	_mav_put_uint32_t(buf, 40, epv);
	_mav_put_uint8_t(buf, 44, severity);
	_mav_put_uint8_t(buf, 45, fixType);
	_mav_put_uint8_t(buf, 46, numSV);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKER_DATA_LEN);
#else
	mavlink_tracker_data_t packet;
	packet.iTOW = iTOW;
	packet.lon = lon;
	packet.lat = lat;
	packet.gpsAlt = gpsAlt;
	packet.baroAlt = baroAlt;
	packet.velN = velN;
	packet.velE = velE;
	packet.velD = velD;
	packet.sAcc = sAcc;
	packet.eph = eph;
	packet.epv = epv;
	packet.severity = severity;
	packet.fixType = fixType;
	packet.numSV = numSV;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKER_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRACKER_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
}

/**
 * @brief Pack a tracker_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param iTOW GPS Time of Week [ms]
 * @param fixType GNSSfix type: 0=Nofix,1=DeadReckoningonly,2=2Dfix,3=3d-fix,4=GNSS+dead reckoning,5=time only fix.
 * @param numSV Number of SVs used in Nav Solution
 * @param lon Longitude [1e-7 deg]
 * @param lat Latitude [1e-7 deg]
 * @param gpsAlt gps Altitude [m]
 * @param baroAlt barometer Altitude [m]
 * @param velN NED north velocity [mm/s]
 * @param velE NED east velocity [mm/s]
 * @param velD NED down velocity [mm/s]
 * @param sAcc Speed accuracy estimate [mm/s]
 * @param eph Horizontal accuracy [mm]
 * @param epv Vertical accuracy [mm]
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tracker_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t severity,uint32_t iTOW,uint8_t fixType,uint8_t numSV,int32_t lon,int32_t lat,float gpsAlt,float baroAlt,int32_t velN,int32_t velE,int32_t velD,uint32_t sAcc,uint32_t eph,uint32_t epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKER_DATA_LEN];
	_mav_put_uint32_t(buf, 0, iTOW);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_float(buf, 12, gpsAlt);
	_mav_put_float(buf, 16, baroAlt);
	_mav_put_int32_t(buf, 20, velN);
	_mav_put_int32_t(buf, 24, velE);
	_mav_put_int32_t(buf, 28, velD);
	_mav_put_uint32_t(buf, 32, sAcc);
	_mav_put_uint32_t(buf, 36, eph);
	_mav_put_uint32_t(buf, 40, epv);
	_mav_put_uint8_t(buf, 44, severity);
	_mav_put_uint8_t(buf, 45, fixType);
	_mav_put_uint8_t(buf, 46, numSV);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TRACKER_DATA_LEN);
#else
	mavlink_tracker_data_t packet;
	packet.iTOW = iTOW;
	packet.lon = lon;
	packet.lat = lat;
	packet.gpsAlt = gpsAlt;
	packet.baroAlt = baroAlt;
	packet.velN = velN;
	packet.velE = velE;
	packet.velD = velD;
	packet.sAcc = sAcc;
	packet.eph = eph;
	packet.epv = epv;
	packet.severity = severity;
	packet.fixType = fixType;
	packet.numSV = numSV;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TRACKER_DATA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TRACKER_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
}

/**
 * @brief Encode a tracker_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tracker_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracker_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tracker_data_t* tracker_data)
{
	return mavlink_msg_tracker_data_pack(system_id, component_id, msg, tracker_data->severity, tracker_data->iTOW, tracker_data->fixType, tracker_data->numSV, tracker_data->lon, tracker_data->lat, tracker_data->gpsAlt, tracker_data->baroAlt, tracker_data->velN, tracker_data->velE, tracker_data->velD, tracker_data->sAcc, tracker_data->eph, tracker_data->epv);
}

/**
 * @brief Encode a tracker_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tracker_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tracker_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tracker_data_t* tracker_data)
{
	return mavlink_msg_tracker_data_pack_chan(system_id, component_id, chan, msg, tracker_data->severity, tracker_data->iTOW, tracker_data->fixType, tracker_data->numSV, tracker_data->lon, tracker_data->lat, tracker_data->gpsAlt, tracker_data->baroAlt, tracker_data->velN, tracker_data->velE, tracker_data->velD, tracker_data->sAcc, tracker_data->eph, tracker_data->epv);
}

/**
 * @brief Send a tracker_data message
 * @param chan MAVLink channel to send the message
 *
 * @param severity Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 * @param iTOW GPS Time of Week [ms]
 * @param fixType GNSSfix type: 0=Nofix,1=DeadReckoningonly,2=2Dfix,3=3d-fix,4=GNSS+dead reckoning,5=time only fix.
 * @param numSV Number of SVs used in Nav Solution
 * @param lon Longitude [1e-7 deg]
 * @param lat Latitude [1e-7 deg]
 * @param gpsAlt gps Altitude [m]
 * @param baroAlt barometer Altitude [m]
 * @param velN NED north velocity [mm/s]
 * @param velE NED east velocity [mm/s]
 * @param velD NED down velocity [mm/s]
 * @param sAcc Speed accuracy estimate [mm/s]
 * @param eph Horizontal accuracy [mm]
 * @param epv Vertical accuracy [mm]
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tracker_data_send(mavlink_channel_t chan, uint8_t severity, uint32_t iTOW, uint8_t fixType, uint8_t numSV, int32_t lon, int32_t lat, float gpsAlt, float baroAlt, int32_t velN, int32_t velE, int32_t velD, uint32_t sAcc, uint32_t eph, uint32_t epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TRACKER_DATA_LEN];
	_mav_put_uint32_t(buf, 0, iTOW);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_float(buf, 12, gpsAlt);
	_mav_put_float(buf, 16, baroAlt);
	_mav_put_int32_t(buf, 20, velN);
	_mav_put_int32_t(buf, 24, velE);
	_mav_put_int32_t(buf, 28, velD);
	_mav_put_uint32_t(buf, 32, sAcc);
	_mav_put_uint32_t(buf, 36, eph);
	_mav_put_uint32_t(buf, 40, epv);
	_mav_put_uint8_t(buf, 44, severity);
	_mav_put_uint8_t(buf, 45, fixType);
	_mav_put_uint8_t(buf, 46, numSV);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKER_DATA, buf, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
#else
	mavlink_tracker_data_t packet;
	packet.iTOW = iTOW;
	packet.lon = lon;
	packet.lat = lat;
	packet.gpsAlt = gpsAlt;
	packet.baroAlt = baroAlt;
	packet.velN = velN;
	packet.velE = velE;
	packet.velD = velD;
	packet.sAcc = sAcc;
	packet.eph = eph;
	packet.epv = epv;
	packet.severity = severity;
	packet.fixType = fixType;
	packet.numSV = numSV;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKER_DATA, (const char *)&packet, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
#endif
}

/**
 * @brief Send a tracker_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_tracker_data_send_struct(mavlink_channel_t chan, const mavlink_tracker_data_t* tracker_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_tracker_data_send(chan, tracker_data->severity, tracker_data->iTOW, tracker_data->fixType, tracker_data->numSV, tracker_data->lon, tracker_data->lat, tracker_data->gpsAlt, tracker_data->baroAlt, tracker_data->velN, tracker_data->velE, tracker_data->velD, tracker_data->sAcc, tracker_data->eph, tracker_data->epv);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKER_DATA, (const char *)tracker_data, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_TRACKER_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tracker_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t severity, uint32_t iTOW, uint8_t fixType, uint8_t numSV, int32_t lon, int32_t lat, float gpsAlt, float baroAlt, int32_t velN, int32_t velE, int32_t velD, uint32_t sAcc, uint32_t eph, uint32_t epv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, iTOW);
	_mav_put_int32_t(buf, 4, lon);
	_mav_put_int32_t(buf, 8, lat);
	_mav_put_float(buf, 12, gpsAlt);
	_mav_put_float(buf, 16, baroAlt);
	_mav_put_int32_t(buf, 20, velN);
	_mav_put_int32_t(buf, 24, velE);
	_mav_put_int32_t(buf, 28, velD);
	_mav_put_uint32_t(buf, 32, sAcc);
	_mav_put_uint32_t(buf, 36, eph);
	_mav_put_uint32_t(buf, 40, epv);
	_mav_put_uint8_t(buf, 44, severity);
	_mav_put_uint8_t(buf, 45, fixType);
	_mav_put_uint8_t(buf, 46, numSV);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKER_DATA, buf, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
#else
	mavlink_tracker_data_t *packet = (mavlink_tracker_data_t *)msgbuf;
	packet->iTOW = iTOW;
	packet->lon = lon;
	packet->lat = lat;
	packet->gpsAlt = gpsAlt;
	packet->baroAlt = baroAlt;
	packet->velN = velN;
	packet->velE = velE;
	packet->velD = velD;
	packet->sAcc = sAcc;
	packet->eph = eph;
	packet->epv = epv;
	packet->severity = severity;
	packet->fixType = fixType;
	packet->numSV = numSV;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TRACKER_DATA, (const char *)packet, MAVLINK_MSG_ID_TRACKER_DATA_MIN_LEN, MAVLINK_MSG_ID_TRACKER_DATA_LEN, MAVLINK_MSG_ID_TRACKER_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE TRACKER_DATA UNPACKING


/**
 * @brief Get field severity from tracker_data message
 *
 * @return Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY.
 */
static inline uint8_t mavlink_msg_tracker_data_get_severity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field iTOW from tracker_data message
 *
 * @return GPS Time of Week [ms]
 */
static inline uint32_t mavlink_msg_tracker_data_get_iTOW(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field fixType from tracker_data message
 *
 * @return GNSSfix type: 0=Nofix,1=DeadReckoningonly,2=2Dfix,3=3d-fix,4=GNSS+dead reckoning,5=time only fix.
 */
static inline uint8_t mavlink_msg_tracker_data_get_fixType(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field numSV from tracker_data message
 *
 * @return Number of SVs used in Nav Solution
 */
static inline uint8_t mavlink_msg_tracker_data_get_numSV(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field lon from tracker_data message
 *
 * @return Longitude [1e-7 deg]
 */
static inline int32_t mavlink_msg_tracker_data_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field lat from tracker_data message
 *
 * @return Latitude [1e-7 deg]
 */
static inline int32_t mavlink_msg_tracker_data_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field gpsAlt from tracker_data message
 *
 * @return gps Altitude [m]
 */
static inline float mavlink_msg_tracker_data_get_gpsAlt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field baroAlt from tracker_data message
 *
 * @return barometer Altitude [m]
 */
static inline float mavlink_msg_tracker_data_get_baroAlt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field velN from tracker_data message
 *
 * @return NED north velocity [mm/s]
 */
static inline int32_t mavlink_msg_tracker_data_get_velN(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field velE from tracker_data message
 *
 * @return NED east velocity [mm/s]
 */
static inline int32_t mavlink_msg_tracker_data_get_velE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field velD from tracker_data message
 *
 * @return NED down velocity [mm/s]
 */
static inline int32_t mavlink_msg_tracker_data_get_velD(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field sAcc from tracker_data message
 *
 * @return Speed accuracy estimate [mm/s]
 */
static inline uint32_t mavlink_msg_tracker_data_get_sAcc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field eph from tracker_data message
 *
 * @return Horizontal accuracy [mm]
 */
static inline uint32_t mavlink_msg_tracker_data_get_eph(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  36);
}

/**
 * @brief Get field epv from tracker_data message
 *
 * @return Vertical accuracy [mm]
 */
static inline uint32_t mavlink_msg_tracker_data_get_epv(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Decode a tracker_data message into a struct
 *
 * @param msg The message to decode
 * @param tracker_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_tracker_data_decode(const mavlink_message_t* msg, mavlink_tracker_data_t* tracker_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	tracker_data->iTOW = mavlink_msg_tracker_data_get_iTOW(msg);
	tracker_data->lon = mavlink_msg_tracker_data_get_lon(msg);
	tracker_data->lat = mavlink_msg_tracker_data_get_lat(msg);
	tracker_data->gpsAlt = mavlink_msg_tracker_data_get_gpsAlt(msg);
	tracker_data->baroAlt = mavlink_msg_tracker_data_get_baroAlt(msg);
	tracker_data->velN = mavlink_msg_tracker_data_get_velN(msg);
	tracker_data->velE = mavlink_msg_tracker_data_get_velE(msg);
	tracker_data->velD = mavlink_msg_tracker_data_get_velD(msg);
	tracker_data->sAcc = mavlink_msg_tracker_data_get_sAcc(msg);
	tracker_data->eph = mavlink_msg_tracker_data_get_eph(msg);
	tracker_data->epv = mavlink_msg_tracker_data_get_epv(msg);
	tracker_data->severity = mavlink_msg_tracker_data_get_severity(msg);
	tracker_data->fixType = mavlink_msg_tracker_data_get_fixType(msg);
	tracker_data->numSV = mavlink_msg_tracker_data_get_numSV(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_TRACKER_DATA_LEN? msg->len : MAVLINK_MSG_ID_TRACKER_DATA_LEN;
        memset(tracker_data, 0, MAVLINK_MSG_ID_TRACKER_DATA_LEN);
	memcpy(tracker_data, _MAV_PAYLOAD(msg), len);
#endif
}
