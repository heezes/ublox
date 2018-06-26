/*
* Vecmocon Technologies
*
* uBlox Gps Library.
*
*@author: Altamash Abdul Rahim.
*/
/*
STAGE Beta: configure ublox 
STAGE 1: retireve lat, lon
STAGE 2: Apply ack check for config Msg
STAGE 3: Apply checksum verify to received Msg
TO DO:
-> Verify checksum function
*/
#include "ublox.h"
#include "queue_aar.h"
#include <string.h>

extern struct Queue* gps_queue;
extern UART_HandleTypeDef huart1;
#define uart_buffer_len 256
uint8_t uart_buffer[uart_buffer_len];
uint8_t *ptr = NULL;
struct ublox_event
{
	uint8_t class_id;
	uint8_t msg_id;
	uint8_t ck_a;
	uint8_t ck_b;
};
/*
UBLOX PROPRIETERY PACKET FORMAT
<--1 Byte--><--1 Byte--> <-1 Byte-> <-1 Byte-><-2 Byte->			   <-2 Byte->
 _______________________________________________________________________________
|Sync Char 1|Sync Char 2|Class     |ID      |Length       |Payload    |CK_A|CK_B|
|___________|___________|__________|________|_____________|___________|____|____|

						 <-----RANGE over Which Checksum Is CALC----->

Length is Little Endian
i.e. int i = 0x01234567;
will be stored as 67452301 

Checksum Formula:
 CK_ values are 8-Bit unsigned integers
 CK_A = 0, CK_B = 0
 For(I=0;I<N;I++)
 {
     CK_A = CK_A + Buffer[I]
     CK_B = CK_B + CK_A
 }
*/
const char UBX_HEADER[] = {0xB5,0x62};

/*Here pkt pointer is provided with +2 offset to remove sync char*/
void calculate_checksum(char* pkt, int len, struct ublox_event *event)
{
	int i = 0;
	uint8_t mck_a = 0, mck_b = 0;
	/*Incremented to ignore Sync data*/
	for(i =2;i<len;i++)
	{
		mck_a += pkt[i];  
		mck_b += mck_a;   
	}
	mck_a &= 0xFF; mck_b &= 0xFF;
	event->ck_a = mck_a;
	event->ck_b = mck_b;
}
int read_packet(uint8_t* pkt, int len, int timeout)
{
	int ret = 0;
	int tickTock = HAL_GetTick();
	int i = 0;
	while(i < len)
	{
		if(gps_queue->size != 0)
		{
			pkt[i] = dequeue(gps_queue);
			i++;	
		}
		if(HAL_GetTick() - tickTock > timeout)
		{
			ret = 1;
			break;
		}
	}
	return ret;
}
int send_packet(char* pkt, int len, int timeout)
{
	return (HAL_UART_Transmit(&huart1, (uint8_t *)pkt, len, timeout)==HAL_OK)? 0 : 1;
}
/*
Return payload len or -1 if fail.
*/
int read_payload(uint8_t* buf, struct ublox_event *event, int timeout)
{
	ptr = uart_buffer;
	int ret = 0;
	while(1)
	{
		if(read_packet(ptr,2,1000)!=0)
		{
			ret = -1;
			break;
		}
		if((ptr[0]==UBX_HEADER[0]) && (ptr[1]==UBX_HEADER[1]))
		{
			ret = 0;
			break;
		}
	}
	if(ret==0)
	{
		uint8_t payload_len = 0;
		ptr += 2;
		if(read_packet(ptr,4,200)!=0)
		{
			//DEBUG("UNABLE TO READ PACKET\r\n");
			return -1;
		}
		payload_len = ((ptr[2]<<0) & 0xFF)+((ptr[3]<<8) & 0xFF00);
		event->class_id = ptr[0];
		event->msg_id = ptr[1];
		payload_len += 2;
		ptr += 4;
		if(read_packet(ptr,payload_len,timeout)!=0)
		{
			//DEBUG("UNABLE TO READ PACKET\r\n");
			return -1;
		}
		payload_len += 4;
		calculate_checksum((char *)&uart_buffer,payload_len, event);
		if((uart_buffer[payload_len]!=event->ck_a) || (uart_buffer[payload_len+1]!=event->ck_b))
		{
			//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
			return -1;
		}
		ret = payload_len - 6;
		memcpy(buf, ptr,ret);
		memset(uart_buffer	,0,uart_buffer_len);
	}
	return ret;
}
int check_ack(struct ublox_event *event)
{
	int ret = 1;
	char body[2];
	if(read_payload((uint8_t *)&body, event, 1000) == -1)
	{return 1;}
	if((0x01==event->msg_id)&&(0x05==event->class_id))
	{
			ret = 0;
	}
	event->msg_id = body[0];
	event->class_id = body[1];
	return ret;
}
int ubx_cfgcfg(void)
{
	int ret = 0;
	struct ublox_event event;
	event.ck_a = 0;
	event.ck_b = 0;
	uint8_t request[] = {UBX_HEADER[0],UBX_HEADER[1],UBX_CFG_CFG,0x0D,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x17,0x00,0x00};
	int packet_len = sizeof(request);
	packet_len -= 2;
	calculate_checksum((char*)&request, packet_len, &event);
	request[packet_len] = event.ck_a;
	request[packet_len+1] = event.ck_b;
	packet_len += 2;
	if(send_packet((char*)&request, packet_len, 100)!=0)
	{
		ret = 1;
		//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
	}
	if(check_ack(&event)!=0)
	{
		ret = 1;
		//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
	}		
	return ret;
}
int disableNmea(void)
{
	int ret = 0;
    // Array of two bytes for CFG-MSG packets payload
    uint8_t messages[][2] = {
        {0xF0, 0x0A}, //DTM
        {0xF0, 0x44}, //GBQ
        {0xF0, 0x09}, //GBS
        {0xF0, 0x00}, //GGA
        {0xF0, 0x01}, //GLL
        {0xF0, 0x43}, //GLQ
        {0xF0, 0x42}, //GNQ
        {0xF0, 0x0D}, //GNS
        {0xF0, 0x40}, //GPQ
        {0xF0, 0x06}, //GRS
        {0xF0, 0x02}, //GSA
        {0xF0, 0x07}, //GST
        {0xF0, 0x03}, //GSV
        {0xF0, 0x04}, //RMC
        {0xF0, 0x41}, //TXT
        {0xF0, 0x0F}, //VLW
        {0xF0, 0x05}, //VTG
        {0xF0, 0x08}, //ZDA
        /*NMEA PUBX*/
        {0xF1, 0x41}, //CONFIG
        {0xF1, 0x00}, //POSITION
        {0xF1, 0x40}, //RATE
        {0xF1, 0x03}, //SVSTATUS
        {0xF1, 0x04}, //TIME
    };

    // CFG-MSG packet buffer
    uint8_t packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x01, // id
        0x03, // length
        0x00, // length
        0x00, // payload (first byte from messages array element)
        0x00, // payload (second byte from messages array element)
        0x00, // payload (not changed in the case)
        0x00, // CK_A
        0x00, // CK_B
    };
    uint8_t packetSize = sizeof(packet);

    // Offset to the place where payload starts
    uint8_t payloadOffset = 6;

    // Iterate over the messages array
    for (uint8_t i = 0; i < sizeof(messages) / sizeof(*messages); i++)
    {
        // Copy two bytes of payload to the packet buffer
        for (uint8_t j = 0; j < sizeof(*messages); j++)
        {
            packet[payloadOffset + j] = messages[i][j];
        }

        // Set checksum bytes to the null
        packet[packetSize - 2] = 0x00;
        packet[packetSize - 1] = 0x00;

        // Calculate checksum over the packet buffer excluding sync (first two) and checksum chars (last two)
        for (uint8_t j = 0; j < packetSize - 4; j++)
        {
            packet[packetSize - 2] += packet[2 + j];
            packet[packetSize - 1] += packet[packetSize - 2];
        }
        //if(send_packet((char*)&packet,packetSize,100)!=0)
        if(HAL_UART_Transmit(&huart1,(uint8_t *)&packet,packetSize,100)!=HAL_OK)
        {
        	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
        }
    }
    return ret;
}
int baud_config(void)
{
	int ret = 0;
	struct ublox_event event;
	uint8_t packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x00, // id
        0x14, // length
        0x00, // length
        0x01, // PORT ID
        0x00, // RESERVED
        0x00, // TX READY
        0x00, // TX READY
        0xC0, // MODE
        0x08, // MODE
        0x00, // MODE
        0x00, // MODE
        0x00, // BAUDRATE
        0xC2, // BAUDRATE
        0x01, // BAUDRATE
        0x00, // BAUDRATE
        0x03, // inProtoMask
        0x00, // inProtoMask
        0x03, // outProtoMask
        0x00, // outProtoMask
        0x00, // Timeout flags
        0x00, // Timeout flags
        0x00, // reserved
        0x00, // RESERVED
        0xC0, // CK_A
        0x7E, // CK_B
    };
    uint8_t packet_len = sizeof(packet);
    packet_len -= 2;
    calculate_checksum((char*)&packet,packet_len,&event);
    packet[packet_len] = event.ck_a;
	packet[packet_len+1] = event.ck_b;
	packet_len += 2;
    if(send_packet((char*)&packet,packet_len,100)!=0)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
    }
   return ret;
}
int set_rate(void)
{
	/*Currently Rate Set to 1 Hz.*/
	int ret = 0;
	struct ublox_event event;
  	uint8_t packet[] = {
 	0xB5,	//SYNC CHAR 1
 	0x62,	//SUNC CHAR 2
 	0x06,	//MSG ID
 	0x08,	//CLASS ID
 	0x06,	//LENGTH
 	0x00,	//LENGTH
 	0xE8,	//MEASURING RATE
 	0x03,	//MEASURING RATE
 	0x01,	//NAV RATE
 	0x00,	//NAV RATE
 	0x00,	//TIME REF
 	0x00,	//TIME REF
 	0x00,	//CK_A
 	0x00	//CK_B
 };
    uint8_t packet_len = sizeof(packet);
    packet_len -= 2;
    calculate_checksum((char*)&packet,packet_len,&event);
    packet[packet_len] = event.ck_a;
	packet[packet_len+1] = event.ck_b;
	packet_len += 2;
    if(send_packet((char*)&packet,packet_len,100)!=0)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
    }
    if(check_ack(&event)!=0)
    {
    	ret = 1;
        //DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
    }
   return ret;
}
// Send a packet to the receiver to enable NAV-PVT messages
int PERIODIC_UBX_NAV_PVT(void)
{
	int ret = 0;
	struct ublox_event event;
    // CFG-MSG packet
    uint8_t packet[] = {
        0xB5, // sync char 1
        0x62, // sync char 2
        0x06, // class
        0x01, // id
        0x03, // length
        0x00, // length
        0x01, // payload
        0x07, // payload
        0x01, // payload
        0x13, // CK_A
        0x51, // CK_B
    };
    if(send_packet((char*)&packet,sizeof(packet),100)!=0)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
    }
    if(check_ack(&event)!=0)
    {
    	ret = 1;
        //DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
    }
   return ret;
}
int POLL_UBX_NAV_POSLLH(struct ublox_pos_data* pos_data)
{
	int ret = 0;
	struct ublox_event event;
	uint8_t packet[] = {UBX_HEADER[0],UBX_HEADER[1],0x01,0x02,0x00,0x00,0x00,0x00};
    uint8_t packet_len = sizeof(packet);
    packet_len -= 2;
    calculate_checksum((char*)&packet,packet_len,&event);
    packet[packet_len] = event.ck_a;
	packet[packet_len+1] = event.ck_b;
	packet_len += 2;
    if(send_packet((char*)&packet,packet_len,100)!=0)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
    }
    HAL_Delay(1000);
    uint8_t request_data[40];
    if(read_payload(request_data,&event, 1000)==-1)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	//ret = 1;
    	return 1;
    }
    pos_data->i = (request_data[0] & 0xff)+((request_data[1]<<8) & 0xff00)+((request_data[2]<<16) & 0xff0000)+((request_data[3]<<24) & 0xff000000);
    pos_data->latitude = (request_data[4] & 0xff)+((request_data[5]<<8) & 0xff00)+((request_data[6]<<16) & 0xff0000)+((request_data[7]<<24) & 0xff000000);
    pos_data->longitude = (request_data[8] & 0xff)+((request_data[9]<<8) & 0xff00)+((request_data[10]<<16) & 0xff0000)+((request_data[11]<<24) & 0xff000000);
    pos_data->height = (request_data[12] & 0xff)+((request_data[13]<<8) & 0xff00)+((request_data[14]<<16) & 0xff0000)+((request_data[15]<<24) & 0xff000000);
    pos_data->hmsl = (request_data[16] & 0xff)+((request_data[17]<<8) & 0xff00)+((request_data[18]<<16) & 0xff0000)+((request_data[19]<<24) & 0xff000000);
    pos_data->lat = (float)pos_data->latitude/10000000;
    pos_data->lon = (float)pos_data->longitude/10000000;
    return ret;
}
int POLL_UBX_NAV_PVT(struct ublox_nav_data *data)
{
	int ret = 0;
	struct ublox_event event;
	uint8_t packet[] = {UBX_HEADER[0],UBX_HEADER[1],0x01,0x07,0x00,0x00,0x00,0x00};
    uint8_t packet_len = sizeof(packet);
    packet_len -= 2;
    calculate_checksum((char*)&packet,packet_len,&event);
    packet[packet_len] = event.ck_a;
	packet[packet_len+1] = event.ck_b;
	packet_len += 2;
    if(send_packet((char*)&packet,packet_len,100)!=0)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
       	return ret;
    }
    HAL_Delay(200);
    uint8_t dt[100];
    if(read_payload(dt,&event, 2000)==-1)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
       	return ret;
    }
    data->fixType = ((dt[20]<<0) & 0xff);
    if(data->fixType != 0 || data->fixType != 5)
    {
		data->iTow = ((dt[0]<<0) & 0xff)+((dt[1]<<8) & 0xff00)+((dt[2]<<16) & 0xff0000)+((dt[3]<<24) & 0xff000000);
		data->year = ((dt[4]<<0) & 0xff)+((dt[5]<<8) & 0xff00);
		data->month = ((dt[6]<<0) & 0xff);
		data->day = ((dt[7]<<0) & 0xff);
		data->hour = ((dt[8]<<0) & 0xff);
		data->min = ((dt[9]<<0) & 0xff);
		data->sec = ((dt[10]<<0) & 0xff);
		data->valid = ((dt[11]<<0) & 0xff);
		data->tAcc = ((dt[12]<<0) & 0xff)+((dt[13]<<8) & 0xff00)+((dt[14]<<16) & 0xff0000)+((dt[15]<<24) & 0xff000000);
		data->nano = ((dt[16]<<0) & 0xff)+((dt[17]<<8) & 0xff00)+((dt[18]<<16) & 0xff0000)+((dt[19]<<24) & 0xff000000);
		data->flags = ((dt[21]<<0) & 0xff);
		data->flags2 = ((dt[22]<<0) & 0xff);
		data->numSv = ((dt[23]<<0) & 0xff);
		data->lon = ((dt[24]<<0) & 0xff)+((dt[25]<<8) & 0xff00)+((dt[26]<<16) & 0xff0000)+((dt[27]<<24) & 0xff000000);
		data->lat = ((dt[28]<<0) & 0xff)+((dt[29]<<8) & 0xff00)+((dt[30]<<16) & 0xff0000)+((dt[31]<<24) & 0xff000000);
		data->height = ((dt[32]<<0) & 0xff)+((dt[33]<<8) & 0xff00)+((dt[34]<<16) & 0xff0000)+((dt[35]<<24) & 0xff000000);
		data->hMSL = ((dt[36]<<0) & 0xff)+((dt[37]<<8) & 0xff00)+((dt[38]<<16) & 0xff0000)+((dt[39]<<24) & 0xff000000);
		data->hAcc = ((dt[40]<<0) & 0xff)+((dt[41]<<8) & 0xff00)+((dt[42]<<16) & 0xff0000)+((dt[43]<<24) & 0xff000000);
		data->vAcc = ((dt[44]<<0) & 0xff)+((dt[45]<<8) & 0xff00)+((dt[46]<<16) & 0xff0000)+((dt[47]<<24) & 0xff000000);
		data->velN = ((dt[48]<<0) & 0xff)+((dt[49]<<8) & 0xff00)+((dt[50]<<16) & 0xff0000)+((dt[51]<<24) & 0xff000000);
		data->velE = ((dt[52]<<0) & 0xff)+((dt[53]<<8) & 0xff00)+((dt[54]<<16) & 0xff0000)+((dt[55]<<24) & 0xff000000);
		data->velD = ((dt[56]<<0) & 0xff)+((dt[57]<<8) & 0xff00)+((dt[58]<<16) & 0xff0000)+((dt[59]<<24) & 0xff000000);
		data->gSpeed = ((dt[60]<<0) & 0xff)+((dt[61]<<8) & 0xff00)+((dt[62]<<16) & 0xff0000)+((dt[63]<<24) & 0xff000000);
		data->headMot = ((dt[64]<<0) & 0xff)+((dt[65]<<8) & 0xff00)+((dt[66]<<16) & 0xff0000)+((dt[67]<<24) & 0xff000000);
		data->sAcc = ((dt[68]<<0) & 0xff)+((dt[69]<<8) & 0xff00)+((dt[70]<<16) & 0xff0000)+((dt[71]<<24) & 0xff000000);
		data->headAcc = ((dt[72]<<0) & 0xff)+((dt[73]<<8) & 0xff00)+((dt[74]<<16) & 0xff0000)+((dt[75]<<24) & 0xff000000);
		data->pDOP = ((dt[76]<<0) & 0xff)+((dt[77]<<8) & 0xff00)+((dt[78]<<16) & 0xff0000)+((dt[79]<<24) & 0xff000000);
		data->headVeh = ((dt[84]<<0) & 0xff)+((dt[85]<<8) & 0xff00)+((dt[86]<<16) & 0xff0000)+((dt[87]<<24) & 0xff000000);
    }
    return ret;
}
int UBX_AID_INI(void)
{
	int ret = 0;
	struct ublox_event event;
	// 0768E2E8->(ECEF (cm)) = 285438310 = 0x11 0x03 0x71 0x66
	// 20972F3E->(ECEF (cm)) = 771911880 = 0x2E 0x02 0x70 0xC8
	uint8_t packet[] = {UBX_HEADER[0],UBX_HEADER[1],0x0B,0x01,0x30,0x00,
			0x66,0x71,0x03,0x11,
			0xC8,0x70,0x02,0x2E,
			0x00,0x00,0x00,0x00,
			0xA0,0x86,0x01,0x00,
			0x00,0x00,
			0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x00,0x00,0x00,0x00,
			0x61,0x00,0x00,0x00,
			0x00,0x00
	};
	//uint8_t packet[] = {0xB5,0x62,0x0B,0x01,0x00,0x00,0x00,0x00};
    uint8_t packet_len = sizeof(packet);
    packet_len -= 2;
    calculate_checksum((char*)&packet,packet_len,&event);
    packet[packet_len] = event.ck_a;
	packet[packet_len+1] = event.ck_b;
	packet_len += 2;
    if(send_packet((char*)&packet,packet_len,100)!=0)
    {
       	//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
       	ret = 1;
       	return ret;
    }
    return ret;
}
int reset(void)
{
	uint8_t packet[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0xFF, 0xFF, 0x02, 0x00, 0x0E, 0x61};
	uint8_t packet_len = sizeof(packet);
	if(send_packet((char*)&packet,packet_len,100)!=0)
	{
		return 1;
	}
	return 0;
}
int UBX_MGA_INI_POS_LLH(void)
{
	int ret = 1;
	struct ublox_event event;
	uint8_t packet[] = {UBX_HEADER[0],UBX_HEADER[1],0x13,0x40,0x14,0x00,
			0x01,				 // Message Type
			0x00,				 // Message Version
			0x00,0x00, 			 // Reserved
			0x66,0x71,0x03,0x11, // Latitude
			0xC8,0x70,0x02,0x2E, // Longitude
			0xD8,0x59,0x00,0x00, // Altitude
			0x20,0xA1,0x07,0x00, // Position Accuracy
	};
	uint8_t packet_len = sizeof(packet);
    packet_len -= 2;
    calculate_checksum((char*)&packet,packet_len,&event);
    packet[packet_len] = event.ck_a;
	packet[packet_len+1] = event.ck_b;
	packet_len += 2;
	if(send_packet((char*)&packet,packet_len,100)!=0)
	{
		//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
		ret = 1;
		return ret;
	}
	char body[10];
	ret = read_payload((uint8_t *)&body, &event, 1000);
	if(ret != -1)
	{
		if((0x60!=event.msg_id)&&(0x13!=event.class_id))
		{
			//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
			ret = 1;
			return ret;
		}
		if(body[0]==0)
		{
			//DEBUG("Failed in %s at Line %d\r\n",__FILE__,__LINE__);
			ret = 1;
			return ret;
		}
	}
	return ret;
}
