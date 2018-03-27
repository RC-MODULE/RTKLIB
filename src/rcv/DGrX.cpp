#include "DGrX.hpp"
#include "rtklib.h"

/* input ublox raw message from stream -----------------------------------------
* fetch next ublox raw data and input a mesasge from stream
* args   : raw_t *raw   IO     receiver raw data control struct
*          unsigned char data I stream data (1 byte)
* return : status (-1: error message, 0: no message, 1: input observation data,
*                  2: input ephemeris, 3: input sbas message,
*                  9: input ion/utc parameter)
*
* notes  : to specify input options, set raw->opt to the following option
*          strings separated by spaces.
*
*          -EPHALL    : input all ephemerides
*          -INVCP     : invert polarity of carrier-phase
*          -TADJ=tint : adjust time tags to multiples of tint (sec)
*          -STD_SLIP=std: slip by std-dev of carrier phase under std
*
*          The supported messages are as follows.
*
*          UBX-RXM-RAW  : raw measurement data
*          UBX-RXM-RAWX : multi-gnss measurement data
*          UBX-RXM-SFRB : subframe buffer
*          UBX-RXM-SFRBX: subframe buffer extension
*
*          UBX-TRK-MEAS and UBX-TRK-SFRBX are based on NEO-M8N (F/W 2.01).
*          UBX-TRK-D5 is based on NEO-7N (F/W 1.00). They are not formally
*          documented and not supported by u-blox.
*          Users can use these messages by their own risk.
*-----------------------------------------------------------------------------*/
extern int input_dgr8(raw_t *raw, unsigned char data) {
 //   trace(5,"input_ubx: data=%02x\n",data);
 //   
 //   /* synchronize frame */
 //   if (raw->nbyte==0) {
 //       if (!sync_ubx(raw->buff,data)) return 0;
 //       raw->nbyte=2;
 //       return 0;
 //   }
 //   raw->buff[raw->nbyte++]=data;
 //   
 //   if (raw->nbyte==6) {
 //       if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
 //           trace(2,"ubx length error: len=%d\n",raw->len);
 //           raw->nbyte=0;
 //           return -1;
 //       }
 //   }
 //   if (raw->nbyte<6||raw->nbyte<raw->len) return 0;
 //   raw->nbyte=0;
 //   
 //   /* decode ublox raw message */
 //   return decode_ubx(raw);
 return 1;
}

/* input ublox raw message from file -------------------------------------------
* fetch next ublox raw data and input a message from file
* args   : raw_t  *raw   IO     receiver raw data control struct
*          FILE   *fp    I      file pointer
* return : status(-2: end of file, -1...9: same as above)
*-----------------------------------------------------------------------------*/
extern int input_dgr8(raw_t *raw, FILE *fp) {
 //   int i,data;
 //   
 //   trace(4,"input_ubxf:\n");
 //   
 //   /* synchronize frame */
 //   if (raw->nbyte==0) {
 //       for (i=0;;i++) {
 //           if ((data=fgetc(fp))==EOF) return -2;
 //           if (sync_ubx(raw->buff,(unsigned char)data)) break;
 //           if (i>=4096) return 0;
 //       }
 //   }
 //   if (fread(raw->buff+2,1,4,fp)<4) return -2;
 //   raw->nbyte=6;
 //   
 //   if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
 //       trace(2,"ubx length error: len=%d\n",raw->len);
 //       raw->nbyte=0;
 //       return -1;
 //   }
 //   if (fread(raw->buff+6,1,raw->len-6,fp)<(size_t)(raw->len-6)) return -2;
 //   raw->nbyte=0;
 //   
 //   /* decode ubx raw message */
 //   return decode_ubx(raw);
 return 1;
}
