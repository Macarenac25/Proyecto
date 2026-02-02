#include "toolbox.h"
#include <utility.h>
#include <userint.h>
#include <ModbusFramework.h>


// Definitions private to the module
#define CRC16		0xA001					// Used in CRC calculation
static unsigned char crc_table_1[256];		// Table for CRC1 calculation
static unsigned char crc_table_2[256];		// Table for CRC2 calculation

typedef struct {			// STRUCTURE TO HANDLE COMMUNICATIONS VIA MODBUS
	int		adr;				// Slave address
	int		bSent;				// Bytes to send
	int		bRcvd;				// Bytes to receive
	unsigned char	msg[256];	// Message towards slave
	unsigned char	ans[256];	// Message from slave
} MB;
static MB mb;


//-------------------------------------------------------------------------------------------------
//                             Initialize tables for CRC calculation
//-------------------------------------------------------------------------------------------------
/// HIFN Init_CRC (void)
/// HIFN Function to inizialize CRC tables used for communicating over Modbus
void Init_CRC (void)
{
	unsigned char i;
	unsigned int mask, crc, mem;

	for (mask = 0; mask < 256; mask++) {
		crc = mask;
		for (i = 0; i < 8; i++) {
			mem = (unsigned int)(crc & 0x0001) ;
			crc /= 2;
			if (mem)
				crc ^= CRC16 ;
		}

		crc_table_2[mask] = (unsigned char) (crc & 0xff); // lobyte
		crc_table_1[mask] = (unsigned char) (crc >> 8);   // hibyte
	}

	return;
}

//-------------------------------------------------------------------------------------------------
//
//          Computes the CRC and append to the buffer with the data to be transmitted
//
//-------------------------------------------------------------------------------------------------
/// HIFN Calc_CRC(unsigned char * buf, unsigned char size)
/// HIFN Function to compute CRC and append to the message to be sent to slave
/// HIPAR buf/Buffer on which to calculate the CRC. Calculated CRC is appended to the buffer
/// HIPAR size/Number of bytes to be transmitted (updated after CRC calculation)
void Calc_CRC (unsigned char * buf, unsigned char *size)
{
	unsigned char car, i;
	unsigned char crc0, crc1;

	crc0 = 0xff;
	crc1 = 0xff;

	for (i = 0; i < *size; i++) {
		car  = buf[i];
		car  = * ((unsigned char *) buf + i);
		car ^= crc0;
		crc0 = (unsigned char) (crc1 ^ crc_table_2[car]);
		crc1 = crc_table_1[car];
	}

	* (buf + *size)     = crc0;  // CRC low
	* (buf + *size + 1) = crc1;  // CRC high
	*size += 2;

	return;
}

//-------------------------------------------------------------------------------------------------
//
//           CRC check and match with the CRC found on the received data buffer
//
//-------------------------------------------------------------------------------------------------
/// HIFN Check_CRC (unsigned char * buf, unsigned char size)
/// HIFN CRC check and match with the CRC found on the received data buffer
/// HIRET 0 if CRC ok, 1 if CRC bad
/// HIPAR buf/Buffer on which to calculate the CRC
/// HIPAR buf/Calculated CRC is compared with the one stored in buf
/// HIPAR size/Lenght of buffer (excluding CRC)
int Check_CRC (unsigned char * buf, unsigned char size)
{
	unsigned char car, i;
	unsigned char crc0, crc1;
	unsigned int  crc_calc, crc_read;

	crc0 = 0xff;
	crc1 = 0xff;

	for(i = 0; i < size; i++) {
		car  = buf[i];
		car  = * ((unsigned char *) buf + i);
		car ^= crc0;
		crc0 = (unsigned char) (crc1 ^ crc_table_2[car]);
		crc1 = crc_table_1[car];
	}

	crc_calc = (crc1 << 8) + crc0;
	crc_read = * (buf + size) + 256 * (* (buf + size + 1));

	return (crc_read != crc_calc);
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a double word from slave to a float for PC
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_dword_to_float (unsigned char * buf, float dvd)
/// HIFN Convert a dword from big-endian to little-endian
/// HIFN Calculated value is cast to a float
/// HIRET The calculated float value
/// HIPAR buf/Buffer in memory that holds the value to be converted
/// HIPAR dvd/Conversion factor to use
float conv_dword_to_float (unsigned char * buf, float dvd)
{
	return ((int) ((* (buf + 0)) << 8)  +
				  ((* (buf + 1)) << 0)  +
				  ((* (buf + 2)) << 24) +
				  ((* (buf + 3)) << 16)) / dvd;
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a word form slave to a float for PC
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_word_to_float (unsigned char * buf, float dvd)
/// HIFN Switch endiannes and calulate the resulting float
/// HIRET The calculated float value
/// HIPAR buf/Buffer in memory that holds the value to be converted
/// HIPAR dvd/Conversion factor to use
float conv_word_to_float (unsigned char * buf, float dvd)
{
	return ((short int) ((* (buf + 0)) << 8) +
						((* (buf + 1)) << 0)) / dvd;
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a float from PC to a double word for slave
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_float_to_dword (float v, int mltp, unsigned char * buf)
/// HIFN Switch endiannes and store bytes in memory for slave
/// HIPAR v/The float value to translate
/// HIPAR mltp/Conversion factor to use
/// HIPAR buf/The buffer in memory where to store calculated value
void conv_float_to_dword (float v, int mltp, unsigned char * buf)
{
	int				vint;
	unsigned char	*p;

	vint = (v > 0.0 ) ? v * mltp + 0.5 : v * mltp - 0.5;

	p = (unsigned char *) &vint;

	* (buf + 0) = * (p + 1);
	* (buf + 1) = * (p + 0);
	* (buf + 2) = * (p + 3);
	* (buf + 3) = * (p + 2);

	return;
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a float from PC to a word for slave
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_float_to_word (float v, int mltp, unsigned char * buf)
/// HIFN Switch endiannes and store bytes in memory for slave
/// HIPAR v/The float value to translate
/// HIPAR mltp/Conversion factor to use
/// HIPAR buf/The buffer in memory where to store calculated value
void conv_float_to_word (float v, int mltp, unsigned char * buf)
{
	short int		vint;
	unsigned char	*p;

	vint = (v > 0.0 ) ? v * mltp + 0.5 : v * mltp - 0.5;

	p = (unsigned char *) &vint;

	* (buf + 0) = * (p + 1);
	* (buf + 1) = * (p + 0);

	return;
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a short from PC to a word for slave
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_short_to_word (float v, int mltp, unsigned char * buf)
/// HIFN Switch endiannes and store bytes in memory for slave
/// HIPAR v/The int value to translate
/// HIPAR buf/The buffer in memory where to store calculated value
void conv_short_to_word (short v, unsigned char * buf)
{
	unsigned char	*p;

	p = (unsigned char *)&v;

	* (buf + 0) = * (p + 1);
	* (buf + 1) = * (p + 0);

	return;
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a word from slave to a int for PC
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_word_to_int (unsigned char * buf)
/// HIFN Switch endianness and return a int
/// HIRET The calculated int value
/// HIPAR buf/Buffer in memory that holds the value to be converted
int conv_word_to_int (unsigned char * buf)
{
	return	((*(buf + 0)) << 8) +
			(*(buf + 1));
}

//-------------------------------------------------------------------------------------------------
//
//			Convert a double word from slave to a int for PC
//
//-------------------------------------------------------------------------------------------------
/// HIFN conv_dword_to_int (unsigned char * buf)
/// HIFN Switch endianness and return a int
/// HIRET The calculated int value
/// HIPAR buf/Buffer in memory that holds the value to be converted
int conv_dword_to_int (unsigned char * buf)
{
	return	((*(buf + 3)) << 24)  +
			((*(buf + 2)) << 16)  +
			((*(buf + 0)) << 8) +
			(*(buf + 1));
}

//-------------------------------------------------------------------------------------------------
//
//			Prepare and send message to query slave memory
//
//-------------------------------------------------------------------------------------------------
/// HIFN QueryMem ()
/// HIFN Prepare and send message to query slave memory
/// HIRET 0 if no error, else RS232 error
/// HIPAR slave/The slave to query
/// HIPAR memAdr/Addres of slave memory area to query
/// HIPAR nRegs/The number of registers to query
/// HIPAR portNo/The serial port to use
/// HIPAR msecWait/Time to wait for the answer (millisecs)
int QueryMem (char slave, short memAdr, short nRegs, int portNo, double msecWait)
{
	int		comerr = 0;
	double	tini;

	// Clear buffers
	memset (mb.msg, 0, 256);
	memset (mb.ans, 0, 256);

	// Prepare message
	mb.adr = slave;
	mb.msg[ 0] = mb.adr;		// Slave address 
  	mb.msg[ 1] = 0x03;			// modbus function number (3)
	conv_short_to_word (memAdr, mb.msg + 2);
	conv_short_to_word (nRegs, mb.msg + 4);

	// Calculate buffer lenght
	mb.bSent = 6;
	mb.bRcvd = 5 + nRegs * 2;

	// Compute CRC and send message
	Calc_CRC (mb.msg, (char *)&mb.bSent);
	comerr = ComWrt (portNo, mb.msg, mb.bSent);
	if (comerr < 0) goto Error1;

	// Wait for slave to answer
	tini = Timer (); 
	msecWait /= 1000.0;
	//while (Timer () - tini < msecWait) ProcessSystemEvents ();
	 	Delay (msecWait); 
	// Check the correct # of bytes received
	if (GetInQLen (portNo) < mb.bRcvd) {
		comerr = kRS_IOTimeOut;
		goto Error;
	}

	// Interpret answer
	ComRd (portNo, mb.ans, Min (mb.bRcvd, 256));	// Read answer
	if (Check_CRC (mb.ans, mb.bRcvd - 2)) {			// Check received CRC
		comerr = kRS_CRCError;
		goto Error;
	}

Error1:
	return comerr < 0 ? comerr : 0;
	
Error:
	return comerr < 0 ? comerr : 0;
}

//-------------------------------------------------------------------------------------------------
//
//			Prepare and send message to write to slave memory
//
//-------------------------------------------------------------------------------------------------
/// HIFN WriteMem ()
/// HIFN Prepare and send message to write to slave memory
/// HIRET 0 if no error, else RS232 error
/// HIPAR slave/The slave to query
/// HIPAR memAdr/Addres of slave memory area to write to
/// HIPAR nRegs/The number of registers to write
/// HIPAR portNo/The serial port to use
/// HIPAR msecWait/Time to wait for the answer (millisecs)
int WriteMem (char slave,char funcion, short memAdr, short nRegs, char *data, int portNo, double msecWait)

// Impostazione della camera climatica
{
	int		size, comerr = 0;
	double	tini;

	memset (mb.msg, 0, 256);
	memset (mb.ans, 0, 256);

	mb.adr = slave;
 	mb.msg[ 0] = mb.adr;			// Slave address 
  	mb.msg[ 1] = funcion;				// modbus function number (16)
	conv_short_to_word (memAdr, mb.msg + 2);
	conv_short_to_word (nRegs, mb.msg + 4);
	mb.msg[ 6] = nRegs * 2;			// Num. of bytes (nRegs * 2)
	memcpy (mb.msg + 7, data, nRegs * 2);	// Data to write to slave memory
	mb.bSent = 7 + nRegs * 2;
  	mb.bRcvd = 8;					// Bytes to receive
	Calc_CRC (mb.msg, (char *)&mb.bSent);
    comerr = ComWrt (portNo, mb.msg, mb.bSent);
	if (comerr < 0) goto Error;

	// Wait for slave to answer
	tini = Timer (); msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();

	// Check the correct # of bytes received
	if (GetInQLen (portNo) < mb.bRcvd) {
		comerr = kRS_IOTimeOut;
		goto Error;
	}

	// Interpret answer
	ComRd (portNo, mb.ans, Min (mb.bRcvd, 256));	// Read answer
	if (Check_CRC (mb.ans, mb.bRcvd - 2)) {			// Check received CRC
		comerr = kRS_CRCError;
		goto Error;
	}

Error:
	return comerr < 0 ? comerr : 0;
}


  //***************************************


int WriteMemHexa (char slave,char funcion, short memAdr, short nRegs, unsigned short int data, int portNo, double msecWait)

// Impostazione della camera climatica
{
	int		size, comerr = 0;
	double	tini;

	memset (mb.msg, 0, 256);
	memset (mb.ans, 0, 256);

	mb.adr = slave;
 	mb.msg[ 0] = mb.adr;			// Slave address 
  	mb.msg[ 1] = funcion;				// modbus function number (16)
	conv_short_to_word (memAdr, mb.msg + 2);
	conv_short_to_word (nRegs, mb.msg + 4);

	mb.msg[ 6] = nRegs * 2;			// Num. of bytes (nRegs * 2)
	
	//memcpy (mb.msg + 7, data, nRegs * 2);	// Data to write to slave memory
	mb.msg[ 7]=(data & 0xff00)>>8;
	mb.msg[ 8]=(data & 0xff);    
	mb.bSent = 7 + nRegs * 2;
  	mb.bRcvd = 8;					// Bytes to receive
	Calc_CRC (mb.msg, (char *)&mb.bSent);
    comerr = ComWrt (portNo, mb.msg, mb.bSent);
	if (comerr < 0) goto Error;

	// Wait for slave to answer
	tini = Timer (); msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();

	// Check the correct # of bytes received
	if (GetInQLen (portNo) < mb.bRcvd) {
		comerr = kRS_IOTimeOut;
		goto Error;
	}

	// Interpret answer
	ComRd (portNo, mb.ans, Min (mb.bRcvd, 256));	// Read answer
	if (Check_CRC (mb.ans, mb.bRcvd - 2)) {			// Check received CRC
		comerr = kRS_CRCError;
		goto Error;
	}

Error:
	return comerr < 0 ? comerr : 0;
}

//-------------------------------------------------------------------------------------------------
//
//			Transfer internal memory content to caller
//
//-------------------------------------------------------------------------------------------------
/// HIFN ReturnMBMessage ()
/// HIFN Returns the internal memory area for message to slave
/// HIPAR msg/The buffer into which to return message area
/// HIPAR bSent/Lenght of last message sent to slave
void ReturnMBMessage (char *msg, int *bSent)
{
	memcpy (msg, &mb.msg, 256);
	*bSent = mb.bSent;
	return;
}

//-------------------------------------------------------------------------------------------------
//
//			Transfer internal memory content to caller
//
//-------------------------------------------------------------------------------------------------
/// HIFN ReturnMBAnswer ()
/// HIFN Returns the internal memory area for answer from slave
/// HIPAR msg/The buffer into which to return answer area
/// HIPAR bRcvd/Lenght of last message received from slave
void ReturnMBAnswer (char *msg, int *bRcvd)
{
	memcpy (msg, &mb.ans, 256);
	*bRcvd = mb.bRcvd;
	return;
}


int Funcion05(char esclavo, short bobina, int puerto,double wait)
{
	short	size, comerr = 0;
	int				i, bSent, bRcvd;
	double			msecWait;
	unsigned char	slave, msg[256], buf[512];
	char funcion=0x05; // Comando Modbus para escribir una bobina
	double	tini; 
	msecWait=wait;
	memset (mb.msg, 0, 256);  //Coloco ceros en los 256 lugares
	memset (mb.ans, 0, 256);
	
	mb.adr = esclavo;// Direccion del esclavo
	mb.msg[ 0] = mb.adr;			// Slave address 
  	mb.msg[ 1] = 0x05;
	conv_short_to_word (bobina, mb.msg + 2) ;
  	mb.msg[ 4] = 0xff;  // Comando en uno
	mb.msg[ 5] = 0x00;
	
	mb.bSent = 6;
  	mb.bRcvd = 8;
	Calc_CRC (mb.msg, (char *)&mb.bSent);
    comerr = ComWrt (puerto, mb.msg, mb.bSent);
	if (comerr < 0) goto Error;
	// Wait for slave to answer
	tini = Timer (); msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	
	// Check the correct # of bytes received
	if (GetInQLen (puerto) < mb.bRcvd) {
		comerr = kRS_IOTimeOut;
		goto Error;
	}
	
	// Interpretando la repuesta
	ComRd (puerto, mb.ans, Min (mb.bRcvd, 256));	// Read answer
	if (Check_CRC (mb.ans, mb.bRcvd - 2)) {			// Check received CRC
		comerr = kRS_CRCError;
		goto Error;
	}
	
//	MessagePopup ("Error Severo ", "Rta Comando OK"); 
	 return 0;
	// Display message sent to slave
//	ReturnMBMessage (buf, &bSent);
	

	// Display answer from slave
//	ReturnMBAnswer (buf, &bRcvd);
	

Error:
	if (comerr < 0) {
		sprintf (buf, "Error %d recibido en escritura de Bobina:\n%s", comerr, GetRS232ErrorString (comerr));
		MessagePopup ("Error Severo ", buf);
	}
	return 0;
		
	}

int Funcion05A(char esclavo, short bobina, int puerto,double wait)
{
	short	size, comerr = 0;
	int				i, bSent, bRcvd;
	double			msecWait;
	unsigned char	slave, msg[256], buf[512];
	char funcion=0x05; // Comando Modbus para escribir una bobina
	double	tini; 
	msecWait=wait;
	memset (mb.msg, 0, 256);  //Coloco ceros en los 256 lugares
	memset (mb.ans, 0, 256);
	
	mb.adr = esclavo;// Direccion del esclavo
	mb.msg[ 0] = mb.adr;			// Slave address 
  	mb.msg[ 1] = 0x05;
	conv_short_to_word (bobina, mb.msg + 2) ;
  	mb.msg[ 4] = 0x00;  // Comando en cero
	mb.msg[ 5] = 0x00;
	
	mb.bSent = 6;
  	mb.bRcvd = 8;
	Calc_CRC (mb.msg, (char *)&mb.bSent);
    comerr = ComWrt (puerto, mb.msg, mb.bSent);
	if (comerr < 0) goto Error;
	// Wait for slave to answer
	tini = Timer (); msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	
	// Check the correct # of bytes received
	if (GetInQLen (puerto) < mb.bRcvd) {
		comerr = kRS_IOTimeOut;
		goto Error;
	}
	
	// Interpretando la repuesta
	ComRd (puerto, mb.ans, Min (mb.bRcvd, 256));	// Read answer
	if (Check_CRC (mb.ans, mb.bRcvd - 2)) {			// Check received CRC
		comerr = kRS_CRCError;
		goto Error;
	}
	
//	MessagePopup ("Error Severo ", "Rta Comando OK"); 
	 return 0;
	// Display message sent to slave
//	ReturnMBMessage (buf, &bSent);
	

	// Display answer from slave
//	ReturnMBAnswer (buf, &bRcvd);
	

Error:
	if (comerr < 0) {
		sprintf (buf, "Error %d recibido en escritura de Bobina:\n%s", comerr, GetRS232ErrorString (comerr));
		MessagePopup ("Error Severo ", buf);
	}
	return 0;
		
	}
 /*
	// Interpret answer
	ComRd (portNo, mb.ans, Min (mb.bRcvd, 256));	// Read answer
	if (Check_CRC (mb.ans, mb.bRcvd - 2)) {			// Check received CRC
		comerr = kRS_CRCError;
		goto Error;
	}

Error:
	return comerr < 0 ? comerr : 0;
*/
	
