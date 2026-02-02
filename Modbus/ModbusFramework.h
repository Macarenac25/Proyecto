#include <ansi_c.h>
#include <rs232.h>


// Public functions
void Init_CRC (void);										// CRC tables initialization
void Calc_CRC (unsigned char * buf, unsigned char *size);	// CRC computing
int Check_CRC (unsigned char * buf, unsigned char size);	// CRC check
int conv_word_to_int (unsigned char * buf);		// Converte da formato del PLC a int (camera Mazzali)
int conv_dword_to_int (unsigned char * buf);	// Converte da formato del PLC a int (camera Angelantoni)
float conv_word_to_float (unsigned char * buf, float dvd);	// Converte dal formato del PLC (WORD) a float (camera Angelantoni)
float conv_dword_to_float (unsigned char * buf, float dvd);	// Converte dal formato del PLC (DWORD) a float (camera Angelantoni)
void conv_short_to_word (short v, unsigned char * buf);		// Convert a short from PC to a word for slave
int QueryMem (char slave, short memAdr, short nRegs, int portNo, double msecWait);
		// Prepare and send message to query slave memory
int WriteMem (char slave,char funcion, short memAdr, short nRegs, char *data, int portNo, double msecWait);
int WriteMemHexa (char slave,char funcion, short memAdr, short nRegs, unsigned short int data, int portNo, double msecWait);    
		// Prepare and send message to write to slave memory
void ReturnMBMessage (char *msg, int *bSent);			// Returns the internal memory area for message to slave
void ReturnMBAnswer (char *msg, int *bRcvd);			// Returns the internal memory area for answer from slave
int Funcion05(char esclavo, short bobina,int puerto,  double wait);   //Escribe un 1 en la bobina
int Funcion05A(char esclavo, short bobina, int puerto,double wait); // Escribe un 0 en la bobina