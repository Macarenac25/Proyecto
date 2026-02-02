
#include <cvirte.h>		
#include <userint.h>
#include "RTUM.h"
#include <ansi_c.h>
#include <formatio.h>
#include <utility.h>
#include <rs232.h>
#pragma C99_extensions_on   
#include "ModbusFramework.h" 
#include <tcpsupp.h>
static int panelHandle,panelHandleChild2, panelHandleChild3, panelHandleChild4, panelHandleChild5;
/*---------------------------------------------------------------------------*/
/* Internal function prototypes                                              */
/*---------------------------------------------------------------------------*/
void DisplayRS232Error (void);
unsigned int cal_crc16 (unsigned char*data,unsigned int length);
void Consultar1T(void);  //Funciones para consultar con el Timer a los estados
/*---------------------------------------------------------------------------*/
/* Agregado para texto                                         */
/*---------------------------------------------------------------------------*/
char Barchivo=0; // Verifica si guarda archivo esta habilitado
unsigned char direccion[270];//="c:\";
int largo, archivo;
char Hora1[20], Fecha1[20];
#define DATETIME_FORMATSTRING "%I:%M:%S %p" //Formato de tiempo para archivo texto
char strgtxt [160]={0}; 
double Ti,Te;// variables de lectura de la temperatura interior y exterior
int archivo, archivo1;

 
/*---------------------------------------------------------------------------*/ 
   /*---------------------------------------------------------------------------*/
/* Macros						                                             */
/*---------------------------------------------------------------------------*/
#define tcpChk(f) if ((g_TCPError=(f)) < 0) {ReportTCPError(); goto Done;} else 

/*---------------------------------------------------------------------------*/
/* Internal function prototypes                                              */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ClientTCPCB (unsigned handle, int event, int error,
                             void *callbackData);
static void ReportTCPError (void);   
/*---------------------------------------------------------------------------*/
/* Module-globals                                                            */
/*---------------------------------------------------------------------------*/
static unsigned int g_hconversation;
static int          g_hmainPanel;
static int			g_connected = 0;
static int			g_TCPError = 0;
unsigned char RTCPVec[16]={0};  ; //Vector para leer ModBus  TCP
unsigned char WTCPVec[16]={0};  ; //Vector para escri bir  ModBus  TCP
unsigned int TramaTcp=0; //indicador de trama TCP Modbud

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* Registros y banderas                                                      */
/*---------------------------------------------------------------------------*/
 
int port_open=0; //Indica si hay un puerto serial abierto
int tcp_open =0; //Indica si hay un puerto TCP abierto
int RS232Error,baudrate,comport, port_open,portindex;//Variables para  puerto serial
char devicename[30]="cerrado";//Indicacion estado puerto
char datoRS[8];// Vector 
char datoRX[55];
double espera=10;// Registro tiempo de espera comunicaciones del ModBus
int offValue=0;
int main (int argc, char *argv[])
{
	if (InitCVIRTE (0, argv, 0) == 0)
		return -1;	/* out of memory */
	
	if ((panelHandle= LoadPanel (0, "RTUM.uir", Principal)) < 0)
		return -1;
	Init_CRC ();//Genera las tablas para el CRC de Modbus
	DisplayPanel (panelHandle);
	//SetCtrlAttribute(panelHandle, Principal_PICTURE, ATTR_ZPLANE_POSITION, VAL_PANEL_BACK);

	RunUserInterface ();
	
	DiscardPanel (panelHandle);
	return 0;
}

int CVICALLBACK SalirRS (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
	DiscardPanel (panelHandleChild2); 
			break;
	}
	return 0;
}

int CVICALLBACK SalirF (int panel, int control, int event,
						void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		QuitUserInterface(0);  
			break;
	}
	return 0;
}

void CVICALLBACK AbrirSerie (int menuBar, int menuItem, void *callbackData,
							 int panel)
{
	if ((panelHandleChild2 = LoadPanel (0, "RTUM.uir", Serial)) < 0)
		return  ;
	
		InstallPopup (panelHandleChild2);
		if (port_open ==1)
			{
            SetCtrlVal (panelHandleChild2, Serial_LED , 1); 
			SetCtrlVal (panelHandleChild2, Serial_STRING , devicename);
			}
		else
			{
            SetCtrlVal (panelHandleChild2, Serial_LED , 0); 
			SetCtrlVal (panelHandleChild2, Serial_STRING , "cerrado");
			}
}

int CVICALLBACK AbrirPuerto (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int valor;    
			GetCtrlVal(panelHandleChild2 , Serial_COMPORT , &comport);
    		GetCtrlVal(panelHandleChild2, Serial_Velocidad, &baudrate);
			GetCtrlIndex(panelHandleChild2, Serial_COMPORT, &portindex);  
			GetLabelFromIndex(panelHandleChild2, Serial_COMPORT, portindex, devicename);
			GetCtrlVal (panelHandleChild2, Serial_WAIT ,  &espera); // Tiempo de espera de la repuesta del MODBUS    
			   port_open = 0;  /* Cierro Puerto */
			   SetCtrlVal (panelHandleChild2, Serial_LED , 0);
            //GetConfigParms ();
            DisableBreakOnLibraryErrors ();
            RS232Error = OpenComConfig (comport,devicename,baudrate,0,8,1,512,512);	
			// paridad par, 8 bits de datos 1 bit de stop 
			
			EnableBreakOnLibraryErrors ();
		 if (RS232Error)
		 {
			 DisplayRS232Error (); 
		 	 return 0;
		 }
			SetComTime(comport,1.0);  //Se cambia a 1  segundos
			 
			
            if (RS232Error == 0)
                {
            port_open = 1;
            SetCtrlVal (panelHandleChild2, Serial_LED , 1); 
			SetCtrlVal (panelHandleChild2, Serial_STRING , devicename);
               // EnablePanelControls (0); /* Enable: no errors */
			
			    }
            else
               // EnablePanelControls (1); /* Disable: errors found */
            DiscardPanel (panelHandleChild2);
			break;
	}
	return 0;
}

int CVICALLBACK CerrarPuerto (int panel, int control, int event,
							  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
 				port_open = 0;  /* Cierro Puerto */  
			   	SetCtrlVal (panelHandleChild2, Serial_LED ,0);  
			   	SetCtrlVal (panelHandleChild2, Serial_STRING ,  "Cerrado"); 
			   	CloseCom (comport); 
			break;
	}
	return 0;
}

//Significado de los datos: suma de comprobación CrC del marco de datos, utilizando 2 bytes.
//Checksum 
// calculo CRC:
unsigned int cal_crc16 (unsigned char*data,unsigned int length)
{
unsigned int i,crc_result = 0xffff;
while(length--)
{
crc_result^=*data++;
for(i=0;i<8;i++)
{
if(crc_result&0x01)
crc_result=(crc_result>>1)^0xa001;

else crc_result=crc_result>>1;

}
}
crc_result=((crc_result&0xff)<<8)|(crc_result>>8);
return(crc_result);
}


/*---------------------------------------------------------------------------*/
/* Display el error del puerto comunicaciones RS232                             */
/*---------------------------------------------------------------------------*/
void DisplayRS232Error (void)
{
    char ErrorMessage[200];
    switch (RS232Error)
        {
        default :
            if (RS232Error < 0)
                {  
                Fmt (ErrorMessage, "%s<RS232 error numero %i", RS232Error);
                MessagePopup ("Mensaje RS232 ", ErrorMessage);
                }
            break;
        case 0  :
            MessagePopup ("Mensaje RS232 ", "No hay errores.");
            break;
        case -2 :
            Fmt (ErrorMessage, "%s", "Invalido numero de puerto  (debe estar en el  "
                                     "rrango 1 a 8).");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;
        case -3 :
            Fmt (ErrorMessage, "%s", "No hay puerto abierto.\n"
                 "Check COM Port setting in Configure.");
            MessagePopup ("Mensaje RS232 ", ErrorMessage);
            break;
			 case -6 :
            Fmt (ErrorMessage, "%s", "No encuentra un puerto serial.\n"
                 "Check COM Port setting in Configure.");
            MessagePopup ("Mensaje RS232 ", ErrorMessage);
            break;
			
        case -99 :
            Fmt (ErrorMessage, "%s", "Timeout error.\n\n"
                 "Either increase timeout value,\n"
                 "       check COM Port setting, or\n"
                 "       check device.");
            MessagePopup ("RS232 Message", ErrorMessage);
            break;
        }
}
 

int CVICALLBACK SalirDa (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		DiscardPanel (panelHandleChild3); //Salir del panel de Datos de Modbus
			break;
	}
	return 0;
}

int CVICALLBACK Abrir1 (int panel, int control, int event,
						void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		 	int valor; //es la salida del popup
			
			 if (port_open==0 && tcp_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere abrir Ventilador 1");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			//Consultar1T(); 
			if (port_open ==1  && tcp_open==0)
			{
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05A(1,0x0,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_3,1);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_2,0);
			//Consultar1T();
			}
			
			if (port_open ==0  && tcp_open==1)
			{
			double	tini, msecWait;// Variables de tiempo
			int datalong=0;
			 WTCPVec[0]= (TramaTcp>>8);//Byte alto la trama
			 WTCPVec[1]= (TramaTcp>>8);// Byte baj la trama
			 WTCPVec[2]= 0;
			 WTCPVec[3]= 0;
			 WTCPVec[4]= 0;
			 WTCPVec[5]= 6;// Cantidad de Bytes
			 WTCPVec[6]= 1;//Esclavo
			 WTCPVec[7]= 5;//Funcion 0x05
			 WTCPVec[8]= 0;//valor alto Coil
			 WTCPVec[9]= 0;//valor bajo Coil
			 WTCPVec[10]= 0;//valor  Abrir
			 WTCPVec[11]= 0;//valor  
			 if (ClientTCPWrite (g_hconversation, WTCPVec, 12, 1000) < 0)
						 goto Error1; 
			TramaTcp =TramaTcp+1;
	 		// Espero repuesta esclavsl
			tini = Timer (); 
			msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	Delay (msecWait); 
	// Check the correct # of bytes received
	
	
	  if (datalong = ClientTCPRead (g_hconversation, RTCPVec, 13 , 1000)< 0)//mb.bRcvd
	{
	 	goto Error1; }
			SetCtrlVal(panelHandleChild3,VerDatos_LED_3,1);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_2,0);
			}
			}
Error1:
			break;
	}
	return 0;
}

int CVICALLBACK Cerrar1 (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int valor; //es la salida del popup
			 if (port_open==0 && tcp_open==0)  
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere cerrar Ventilador 1");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
				if (port_open ==1  && tcp_open==0)// Modbus Serial
			{
			//Consultar1T(); 
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05(1,0x0,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_3,0);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_2,1);
			//Consultar1T();
			}
			
			if (port_open ==0  && tcp_open==1)
			{
			double	tini, msecWait;// Variables de tiempo
			int datalong=0;
			 WTCPVec[0]= (TramaTcp>>8);//Byte alto la trama
			 WTCPVec[1]= (TramaTcp>>8);// Byte baj la trama
			 WTCPVec[2]= 0;
			 WTCPVec[3]= 0;
			 WTCPVec[4]= 0;
			 WTCPVec[5]= 6;// Cantidad de Bytes
			 WTCPVec[6]= 1;//Esclavo
			 WTCPVec[7]= 5;//Funcion 0x05
			 WTCPVec[8]= 0;//valor alto Coil 0
			 WTCPVec[9]= 0;//valor bajo Coil 0
			 WTCPVec[10]= 0xFF;//valor  Cerrar
			 WTCPVec[11]= 0;//valor  
			 if (ClientTCPWrite (g_hconversation, WTCPVec, 12, 1000) < 0)
						 goto Error1; 
			TramaTcp =TramaTcp+1;
	 		// Espero repuesta esclavsl
			tini = Timer (); 
			msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	Delay (msecWait); 
	// Check the correct # of bytes received
	
	
	  if (datalong = ClientTCPRead (g_hconversation, RTCPVec, 13 , 1000)< 0)//mb.bRcvd
	{
	 	goto Error1; }
			SetCtrlVal(panelHandleChild3,VerDatos_LED_3,0);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_2,1);
			}
			}
			
Error1:
			break;
	}
	return 0;
}

int CVICALLBACK Abrir2 (int panel, int control, int event,
						void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			int valor; //es la salida del popup
			 if (port_open==0 && tcp_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere abrir Ventilador 2");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			//Consultar1T(); 
				if (port_open ==1  && tcp_open==0)// Modbus Serial
			{
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05A(1,0x1,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_5,0);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_4,1);
			//Consultar1T();
			}
			if (port_open ==0  && tcp_open==1)// Sale por TCP
			{
			double	tini, msecWait;// Variables de tiempo
			int datalong=0;
			 WTCPVec[0]= (TramaTcp>>8);//Byte alto la trama
			 WTCPVec[1]= (TramaTcp>>8);// Byte baj la trama
			 WTCPVec[2]= 0;
			 WTCPVec[3]= 0;
			 WTCPVec[4]= 0;
			 WTCPVec[5]= 6;// Cantidad de Bytes
			 WTCPVec[6]= 1;//Esclavo
			 WTCPVec[7]= 5;//Funcion 0x05
			 WTCPVec[8]= 0;//valor alto Coil 1
			 WTCPVec[9]= 1;//valor bajo Coil 1
			 WTCPVec[10]= 0;//valor  Abrir
			 WTCPVec[11]= 0;//valor  
			 if (ClientTCPWrite (g_hconversation, WTCPVec, 12, 1000) < 0)
						 goto Error1; 
			TramaTcp =TramaTcp+1;
	 		// Espero repuesta esclavsl
			tini = Timer (); 
			msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	Delay (msecWait); 
	// Check the correct # of bytes received
	
	
	  if (datalong = ClientTCPRead (g_hconversation, RTCPVec, 13 , 1000)< 0)//mb.bRcvd
	{
	 	goto Error1; }
			SetCtrlVal(panelHandleChild3,VerDatos_LED_5,0);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_4,1);
			}
			}
Error1:
			break;
	}
	return 0;
}

int CVICALLBACK Cerrar2 (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
	int valor; //es la salida del popup
			 if (port_open==0 && tcp_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere cerrar Ventilador 2");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			if (port_open ==1  && tcp_open==0)// Modbus Serial
			{
			//Consultar1T(); 
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05(1,0x1,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_5,1);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_4,0);
			//Consultar1T();
			}
			
			if (port_open ==0  && tcp_open==1)
			{
			double	tini, msecWait;// Variables de tiempo
			int datalong=0;
			 WTCPVec[0]= (TramaTcp>>8);//Byte alto la trama
			 WTCPVec[1]= (TramaTcp>>8);// Byte baj la trama
			 WTCPVec[2]= 0;
			 WTCPVec[3]= 0;
			 WTCPVec[4]= 0;
			 WTCPVec[5]= 6;// Cantidad de Bytes
			 WTCPVec[6]= 1;//Esclavo
			 WTCPVec[7]= 5;//Funcion 0x05
			 WTCPVec[8]= 0;//valor alto Coil 1
			 WTCPVec[9]= 1;//valor bajo Coil 1
			 WTCPVec[10]= 0xFF;//valor  Cerrar
			 WTCPVec[11]= 0;//valor  
			 if (ClientTCPWrite (g_hconversation, WTCPVec, 12, 1000) < 0)
						 goto Error1; 
			TramaTcp =TramaTcp+1;
	 		// Espero repuesta esclavsl
			tini = Timer (); 
			msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	Delay (msecWait); 
	// Check the correct # of bytes received
	
	
	  if (datalong = ClientTCPRead (g_hconversation, RTCPVec, 13 , 1000)< 0)//mb.bRcvd
	{
	 	goto Error1; }
			SetCtrlVal(panelHandleChild3,VerDatos_LED_5,1);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_4,0);
			}
			}
			
Error1:
			break;
	}
	return 0;
}

void CVICALLBACK DatosM (int menuBar, int menuItem, void *callbackData,
						 int panel)
{
	if ((panelHandleChild3 = LoadPanel (0, "RTUM.uir", VerDatos)) < 0)
		return  ;
	
		InstallPopup (panelHandleChild3);
		if (port_open ==1)
			{
            SetCtrlVal (panelHandleChild3, VerDatos_LED , 1); 
			SetCtrlVal (panelHandleChild3, VerDatos_STRING_2 , devicename);
			}

		else 
		{
		SetCtrlVal (panelHandleChild3, VerDatos_LED , 0); 	
		SetCtrlVal (panelHandleChild3, VerDatos_STRING_2 , "cerrado");	
		}
		
		if (tcp_open ==1)
			{
            SetCtrlVal (panelHandleChild3, VerDatos_LED_6 , 1); 
			 }

		else 
		{
		SetCtrlVal (panelHandleChild3, VerDatos_LED_6 , 0); 	
		}
}

int CVICALLBACK LeerT (int panel, int control, int event,
					   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
			short	memAdr, nRegs, comerr = 0;    
			int		i, bSent, bRcvd, n;
			unsigned char	slave, buf[512];
			double Ti,Te;
			 if (port_open==0 && tcp_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar enlace comunicaciones");
				 return 0; 
			 }
	 		if (port_open ==1  && tcp_open==0)
			{
			FlushInQ (comport); 
			slave=1; //direccion de esclavo
			memAdr=0x00;// Direccion del Modulo IA
	        nRegs=0x08;// Leo los registroa IA,IB, IC y IT
			comerr = QueryMem (slave, memAdr, nRegs, comport, espera);
			if (comerr < 0) goto Error;
	 		// Analizo la repuesta del esclavo  
	  		ReturnMBAnswer (buf, &bRcvd);
			Ti =(buf[11]<<8) +(buf[12]) ; // hay que dividir por 16
			Te =(buf[13]<<8) +(buf[14]) ; // hay que dividir por 100
            
			Ti=Ti/100.0; //Temperatura interior  Max31865
		    Te=Te/16.0; //Temperatura  exteriior DS18B20
			
			SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM , Ti);
	 		SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM_2 , Te);
Error:
	if (comerr < 0) {
		if (comerr==-99)	//se fué por tiempo
		{
		sprintf (buf, "Finalizó el tiempo de respuesta");
		MessagePopup ("Error Comunicaciones", buf);
		}
		else
		{
		sprintf (buf, "Error %d received in QuerySlave:\n%s", comerr, GetRS232ErrorString (comerr));
		MessagePopup ("Error Severo", buf);
		}
	}
			}//fin comunicaciones Serial
			
	if (port_open ==0  && tcp_open==1)
			{
			double	tini, msecWait;// Variables de tiempo
			int datalong=0;
			 WTCPVec[0]= (TramaTcp>>8);//Byte alto la trama
			 WTCPVec[1]= (TramaTcp>>8);// Byte baj la trama
			 WTCPVec[2]= 0;
			 WTCPVec[3]= 0;
			 WTCPVec[4]= 0;
			 WTCPVec[5]= 6;
			 WTCPVec[6]= 1;//Esclavo
			 WTCPVec[7]= 3;//Funcion
			 WTCPVec[8]= 0;//valor alto direccion
			 WTCPVec[9]= 4;//valor bajo direccion
			 WTCPVec[10]= 0;//valor alto cantidad
			 WTCPVec[11]= 2;//valor bajo cantidad
			 if (ClientTCPWrite (g_hconversation, WTCPVec, 12, 1000) < 0)
						 goto Error1; 
			TramaTcp =TramaTcp+1;
	 		// Espero repuesta esclavsl
			tini = Timer (); 
			msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	Delay (msecWait); 
	// Check the correct # of bytes received
	
	
	  if (datalong = ClientTCPRead (g_hconversation, RTCPVec, 13 , 1000)< 0)//mb.bRcvd
	{
	comerr = kRS_IOTimeOut;
	goto Error1; }
			}
			Ti =(RTCPVec[9]<<8) +(RTCPVec[10]) ; // hay que dividir por 16
			Te =(RTCPVec[11]<<8) +(RTCPVec[12]) ; // hay que dividir por 100
            
			Ti=Ti/100.0; //Temperatura interior  Max31865
		    Te=Te/16.0; //Temperatura  exteriior DS18B20
			
			SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM , Ti);
	 		SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM_2 , Te);
	
Error1:
			break;
	}
	return 0;
}

void CVICALLBACK SalirM (int menuBar, int menuItem, void *callbackData,
						 int panel)
{
	QuitUserInterface(0); 
}

int CVICALLBACK Disparo (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_TIMER_TICK:
			if (port_open==0 && tcp_open==0) 
			 {
				 SetCtrlAttribute (panelHandleChild3, VerDatos_TIMER,ATTR_ENABLED ,0); //Apagar timer  
				SetCtrlAttribute (panelHandleChild3, VerDatos_COMMANDBUTTON_9,ATTR_CMD_BUTTON_COLOR ,VAL_RED);
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar enlace de comunicaciones");
				 return 0; 
			 }
			 Consultar1T();
			 if (Barchivo !='\0')
	{	
	    	strcpy(Hora1,TimeStr());	
			Fmt (strgtxt, "%s<%s%f[w9p2]%s%f[w9p2]%s%s","         ",    Ti  ,"                ", Te,"                ",Hora1);//
			WriteLine (archivo1,strgtxt, 81);	//Escribe el string "strgtxt" en el txt que se llama "archivo". La longitud del string es de 65 caracteres.
	}
			break;
	}
	return 0;
}

int CVICALLBACK ConfT (int panel, int control, int event,
					   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{

		
		case EVENT_COMMIT:
		
		GetCtrlAttribute(panelHandleChild3, VerDatos_TIMER, ATTR_ENABLED, &offValue );
		
		if(offValue==1)
		{
		SetCtrlAttribute (panelHandleChild3, VerDatos_TIMER,ATTR_ENABLED ,0); //Apagar timer  
		SetCtrlAttribute (panelHandleChild3, VerDatos_COMMANDBUTTON_9,ATTR_CMD_BUTTON_COLOR ,VAL_RED);
		Barchivo-0; //Desconecto almacenamiento en archivo
		}
		 
		  if(offValue==0)
		{
		SetCtrlAttribute (panelHandleChild3, VerDatos_TIMER,ATTR_ENABLED ,1); //Encender timer  
		SetCtrlAttribute (panelHandleChild3, VerDatos_COMMANDBUTTON_9,ATTR_CMD_BUTTON_COLOR ,VAL_GREEN);
		}
			
			break;
	}
	return 0;
}

void CVICALLBACK Grafico (int menuBar, int menuItem, void *callbackData,
						  int panel)
{
	if ((panelHandleChild4 = LoadPanel (0, "RTUM.uir", PANELG)) < 0)
		return  ;
	
		InstallPopup (panelHandleChild4);
		if (port_open ==1)
			{
            SetCtrlVal (panelHandleChild4, PANELG_LED  , 1); 
			SetCtrlVal (panelHandleChild4, PANELG_STRING , devicename);
			}
		else 
			{
            SetCtrlVal (panelHandleChild4, PANELG_LED  , 0); 
			SetCtrlVal (panelHandleChild4, PANELG_STRING , "cerrado");
			}
}

int CVICALLBACK SalirG (int panel, int control, int event,
						void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
	DiscardPanel (panelHandleChild4);
			break;
	}
	return 0;
}

void Consultar1T(void)  //Funciones para consultar con el Timer al equipo con Modbus
{
	short	memAdr, nRegs, comerr = 0;    
			int		i, bSent, bRcvd, n;
			unsigned char	slave, buf[512];
		 
			 if (port_open==0 && tcp_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Enlace de comunicaciones ");
				 return ; 
			 }
	 if (port_open==1 && tcp_open==0)
	 		{
		 	FlushInQ (comport); 
			slave=1; //direccion de esclavo
			memAdr=0x00;// Direccion del Modulo IA
	        nRegs=0x08;// Leo los registroa IA,IB, IC y IT
			comerr = QueryMem (slave, memAdr, nRegs, comport, espera);
			if (comerr < 0) goto Error;
	 		// Analizo la repuesta del esclavo  
	  		ReturnMBAnswer (buf, &bRcvd);
			Ti =(buf[11]<<8) +(buf[12]) ; // hay que dividir por 16
			Te =(buf[13]<<8) +(buf[14]) ; // hay que dividir por 100
            
		    Te=Te/16.0; //Temperatura exterior DS18B29
			Ti=Ti/100.0; //Temperatura interior Max31865
			SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM , Ti);
	 		SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM_2 , Te);
Error:
	if (comerr < 0) {
		if (comerr==-99)	//se fué por tiempo
		{
		sprintf (buf, "Finalizó el tiempo de respuesta");
		MessagePopup ("Error Comunicaciones", buf);
		}
		else
		{
		sprintf (buf, "Error %d received in QuerySlave:\n%s", comerr, GetRS232ErrorString (comerr));
		MessagePopup ("Error Severo", buf);
		}
	}
			}
			 
	if (port_open==0 && tcp_open==1)
	 		{
			double	tini, msecWait;// Variables de tiempo
			int datalong=0;
			 WTCPVec[0]= (TramaTcp>>8);//Byte alto la trama
			 WTCPVec[1]= (TramaTcp>>8);// Byte baj la trama
			 WTCPVec[2]= 0;
			 WTCPVec[3]= 0;
			 WTCPVec[4]= 0;
			 WTCPVec[5]= 6;
			 WTCPVec[6]= 1;//Esclavo
			 WTCPVec[7]= 3;//Funcion
			 WTCPVec[8]= 0;//valor alto direccion
			 WTCPVec[9]= 4;//valor bajo direccion
			 WTCPVec[10]= 0;//valor alto cantidad
			 WTCPVec[11]= 2;//valor bajo cantidad
			 if (ClientTCPWrite (g_hconversation, WTCPVec, 12, 1000) < 0)
						 goto Error1; 
			TramaTcp =TramaTcp+1;
	 		// Espero repuesta esclavsl
			tini = Timer (); 
			msecWait /= 1000.0;
	while (Timer () - tini < msecWait) ProcessSystemEvents ();
	Delay (msecWait); 
	// Check the correct # of bytes received
	
	
	  if (datalong = ClientTCPRead (g_hconversation, RTCPVec, 13 , 1000)< 0)//mb.bRcvd
		{
		comerr = kRS_IOTimeOut;
		goto Error1; 
		}
			
			Ti =(RTCPVec[9]<<8) +(RTCPVec[10]) ; // hay que dividir por 16
			Te =(RTCPVec[11]<<8) +(RTCPVec[12]) ; // hay que dividir por 100
            
			Ti=Ti/100.0; //Temperatura interior  Max31865
		    Te=Te/16.0; //Temperatura  exteriior DS18B20
			
			SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM , Ti);
	 		SetCtrlVal(panelHandleChild3, VerDatos_NUMERICTHERM_2 , Te);	
			}
Error1:
			return ;
}

int CVICALLBACK Arch (int panel, int control, int event,
					  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		int stat;
		char filestring[100];  
		if (Barchivo){
		 stat = FileSelectPopupEx ("", "*.txt", "", "Abrir archivo de texto", VAL_LOAD_BUTTON ,0, 0, direccion);
		sprintf (filestring, "notepad.exe %s", direccion);
        LaunchExecutable (filestring);
			}
			break;
		}
	return 0;
}


int CVICALLBACK CrearArch (int panel, int control, int event,
						   void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		 char directorio_inicial[] = "C:\\Trafo\datos"; 
		//char i, filePath[512];
		unsigned char y;
		char strgtxt [160]={0};    
		char fecha [20], hora [20]  ; 
		int resultado;
    	char ruta_archivo[260];
    
    // Configura todos los argumentos
       	char archivo_por_defecto[] = "*.txt";
   	 	char tipos_archivos[] = "Archivos de texto (*.txt)|*.txt|Todos los archivos (*.*)|*.*";
    	char titulo[] = "Seleccione un archivo de texto";
    	int etiqueta_boton = VAL_OK_BUTTON; // Para crear un archivo existente
    	int restringir_directorio = 0; // Permitir cambiar de directorio
    	int restringir_extension = 1;  // Forzar la extensión .txt

    // Copia el directorio inicial a la variable de salida
    	sprintf(ruta_archivo, "%s", directorio_inicial);

    // Llama a la función con todos los argumentos
    	resultado = FileSelectPopupEx(
        directorio_inicial,     // Directorio por defecto
        archivo_por_defecto,    // Especif. de archivo por defecto
        tipos_archivos,         // Lista de tipos de archivo
        titulo,                 // Título
        etiqueta_boton,         // Etiqueta del botón
        restringir_directorio,  // Restringir directorio
        restringir_extension,   // Restringir extensión
        ruta_archivo            // Variable para la ruta seleccionada
    );
			
		
			if (ruta_archivo[0] == '\0')
                {
                MessagePopup ("Error", "Debe incluir un directorio válido para el Archivo txt");
                return 0; 
                }
			 strcpy(hora,TimeStr());			//Copia el pointer string que devuelve TimeStr() y lo pone en el string Hora
			 strcpy(fecha,DateStr());		//Copia el pointer string que devuelve DateStr() y lo pone en el string Fecha
			 
			 archivo1=OpenFile(ruta_archivo,VAL_WRITE_ONLY,VAL_OPEN_AS_IS,VAL_ASCII);	//Crea el archivo txt en la direccion "filePath". 
			 //Se usa despues la variable "archivo" para identificar el txt al cual se escribe  
			 
			 Fmt (strgtxt, "%s<%s%s%s%s","       Fecha=", fecha ,"        Hora=",hora  );
			 WriteLine (archivo1,strgtxt, 100);
			 
			 WriteLine (archivo1,"Temperature Interna(°c)        Temperatura Externa(°c)      Hora", 64);	  //Apenas creado el archivo se escribe una fila para identificar las 6 variables
				 
			 Barchivo=1; //Indico que quiero guardar los datos en un archivo
			SetCtrlVal (panelHandleChild3, VerDatos_STRING ,ruta_archivo); 
			break;
	}
	return 0;
}

int CVICALLBACK LeerTXT (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		int resultado;
    	char ruta_archivo[260];
	// Configura todos los argumentos
		char directorio_inicial[] = "C:\\Trafo\datos"; 
       	char archivo_por_defecto[] = "*.txt";
   	 	char tipos_archivos[] = "Archivos de texto (*.txt)|*.txt|Todos los archivos (*.*)|*.*";
    	char titulo[] = "Seleccione un archivo de texto";
    	int etiqueta_boton = VAL_SELECT_BUTTON ; // Para abrir un archivo existente
    	int restringir_directorio = 0; // Permitir cambiar de directorio
    	int restringir_extension = 1;  // Forzar la extensión .txt

    // Copia el directorio inicial a la variable de salida
    	sprintf(ruta_archivo, "%s", directorio_inicial);

    // Llama a la función con todos los argumentos
    	resultado = FileSelectPopupEx(
        directorio_inicial,     // Directorio por defecto
        archivo_por_defecto,    // Especif. de archivo por defecto
        tipos_archivos,         // Lista de tipos de archivo
        titulo,                 // Título
        etiqueta_boton,         // Etiqueta del botón
        restringir_directorio,  // Restringir directorio
        restringir_extension,   // Restringir extensión
        ruta_archivo            // Variable para la ruta seleccionada
    );
 
    FILE *file = fopen(ruta_archivo, "r");
    if (file == NULL) {
        return -1; // Error al abrir el archivo
    }

    int lines = 0;
    int c;
    while ((c = fgetc(file)) != EOF) {
        if (c == '\n') {
            lines++;
        }     }
		
		if (lines <= 0) {
        // Manejar el caso de archivo vacío o error
		fclose(file);
        return 0;
   		 }
	//lines=lines-2; // las dos primeras filas no hay que contemplarlas
		  // Asignar memoria dinámicamente
    double *x_data = (double*)malloc(sizeof(double) * lines);
    double *y_data = (double*)malloc(sizeof(double) * lines);

    if (x_data == NULL || y_data == NULL) {
        // Manejar error de asignación de memoria
        if (x_data) free(x_data);
        if (y_data) free(y_data);
		fclose(file);
        return 0;
    }
    
     // Abrir el archivo por segunda vez para leer los datos
    file = fopen(ruta_archivo, "r");
    if (file == NULL) {
        free(x_data);
        free(y_data);
		fclose(file);
        return;
    }

    // Leer los datos línea por línea
   
	
	char line_buffer[256];
    double x_val, y_val;
	int numPoints = 0;
    int hour, minute, second;
	double time_data[lines]; // Almacena el tiempo como segundos desde la medianoche

// Leer el archivo línea por línea
    while (fgets(line_buffer, sizeof(line_buffer), file) != NULL && numPoints < (lines-2)) {
        // Analizar la línea usando sscanf
        if (sscanf(line_buffer, "%lf %lf %d:%d:%d", &x_val, &y_val, &hour, &minute, &second) == 5) {
            x_data[numPoints] = x_val;
            y_data[numPoints] = y_val;
             time_data[numPoints] = (double)hour * 3600 + (double)minute * 60 + (double)second;
            numPoints++;
        }    }
		
		fclose(file);
		if (numPoints > 0) {
        // Borrar el gráfico existente
        DeleteGraphPlot(panelHandleChild4, PANELG_GRAPH, -1, VAL_IMMEDIATE_DRAW );
        
        // Graficar los datos (aquí se grafica temperatura interna y externa vs. tiempo)
        int plotHandle1=PlotXY(panelHandleChild4, PANELG_GRAPH, time_data, x_data, numPoints, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_BLUE);
        SetPlotAttribute(panelHandleChild4, PANELG_GRAPH, plotHandle1, ATTR_PLOT_LG_TEXT, "Temperatura Interna");

		int plotHandle2 =PlotXY(panelHandleChild4, PANELG_GRAPH, time_data, y_data, numPoints, VAL_DOUBLE, VAL_DOUBLE, VAL_THIN_LINE, VAL_EMPTY_SQUARE, VAL_SOLID, 1, VAL_RED);
       	SetPlotAttribute(panelHandleChild4, PANELG_GRAPH, plotHandle2, ATTR_PLOT_LG_TEXT, "Temperatuta Externa");

        // Configurar el eje X para mostrar el tiempo
        SetCtrlAttribute(panelHandleChild4, PANELG_GRAPH, ATTR_XFORMAT, VAL_ABSOLUTE_TIME_FORMAT);
        SetAxisTimeFormat(panelHandleChild4, PANELG_GRAPH, VAL_BOTTOM_XAXIS, VAL_ABSOLUTE_TIME_FORMAT, "%H:%M:%S");

        SetCtrlAttribute(panelHandleChild4, PANELG_GRAPH, ATTR_XMAP_MODE, VAL_LINEAR);
        SetCtrlAttribute(panelHandleChild4, PANELG_GRAPH, ATTR_YMAP_MODE, VAL_LINEAR);
		
		// Cambiar el color de fondo del área de ploteo a un azul oscuro
    	SetCtrlAttribute(panelHandleChild4, PANELG_GRAPH, ATTR_PLOT_BGCOLOR, VAL_DK_BLUE);

    // Cambiar el color de fondo del control completo a un gris claro
    	SetCtrlAttribute(panelHandleChild4, PANELG_GRAPH, ATTR_PLOT_BGCOLOR, VAL_TRANSPARENT);

			break;
	} }
	return 0;
}

int CVICALLBACK CursorM (int panel, int control, int event,
						 void *callbackData, int eventData1, int eventData2)
{
	int    activeCursor;
    double cursorX, cursorY;
    char time_string[10]; // Búfer para la cadena de tiempo (ej. "HH:MM:SS\0")

    if ((event == EVENT_VAL_CHANGED)||(event == EVENT_COMMIT))
        {
        GetActiveGraphCursor (panelHandleChild4, PANELG_GRAPH , &activeCursor);
        GetGraphCursor (panelHandleChild4, PANELG_GRAPH , activeCursor, &cursorX, &cursorY);
       
            SetCtrlVal (panelHandleChild4, PANELG_NUMERIC_2 , cursorY);
             int total_seconds = cursorX;
        
        int hours = total_seconds / 3600;
        int minutes = (total_seconds % 3600) / 60;
        int seconds = total_seconds % 60;

        // Formatear las variables de tiempo en una única cadena
        sprintf(time_string, "%02d:%02d:%02d", hours, minutes, seconds);
     
        // Actualizar el control de cadena con el valor formateado
         SetCtrlVal(panelHandleChild4, PANELG_STRING_2, time_string);
           
        }
	return 0;
}

int CVICALLBACK SalirTCP (int panel, int control, int event,
						  void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		DiscardPanel (panelHandleChild5); 
			break;
	}
	return 0;
}

int CVICALLBACK AbrirTCPP (int panel, int control, int event,
						   void *callbackData, int eventData1, int eventData2)
{
	switch (event) //Es para conversor RS232 a TCP virtual
	{
		case EVENT_COMMIT:
 	int  portNum;
    char tempBuf[256] = "192.168.0.139";
	//char tempBuf[256] = "172.23.91.110";   
	//char tempBuf[256] = "10.26.48.100";  
//	char tempBuf[256] = "172.23.56.151";  
    char portNumStr[32];
	int boton=0;
    
    /* Prompt for the name of the server to connect to */
    //PromptPopup ("Server Name?", "Escriba la IP del Conversor de Medios "
    //             ".\n\n(Ejemplo xxx.xxx.xxx.xxx)",  tempBuf, 255);
	
	
    /* Prompt for the port number on the server */
    //PromptPopup ("Número de Puerto?","Escriba el número de puerto del Conversor de Medios "
    //             "\n\n(example: 10001)",portNumStr, 31);
    //portNum = atoi (portNumStr);
	if (port_open==1) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Ya está conectado a un Puerto Serial");
				 return 0; 
			 }
	portNum = 1502; 
	
	GetCtrlVal(panelHandleChild5, TCP1_RADIOBUTTON, &boton);
    if (boton==1)
	{
		//SetCtrlVal (panelHandleChild5, TCP1_SERVER_IP_2, "");
		GetCtrlVal(panelHandleChild5, TCP1_SERVER_IP_2, tempBuf);
				
	}
	/* Attempt to connect to TCP server... */
    SetWaitCursor (1);
    if (ConnectToTCPServer (&g_hconversation, portNum, tempBuf, NULL, //(&g_hconversation, portNum, tempBuf, ClientTCPCB,
                            NULL, 5000) < 0)
        MessagePopup("TCP Client", "Falló la conxión al Dispositivo !");
    else
        {
        SetWaitCursor (0);
        g_connected = 1;
        
        /* We are successfully connected -- gather info */
        SetCtrlVal (panelHandleChild5, TCP1_CONNECTED , 1);
  		if (GetTCPHostAddr (tempBuf, 256) >= 0)
	        SetCtrlVal (panelHandleChild5, TCP1_CLIENT_NAME, tempBuf);
        tcpChk (GetTCPPeerAddr (g_hconversation, tempBuf, 256));
        SetCtrlVal (panelHandleChild5, TCP1_SERVER_IP, tempBuf);
        //tcpChk (GetTCPPeerName (g_hconversation, tempBuf, 256));
        //SetCtrlVal (g_hmainPanel, MAINPNL_SERVER_NAME, tempBuf);
        SetCtrlAttribute (panelHandleChild5, TCP1_COMMANDBUTTON_6, ATTR_DIMMED ,0);
		SetCtrlAttribute (panelHandleChild5, TCP1_COMMANDBUTTON_5, ATTR_DIMMED ,1);
        SetCtrlVal (panelHandleChild5, TCP1_LED, 1); 
        }
    
Done:
    /* Disconnect from the TCP server */
   // if (g_connected)
    //	DisconnectFromTCPServer (g_hconversation);

    /* Free resources and return */
  		tcp_open =1;		
			
			break;
	}
	return 0;
}

int CVICALLBACK CerrarTCPIP (int panel, int control, int event,
							 void *callbackData, int eventData1, int eventData2)
{
	switch (event)
	{
		case EVENT_COMMIT:
		SetCtrlAttribute (panelHandleChild5, TCP1_COMMANDBUTTON_6, ATTR_DIMMED ,1);
		SetCtrlAttribute (panelHandleChild5, TCP1_COMMANDBUTTON_5, ATTR_DIMMED ,0);
		port_open = 0;  /* Cierro Puerto */  
		DisconnectFromTCPServer (g_hconversation); 
		SetCtrlVal (panelHandleChild5, TCP1_LED, 0);
		SetCtrlVal (panelHandleChild5, TCP1_CONNECTED, 0); 
		tcp_open =0;
			break;
	}
	return 0;
}

void CVICALLBACK AbrirTCP (int menuBar, int menuItem, void *callbackData,
						   int panel)
{
	if ((panelHandleChild5 = LoadPanel (0, "RTUM.uir", TCP1)) < 0)
		return  ;
	
		InstallPopup (panelHandleChild5);
}





/*---------------------------------------------------------------------------*/
/* Report TCP Errors if any                         						 */
/*---------------------------------------------------------------------------*/
static void ReportTCPError(void)
{
	if (g_TCPError < 0)
		{
		char	messageBuffer[1024];
		sprintf(messageBuffer, 
			"TCP library error message: %s\nSystem error message: %s", 
			GetTCPErrorString (g_TCPError), GetTCPSystemErrorString());
		MessagePopup ("Error", messageBuffer);
		g_TCPError = 0;
		}
}


/*---------------------------------------------------------------------------*/
/* This is the TCP client's TCP callback.  This function will receive event  */
/* notification, similar to a UI callback, whenever a TCP event occurs.      */
/* We'll respond to the DATAREADY event and read in the available data from  */
/* the server and display it.  We'll also respond to DISCONNECT events, and  */
/* tell the user when the server disconnects us.                             */
/*---------------------------------------------------------------------------*/
int CVICALLBACK ClientTCPCB (unsigned handle, int event, int error,
                             void *callbackData)
{
    char receiveBuf[256] = {0};
    ssize_t dataSize         = sizeof (receiveBuf) - 1;

    switch (event)
        {
        
	 case TCP_DATAREADY:
             //if ((dataSize = ClientTCPRead (g_hconversation, receiveBuf,
             //                               dataSize, 1000))
              //   < 0)
             //   {
              //   MessagePopup ("TCP Client", "Error en la recepción de datos!"); 
		 		
             //    }
            // else
             //	{
            	//receiveBuf[dataSize] = '\0';
                
             //    }
             break;
		  
        case TCP_DISCONNECT:
            MessagePopup ("TCP Client", "Server cerró la conexión!");
            SetCtrlVal (panelHandleChild5, TCP1_CONNECTED, 0);
            g_connected = 0;
            
            break;
    }
    return 0;
}
