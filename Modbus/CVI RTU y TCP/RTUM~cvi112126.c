#include <cvirte.h>		
#include <userint.h>
#include "RTUM.h"
#include <ansi_c.h>
#include <formatio.h>
#include <utility.h>
#include <rs232.h>
#pragma C99_extensions_on   
#include "ModbusFramework.h" 
static int panelHandle,panelHandleChild2, panelHandleChild3, panelHandleChild4;
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
/* Registros y banderas                                                      */
/*---------------------------------------------------------------------------*/
 
int port_open=0; //Indica si hay un puerto serial abierto
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
			 if (port_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere abrir Ventilador 1");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			//Consultar1T(); 
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05A(1,0x0,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_3,1);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_2,0);
			//Consultar1T();
			}
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
			 if (port_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere cerrar Ventilador 1");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			//Consultar1T(); 
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05(1,0x0,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_3,0);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_2,1);
			//Consultar1T();
			}
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
			 if (port_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere abrir Ventilador 2");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			//Consultar1T(); 
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05A(1,0x1,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_5,0);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_4,1);
			//Consultar1T();
			}
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
			 if (port_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
			// if (TimerOn()==0) return 0;  // es para evitar que se superponga con la pregunta periodica
			valor=ConfirmPopup ("Consulta de operación", "Realmente quiere cerrar Ventilador 2");
			if (valor==1)  //Si valor es =1 quiere hacer el comando
			{
			//Consultar1T(); 
			FlushInQ (comport); 
			FlushOutQ (comport); 
			Funcion05(1,0x1,comport, espera );  // Esclavo 2 bobina 0x4004
			SetCtrlVal(panelHandleChild3,VerDatos_LED_5,1);
			SetCtrlVal(panelHandleChild3,VerDatos_LED_4,0);
			//Consultar1T();
			}
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
			 if (port_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return 0; 
			 }
	 
			FlushInQ (comport); 
			slave=1; //direccion de esclavo
			memAdr=0x00;// Direccion del Modulo IA
	        nRegs=0x08;// Leo los registroa IA,IB, IC y IT
			comerr = QueryMem (slave, memAdr, nRegs, comport, espera);
			if (comerr < 0) goto Error;
	 		// Analizo la repuesta del esclavo  
	  		ReturnMBAnswer (buf, &bRcvd);
			Ti =(buf[13]<<8) +(buf[14]) ; // hay que dividir por 16
			Te =(buf[11]<<8) +(buf[12]) ; // hay que dividir por 100
            
		    Ti=Ti/16.0; //Temperatura Ds
			Te=Te/250.0; //Temperatura Ds
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
			if (port_open==0) 
			 {
				 SetCtrlAttribute (panelHandleChild3, VerDatos_TIMER,ATTR_ENABLED ,0); //Apagar timer  
				SetCtrlAttribute (panelHandleChild3, VerDatos_COMMANDBUTTON_9,ATTR_CMD_BUTTON_COLOR ,VAL_RED);
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
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
		 
			 if (port_open==0) 
			 {
				 MessagePopup (" Puerto de comunicaciones", "Falta seleccionar Puerto");
				 return ; 
			 }
	 
			FlushInQ (comport); 
			slave=1; //direccion de esclavo
			memAdr=0x00;// Direccion del Modulo IA
	        nRegs=0x08;// Leo los registroa IA,IB, IC y IT
			comerr = QueryMem (slave, memAdr, nRegs, comport, espera);
			if (comerr < 0) goto Error;
	 		// Analizo la repuesta del esclavo  
	  		ReturnMBAnswer (buf, &bRcvd);
			Ti =(buf[13]<<8) +(buf[14]) ; // hay que dividir por 16
			Te =(buf[11]<<8) +(buf[12]) ; // hay que dividir por 100
            
		    Ti=Ti/16.0; //Temperatura Ds
			Te=Te/250.0; //Temperatura Ds
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
        return 0;
    }
    
     // Abrir el archivo por segunda vez para leer los datos
    file = fopen(ruta_archivo, "r");
    if (file == NULL) {
        free(x_data);
        free(y_data);
        return;
    }

    // Leer los datos línea por línea
    for (int i = 0; i <= lines; i++) {
        // Asumiendo formato "X, Y" por línea
        fscanf(file, "%lf, %lf", &x_data[i], &y_data[i]);
    }
	
	char line_buffer[512];
    double x_val, y_val;
	int numPoints = 0;
    int hour, minute, second;
// Leer el archivo línea por línea
    while (fgets(line_buffer, sizeof(line_buffer), file) != NULL && numPoints < lines) {
        // Analizar la línea usando sscanf
        if (sscanf(line_buffer, "%lf %lf %d:%d:%d", &x_val, &y_val, &hour, &minute, &second) == 5) {
            x_data[numPoints] = x_val;
            y_data[numPoints] = y_val;
            //time_data[numPoints] = (double)hour * 3600 + (double)minute * 60 + (double)second;
            numPoints++;
        }
    }



			break;
	}
	return 0;
}
