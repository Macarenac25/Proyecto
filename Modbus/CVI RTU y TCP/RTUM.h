/**************************************************************************/
/* LabWindows/CVI User Interface Resource (UIR) Include File              */
/*                                                                        */
/* WARNING: Do not add to, delete from, or otherwise modify the contents  */
/*          of this include file.                                         */
/**************************************************************************/

#include <userint.h>

#ifdef __cplusplus
    extern "C" {
#endif

     /* Panels and Controls: */

#define  PANELG                           1
#define  PANELG_COMMANDBUTTON             2       /* control type: command, callback function: SalirG */
#define  PANELG_LED                       3       /* control type: LED, callback function: (none) */
#define  PANELG_STRING                    4       /* control type: string, callback function: (none) */
#define  PANELG_COMMANDBUTTON_2           5       /* control type: command, callback function: LeerTXT */
#define  PANELG_GRAPH                     6       /* control type: graph, callback function: CursorM */
#define  PANELG_NUMERIC_2                 7       /* control type: numeric, callback function: (none) */
#define  PANELG_STRING_2                  8       /* control type: string, callback function: (none) */

#define  Principal                        2
#define  Principal_COMMANDBUTTON_1        2       /* control type: command, callback function: SalirF */
#define  Principal_PICTURE                3       /* control type: picture, callback function: (none) */

#define  Serial                           3
#define  Serial_COMMANDBUTTON_1           2       /* control type: command, callback function: SalirRS */
#define  Serial_LED                       3       /* control type: LED, callback function: (none) */
#define  Serial_STRING                    4       /* control type: string, callback function: (none) */
#define  Serial_Velocidad                 5       /* control type: slide, callback function: (none) */
#define  Serial_COMPORT                   6       /* control type: slide, callback function: (none) */
#define  Serial_WAIT                      7       /* control type: numeric, callback function: (none) */
#define  Serial_COMMANDBUTTON_2           8       /* control type: command, callback function: AbrirPuerto */
#define  Serial_COMMANDBUTTON_3           9       /* control type: command, callback function: CerrarPuerto */
#define  Serial_PICTURE_2                 10      /* control type: picture, callback function: (none) */

#define  TCP1                             4
#define  TCP1_COMMANDBUTTON_1             2       /* control type: command, callback function: SalirTCP */
#define  TCP1_CONNECTED                   3       /* control type: LED, callback function: (none) */
#define  TCP1_PICTURE_2                   4       /* control type: picture, callback function: (none) */
#define  TCP1_CLIENT_NAME                 5       /* control type: string, callback function: (none) */
#define  TCP1_SERVER_IP_2                 6       /* control type: string, callback function: (none) */
#define  TCP1_SERVER_IP                   7       /* control type: string, callback function: (none) */
#define  TCP1_COMMANDBUTTON_5             8       /* control type: command, callback function: AbrirTCPP */
#define  TCP1_LED                         9       /* control type: LED, callback function: (none) */
#define  TCP1_DECORATION                  10      /* control type: deco, callback function: (none) */
#define  TCP1_RADIOBUTTON                 11      /* control type: radioButton, callback function: (none) */
#define  TCP1_COMMANDBUTTON_6             12      /* control type: command, callback function: CerrarTCPIP */

#define  VerDatos                         5
#define  VerDatos_COMMANDBUTTON_1         2       /* control type: command, callback function: SalirDa */
#define  VerDatos_LED_6                   3       /* control type: LED, callback function: (none) */
#define  VerDatos_LED                     4       /* control type: LED, callback function: (none) */
#define  VerDatos_STRING_2                5       /* control type: string, callback function: (none) */
#define  VerDatos_PICTURE_2               6       /* control type: picture, callback function: (none) */
#define  VerDatos_LED_4                   7       /* control type: LED, callback function: (none) */
#define  VerDatos_LED_5                   8       /* control type: LED, callback function: (none) */
#define  VerDatos_COMMANDBUTTON_7         9       /* control type: command, callback function: Cerrar2 */
#define  VerDatos_COMMANDBUTTON_8         10      /* control type: command, callback function: Abrir2 */
#define  VerDatos_LED_3                   11      /* control type: LED, callback function: (none) */
#define  VerDatos_LED_2                   12      /* control type: LED, callback function: (none) */
#define  VerDatos_COMMANDBUTTON_5         13      /* control type: command, callback function: Cerrar1 */
#define  VerDatos_COMMANDBUTTON_6         14      /* control type: command, callback function: Abrir1 */
#define  VerDatos_TEXTMSG                 15      /* control type: textMsg, callback function: (none) */
#define  VerDatos_NUMERICTHERM_2          16      /* control type: scale, callback function: (none) */
#define  VerDatos_NUMERICTHERM            17      /* control type: scale, callback function: (none) */
#define  VerDatos_TEXTMSG_2               18      /* control type: textMsg, callback function: (none) */
#define  VerDatos_COMMANDBUTTON_9         19      /* control type: command, callback function: ConfT */
#define  VerDatos_COMMANDBUTTON_11        20      /* control type: command, callback function: CrearArch */
#define  VerDatos_COMMANDBUTTON_10        21      /* control type: command, callback function: Arch */
#define  VerDatos_COMMANDBUTTON           22      /* control type: command, callback function: LeerT */
#define  VerDatos_TIMER                   23      /* control type: timer, callback function: Disparo */
#define  VerDatos_STRING                  24      /* control type: string, callback function: (none) */


     /* Control Arrays: */

          /* (no control arrays in the resource file) */


     /* Menu Bars, Menus, and Menu Items: */

#define  MENUBAR                          1
#define  MENUBAR_MENU1                    2       /* callback function: AbrirSerie */
#define  MENUBAR_MENU1_2                  3       /* callback function: AbrirTCP */
#define  MENUBAR_MENU2                    4       /* callback function: DatosM */
#define  MENUBAR_MENU3                    5       /* callback function: Grafico */
#define  MENUBAR_Salir                    6       /* callback function: SalirM */


     /* Callback Prototypes: */

int  CVICALLBACK Abrir1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Abrir2(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK AbrirPuerto(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
void CVICALLBACK AbrirSerie(int menubar, int menuItem, void *callbackData, int panel);
void CVICALLBACK AbrirTCP(int menubar, int menuItem, void *callbackData, int panel);
int  CVICALLBACK AbrirTCPP(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Arch(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Cerrar1(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK Cerrar2(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK CerrarPuerto(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK CerrarTCPIP(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK ConfT(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK CrearArch(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK CursorM(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
void CVICALLBACK DatosM(int menubar, int menuItem, void *callbackData, int panel);
int  CVICALLBACK Disparo(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
void CVICALLBACK Grafico(int menubar, int menuItem, void *callbackData, int panel);
int  CVICALLBACK LeerT(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK LeerTXT(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SalirDa(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SalirF(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SalirG(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
void CVICALLBACK SalirM(int menubar, int menuItem, void *callbackData, int panel);
int  CVICALLBACK SalirRS(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);
int  CVICALLBACK SalirTCP(int panel, int control, int event, void *callbackData, int eventData1, int eventData2);


#ifdef __cplusplus
    }
#endif