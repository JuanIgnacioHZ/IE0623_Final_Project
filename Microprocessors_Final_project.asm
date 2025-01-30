;===============================================================================
;
;                           Universdad de Costa Rica
;                        Escuela de Ingeniería Eléctrica
;                           Microprocesadores IE0623
;                                Proyecto Final
;                                  Radar 623
;
;===============================================================================
;       Explicación general:
;
;       El programa propuesto implementa una analogía de radar de detección de
;    velocidad de automóviles, donde se tienen dos sensores ultrasónicos que se
;    activan ante el paso de un vehículo, una pantalla para que quien conduzca
;    pueda saber su velocidad promedio actual, la velocidad límites y si se
;    encuentra dentro de los límites definidos de 45 km/h y 99 km/h. Además, se
;    cuenta con un pánel de control donde se puede modificar la velocidad máxima
;    permitida en la calle que se instale el sistema.
;
;       Desglose de subrutinas y tareas:
;               Nombre: Línea de inicio.
;               1. Tarea_LeerDS: .
;               2. Tarea_Modo_Inactivo: .
;               3. Tarea_Configurar: .
;               4. Tarea_EnServicio: .
;               5. Tarea_DsplzLeds: .
;               6. Tarea_Brillo: .
;               7. Tarea_Teclado: .
;               8. Tarea_Led_Testigo: .
;               9. Tarea_Leer_PB0 y Tarea_Leer_PB1: .
;               10. Tarea_PantallaMUX: .
;               11. Tarea_LCD y Send_LCD: .
;               12. Subrutina BCD_BIN: .
;               13. Subrutina Calcula: .
;               14. Subrutina BIN_BCD_MUXP: .
;               15. Subrutina BCD_7SEG: .
;               16. Subrutina Borrar_NumArray: .
;               17. Máquina de tiempos: .
;
; Estudiante: Juan Ignacio Hernández Zamora
; Carné: B93826
; Fecha: 26 de junio de 2024
;
;===============================================================================

#include registers.inc

;===============================================================================
;                    RELOCALIZACION DE VECTOR DE INTERRUPCION
;===============================================================================
                Org $3E64
                dw Maquina_Tiempos
;===============================================================================
;                             DEFINICION DE VALORES
;===============================================================================
; Nota: Las definiciones de valores se presentan en el orden dado para las
;       las subrutinas.
;____ Valores I.  Generales ____________________________________________________
MENSAJES:       EQU $1200       ; Dirección de inicio de la tabla de mensajes.
Dir_ESTR_DATOS: EQU $1000       ; Ubicación en memoria a partir de la cual se
                                ; colocan las estructuras de datos.
Dir_T_PantMUX:  EQU $1020       ; Ubicación en memoria de las estructuras de
                                ; datos de Tarea_PantallaMUX.
Dir_BANDERAS:   EQU $1070       ; Ubicación en memoria de las estructuras de
                                ; datos del registro de banderas.
Dir_Generales:  EQU $1080       ; Ubicación en memoria de las variables de uso
                                ; general.
Dir_T_TIMERS:   EQU $1500       ; Ubicación en memoria de la tabla de timers.
MAIN:           EQU $2000       ; Dirección de memoria a partir de la cual se
                                ; ubica el programa principal.
Mask_LD_Br_MD:  EQU $F8         ; Máscara para apagar los leds de todos los
                                ; modos de operación.

;____ Valores #1.  Tarea_LeerDS ________________________________________________
tTimerRebDS:    EQU 10          ; 10 mS para rechazo de rebote de los DS.
PortDS:         EQU PTIH        ; Puerto para leer los DS.
Mask_Mode_Sel:  EQU $C0         ; Constante para detectar qué dipswitches
                                ; activan cuáles modos.

;____ Valores #2.  Tarea_Modo_Inactivo _________________________________________
LDInactivo:     EQU $01         ; Led PB0. Led a encender en el modo inactivo.
tTimerVLim:     EQU 30          ; 3 segundos = 30*100mS
Mask_DS_MD_Ina: EQU 0           ; Disposición de los dipswitches para el modo
                                ; inactivo.

;____ Valores #3.  Tarea_Configurar ____________________________________________
LDConfig:       EQU $02         ; Led PB1. Led a encender en el modo configurar.
LimMax:         EQU 90          ; Límite de velocidad máxima.
LimMin:         EQU 65          ; Límite mínimo de la velocidad.
Mask_DS_MD_Cfg: EQU $40         ; Disposición de los dipswitches para el modo
                                ; configurar.
Def_Vel_LIM:    EQU 65          ; Valor por defecto (después del poweron-reset)
                                ; para la variable Vel_LIM

;____ Valores #4.  Tarea_EnServicio ____________________________________________
LDEnServ:       EQU $04         ; Led PB2.
tTimerVel:      EQU 100         ; Tiempo para esperar a que el vehículo pase por
                                ; los sensores.
tTimerError:    EQU 30          ; Timer de 3 segundos en base 100mS
VelocMax:       EQU 99          ; Límite de velocidad máxima.
VelocMin:       EQU 45          ; Límite mínimo de la velocidad.
Mask_DS_MD_Srv: EQU $80         ; Disposición de los dipswitches para el modo
                                ; en servicio.

;____ Valores #5.  Tarea_DsplzLeds _____________________________________________
tTimerDplzLeds: EQU 1           ; Variable actual de la máquina de DsplzLeds.
LED_Sup:        EQU %10000000   ; Valor del led superior.
LED_Inf:        EQU %00001000   ; Valor del led inferior.
Mask_Dspz_LEDS: EQU $07         ; Máscara para borrar los leds a desplazar y
                                ; dejar los de operación.

;____ Valores #6.  Tarea_Brillo ________________________________________________
tTimerBrillo:   EQU 4           ; Valor de cuenta de 400 mS mientras se realza
                                ; el ciclo de conversión. (4x100mS)
MaskSCF:        EQU $80         ; Valor para determinar cuando se termina el
                                ; ciclo de conversión. Corresponde con el
                                ; registro ATD0STAT0
ATD_Read_CMD:   EQU $87         ; Valor a escribir en el registo de control 5
                                ; del conversor analógico digital para iniciar
                                ; la lectura. Análogo a un comando.
Cte_Div_Vl_ATD: EQU 255         ; Constante de división para representar la
                                ; pendiente de la recta de la regresión lineal
                                ; para convertir el valor leído del ATD al rango
                                ; de brillo: de [0,255] a [0,100].
Cte_Mul_Vl_ATD: EQU 100         ; Constante de multiplicación para la regresión
                                ; lineal.

;____ Valores #7.  Tarea_Teclado _______________________________________________
PortTCL:        EQU PORTA       ; Registro de datos del puerto A
Teclas_MAX:     EQU 2           ; Cantidad máxima de teclas a leer del teclado
Teclas:         EQU $1110       ; Etiqueta de ubicación de la tabla de Teclas
Tecla_BORRAR:   EQU $0B         ; Para detectar la tecla borrar.
Tecla_ENTER:    EQU $0E         ; Para detectar la tecla enter.
Ninguna_Tecla:  EQU $FF         ; Caso que no se presione ninguna tecla.
Num_Array:      EQU $1010       ; Arreglo de destino de los valores leídos.
;     Leer_teclado
LT_Fin_Ciclo:   EQU $F0         ; Constante de finalización de ciclo para la
                                ; lectura del teclado.

;____ Valores #8.  Tarea_Led_Testigo ___________________________________________
tTimerLDTst:    EQU 5           ; Tiempo de parpadeo de LED testigo centenas
                                ; de microsegundos

;____ Valores #9.  Tarea_Leer_PB0 y Tarea_Leer_PB1 _____________________________
;     Generales
PortPB:         EQU PTIH        ; Puerto donde se ubica el PB
tSupRebPB:      EQU 10          ; Contador para supresión de rebotes, 10 mS
tShortP:        EQU 25          ; Contador para detección de short press, 250 mS
tLongP:         EQU 2           ; Contador para detección de long press, 2 S
;     PB0
MaskPB0:        EQU $01         ; Ubicación del SW5 en el puerto H.
;     PB1
MaskPB1:        EQU $08         ; Ubicación del SW2 en el puerto H.

;____ Valores #10. Tarea_PantallaMUX ___________________________________________
tTimerDigito:   EQU 2           ; Tiempo de encendido de cada display = 2ms
MaxCountTicks:  EQU 100         ; Tiempo en ticks entre dos periodos de display
Segment:        EQU $1100       ; Tabla de equivalencias para códigos BCD-7seg
OFF:            EQU 11          ; Offset de la tabla segment para apagar Dsp
GUIONES:        EQU 10          ; Offset de la tabla segment para mostrar "-"

;____ Valores #11. Tarea_LCD y Send_LCD ________________________________________
;     Tarea_LCD
tTimer2mS:      EQU 2           ; Valor a asignar en el timer: 2 x 1mS
tTimer260uS:    EQU 13          ; Valor a asignar en el timer: 13 x 1 tick
tTimer40uS:     EQU 2           ; Valor a asigner en el timer: 2 x 1 tick
EOB:            EQU $FF         ; Caracter de finalización de bloque
Clear_LCD:      EQU $01         ; Comando para limpiar la pantalla
ADD_L1:         EQU $80         ; Dirección (en pantalla) de inicio de msg
                                ; en línea 1
ADD_L2:         EQU $C0         ; Dirección (en pantalla) de inicio de msg
                                ; en línea 2
IniDsp:         EQU $102D       ; Posición de memoria del arreglo de comandos
                                ; de inicialización de la LCD.

;____ Valores #12. Subrutina BCD_BIN ___________________________________________

;____ Valores #13. Subrutina Calcula ___________________________________________
Cte_Dvndo_Vel:  EQU 1440        ; Constante de dividendo para obtener la
                                ; velocidad.
Cte_Dvndo_TP:   EQU 7200        ; Constante de dividendo para obtener TimerPant
Cte_Dvndo_TFP:  EQU 10800       ; Constante de dividendo para obtener
                                ; TimerFinPant.

;____ Valores #14. Subrutina BIN_BCD_MUXP ______________________________________

;____ Valores #15. Subrutina BCD_7SEG __________________________________________

;____ Valores #16. Subrutina Borrar_NumArray ___________________________________
BNA_Blank_Val:  EQU $FF         ; Valor en blanco para borrar NumArray

;____ Valores #17. Máquina de tiempos __________________________________________
; Nota sobre la máquina de tiempos con output comare:
; La frecuencia de interrupción es de 50kHz, lo que implica un periodo entre
; interrupciones de 1/(50kHz) = 2x10^(-5)s = 20uS, con base en esto fue que se
; redefinieron los valores de las constantes.
Contante_OC5_MT:EQU 30          ; Constante que se suma a TCNT para que la
                                ; interrupción por output compare sea
                                ; regular en el tiempo.
tTimer1mS:      EQU 50          ; Base de tiempo de 1 mS (20 uS x 50)
tTimer10mS:     EQU 500         ; Base de tiempo de 10 mS (20 uS x 500)
tTimer100mS:    EQU 5000        ; Base de tiempo de 100 mS (20 uS x 5000)
tTimer1S:       EQU 50000       ; Base de tiempo de 1 segundo (20 uS x 50000)
RTI_Flag:       EQU $80         ; Eqtiqueta para reactivar la interrupción RTI

;===============================================================================
;                       DEFINICION DE ESTRUCTURAS DE DATOS
;===============================================================================
; Nota: Dado que el posicionamiento de las estructuras de datos es crucial para
;       el desarrollo del proyecto, se ordenaron tomando como referencia la
;       tabla de estructuras de datos y no la lista de subrutinas dada en el
;       enunciado del profesor. La morfología utilizada en el listado se
;       encuentra en la siguiente línea.
;Etiqueta       Def  Dir        Descripción
;       Cuando el formato no se puede cumplir, se coloca la directiva "ds" y
;       la información subsiguiente una línea después.
; Se colocan las estructuras de datos en las posiciones de memoria solicitadas.
                Org Dir_ESTR_DATOS
;____ Variables #1.  Tarea_Teclado _____________________________________________
MAX_TCL:        ds 1 ;$1000     Cantidad máxima de teclas a leer
Tecla:          ds 1 ;$1001     Valor de tecla leído
Tecla_IN:       ds 1 ;$1002     Valor temporal de tecla, usado para detectar
                     ;          los rebotes y evitar teclas presionadas.
Cont_TCL:       ds 1 ;$1003     Offset para recorrer el arreglo de números
                     ;          leídos y determinar cúantos se tienen.
PATRON:         ds 1 ;$1004     Patrón a escribir en el puerto A del teclado
Est_Pres_TCL:   ds 2 ;$1005     Variable de estado de la máquina de estados TCL

;____ Variables #2.  Tarea_PantallaMUX _________________________________________
                Org Dir_T_PantMUX
EstPres_PantallaMUX: ;$1020     Estado actual de la máquina de pantalla
                ds 2 ;          multiplexada.
Dsp1:           ds 1 ;$1022     Valor del dígito 1 codificado para 8 segmentos
Dsp2:           ds 1 ;$1023     Valor del dígito 2 codificado para 8 segmentos
Dsp3:           ds 1 ;$1024     Valor del dígito 3 codificado para 8 segmentos
Dsp4:           ds 1 ;$1025     Valor del dígito 4 codificado para 8 segmentos
LEDS:           ds 1 ;$1026     Variable que almacena el estado actual de leds
                     ;          en PortB
Cont_Dig:       ds 1 ;$1027     Contador de dígitos colocados en el display
Brillo:         ds 1 ;$1028     Cantidad de ticks que pasa el display apagado

;____ Variables #3.  Variables para subrutinas de Conversión ___________________
BCD:            ds 1 ;$1029     Valor binario en BCD
Cont_BCD:       ds 1 ;$102A     Contador de valores BCD
BCD1:           ds 1 ;$102B     Valor binario 1 en BCD
BCD2:           ds 1 ;$102C     Valor binario 2 en BCD

;____ Variables #4.  Tarea LCD _________________________________________________
                Org IniDsp      ; Se inicia como etiqueta para poder barrer los
                                ; caracteres como un array.
                dB $28          ; Comando function set: 2 líneas con bus de 4
                dB $28          ; bits y caracteres de 5x8 puntos.
                dB $06          ; Entry mode set: escribe de izq. a der.
                dB $04          ; Display ON/OFF: Display ON, Cursor OFF, no
                                ; blinking.
                dB  $FF         ; Fin de mensaje.
Punt_LCD:       ds 2 ;$1032     Puntero de LCD al mensaje a enviar.
CharLCD:        ds 1 ;$1034     Caracter a enviar a la pantalla
Msg_L1:         ds 2 ;$1035     Mensaje de la línea 1 de la LCD
Msg_L2:         ds 2 ;$1037     Mensaje de la línea 2 de la LCD
EstPres_SendLCD:ds 2 ;$1039     Estado actual de la máquina SendLCD
EstPres_TareaLCD:
                ds 2 ;$103B     Estado actual de la máquina TareaLCD

;____ Variables #5.  Tarea Leer PB0 y Tarea Leer PB1 ___________________________
Est_Pres_LeerPB0:
                ds 2 ;$103D     Estado de la máquina de estados para leer PB0.
Est_Pres_LeerPB1:
                ds 2 ;$103F     Estado actual de la máquina que lee PH3.

;____ Variables #6.  Tarea Configurar __________________________________________
Est_Pres_TConfig:
                ds 2 ;$1041     Estado actual de la máquina de la tarea config.
ValorLIM:       ds 1 ;$1043     Variable donde se almacena temporalmente la 
                     ;          velocidad límite.
Vel_LIM:        ds 1 ;$1044     Valor actual de la velocidad límite.

;____ Variables #7.  Tarea Inactivo ____________________________________________
Est_Pres_TInac: ds 2 ;$1045     Estado actual de la máquina para tarea inactivo

;____ Variables #8.  Tarea EnServicio __________________________________________
Est_Pres_TServ: ds 2 ;$1047     Estado actual de la máquina en servicio.
Vel_Calc:       ds 1 ;$1049     Valor calculado de velocidad
DeltaT:         ds 1 ;$104A     Diferencia de tiempo, en ticks, entre la
                     ;          activación de los sensores.

;____ Variables #9.  Tarea Brillo ______________________________________________
Est_Pres_TBrillo:
                ds 2 ;$105B     Estado actual de la máquina que actualiza el
                     ;          valor de brillo.

;____ Variables #10. Tarea Leer DS _____________________________________________
Est_Pres_LeerDS:ds 2 ;$105D     Estado actual de la máquina que lee dipswitches
Temp_DS:        ds 1 ;$105F     Valor temporal de DS mientras se rechaza los
                     ;          rebotes.
Valor_DS:       ds 1 ;$1050     Valor final de los dipsiwtches luego de
                     ;          rechazar los rebotes.

;____ Variables #11. Tarea Desplazar Leds ______________________________________
Est_Pres_DsplzLeds:
                ds 2 ;$1051     Estado actual de la máquina que desplaza los
                     ;          cinco les más significativos del puerto B.
DplzLeds:       ds 1 ;$1053     Variable con el estado actual de los leds de
                     ;          alarma.

;____ Variables #12. BANDERAS __________________________________________________
                Org Dir_BANDERAS
Banderas_1:     ds 1 ;$1070     Registro de banderas 1
Banderas_2:     ds 1 ;$1071     Registro de banderas 2
;____ Valores de los registros de banderas ______________________________________
;     Banderas_1
ShortP0:        EQU $01         ; Valor para identificar un pulso corto en PH.0.
LongP0:         EQU $02         ; Valor para identificar un pulso largo en PH.0.
ShortP1:        EQU $04         ; Valor para identificar un pulso corto en PH.3.
LongP1:         EQU $08         ; Valor para identificar un pulso largo en PH.3.
ARRAY_OK:       EQU $10         ; Etiqueta para detectar que Num_Array esté
                                ; listo.
;     Banderas_2
RS:             EQU $01         ; Bandera para enviar un .
LCD_Ok:         EQU $02         ; Determina si la LCD ya tiene los mensajes
                                ; deseadps en pantalla.
FinSendLCD:     EQU $04         ; Se pone en alto cuando se terminó de enviar
                                ; un mensaje.
Second_Line:    EQU $08         ; Diferencia entre enviar la primera o segunda
                                ; línea.
Alarma:         EQU $10         ; Bandera para activar la alarma desde la tarea
                                ; EnServicio.
DspIzquierda:   EQU $20         ; En activo indica si se desplaza los LEDS
                                ; hacia la izquierda.

;____ Variables #13. Generales _________________________________________________
                Org Dir_Generales
LED_Testigo:    ds 1 ;$1080     Valor actual del led testigo.

;____ Variables #14. TABLAS ____________________________________________________
;     Tabla Segment
                Org Segment
                dB $3F          ; Valor de 0 (BCD) codificado para 7 segmentos
                dB $06          ; Valor de 1 (BCD) codificado para 7 segmentos
                dB $5B          ; Valor de 2 (BCD) codificado para 7 segmentos
                dB $4F          ; Valor de 3 (BCD) codificado para 7 segmentos
                dB $66          ; Valor de 4 (BCD) codificado para 7 segmentos
                dB $6D          ; Valor de 5 (BCD) codificado para 7 segmentos
                dB $7D          ; Valor de 6 (BCD) codificado para 7 segmentos
                dB $07          ; Valor de 7 (BCD) codificado para 7 segmentos
                dB $7F          ; Valor de 8 (BCD) codificado para 7 segmentos
                dB $6F          ; Valor de 9 (BCD) codificado para 7 segmentos
                dB $40          ; Valor de guión e pantalla de 7 segmentos
                dB $00          ; Valor de pantalla apagada para 7 segmentos

;     Tabla Teclas 
                Org Teclas
                dB $01          ; Constante para identificar la tecla 1
                dB $02          ; Constante para identificar la tecla 2
                dB $03          ; Constante para identificar la tecla 3
                dB $04          ; Constante para identificar la tecla 4
                dB $05          ; Constante para identificar la tecla 5
                dB $06          ; Constante para identificar la tecla 6
                dB $07          ; Constante para identificar la tecla 7
                dB $08          ; Constante para identificar la tecla 8
                dB $09          ; Constante para identificar la tecla 9
                dB $0B          ; Constante para identificar la tecla Borrar
                dB $00          ; Constante para identificar la tecla 0
                dB $0E          ; Constante para identificar la tecla Enter

;____ Variables #15. MENSAJES __________________________________________________
                Org MENSAJES
MSG_Md_Inact_L1:Fcc "   RADAR 623    "
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_Md_Inact_L2:Fcc " MODO INACTIVO  "
                dB  $FF         ; Mensaje seguido de caracter de finalización.

MSG_Config_L1:  Fcc " MODO CONFIGURAR"
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_Config_L2:  Fcc "         VEL_LIM"
                dB  $FF         ; Mensaje seguido de caracter de finalización.

MSG_Inicial_L1: Fcc "   RADAR  623   "
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_Inicial_L2: Fcc "   ESPERANDO... "
                dB  $FF         ; Mensaje seguido de caracter de finalización.

MSG_EnServ_L1:  Fcc "MODO EN SERVICIO"
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_EnServ_L2:  Fcc "V_LIM  SU_VELOC "
                dB  $FF         ; Mensaje seguido de caracter de finalización.

MSG_Clcndo_L1:  Fcc "  RADAR  623    "
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_Clcndo_L2:  Fcc "  CALCULANDO... "
                dB  $FF         ; Mensaje seguido de caracter de finalización.

MSG_Alerta_L1:  Fcc "** VELODIDAD ** "
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_Alerta_L2:  Fcc "*FUERA DE RANGO*"
                dB  $FF         ; Mensaje seguido de caracter de finalización.

MSG_Alarma_L1:  Fcc "*V_LIM EXCEDIDA*"
                dB  $FF         ; Mensaje seguido de caracter de finalización.
MSG_Alarma_L2:  Fcc "V_LIM  SU_VELOC "
                dB  $FF         ; Mensaje seguido de caracter de finalización.

;____ Varialbes #16. TABLA DE TIMERS ___________________________________________
                Org Dir_T_TIMERS
Tabla_Timers_BaseT:

Timer1mS:       ds 2    ;$1500
Timer10mS:      ds 2    ;$1502
Timer100mS:     ds 2    ;$1504
Timer1S:        ds 2    ;$1506
CounterTicks:   ds 2    ;$1508
Timer260uS:     ds 2    ;$150A
Timer40uS:      ds 2    ;$150C
                        ;$150E
Fin_BaseT:      dW $FFFF

Tabla_Timers_Base1mS:

Timer_RebPB0:   ds 1    ;$1510
Timer_RebPB1:   ds 1    ;$1511
Timer_RebTCL:   ds 1    ;$1512
Timer_RebDS:    ds 1    ;$1513
TimerDigito:    ds 1    ;$1514
Timer2mS:       ds 1    ;$1515

Fin_Base1mS:    dB $FF  ;$1516

Tabla_Timers_Base10mS:

Timer_SHP0:     ds 1    ;$1517
Timer_SHP1:     ds 1    ;$1518

Fin_Base10ms:   dB $FF  ;$1519

Tabla_Timers_Base100mS:

Timer_DplzLeds: ds 1    ;$151A
TimerVel:       ds 1    ;$151B
TimerError:     ds 1    ;$151C
TimerPant:      ds 1    ;$151D
TimerFinPant:   ds 1    ;$151E
TimerBrillo:    ds 1    ;$151F
Timer_LED_Testigo:
                ds 1    ;$1520
Fin_Base100mS:  dB $FF  ;$1521

Tabla_Timers_Base1S:

Timer_LP0:      ds 1    ;$1522
Timer_LP1:      ds 1    ;$1523

Fin_Base1S:     dB $FF  ;$1524

;===============================================================================
;                         INICIO DEL CODIGO DEL PROGRAMA
;===============================================================================
                Org MAIN
;===============================================================================
;                         CONFIGURACION DE PERIFERICOS
;===============================================================================

        Bset DDRB,$FF   ; Habilitacion de LEDS para pantallas y PB
        Bset DDRP,$FF   ; Habilitación del LED testigo y los displays de 7s
        Bset DDRJ,$02   ; Setea pin 1 del puerto J como salida, esto para
                        ; conectar los LEDS a VCC
        BClr PTJ,$02    ; Conecta el cátodo de los LEDS para encenderlos
        
        Movb #$F0,DDRA  ; Establece los canales PA0-3 como salidas y los PA4-7
                        ; como entradas para leer el teclado multiplexado
        Movb #$01,PUCR  ; Se setea el puerto A como pull-up.
        
        Movb #$00,DDRH  ; Se activan todos los dipswitches según se solicitó.

        ; Configuración de módulo de timer
        Movb #$90,TSCR1 ; Enciende el módulo de timer y activa el borrado
                        ; automático de banderas.
        Movb #$04,TSCR2 ; Selecciona un valor de 16 en el preescalador.
        Movb #$20,TIOS  ; Activa la interrupción por comparación en el canal 5.

        Movb #$80,ATD0CTL2      ; Señal de activación del ATD.
        Ldaa #160               ; Se carga una constante para restar mientras
                                ; se espera a que termine de inicializarse el
                                ; ATD.
ATD_Init_Loop:                  ; Analogue to digital converter initialization
                                ; loop.
        Dbne A,ATD_Init_Loop    ; Se mantiene restando 1 a A mientras se espera
        Movb #$10,ATD0CTL3      ; Configura el conversor para realizar dos
                                ; conversiones.
        Movb #$B1,ATD0CTL4      ; Configura el convertidor para operar a 8
                                ; bits, setea PRS=17 de modo que la frecuencia
                                ; de muestreo sea ~650kHz, y utiliza 4 ciclos
                                ; de reloj en cada conversión.

        Movb #$20,TIE           ; Cuenta inicial del módulo timer.
        
        ; Realoja tiempo en el contador del canal 5 que se atiende por OC.
        Ldx  TCNT               ; Carga la cuenta actual y la cantidad de ticks
                                ; en el futuro que dispararán la siguiente
                                ; interrupción.
        Ldab #Contante_OC5_MT
        Abx                     ; Suma ambos valores.
        Stx  TC5                ; Salva el valor de la siguiente interrupción
                                ; en su registro respectivo.

;===============================================================================
;                    INICIALIZACION DE ESTRUCTURAS DE DATOS
;===============================================================================
;____ Timers de la máquina de tiempos __________________________________________
        Movw #tTimer1mS,Timer1mS
        Movw #tTimer10mS,Timer10mS
        Movw #tTimer100mS,Timer100mS
        Movw #tTimer1S,Timer1S

;____ Variables de Led Testigo _________________________________________________
                                ; Timer de parpadeo
        Movb #tTimerLDTst,Timer_LED_Testigo
        Bset PTP,$00            ; Pone en azul el LED testigo
                                ; RGB.
        Clr  LED_Testigo        ; Inicia el la variable del led testigo en azul.

;____ Variables referentes a Tarea_Teclado y Leer_Teclado ______________________
        Movb #Teclas_MAX,MAX_TCL; Inicializa las variables para el driver del
        Movb #$FF,Tecla         ; teclado: Tarea_Teclado y Leer_Teclado.
        Movb #$FF,Tecla_IN
        Clr  Cont_TCL
        Clr  PATRON
        Jsr  Borrar_NumArray    ; Reinicia los valores de
                                ; Num_Array.

;____ Inicialización de los registros de banderas ______________________________
        Clr  Banderas_1         ; Resetea todas las banderas
        Clr  Banderas_2         ; Resetea todas las banderas

;____ Variables de la tarea y subrutinas de los displays de de 7 segmentos _____
        Movb #0,TimerDigito     ; Timer de dígito se inicia en 0
        Movb #1,Cont_Dig        ; Se selecciona el primer dígito de los displays
                                ; para mostrar.
        Clr  LEDS               ; Los LEDS inician apagados

;____ Variables para la lectura de los dipswitches ______________________________
        Clr  Temp_DS            ; Se limpian las variables de
        Clr  Valor_DS           ; registro de los dipswitches.

;____ Variables para el desplazamiento de los leds de alarma ___________________
        Movb #$80,DplzLeds      ; Estado inicial de los leds a desplazar.
        Bclr Banderas_2,DspIzquierda
                                ; Se empieza por desplazar los leds de alarma
                                ; a la derecha.

;____ Se mueve el valor inicial de la velocidad límite a su variable ___________
        Movb #Def_Vel_LIM,Vel_LIM
                                ; Valor por inicial de la velocidad máxima.

;____ Inicialización de la pila ________________________________________________
        Lds  #$3BFF             ; Inicializa la pila para la Dragon12+ con
                                ; debug12.

;===============================================================================
;                     INICIALIZACIÓN DE MAQUINAS DE ESTADO
;===============================================================================
; Nota: Todas las máquinas de estado se inicializan en su estado inicial, o sea
;       el 1. De ahí que no se comenten.
        Movw #LeerPB0_Est1,Est_Pres_LeerPB0
        Movw #LeerPB1_Est1,Est_Pres_LeerPB1
        Movw #Teclado_Est1,Est_Pres_TCL
        Movw #PantallaMUX_Est1,EstPres_PantallaMUX
        Movw #Tarea_LCD_Est1,EstPres_TareaLCD
        Movw #Brillo_Est1,Est_Pres_TBrillo
        Movw #LeerDS_Est1,Est_Pres_LeerDS
        Movw #DsplzLeds_Est1,Est_Pres_DsplzLeds
        Movw #Configurar_Est1,Est_Pres_TConfig
        Movw #Modo_Inactivo_Est1,Est_Pres_TInac
        Movw #EnServicio_Est1,Est_Pres_TServ

;____ Activación de las interrupciones _________________________________________
        Cli                     ; Activa las interrupciones

;===============================================================================
;                      INICIALIZACION DE LA PANTALLA LCD
;===============================================================================
;       Configura la pantalla para usarla según la tarea y termina por
; reiniciarla.
;===============================================================================
Init_LCD:
        Movw #0,Timer40uS       ; Se inicializan en cero los contadores de la
        Movw #0,Timer260uS      ; LCD.
        Movb #0,Timer2mS
        
        Movb #$FF,DDRK          ; Activa las seis primeras líneas del puerto K

        Movb #0,Punt_LCD        ; Se inicializan las banderas de acuerdo al
        Bclr Banderas_2,RS      ; algoritmo
        Bclr Banderas_2,Second_Line
        Bset Banderas_2,LCD_Ok

        Movw #SendLCD_Est1,EstPres_SendLCD

        Ldx  #IniDsp             ; Obtiene la dirección de inicio de la tabla
                                ; de comandos de inicialización.
Init_LCD_Loop_Init_Cmds:        ; Loop de comandos de inicialización
        Ldaa 1,X+               ; Obtiene el dato actual, pasa al siguiente y
                                ; salta si se llegó al final.
        Staa CharLCD            ; Si se sigue enviando datos, se pasan memoria.
        Cmpa #EOB
        Bne  Init_LCD_Loop_Init_Cmds_Keep

        Movb #Clear_LCD,CharLCD ; Selecciona el comando clr en memoria.
Init_LCD_Loop_Clr:              ; Loop para envío de clear
        Jsr  Tarea_SendLCD
        Brclr Banderas_2,FinSendLCD,Init_LCD_Loop_Clr

        Movb #tTimer2mS,Timer2mS; Inicia el contador de 2mS mientras se ejecuta
                                ; el comando clear.
Init_LCD_Loop_2mS:              ; Se mantiene esperando a que pasen los 2mS.
        Ldaa Timer2mS
        Bne  Init_LCD_Loop_2mS

        Bra  Init_LCD_Fin

Init_LCD_Loop_Init_Cmds_Keep:   ; Loop de comandos de inicialización: Mantenerse
        Pshx                    ; Salva la dir actual en la tabla de init.
                                ; en pila.
Init_LCD_Loop_Cmd:              ; Loop mientras envía un comando.
        Jsr  Tarea_SendLCD
        Brclr Banderas_2,FinSendLCD,Init_LCD_Loop_Cmd
                                ; Borra la bandera de caracter enviado.
        Bclr Banderas_2,FinSendLCD
        Pulx                    ; Recupera dir actual en tabla.
        Bra Init_LCD_Loop_Init_Cmds
Init_LCD_Fin:

;===============================================================================
;                   PROGRAMA PRINCIPAL: DESPACHADOR DE TAREAS
;===============================================================================
Despachador_Tareas:
        Brset Banderas_2,LCD_Ok,DT_Otras_Tareas
        Jsr Tarea_LCD           ; Si hay un mensaje nuevo, lo imprime en la LCD.   

DT_Otras_Tareas:                ; De no haber mensajes pendientes, ejecuta las
        Jsr Tarea_Led_Testigo   ; demás tareas.
        Jsr Tarea_Leer_PB0      ;
        Jsr Tarea_Leer_PB1      ;
        Jsr Tarea_Teclado       ;
        Jsr Tarea_Brillo        ;
        Jsr Tarea_PantallaMUX   ;
        Jsr Tarea_LeerDS        ;
        Jsr Tarea_Modo_Inactivo ;
        Jsr Tarea_Configurar    ;
        Jsr Tarea_EnServicio    ;
        Jsr Tarea_DsplzLeds     ;
        Bra Despachador_Tareas

;===============================================================================
;    I. Tarea_LeerDS. Inicio
;===============================================================================
;       Lee mediante el conversor analógico digital el valor actual del
;    potenciómetro del pad 7, promedia dos muestreos y pasa el valor leído al
;    rango de 0 a 100 que admite la variable Brillo.
;===============================================================================
Tarea_LeerDS:
        Ldx  Est_Pres_LeerDS
        Jsr  0,X
Fin_Tarea_LeerDS:
        Rts

;==== Tarea_LeerDS ESTADO 1 ====================================================
LeerDS_Est1:
                                ; Se lee el valor temporal de los diswitches y
                                ; se inicia el temporizador de rechazo de
                                ; de rebote.
        Movb PortDS,Temp_DS
        Movb #tTimerRebDS,Timer_RebDS

        Movw #LeerDS_Est2,Est_Pres_LeerDS

LeerDS_Est1_END:
        Rts

;==== Tarea_LeerDS ESTADO 2 ====================================================
LeerDS_Est2:
        Tst  Timer_RebDS        ; Se espera a que termine el timer de rechazo
        Bne  LeerDS_Est2_END    ; de rebotes.

        Ldaa PortDS             ; Obtiene el valor actual del puerto.

        Cmpa Temp_DS            ; Compara el valor actual con el valor anterior
                                ; de modo que si son iguales, se tiene una
                                ; lectura exitosa y se salva el valor. Caso
                                ; contrario se vuelve a empezar.
        Bne  LeerDS_Est2_Terminar

        Staa Valor_DS           ; Salva el valor correcto en memoria.
        Bra  LeerDS_Est2_Terminar
LeerDS_Est2_Terminar:
                                ; Cuando se termina el proceso, se retorna al
                                ; estado 1, si la lectura fue exitosa, se
                                ; encuentra salva memoria el valor leído, si
                                ; no, se deja el anterior.
        Movw #LeerDS_Est1,Est_Pres_LeerDS

LeerDS_Est2_END:
        Rts
;===============================================================================
;    I.  Tarea_LeerDS. Final
;===============================================================================

;===============================================================================
;    II. Tarea_Modo_Inactivo. Inicio
;===============================================================================
;       Estado de operación ocioso del sistema, muestra un mensaje de
;    inactivdad y ante cualquier long press muestra la velocidad máxima actual.
;===============================================================================
Tarea_Modo_Inactivo:

        Ldaa Valor_DS           ; Obtiene el estado actual de los dipswitches.
        Anda #Mask_Mode_Sel     ; Deja sólo los valores de interés.

                                ; Compara la configuración de los dipsiwtches
                                ; con la de la tarea modo inactivo, si no
                                ; corresponden, termina la subrutina. Caso
                                ; contrario la ejecuta.
        Cmpa #Mask_DS_MD_Ina
        Bne  Fin_Tarea_Modo_Inactivo

        Ldx  Est_Pres_TInac
        Jsr  0,X
Fin_Tarea_Modo_Inactivo:
        Rts

;==== Tarea_Modo_Inactivo ESTADO 1 =============================================
Modo_Inactivo_Est1:

        Movw #Configurar_Est1,Est_Pres_TConfig
        Movw #EnServicio_Est1,Est_Pres_TServ

        Ldaa LEDS               ; Activa el LED correspondiente al modo de
        Anda #Mask_LD_Br_MD     ; operación inactivo.
        Oraa #LDInactivo
        Staa LEDS
                                ; Se coloca el Mensaje Modo Inactivo en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_Md_Inact_L1,Msg_L1
        Movw #MSG_Md_Inact_L2,Msg_L2

        Ldx  #Segment           ; Coloca los displays de 7 segmentos en estado
        Ldaa #OFF               ; OFF.
        Movb A,X,Dsp1
        Movb A,X,Dsp2
        Movb A,X,Dsp3
        Movb A,X,Dsp4

        Movw #Modo_Inactivo_Est2,Est_Pres_TInac

Modo_Inactivo_Est1_END:
        Rts

;==== Tarea_Modo_Inactivo ESTADO 2 =============================================
Modo_Inactivo_Est2:
                                ; Empieza por detectar si se recibió algún long
                                ; press en PH3 ó PH0. Hasta que esto no suceda
                                ; se mantiene esperando.
        Brset Banderas_1,LongP0,Modo_Inactivo_Est2_Presentar
        Brset Banderas_1,LongP1,Modo_Inactivo_Est2_Presentar

        Bra  Modo_Inactivo_Est2_END

Modo_Inactivo_Est2_Presentar:
        Ldaa Vel_LIM            ; Obtiene el valor actual de velocidad límite
        Jsr  BIN_BCD_MUXP       ; y lo coloca en los displays de 7 segmentos
        Movb BCD,BCD1           ; 3 y 4.
        Jsr  BCD_7SEG

        Ldx  #Segment           ; Coloca los displays de 7 segmentos en estado
        Ldaa #OFF               ; OFF.
        Movb A,X,Dsp1
        Movb A,X,Dsp2
       
        Movb #tTimerVLim,TimerPant

        Movw #Modo_Inactivo_Est3,Est_Pres_TInac

Modo_Inactivo_Est2_END:
        Rts

;==== Tarea_Modo_Inactivo ESTADO 3 =============================================
Modo_Inactivo_Est3:
        Tst  TimerPant          ; Espera a que termine el tiempo de 3 segundos,
                                ; luego retorna al estado 1.
        Bne  Modo_Inactivo_Est3_END

        Bclr Banderas_1,LongP0  ; Se resetean los valores de long press.
        Bclr Banderas_1,LongP1

        Movw #Modo_Inactivo_Est1,Est_Pres_TInac

Modo_Inactivo_Est3_END:
        Rts
;===============================================================================
;    II. Tarea_Modo_Inactivo. Final
;===============================================================================

;===============================================================================
;    III. Tarea_Configurar. Inicio
;===============================================================================
;       Permite cambiar el valor de la velocidad límite en tiempo de ejecución.
;===============================================================================
Tarea_Configurar:

        Ldaa Valor_DS           ; Obtiene el estado actual de los dipswitches.
        Anda #Mask_Mode_Sel     ; Deja sólo los valores de interés.

                                ; Compara la configuración de los dipsiwtches
                                ; con la de la tarea configurar, si no
                                ; corresponden, termina la subrutina. Caso
                                ; contrario la ejecuta.
        Cmpa #Mask_DS_MD_Cfg
        Bne  Fin_Tarea_Configurar

        Ldx  Est_Pres_TConfig
        Jsr  0,X
Fin_Tarea_Configurar:
        Rts

;==== Tarea_Configurar ESTADO 1 ================================================
Configurar_Est1:

        Movw #Modo_Inactivo_Est1,Est_Pres_TInac
        Movw #EnServicio_Est1,Est_Pres_TServ

        Ldaa LEDS               ; Activa el LED correspondiente al modo de
        Anda #Mask_LD_Br_MD     ; operación de configuración.
        Oraa #LDConfig
        Staa LEDS
                                ; Se coloca el Mensaje Configurar en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_Config_L1,Msg_L1
        Movw #MSG_Config_L2,Msg_L2

        Ldaa Vel_LIM            ; Obtiene el valor actual de velocidad límite
        Jsr  BIN_BCD_MUXP       ; y lo coloca en los displays de 7 segmentos
        Movb BCD,BCD1           ; 3 y 4.
        Jsr  BCD_7SEG

        Ldx  #Segment           ; Coloca los displays de 7 segmentos en estado
        Ldaa #OFF               ; OFF.
        Movb A,X,Dsp1
        Movb A,X,Dsp2

        Jsr  Borrar_NumArray    ; Borra el contenido del array de números.

                                ; Pasa al siguiente estado.
        Movw #Configurar_Est2,Est_Pres_TConfig

Configurar_Est1_END:
        Rts

;==== Tarea_Configurar ESTADO 2 ================================================
Configurar_Est2:
                                ; Espera a que se ingresen datos mediante el
                                ; teclado.
        Brclr Banderas_1,ARRAY_OK,Configurar_Est2_END

        Jsr  BCD_BIN            ; Convierte el valor leído a binario y lo salva
                                ; en Vel_LIM.

        Ldaa ValorLIM            ; Obtiene el valor actual de velocidad límite

        Cmpa #LimMin            ; Detecta valor inválido: número ingresado es
                                ; menor al límite mínimo, en cuyo caso se
                                ; descarta lo ingresado.
        Blo  Configurar_Est2_Reiniciar

        Cmpa #LimMax            ; Detecta valor inválido: número ingresado es
                                ; menor al límite mínimo, en cuyo caso se
                                ; descarta lo ingresado.
        Bhi  Configurar_Est2_Reiniciar
    
        ; De acá en adelante, la el segundo estado corresponde con el primero.
        Jsr  BIN_BCD_MUXP       ; y lo coloca en los displays de 7 segmentos
        Movb BCD,BCD1           ; 3 y 4.
        Jsr  BCD_7SEG

        Ldx  #Segment           ; Coloca los displays de 7 segmentos en estado
        Ldaa #OFF               ; OFF.
        Movb A,X,Dsp1
        Movb A,X,Dsp2

        Movb ValorLIM,Vel_LIM   ; Si valor ingresado es válido se mueve de la
                                ; variable temporal a la de uso.

Configurar_Est2_Reiniciar:
        Jsr  Borrar_NumArray    ; Borra el contenido del array de números.

Configurar_Est2_END:
        Rts
;===============================================================================
;    III.  Tarea_Configurar. Final
;===============================================================================

;===============================================================================
;    IV. Tarea_EnServicio. Inicio
;===============================================================================
;       Se encarga de la ejecución del Radar623, llamadas a subrutina Calcular
;   y coloca mensajes requeridos en pantalla.
;===============================================================================
Tarea_EnServicio:

        Ldaa Valor_DS           ; Obtiene el estado actual de los dipswitches.
        Anda #Mask_Mode_Sel     ; Deja sólo los valores de interés.

                                ; Compara la configuración de los dipsiwtches
                                ; con la de la tarea en servicio, si no
                                ; corresponden, termina la subrutina. Caso
                                ; contrario la ejecuta.
        Cmpa #Mask_DS_MD_Srv
        Bne  Fin_Tarea_EnServicio

        Ldx  Est_Pres_TServ
        Jsr  0,X
Fin_Tarea_EnServicio:
        Rts

;==== Tarea_EnServicio ESTADO 1 ================================================
EnServicio_Est1:

        Movw #Configurar_Est1,Est_Pres_TConfig
        Movw #Modo_Inactivo_Est1,Est_Pres_TInac

        Ldaa LEDS               ; Activa el LED correspondiente al modo de
        Anda #Mask_LD_Br_MD     ; operación de configuración.
        Oraa #LDEnServ
        Staa LEDS
                                ; Se coloca el Mensaje Inicial en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_Inicial_L1,Msg_L1
        Movw #MSG_Inicial_L2,Msg_L2

        Ldx  #Segment           ; Coloca los displays de 7 segmentos en estado
        Ldaa #OFF               ; OFF.
        Movb A,X,Dsp1
        Movb A,X,Dsp2
        Movb A,X,Dsp3
        Movb A,X,Dsp4

                                ; Pasa al estado 2.
        Movw #EnServicio_Est2,Est_Pres_TServ

EnServicio_Est1_END:
        Rts

;==== Tarea_EnServicio ESTADO 2 ================================================
EnServicio_Est2:
                                ; Espera a que se reciba un short press el botón
                                ; PH3, que representa el paso del vehículo por
                                ; el primer sensor.
        Brclr Banderas_1,ShortP1,EnServicio_Est2_END

                                ; Se coloca el Mensaje Calculando en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_Clcndo_L1,Msg_L1
        Movw #MSG_Clcndo_L2,Msg_L2

        Bclr Banderas_1,ShortP1 ; Borra la bandera de detección de PH3.

        Movb #tTimerVel,TimerVel; Inicia el contador para obtener DeltaT.

                                ; Pasa al estado 3.
        Movw #EnServicio_Est3,Est_Pres_TServ

EnServicio_Est2_END:
        Rts

;==== Tarea_EnServicio ESTADO 3 ================================================
EnServicio_Est3:
                                ; Espera a que se reciba un short press el botón
                                ; PH0, que representa el paso del vehículo por
                                ; el segundo sensor.
        Brclr Banderas_1,ShortP0,EnServicio_Est3_END

        Jsr  Calcula            ; Lee el timpo transcurrido entre el paso del
                                ; vehículo por los sensores. Con base en esto
                                ; calcula el la velocidad (en km/h) que trae,
                                ; además de los tiemps (en décimas de segundo)
                                ; del tiempo de activación y apagado de la
                                ; pantalla.

        Bclr Banderas_1,ShortP0 ; Borra la bandera de detección de PH0.

        Ldaa Vel_Calc           ; Carga el valor de la velocidad calculada
                                ; anteriormente.

                                ; Luego revisa si la velocidad es inválida, en
                                ; cuyo caso muestra el mensaje de error y pasa
                                ; al estado 5. Si no, pone el timer de
                                ; activación de la pantalla y pasa al estado 4.
        Cmpa #VelocMax
        Bhi  EnServicio_Est3_Vel_Inv

        Cmpa #VelocMin
        Blo  EnServicio_Est3_Vel_Inv

                                ; Si la velocidad es válida, pasa al estado 4.
        Movw #EnServicio_Est4,Est_Pres_TServ

        Bra  EnServicio_Est3_END

EnServicio_Est3_Vel_Inv:
                                ; Se coloca el Mensaje de Alerta en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_Alerta_L1,Msg_L1
        Movw #MSG_Alerta_L2,Msg_L2

        Ldx  #Segment           ; Coloca guiones en los displays de 7 segmentos
        Ldaa #GUIONES           ; Obtiene el índice los guiones a usar.
        Movb A,X,Dsp1
        Movb A,X,Dsp2
        Movb A,X,Dsp3
        Movb A,X,Dsp4
                                ; Inicia el temporizador de error.
        Movb #tTimerError,TimerError
                                ; Pasa al estado 5.
        Movw #EnServicio_Est5,Est_Pres_TServ

EnServicio_Est3_END:
        Rts

;==== Tarea_EnServicio ESTADO 4 ================================================
EnServicio_Est4:
        Tst  TimerPant          ; Espera a que el temporizador para encender
        Bne  EnServicio_Est4_END; la pantalla llegue a cero.

                                ; Coloca los valores de velocidad en los
                                ; displays de 7 segmentos según se requiere.
        Ldaa Vel_Calc           ; Obtiene el valor de la velocidad calculada
        Jsr  BIN_BCD_MUXP       ; y lo coloca en los displays de 7 segmentos
        Movb BCD,BCD1           ; 3 y 4.
        
        Ldaa Vel_LIM            ; Obtiene el valor actual de velocidad límite
        Jsr  BIN_BCD_MUXP       ; y lo coloca en los displays de 7 segmentos
        Movb BCD,BCD2           ; 1 y 2.
        Jsr  BCD_7SEG
       
                                ; Pasa al estado 6 independientemente de la
                                ; relación entre las velocidades.
        Movw #EnServicio_Est6,Est_Pres_TServ

        Ldaa Vel_Calc           ; Carga la velocidad calculada para compararla
                                ; con el límite de velocidad.
        Cmpa Vel_LIM            ; Si se supera el límite, se imprime el mensaje
                                ; de exceso de velocidad.
        Bhi  EnServicio_Est4_Exceso_Vel

                                ; Se coloca el Mensaje de En Servicio en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_EnServ_L1,Msg_L1
        Movw #MSG_EnServ_L2,Msg_L2

        Bra  EnServicio_Est4_END

EnServicio_Est4_Exceso_Vel:
                                ; Se coloca el Mensaje de Alarma en los
                                ; punteros de impresión para colocarlo en la
                                ; pantalla LCD.
        Bclr Banderas_2,LCD_OK
        Movw #MSG_Alarma_L1,Msg_L1
        Movw #MSG_Alarma_L2,Msg_L2

        Bset Banderas_2,Alarma  ; Activa la bandera de alarma.

EnServicio_Est4_END:
        Rts

;==== Tarea_EnServicio ESTADO 5 ================================================
EnServicio_Est5:
        Tst  TimerError         ; Espera a que termine el tiempo del temporizdor
        Bne  EnServicio_Est5_END; del mensaje de error.

                                ; Regresa al estado 1 para esperar el siguiente
                                ; vehículo.
        Movw #EnServicio_Est1,Est_Pres_TServ

EnServicio_Est5_END:
        Rts

;==== Tarea_EnServicio ESTADO 6 ================================================
EnServicio_Est6:
        Tst  TimerFinPant       ; Espera a que termine el tiempo del temporizdor
        Bne  EnServicio_Est6_END; del mensaje en la pantalla de velocidad.

        Bclr Banderas_2,Alarma  ; Desactiva la bandera de alarma.

                                ; Regresa al estado 1 para esperar el siguiente
                                ; vehículo.
        Movw #EnServicio_Est1,Est_Pres_TServ

EnServicio_Est6_END:
        Rts
;===============================================================================
;    IV. Tarea_EnServicio. Final
;===============================================================================

;===============================================================================
;    V.  Tarea_DsplzLeds. Inicio
;===============================================================================
;       Activa un led de manera secuencial entre entre puertoB.7 y puertoB.3
;    para simbolizar la luz de alarma cuando esta se activa.
;===============================================================================
Tarea_DsplzLeds:
        Ldx  Est_Pres_DsplzLeds
        Jsr  0,X
Fin_Tarea_DesplzLeds:
        Rts

;==== Tarea_DsplzLeds ESTADO 1 =================================================
DsplzLeds_Est1:
                                ; Si no se ha activado la alarma, no se hace
                                ; nada.
        Brclr Banderas_2,Alarma,DsplzLeds_Est1_END

                                ; Empieza el timer de desplazamiento de LEDS.
        Movb #tTimerDplzLeds,Timer_DplzLeds

        Ldaa LEDS               ; Obtiene el valor de actual en los leds y
        Anda #Mask_Dspz_LEDS    ; coloca el valor de leds a desplazar en la
        Oraa DplzLeds           ; parte alta.
        Staa LEDS

                                ; Pasa al estado 2.
        Movw #DsplzLeds_Est2,Est_Pres_DsplzLeds

DsplzLeds_Est1_END:
        Rts

;==== Tarea_DsplzLeds ESTADO 2 =================================================
DsplzLeds_Est2:
                                ; Espera a que termine el periodo de permanencia
                                ; de cada led antes de cambiarlo.
        Tst  Timer_DplzLeds
        Bne  DsplzLeds_Est2_END

                                ; Determina si se llegó al final de un semiciclo
                                ; de desplazamiento: salta a altermar la
                                ; la dirección cuando se tiene el led 7 y el led
                                ; 3 de la Dragon12.
        Brset LEDS,LED_Sup,DsplzLeds_Est2_Cambio_IaD
        Brset LEDS,LED_Inf,DsplzLeds_Est2_Cambio_DaI

                                ; En los casos intermedios se desplaza según la
                                ; dirección.
        Bra  DsplzLeds_Est2_Desplazar

DsplzLeds_Est2_Cambio_IaD:
                                ; Cambia la dirección de der a izq, a izq a der.
        Bclr Banderas_2,DspIzquierda
        Bra DsplzLeds_Est2_Desplazar

DsplzLeds_Est2_Cambio_DaI:
                                ; Cambia la dirección de izq a der, a der a izq.
        Bset Banderas_2,DspIzquierda

DsplzLeds_Est2_Desplazar:
                                ; El desplazamiento se realiza según la
                                ; dirección seleccionada en DspIzquierda.
        Brset Banderas_2,DspIzquierda,DsplzLeds_Est2_DesplazarIzq
        Lsr  DplzLeds           ; Desplaza a la derecha.
        Bra  DsplzLeds_Est2_Escribir_Cambios
    
DsplzLeds_Est2_DesplazarIzq:
        Lsl  DplzLeds           ; Desplaza a la izquierda.

DsplzLeds_Est2_Escribir_Cambios:
        Ldaa LEDS               ; Obtiene el valor de actual en los leds y
        Anda #Mask_Dspz_LEDS    ; coloca el valor de leds a desplazar en la
        Oraa DplzLeds           ; parte alta.
        Staa LEDS

                                ; Si se apagó la bandera de alarma, se termina
                                ; la subrutina y deja todo listo para volver a
                                ; ejecutarse.
        Brclr Banderas_2,Alarma,DsplzLeds_Est2_Terminar
        Movb #tTimerDplzLeds,Timer_DplzLeds

        Bra  DsplzLeds_Est2_END

DsplzLeds_Est2_Terminar:        
        Movb #LED_Sup,DplzLeds  ; Valor original de los leds a desplazar.
        Ldaa LEDS
        Anda #Mask_Dspz_LEDS    ; Borra los valores de DplzLeds de los leds.
        Staa LEDS
                                ; Selecciona la dirección original de
                                ; desplazamiento.
        Bclr Banderas_2,DspIzquierda

        Movw #DsplzLeds_Est1,Est_Pres_DsplzLeds

DsplzLeds_Est2_END:
        Rts
;===============================================================================
;    V.  Tarea_DsplzLeds. Final
;===============================================================================

;===============================================================================
;    VI. Tarea_Brillo. Inicio
;===============================================================================
;       Lee mediante el conversor analógico digital el valor actual del
;    potenciómetro del pad 7, promedia dos muestreos y pasa el valor leído al
;    rango de 0 a 100 que admite la variable Brillo.
;===============================================================================
Tarea_Brillo:
        Ldx  Est_Pres_TBrillo
        Jsr  0,X
Fin_Tarea_Brillo:
        Rts

;==== Tarea_Brillo ESTADO 1 ====================================================
Brillo_Est1:
                                ; Se inicia por colocar el valor de 4x100mS en
                                ; el contador correspondiente para esperar a
                                ; a que ocurra una conversión.
        Movb #tTimerBrillo,TimerBrillo

                                ; Inicia una conversión en el canal 7 del ATD.
        Movb #ATD_Read_CMD,ATD0CTL5
                                ; Selecciona el estado 2 como el siguiente.
        Movw #Brillo_Est2,Est_Pres_TBrillo

Brillo_Est1_END:
        Rts

;==== Tarea_Brillo ESTADO 2 ====================================================
Brillo_Est2:
        Tst  TimerBrillo        ; Se muestrea el valor actual del timer, si no
        Bne  Brillo_Est2_END    ; ha llegado a cero se mantiene esperando.

                                ; Inicia una conversión en el canal 7 del ATD.
        Movb #ATD_Read_CMD,ATD0CTL5
                                ; Selecciona el estado 2 como el siguiente.
        Movw #Brillo_Est3,Est_Pres_TBrillo
        

Brillo_Est2_END:
        Rts

;==== Tarea_Brillo ESTADO 3 ====================================================
Brillo_Est3:
                                ; Espera a que termine la conversión del ATD.
        Brclr ATD0STAT0,MaskSCF,Brillo_Est3_END
        
        Ldd  ADR00H             ; Lee y suma los valores de las dos
        Addd ADR01H             ; conversiones realizadas.
        Lsrd                    ; Divide entre 2 para obtener el promedio.

        Ldx  #Cte_Div_Vl_ATD    ; Carga las constantes de conversión para pasar
        Ldy  #Cte_Mul_Vl_ATD    ; de la magnitud leída por el ATD [0,255] al
                                ; rango que admite la variable Brillo.
                                ; Esto se logra mediante la progresión lineal
                                ; Brillo = V_atd*100/255
        Emul                    ; Multiplica D por 100 y deja el resultado en D

        Idiv                    ; Divide D/X y deja el resultado en X.

        XgdX                    ; Intercambia el valor final del brillo y lo
                                ; que se tenga en el índice X para salvar el
                                ; primero en memoria.

        Stab  Brillo            ; Salva el valor convertido en la variable de
                                ; destino.

        Movw #Brillo_Est1,Est_Pres_TBrillo

Brillo_Est3_END:
        Rts
;===============================================================================
;    VI. Tarea_Brillo. Final
;===============================================================================

;===============================================================================
;    VII. Tarea_Teclado. Inicio
;===============================================================================
;       Tarea que implementa la lectura del teclado mediante temporizadores de
; rechazo de rebotes y rechazo de presión constante de tecla.
;===============================================================================
Tarea_Teclado:
        Ldx  Est_Pres_TCL
        Jsr  0,X
Fin_Tarea_Teclado:
        Rts

;==== Teclado Estado 1 =========================================================
Teclado_Est1:
        Jsr  Leer_Teclado       ; Obtiene el valor actual del teclado.
        Ldaa Tecla
        Cmpa #Ninguna_Tecla     ; Si no se presionó una tecla, se mantiene
                                ; esperando a que se presione una.
        Beq  Teclado_Est1_END
        Staa Tecla_IN           ; Si el PC llega acá, se presionó una tecla,
                                ; se salva el valor en memoria para compararlo
                                ; después.
                                ; Luego, se inician los contadores para rechazar
                                ; rebotes mecánicos y se pasa al estado 2.
        Movb #tSupRebPB,Timer_RebTCL        
        Movw #Teclado_Est2,Est_Pres_TCL

Teclado_Est1_END:
        Rts

;==== Teclado Estado 2 =========================================================
Teclado_Est2:
        Tst  Timer_RebTCL       ; Se prueba el contador de tiempo de rechazo de
                                ; rebotes para determinar si fue un falso
                                ; positivo, si el tiempo no ha pasado, se
                                ; se mantiene esperando.
        Bne  Teclado_Est2_END
        Jsr  Leer_Teclado       ; Se vuelve a leer el teclado.
        Ldaa Tecla              ; Obtiene el valor de la tecla actual.
        Cmpa Tecla_IN           ; Se compara la tecla actual con la que se
                                ; detectó antes para determinar si fue un error
                                ; o no.
        Bne  TE2_Falso_Positivo ; Si las teclas detectadas son diferentes,
                                ; se tiene un falso positivo o a alguien con un
                                ; generador de seÃ±ales muy bueno.
                                ; Si, por otro lado, se tiene una técla válida,
                                ; se pasa al siguiente estado.
        Movw #Teclado_Est3,Est_Pres_TCL
        Bra  Teclado_Est2_END

TE2_Falso_Positivo:             ; Teclado estado 2 falso positivo
        Movw #Teclado_Est1,Est_Pres_TCL

Teclado_Est2_END:
        Rts

;==== Teclado Estado 3 =========================================================
Teclado_Est3:
        Jsr  Leer_Teclado       ; Vuelve a leer estado del teclado.
        Ldaa Tecla              ; Obtiene el valor actual de las teclas
                                ; presionadas. Eso sucede después de determinar
                                ; que la tecla ingresada es válida.
        Cmpa #Ninguna_Tecla     ; Si se lee $FF, se soltó la tecla y se sigue
        Beq  TE3_Tecla_Suelta   ; con el estado 4, caso contrario se mantiene
                                ; esperando.
        Bra  Teclado_Est3_END

TE3_Tecla_Suelta:
        Movw #Teclado_Est4,Est_Pres_TCL

Teclado_Est3_END:
        Rts

;==== Teclado Estado 4 =========================================================
Teclado_Est4:
        Ldaa MAX_TCL            ; Se cargan las contantes de trabajo para
        Ldab Tecla_IN           ; comparar los resultados obtenidos.

        Cmpa  Cont_TCL          ; Se empieza por determinar si
        Beq   TE4_Array_Lleno   ; Si se llenó el array, se pasa a la sección de
                                ; atención correspondiente.
        Tst  Cont_TCL           ; Luego, se verifica si no se ha ingresado
        Beq  TE4_Primera_Tecla  ; números al array.

        Cmpb #Tecla_BORRAR      ; Si hay elementos en Num_Array, este no está
        Beq  TE4_Verif_Borrar   ; lleno y se ingresa la tecla BORRAR, se pasa
                                ; a borrar verificar si borrar o no el elemento.

        Cmpb #Tecla_ENTER       ; Si la tecla ingresada es ENTER, hay más de uno
        Beq  TE4_OP_Enter       ; y menos de MAX_TCL elementos en el array, se
                                ; procede a terminar la subrutina con enter.
       
        Bra  TE4_Ingresar_Valor ; Si no sucedió ninguno de los casos, se agrega
                                ; el elemento al array. 

TE4_Verif_Borrar:               ; Tecaldo estado 4, verificación de borrado.
        Tst  Cont_TCL           ; Determina si hay cero elementos en el array,
        Beq  TE4_Terminar       ; en cuyo caso termina, si no, pasa a borrar
        Bra  TE4_OP_Borrar      ; el último elemento ingresado.

TE4_Primera_Tecla:              ; Tecaldo estado 4, array vacío
        Cmpb #Tecla_BORRAR      ; Se deterca si se ingresó la tecla BORRAR.
        Beq  TE4_Terminar; Si se ingresó BORRAR, se pasa a verificar
                                ; si se tiene que 
        Cmpb #Tecla_ENTER       ; Se detecta si se ingresó ENTER.
        Beq  TE4_Terminar       ; Igualmente, si el array está vacío y se
                                ; ingresa ENTER, se ignora.
        Bra  TE4_Ingresar_Valor ; Caso contrario, se ingresa el valor al array.

TE4_Ingresar_Valor:
        Ldx  #Num_Array        ; Obtiene la dirección de inicio de Tecaldo
                                ; estado 4, luego, se salva el valor leído
        Ldaa Cont_TCL           ; en la posición
        Stab A,X
        Inc  Cont_TCL           ; más el offset, y se incrementa el contador.
        Bra  TE4_Terminar       ; Se pasa a terminar la subrutina.

TE4_Array_Lleno:                ; NUM_Array condición de array lleno.
        Cmpb #Tecla_BORRAR      ; Detecta si la tecla ingresada es la de borrar.
        Bne TE4_Verif_ENTER

TE4_OP_Borrar:                 ; NUM_Array, operación borrar.
        Ldx  #Num_Array
        Dec  Cont_TCL           ; Se decrementa en uno la cantidad de elementos
                                ; en la tabla.
        Ldaa Cont_TCL
        Movb #Ninguna_Tecla,A,X ; Se borra la posición actual de la tabla al
                                ; establecer la entrada como $FF.
        Bra  TE4_Terminar       ; Pasa a terminar la subrutina.

TE4_Verif_ENTER:                ; NUM_Array, verificar si se ingresó ENTER.
        Cmpb #Tecla_ENTER       ; En caso de que no se ingresara ENTER, se
        Bne  TE4_Terminar       ; termina la subrutina al limpiar las teclas
                                ; leídas,de modo que se ignoran estas.
TE4_OP_Enter:                   ; NUM_Array, operación enter.
        Clr  Cont_TCL           ; Caso contrario, se activa la bandera de
                                ; ARRAY_OK, se borra la cantidad de teclas
                                ; ingresadas y se setea el estado siguiente
                                ; como el número 1. Luego, se pasa a terminar.
        Bset Banderas_1,ARRAY_OK
        Bra  TE4_Terminar

TE4_Terminar:
                                ; Se reinician las variables de trabajo a sus
                                ; valores por defecto.
        Movb #Ninguna_Tecla,Tecla
        Movb #Ninguna_Tecla,Tecla_IN
        Movw #Teclado_Est1,Est_Pres_TCL

Teclado_Est4_END:
        Rts

;*******************************************************************************
;    Subrutina Leer_Teclado. Inicio
;*******************************************************************************
;       Subrutina que se encarga de leer el teclado matricial de manera
; multiplexada.
;*******************************************************************************
Leer_Teclado:
        Clra                    ; Limpia las varables a utilizar.
        Clrb
        Ldx  #Teclas            ; Carga las constantes con las que se trabajará
        Movb #$EF,PATRON
        Movb #$FF,Tecla         ; Se asume que no se presionó una tecla, y se
                                ; cambiará en caso contrario.
        
LT_Ciclo:                       ; Ciclo de subrutina Leer_Teclado.
        Clr  PortTCL
        Movb PATRON,PortTCL     ; Escribe el patrón de lectura en el teclado.
        Brclr PATRON,$F0,LT_FIN; Si se llega al final de las filas de teclas
                                ; sin detectar pulsaciones, se termina la
                                ; subrutina.
        Ldaa PortTCL            ; Obtiene el estado actual del teclado.

LT_IF_Tecla_Rec:                ; If de tecla recivida de Leet_Teclado
                                ; Si ninguna tecla se presionó, se pasa al
                                ; siguiente ciclo o se termina la subrutina.
                                ; Si no se presionó ninguna tecla, se salta
                                ; la columna actual.
        Brset PortTCL,$0F,LT_Rev_Siguiente
LT_IF_Tc_1:                     ; IF para detectar si se presionó la primera
                                ; tecla del array.
        Brset PortTCL,1,LT_IF_Tc_2
        Bra  LT_Tecla_Presionada; No se aumenta el offset de la tabla Teclas.

LT_IF_Tc_2:                     ; IF para detectar si se presionó la segunda
                                ; tecla del array.

        Brset PortTCL,2,LT_IF_Tc_3
        Ldab #1                    ; Selecciona la tecla correspondiente (+1)
        Bra  LT_Tecla_Presionada; según se requiera en el offset de Teclas.


LT_IF_Tc_3:                     ; IF para detectar si se presionó la tercera
                                ; tecla del array.

        Brset PortTCL,4,LT_Rev_Siguiente
        Ldab #2                 ; Selecciona la tecla correspondiente (+2)
        Bra  LT_Tecla_Presionada; según se requiera en el offset de Teclas.


LT_Rev_Siguiente:               ; Revisión de siguiente entrada de la tabla
        Inx                     ; Aumenta en 3 las teclas posibles a detectar
        Inx
        Inx
        Clrb
        Rol  PATRON             ; Obtiene el siguiente patrón para escribir
                                ; en el pueto
        Bra  LT_Ciclo           ; Retorna al inicio del ciclo

LT_Tecla_Presionada:            ; Tecla presionada en Leer_Teclado
        Movb  B,X,Tecla
LT_FIN:                         ; Fin de Leer_Teclado
        Rts                     ; Retorna la subrutina
;*******************************************************************************
;    Subrutina Leer_Teclado. Final
;*******************************************************************************
;===============================================================================
;    VII. Tarea_Teclado. Final
;===============================================================================

;==============================================================================
;    VIII. Tarea_Led_Testigo. Inicio
;==============================================================================
Tarea_Led_Testigo:
        Tst Timer_LED_Testigo
        Bne FinLedTest
        Movb #tTimerLDTst,Timer_LED_Testigo
        Ldaa LED_Testigo
        Anda #$F0
        Cmpa #$40
        Beq  TLT_Set_Green
        Cmpa #$20
        Beq  TLT_Set_Blue
        Bset LED_Testigo,$40    ; Setea el led RGB en rojo
        Bclr LED_Testigo,$30
        Bra  TLT_Terminar

TLT_Set_Green:                  ; Tarea_Led_Testigo Setear en verde
        Bset LED_Testigo,$20    ; Setea el led RGB en verde
        Bclr LED_Testigo,$50
        Bra  TLT_Terminar

TLT_Set_Blue:                   ; Tarea_Led_Testigo Setear en azul
        Bset LED_Testigo,$10    ; Setea el led RGB en azul
        Bclr LED_Testigo,$60
        Bra  TLT_Terminar

TLT_Terminar:
        Ldaa PTP                ; Coloca el valor deseado del led testigo
                                ; en su puerto respectivo.
        Anda #$0F               ; Elimina el valor anterior y preserva el.
                                ; estado de los displays de 7 segmentos.
        Oraa LED_Testigo        ; Salva el valor elegido.
        Staa PTP                ; Escribe el resultado en el puerto.

FinLedTest:
        Rts
;===============================================================================
;    VIII. Tarea_Led_Testigo. Final
;===============================================================================

;===============================================================================
;    IX. Tarea_Leer_PB0 y Tarea_Leer_PB1. Inicio
;===============================================================================
;    Leer_PB0
;       Método para implementar la máquina de estados Leer_PB para el botón PH0.
;
;       Se inicializa la variable de estado Est_Pres_LeerPB0 en el programa
;    principal.
;       Luego, en cada estado se actualiza la variable de estado con la
;    dirección del próximo estado.
;===============================================================================

Tarea_Leer_PB0:
        Ldx  Est_Pres_LeerPB0
        Jsr  0,X
Fin_Tarea_Leer_PB0:
        Rts

;==== LEER PB ESTADO 1 =========================================================
LeerPB0_Est1:
        ; El PB está en el bit 0 del puerto H
        ; Si no se ha presionado el botón, se sigue en el estado 1
        Brset PortPB,MaskPB0,LeerPB0_Est1_END
        Movb #tSupRebPB,Timer_RebPB0    ; En caso de que se detecte que se
        Movb #tShortP,Timer_SHP0        ; presionó el botón, se inician los
        Movb #tLongP,Timer_LP0          ; contadores respectivos.

        Movw #LeerPB0_Est2,Est_Pres_LeerPB0       ; Para ir al estado 2

LeerPB0_Est1_END:
        Rts

;==== LEER PB ESTADO 2 =========================================================
LeerPB0_Est2:
        Tst  Timer_RebPB0       ; Determina si se pasó el tiempo de espera para
        Bne  LeerPB0_Est2_END    ; rechazar el rebote.

        Brset PortPB,MaskPB0,LeerPB0_Est2_FP     ; Si se soltó el botó era un
                                                ; falso positivo y se reinicia.
        ; El botón siguió presionado después de Timer_RebPF.
        Movw #LeerPB0_Est3,Est_Pres_LeerPB0      ; Siguiente estado = 3ero.
        Bra  LeerPB0_Est2_END                    ; Termina la subrutina.
LeerPB0_Est2_FP:
        Movw #LeerPB0_Est1,Est_Pres_LeerPB0     ; Siguiente estado = 1ero.
LeerPB0_Est2_END:
        Rts

;==== LEER PB ESTADO 3 =========================================================
LeerPB0_Est3:
;        Bset PORTB,$01 ; DEBUG
        Tst  Timer_SHP0         ; Detecta si pasó o no el tiempo de short press.
        Bne  LeerPB0_Est3_END    ; Si no, se mantiene esperando.

        Brset PortPB,MaskPB0,LeerPB0_Est3_SP_Listo       ; Si se soltó el botón
                                                        ; luego del periodo
                                                        ; Timer_SHP0, se termina
                                                        ; acá la máquina.
        Movw #LeerPB0_Est4,Est_Pres_LeerPB0       ; Siguiente estado = 4to.
        Bra  LeerPB0_Est3_END    ; Termina la subrutina.

LeerPB0_Est3_SP_Listo:
        Bset Banderas_1,ShortP0 ; Si se detectó un press válido antes de
                                ; Timer_LP0, se lee un short press.
        Movw #LeerPB0_Est1,Est_Pres_LeerPB0       ; Siguiente estado = 1ero.

LeerPB0_Est3_END:
        Rts

;==== LEER PB ESTADO 4 =========================================================
LeerPB0_Est4:
        Tst  Timer_LP0
        Bne  LeerPB0_Est4_SP_Verif
        Brclr PortPB,MaskPB0,LeerPB0_Est4_END
        Bset Banderas_1,LongP0  ; Si se detectó un press válido después de
                                ; Timer_LP0, se lee un long press.
        Movw #LeerPB0_Est1,Est_Pres_LeerPB0       ; Siguiente estado = 1ero
        Bra  LeerPB0_Est4_END

LeerPB0_Est4_SP_Verif:
        Brclr PortPB,MaskPB0,LeerPB0_Est4_END
        Bset Banderas_1,ShortP0 ; Si se detectó un press válido antes de
                                ; Timer_LP0, se lee un short press.
        Movw #LeerPB0_Est1,Est_Pres_LeerPB0       ; Siguiente estado = 1ero

LeerPB0_Est4_END:
        Rts

;===============================================================================
;    Leer_PB1
;       Método para implementar la máquina de estados Leer_PB para el botón PH3
;
;       Se inicializa la variable de estado Est_Pres_LeerPB1 en el programa
;    principal.
;       Luego, en cada estado se actualiza la variable de estado con la
;    dirección del próximo estado.
;===============================================================================
Tarea_Leer_PB1:
        Ldx  Est_Pres_LeerPB1
        Jsr  0,X
Fin_Tarea_Leer_PB1:
        Rts

;==== LEER PB1 ESTADO 1 ========================================================
LeerPB1_Est1:
        ; El PB está en el bit 0 del puerto H
        ; Si no se ha presionado el botón, se sigue en el estado 1
        Brset PortPB,MaskPB1,LeerPB1_Est1_END
        Movb #tSupRebPB,Timer_RebPB1    ; En caso de que se detecte que se
        Movb #tShortP,Timer_SHP1        ; presionó el botón, se inician los
        Movb #tLongP,Timer_LP1          ; contadores respectivos.

        Movw #LeerPB1_Est2,Est_Pres_LeerPB1      ; Para ir al estado 2

LeerPB1_Est1_END:
        Rts

;==== LEER PB1 ESTADO 2 ========================================================
LeerPB1_Est2:
        Tst  Timer_RebPB1        ; Determina si se pasó el tiempo de espera para
        Bne  LeerPB1_Est2_END    ; rechazar el rebote.

        Brset PortPB,MaskPB1,LeerPB1_Est2_FP    ; Si se soltó el botó era un
                                                ; falso positivo y se reinicia.
        ; El botón siguió presionado después de Timer_RebPF.
        Movw #LeerPB1_Est3,Est_Pres_LeerPB1     ; Siguiente estado = 3ero.
        Bra  LeerPB1_Est2_END                   ; Termina la subrutina.
LeerPB1_Est2_FP:
        Movw #LeerPB1_Est1,Est_Pres_LeerPB1     ; Siguiente estado = 1ero.
LeerPB1_Est2_END:
        Rts

;==== LEER PB1 ESTADO 3 ========================================================
LeerPB1_Est3:
;        Bset PORTB,$01 ; DEBUG
        Tst  Timer_SHP1          ; Detecta si pasó o no el tiempo de short press.
        Bne  LeerPB1_Est3_END    ; Si no, se mantiene esperando.

        Brset PortPB,MaskPB1,LeerPB1_Est3_SP_Listo        ; Si se soltó el botón
                                                        ; luego del periodo
                                                        ; Timer_SHP1, se termina
                                                        ; acá la máquina.
        Movw #LeerPB1_Est4,Est_Pres_LeerPB1      ; Siguiente estado = 4to.
        Bra  LeerPB1_Est3_END    ; Termina la subrutina.

LeerPB1_Est3_SP_Listo:
        Bset Banderas_1,ShortP1 ; Si se detectó un press válido antes de
                                ; Timer_LP1, se lee un short press.
        Movw #LeerPB1_Est1,Est_Pres_LeerPB1      ; Siguiente estado = 1ero.

LeerPB1_Est3_END:
        Rts

;==== LEER PB1 ESTADO 4 ========================================================
LeerPB1_Est4:
        Tst  Timer_LP1
        Bne  LeerPB1_Est4_SP_Verif
        Brclr PortPB,MaskPB1,LeerPB1_Est4_END
        Bset Banderas_1,LongP1  ; Si se detectó un press válido después de
                                ; Timer_LP1, se lee un long press.
        Movw #LeerPB1_Est1,Est_Pres_LeerPB1      ; Siguiente estado = 1ero
        Bra  LeerPB1_Est4_END

LeerPB1_Est4_SP_Verif:
        Brclr PortPB,MaskPB1,LeerPB1_Est4_END
        Bset Banderas_1,ShortP1 ; Si se detectó un press válido antes de
                                ; Timer_LP1, se lee un short press.
        Movw #LeerPB1_Est1,Est_Pres_LeerPB1      ; Siguiente estado = 1ero

LeerPB1_Est4_END:
        Rts
;===============================================================================
;    IX. Tarea_Leer_PB0 y Tarea_Leer_PB1. Final
;===============================================================================

;===============================================================================
;    X. Tarea_PantallaMUX. Inicio
;===============================================================================
;       Tarea que coloca los dígitos de DISP1 a DISP4 en los displays de 7
;    segmentos.
;===============================================================================
Tarea_PantallaMUX:
        Ldx  EstPres_PantallaMUX
        Jsr  0,X
Fin_Tarea_PantallaMUX:
        Rts

;==== PantallaMUX Estado 1 =====================================================
PantallaMUX_Est1:
        Tst  TimerDigito        ; Detecta si ya se terminó un periodo de un
                                ; dígito. Si no, se mantiene esperando.
        Bne  PantallaMUX_Est1_END
                                ; Una vez se termina el contador, se reinicia
        Movb #tTimerDigito,TimerDigito
        Bclr PTP,$0F            ; Desactiva los displays de 7 segmentos
        Bset PTJ,$02            ; Contecta el cátodo de los leds para usarlos.
        Ldaa Cont_Dig
        Cmpa #1                 ; Se implementa un switch-case para determinar
        Beq  PM_Dig_1           ; cuál dígito encender. Si ningún caso es válido
        Cmpa #2                 ; se pasa a encender los LEDS.
        Beq  PM_Dig_2           ; Nota sobre la asignación de los dígitos:
        Cmpa #3                 ; se cuentan de izquierda a derecha, de modo
        Beq  PM_Dig_3           ; que los dígitos físicos se cuentan en el orden
        Cmpa #4                 ; que se lee en la tarjeta.
        Beq  PM_Dig_4
                                ; Si no salta, significa que tiene que encender
                                ; los LEDs.
        Bset PTP,$0F            ; Desactiva los displays de 7 segmentos
        Bclr PTJ,$02            ; Contecta el cátodo de los leds para usarlos.
        Movb LEDS,PORTB         ; Mueve el dato de memoria al puerto leds/disp7s
        Movb #1,Cont_Dig        ; Reinicia el contador de dígitos
        Bra  PantallaMUX_Est1_Terminar
PantallaMUX_Est1_END:
        Rts

PM_Dig_1:                       ; PantallaMUX Dígito 1
        Bset PTP,$07            ; Activa el display Disp1
        Movb Dsp3,PORTB         ; Mueve el dato de memoria al puerto leds/disp7s
        Inc Cont_Dig            ; Selecciona el siguiente dígito para el
                                ; siguiente estado

        Bra  PantallaMUX_Est1_Terminar
PM_Dig_2:                       ; PantallaMUX Dígito 2
        Bset PTP,$0B            ; Activa el display Disp2
        Movb Dsp4,PORTB         ; Mueve el dato de memoria al puerto leds/disp7s
        Inc Cont_Dig            ; Selecciona el siguiente dígito para el
                                ; siguiente estado

        Bra  PantallaMUX_Est1_Terminar
PM_Dig_3:                       ; PantallaMUX Dígito 3
        Bset PTP,$0D            ; Activa el display Disp3
        Movb Dsp1,PORTB         ; Mueve el dato de memoria al puerto leds/disp7s
        Inc Cont_Dig            ; Selecciona el siguiente dígito para el
                                ; siguiente estado

        Bra  PantallaMUX_Est1_Terminar
PM_Dig_4:                       ; PantallaMUX Dígito 4
        Bset PTP,$0E            ; Activa el display Disp4
        Movb Dsp2,PORTB         ; Mueve el dato de memoria al puerto leds/disp7s
        Inc Cont_Dig            ; Selecciona el siguiente dígito para el
                                ; siguiente estado


PantallaMUX_Est1_Terminar:      ; Terminar el estado 1 para pasar al dos
                                ; Inicia la cuenta regresiva de la cantidad
                                ; máxima de ticks para tener activados los LEDS
        Ldd  #0
        Ldab Brillo
        Std  CounterTicks
        Movw #PantallaMUX_Est2,EstPres_PantallaMUX
        Bra PantallaMUX_Est1_END

;==== PantallaMUX Estado 2 =====================================================
PantallaMUX_Est2:
        Ldd  CounterTicks       ; Espera a que pase el tiempo de
                                ; activación de los displays para
                                ; apagarlos.
        Bne  PantallaMUX_Est2_END
        Bset PTP,$0F            ; Desactiva los displays de 7 segmentos
        Bset PTJ,$02            ; Desconecta el cátodo de los LEDs para que no
                                ; se enciendan.
                                ; Coloca el puntero de estado actual para pasar
                                ; al primero. 
;        Movb #0,PORTB
        Movw #PantallaMUX_Est1,EstPres_PantallaMUX
        
PantallaMUX_Est2_END:
        Rts
;===============================================================================
;    X. Tarea_PantallaMUX. Final
;===============================================================================

;===============================================================================
;    XI. Tarea_LCD y Send_LCD. Inicio
;===============================================================================
;       Tarea_LCD
;
;       Envía datos a la pantalla LCD
;===============================================================================
Tarea_LCD:
        Ldx  EstPres_TareaLCD
        Jsr  0,X
Fin_Tarea_LCD:
        Rts

;==== Tarea LCD Estado 1 =======================================================
Tarea_LCD_Est1:
                                ; Se borran las banderas de modo que se pueda
                                ; enviar un comando y esperar a que termine.
        Bclr Banderas_2,FinSendLCD
        Bclr Banderas_2,RS

        Brclr Banderas_2,Second_Line,Tarea_LCD_Est1_FirstLine

        Movb #ADD_L2,CharLCD    ; Selecciona el comando para colocar el puntero
                                ; al inicio de la segunda línea.
        Movw Msg_L2,Punt_LCD    ; Coloca el puntero al mensaje deseado.
        Bra  Tarea_LCD_Est1_Enviar

Tarea_LCD_Est1_FirstLine:       ; Tarea LCD estado 1, enviar comando 1era Lin.
        Movb #ADD_L1,CharLCD    ; Selecciona el comando para colocar el puntero
                                ; al inicio de la primera línea.
        Movw Msg_L1,Punt_LCD    ; Coloca el puntero al mensaje deseado.

Tarea_LCD_Est1_Enviar:

        Jsr  Tarea_SendLCD      ; Pasa a enviar el mensaje y selecciona como
                                ; siguiente el estado 4.
        Movw #Tarea_LCD_Est2,EstPres_TareaLCD

Tarea_LCD_Est1_END:
        Rts

;==== Tarea LCD Estado 2 =======================================================
Tarea_LCD_Est2:
                                ; Revisa si se terminó de enviar el mensaje,
                                ; si no, se mantiene esperando, cuando
                                ; eventualmente termina, borra la bandera,
                                ; Setea RS para enviar un dato y continua
                                ; con la tira de caracteres.
        Brclr Banderas_2,FinSendLCD,Tarea_LCD_Est2_Mantenerse

        Bclr Banderas_2,FinSendLCD
        Bset Banderas_2,RS

        Ldx  Punt_LCD           ; Obtiene el valor actual a enviar al cargar
        Ldaa 1,X+               ; el puntero y leerlo mediante direccionamiento
        Staa CharLCD            ; indexado de postincremento en X, así deja
        Stx  Punt_LCD           ; seleccionada la siguiente entrada de la
                                ; tira de caracteres para cuando vuelva.
                                ; Luego salva el caracter a enviar en el
                                ; registro que lee la tarea SendLCD y el
                                ; puntero actual en memoria.

        Cmpa  #EOB              ; Condición de salida.
        Beq   Tarea_LCD_Est2_Sig; Si se llega al final del mensaje se pasa al
                                ; siguiente o se termina dependiendo del
                                ; contexto.

Tarea_LCD_Est2_Mantenerse:      ; Tarea LCD Estado 2, mantenerse esperando.
        Jsr  Tarea_SendLCD      ; Se mantiene enviando el mensaje.
        Bra  Tarea_LCD_Est2_END

Tarea_LCD_Est2_Sig:             ; Tarea LCD Estado 2 Siguiente mensaje
        Brclr Banderas_2,Second_Line,Tarea_LCD_Est2_Send_L2
        Bclr Banderas_2,Second_Line
        Bset Banderas_2,LCD_Ok
        Bra  Tarea_LCD_Est2_Sig_Terminar

Tarea_LCD_Est2_Send_L2:         ; Tarea LCD, estado 2 enviar línea 2
        Bset Banderas_2,Second_Line

Tarea_LCD_Est2_Sig_Terminar:    ; Tarea LCD, estado 2 terminar envío de
                                ; mensaje siguiente retornando al estado 1.
        Movw #Tarea_LCD_Est1,EstPres_TareaLCD

Tarea_LCD_Est2_END:
        Rts

;===============================================================================
;       Tarea SendLCD
;
;       Implementa la comunicación estroboscópica para enviar caracteres y
;     comandos a la LCD.
;===============================================================================
Tarea_SendLCD:
        Ldx  EstPres_SendLCD
        Jsr  0,X
Fin_Tarea_SendLCD:
        Rts
;==== Tarea SendLCD Estado 1 ===================================================
SendLCD_Est1:
        Ldaa CharLCD           ; Obtiene el mensaje a enviar
        Anda #$F0               ; Elimina la parte baja, se tiene A=$X0
        Lsra                    ; Desplaza hacia la izquierda para que A
        Lsra                    ; corresponda con el formato necesario,
                                ; siendo este A=%00XXXX00
        Staa PortK              ; Pasa el valor al puerto

SndLCD_E1_IF_CMD:               ; SendLCD if para determinar si se envia cmd
                                ; Si no es un comando, se pasa a cambiar RS
                                ; como se necesite
        Brset Banderas_2,RS,SendLCD_E1_No_Cmd
        Bclr PORTK,RS           ; Cuando se tiene un comando, se setea RS en 0
        Bra  SndLCD_E1_IF_CMD_Fin
SendLCD_E1_No_Cmd:
        Bset PORTK,RS           ; Cuando no se tiene un comando, se setea RS=1
SndLCD_E1_IF_CMD_Fin:           ; Fin de SndLCD_E1_IF_CMD.

        Bset PORTK,$02          ; Activa la línea Enable de la LCD y pasa a
                                ; esperar que el tiempo requerido.

                                ; Inicia el temporizador.
        Movw #tTimer260uS,Timer260uS
                                ; Selecciona el estado 2 como el siguiente.
        Movw #SendLCD_Est2,EstPres_SendLCD

SendLCD_Est1_END:
        Rts                     ; Retorna

;==== Tarea SendLCD Estado 2 ===================================================
SendLCD_Est2:
        Ldx  Timer260uS         ; Carga el valor del contador de 260 uS,
        Bne  SendLCD_Est2_END   ; determina si terminó y se mantiene esperando
                                ; en caso de que no.
        Bclr PORTK,$02          ; Baja la señal de enable de la pantalla.

                                ; Repite el proceso de envío de dato pero con
                                ; la parte baja de CharLCD
        Ldaa CharLCD            ; Obtiene el mensaje a enviar
        Anda #$0F               ; Elimina la parte alta, se tiene A=$0X
        Lsla                    ; Desplaza hacia la derecha para que A
        Lsla                    ; corresponda con el formato necesario,
                                ; siendo este A=%00XXXX00
        Staa PortK              ; Pasa el valor al puerto

SndLCD_E2_IF_CMD:               ; SendLCD if para determinar si se envia cmd
                                ; Si no es un comando, se pasa a cambiar RS
                                ; como se necesite
        Brclr Banderas_2,RS,SendLCD_E2_Cmd
        Bset PORTK,RS           ; Cuando no se tiene un comando, se setea RS=1
        Bra  SndLCD_E2_IF_CMD_Fin
SendLCD_E2_Cmd:
        Bclr PORTK,RS           ; Cuando se tiene un comando, se setea RS en 0
SndLCD_E2_IF_CMD_Fin:           ; Fin de SndLCD_E2_IF_CMD.

        Bset PORTK,$02          ; Activa la línea Enable de la LCD y pasa a
                                ; esperar que el tiempo requerido.
                                ; Inicia el temporizador.
        Movw #tTimer260uS,Timer260uS
                                ; Selecciona el estado 2 como el siguiente.
        Movw #SendLCD_Est3,EstPres_SendLCD

SendLCD_Est2_END:
        Rts

;==== Tarea SendLCD Estado 3 ===================================================
SendLCD_Est3:
        Ldx  Timer260uS         ; Carga el valor del contador de 260 uS,
        Bne  SendLCD_Est3_END   ; determina si terminó y se mantiene esperando

        Bclr PORTK,$02          ; Baja la señal de enable de la pantalla.
                                ; Inicia el temporizador de 40uS.
        Movw #tTimer40uS,Timer40uS
        
        Movw #SendLCD_Est4,EstPres_SendLCD

SendLCD_Est3_END:
        Rts

;==== Tarea SendLCD Estado 4 ===================================================
SendLCD_Est4:
        Ldx  Timer40uS          ; Carga el valor del contador de 260 uS,
        Bne  SendLCD_Est4_END   ; determina si terminó y se mantiene esperando
                                ; Activa la bandera de fin del envío.
        Bset Banderas_2,FinSendLCD
                                ; Pasa al estado 1.
        Movw #SendLCD_Est1,EstPres_SendLCD

SendLCD_Est4_END:
        Rts

;===============================================================================
;    XI. Tarea_LCD y Send_LCD. Final
;===============================================================================

;*******************************************************************************
;    XII. Subrutina BCD_BIN. Inicio
;*******************************************************************************
;       Convierte los valores de Num_Array (que se salvan en BCD) a binario 
;    para la tarea configurar.
;*******************************************************************************
BCD_BIN:
        Ldx  #Num_Array         ; Obtiene la dirección del arreglo de datos
        Ldd  #0                 ; ingresados y limpia D.

        Ldab 0,X                ; Obtiene el valor de las decenas en B y carga
        Ldaa #10                ; la constante de multiplicación en A.
        Mul                     ; Multiplica los valores y los aloja en B.

        Addb 1,X                ; Suma las unidades, que en BCD siempre
                                ; corresponden con binario, a las decenas.

        Stab ValorLIM            ; Salva en valor donde se requiere en memoria.

        Rts
;*******************************************************************************
;    XII. Subrutina BCD_BIN. Inicio
;*******************************************************************************

;*******************************************************************************
;    XIII. Subrutina Calcula. Inicio
;*******************************************************************************
;       Partiendo de la lectura de TimerVel obtiene los valores de DeltaT,
;    Vel_Calc (km/h), TimerPant (base 100mS) y TimerFinPant (base 100mS).
;       Se detallan las ecuaciones utilizadas para cada variable:
;       - DeltaT:
;               El tiempo transcurrido, el que le toma al vehículo pasar por
;           ambos sensores, está dado por la resta de el tiempo máximo y el
;           tiempo restante. El valor obtenido ya está en decimas de segundo.
;               DeltaT = tTimerVel - TimerVel
;       - Vel_Calc:
;               La velocidad calculada se obtiene de dividir la distancia
;           recorrida (40 metros) entre el tiempo transcurrido y multiplicarla
;           por las constantes de conversión para pasar de m/s a km/h. También
;           se requiere pasar DeltaT de décimas de segundo (dS) a segundos.
;               Vel_Cacl = (40m/DeltaT)*(1/(1s/10dS))*(1km/1000m)*(3600s/1h)
;               Vel_Calc = 1440/DeltaT
;       - TimerPant:
;               El tiempo para activar la pantalla es aquel en que el vehículo
;           recorre 200m luego de pasar por el segundo sensor. Vel_Calc está en
;           km/h.
;               TimerPant = 200m/Vel_Calc
;               TimerPant = (200m/Vel_Calc)*(1/(1000m/km)*(1h/3600s))*(10dS/1s)
;               TimerPant = 7200/Vel_Calc
;       - TimerFinPant:
;               El ¿alculo del tiempo para apagar la pantalla es análogo al de
;           TimerPant, la diferencia está en que se tiene que recorrer 300m en
;           lugar de 200m.
;               TimerFinPant = 300m/Vel_Calc
;               TimerFinPant = (300m/Vel_Calc)*(1/(1000m/km)*(1h/3600s))
;                              *(10dS/1s)
;               TimerFinPant = 10800/Vel_Calc
;*******************************************************************************
Calcula:
        ; DeltaT
        Ldaa #tTimerVel         ; Obtiene el valor máximo de tiempo a
                                ; transcurrir.
        Ldab TimerVel           ; Obtiene el valor restante, en décimas de
                                ; segundo, entre el valor máximo de tiempo
                                ; y 0 para la activación de ambos sensores.
        Sba                     ; Resta al valor máximo el tiempo restante
                                ; para obtener el tiempo transcurrido.
        Staa DeltaT             ; Salva el valor leído en memoria.

        ; Vel_Calc
        Ldx  #Cte_Dvndo_Vel     ; Carga la constante a dividir para obtener la
                                ; velocidad.
        Clra                    ; Como DeltaT es de 8 bits, se limpia A para
                                ; obtener D=0:DeltaT y luego pasarlo a X.
        Ldab  DeltaT            ; Para obtener el valor deseado se carga el
                                ; tiempo transcurrido como divisor.
        Xgdx                    ; Intercambia los valores para tener como
                                ; dividento la constante y como divisor DeltaT.
        Idiv                    ; Realiza la división para obtener la velocidad
        Xgdx                    ; Intercambia los valores de manera que se tenga
                                ; D=0:Vel_Calc
        Stab Vel_Calc           ; Salva el valor de velocidad calculado en
                                ; memoria.

        ; TimerPant
        Ldx  #Cte_Dvndo_TP      ; Carga la constante a dividir para obtener el
                                ; tiempo a esperar para encender la pantalla.
        Clra                    ; Limpia el acumulador A para tener
                                ; D=0:Vel_Calc.
        Ldab  Vel_Calc          ; Para obtener el valor deseado se carga la
                                ; velocidad actual como divisor.
        Xgdx                    ; Intercambia los valores para tener dividendo
                                ; y el divisor según ser requiere.
        Idiv                    ; Realiza la división para obtener TimerPant
        Xgdx                    ; Coloca el valor calculado en D, dado que es
                                ; de 8 bits, para salvar el valor en memoria.
        Stab TimerPant          ; Salva el valor calculado de tiempo a memoria
                                ; en TimerPant.

        ; TimerFinPant
        Ldx  #Cte_Dvndo_TFP     ; Carga la constante a dividir para obtener el
                                ; tiempo a esperar para apagar la pantalla.
        Clra                    ; Limpia el acumulador A. De modo que se tenga
                                ; D=0:Vel_Calc.
        Ldab Vel_Calc           ; Para obtener el valor deseado se carga la
                                ; velocidad actual como divisor.
        Xgdx                    ; Intercambia el X y D de manera que se tenga
                                ; D=Dividendo y X=Divisor.
        Idiv                    ; Realiza la división para obtener TimerFinPant
        Xgdx                    ; Coloca el valor calculado en D, dado que es
                                ; de 8 bits, para salvar el valor en memoria.
        Stab TimerFinPant       ; Salva el valor calculado de tiempo a memoria
                                ; en TimerFinPant.

        Rts
;*******************************************************************************
;    XIII. Subrutina Calcula. Final
;*******************************************************************************

;*******************************************************************************
;    XIV. SUBRUTINA BIN_BCD_MUXP. Inicio
;*******************************************************************************
;       Convierte un número binario alojado en el acumulador A a BCD y lo
; guarda en la dirección BCD.
;*******************************************************************************
BIN_BCD_MuxP:                   ; Subrutina que convierte un número binario de
                                ; 8 bits a BCD.
        Clr  BCD                ; Se setea la dirección del resultado en cero
        Ldab #0                 ; Se usa B como acumulador temporal
        Ldx  #7                 ; X = Contador de cantidad de bits a procesar.
XS3:                            ; Este ciclo aplica el algoritmo XS3 visto en
                                ; clase.
        Asla
        Rol  BCD
        Pshx                    ; Se salva el contador de bits faltantes.
        Psha                    ; Salva el valor binario actual en la pila
Ret_XS3:
        Ldaa BCD                ; Recupera lo que va de la conversión.
        Anda #$0F               ; Se toma nibble más bajo para verificar si es
                                ; mayor o igual a 5.
        Cmpa #$05
IF_GT05:
            Blt  IF_GT05_END    ; Si el nibble en cuestión es >=5 se le suma 3.
            Adda #$03
IF_GT05_END:
        Tab                     ; Se salva el valor a medio convertir en B
        Ldaa BCD                ; Recupera lo que va de la conversión

        Anda #$F0               ; Selecciona la parte alta de BIN.
        Cmpa #$50               ; Verificación que pide el algoritmo XS3.
IF_GT50:
                Blt  IF_GT50_END; Si el nibble en cuestión es >=$50 se le suma
                Adda #$30       ; $30.                                              
IF_GT50_END:
        Aba                     ; Suma ambas partes
BCD_Procesado:
        Staa BCD
        Pula                    ; Recupera el valor binario actual
        Pulx                    ; Recupera el la cantidad de bits a procesar
        Dex                     ; Se decrementa la cantidad de bits procesados.
        Cpx  #$0000             ; En caso de que X==0, se termina la rutina al
        Bne  XS3                ; hacer el último shift, que no conlleva ninguna
        Asla                    ; verificación de >=5, caso contrario se
        Rol  BCD                ; Termina la subrutina con el último
    
        Rts                     ; Retorna

;*******************************************************************************
;    XIV. SUBRUTINA BIN_BCD_MUXP. Final
;*******************************************************************************

;*******************************************************************************
;    XV. SUBRUTINA BCD_7SEG. Inicio
;*******************************************************************************
;       Lee dos números en BCD en BCD1 y BCD2, los codifica como códigos para
; displays de 7 segmenos y los aloja en las posiciones de memoria requeridas
; para que se muestren en las pantallas.
;*******************************************************************************
BCD_7SEG:
        Ldab BCD1               ; Obtiene los valores de trabajo de la
        Pshb                    ; subrutina de manera que se saquen de la pila
        Pshb                    ; según se vayan convirtiendo a códigos de 7
        Ldab BCD2               ; segmentos.
        Pshb
     
        Ldx  #Segment           ; Carga en el índice X la dirección de incio de
                                ; la tabla.
        Andb #$0F               ; El primer dígito se obtiene quitando el primer
        Movb B,X,Dsp1           ; nibble. Se salva en para mostrarlo en el Disp1

        Pulb                    ; Para el segundo, se recupera el valor de la
        Andb #$F0               ; pila y se toma el segundo nibbble. Luego,
        Rorb                    ; se rota hacia la derecha para obtener su valor
        Rorb                    ; unitario.
        Rorb
        Rorb
        Movb B,X,Dsp2

        Pulb                    ; Los segundos dos dígitos son análogos a los
        Andb #$0F               ; primeros con la diferencia de que se leyeron
        Movb B,X,Dsp3           ; de BCD1.

        Pulb
        Andb #$F0
        Rorb
        Rorb
        Rorb
        Rorb
        Movb B,X,Dsp4

BCD_7SEG_END:
        Rts                     ; Retorna

;*******************************************************************************
;    XV. SUBRUTINA BCD_7SEG. Final
;*******************************************************************************

;******************************************************************************
;    XVI. Subrutina Borrar_NumArray. Inicio
;******************************************************************************
;       Resetea el arreglo Num_Array a sus valores por defecto.
;******************************************************************************
Borrar_NumArray:
                                ; Declara el array como inválido.
        Bclr  Banderas_1,ARRAY_OK

        Ldaa MAX_TCL            ; Obtiene la catidad de valores a borrar.
        Ldx  #Num_Array
RNA_Ciclo:                      ; Ciclo de reinicio de Num_Array
        Movb #BNA_Blank_Val,1,X+; Guarda un $FF en la entrada actual de la
                                ; tabla y selecciona el siguiente.
        Deca                    ; Decrementa la cantidad de elementos restantes,
                                ; cambia la bandera Z, de modo que no requiere
                                ; Cmpa acá.
        Beq  Borrar_NumArray_END
        Bra  RNA_Ciclo          ; Repite el ciclo

Borrar_NumArray_END:
        Rts                     ; Retorna
;*******************************************************************************
;    XVI. Subrutina Borrar_NumArray. Inicio
;*******************************************************************************

;*******************************************************************************
;    XVII. Máquina de tiempos. Inicio
;*******************************************************************************
;       Se encarga de la temporización del sistema, decrementa los contadores
;    que se le indiquen a un ritmo regular según la base de tiempo seleccionada.
;*******************************************************************************
Maquina_Tiempos:
        ; Empieza por realojar tiempo en el contador del canal 5 que se atiende
        ; por OC.
        Ldx  TCNT               ; Carga la cuenta actual y la cantidad de ticks
                                ; en el futuro que dispararán la siguiente
                                ; interrupción.
        Ldab #Contante_OC5_MT
        Abx                     ; Suma ambos valores.
        Stx  TC5                ; Salva el valor de la siguiente interrupción
                                ; en su registro respectivo.

        Ldx  #Tabla_Timers_BaseT        ; Obtiene la cabecera de la tabla de
                                        ; timers.
        Jsr  Decre_Timers_BaseT
Comp_T1mS:
        Ldx  Timer1mS
        Cpx  #0
        Bne  Comp_T10mS
        Movw #tTimer1mS,Timer1mS
        Ldx  #Tabla_Timers_Base1mS
        Jsr  Decre_Timers
Comp_T10mS:
        Ldx  Timer10mS
        Cpx  #0
        Bne  Comp_T100mS
        Movw #tTimer10mS,Timer10mS
        Ldx  #Tabla_Timers_Base10mS
        Jsr  Decre_Timers
Comp_T100mS:
        Ldx  Timer100mS
        Cpx  #0
        Bne  Comp_T1S
        Movw #tTimer100mS,Timer100mS
        Ldx  #Tabla_Timers_Base100mS
        Jsr  Decre_Timers
Comp_T1S:
        Ldx  Timer1S
        Cpx  #0
        Bne  Maquina_Tiempos_FIN
        Movw #tTimer1S,Timer1S
        Ldx  #Tabla_Timers_Base1S
        Jsr  Decre_Timers
Maquina_Tiempos_FIN:
        Bset CRGFLG,#RTI_Flag
        RTI

;******************************************************************************
;    XVII.I. Subrutina Decre_Timers_BaseT
;******************************************************************************
;       Decrementa en uno los timers base de la máquina de tiempos.
;******************************************************************************
Decre_Timers_BaseT:
        Ldy  2,X+                       ; Carga la entrada X-ésima de la tabla
                                        ; de contadores de generales.
        Cpy  #0                         ; Si el contador es cero, se salta.
        Beq  Decre_Timers_BaseT
        Cpy  #$FFFF                     ; Detecta el final de la tabla
        Bne  Decre_Timers_BaseT_Siguiente ; Si se llega al final de la tabla,
        Rts                             ; se termina la subrutina, caso
                                        ; contrario se decrementa en 1 el cont.
Decre_Timers_BaseT_Siguiente:
        Dey                             ; Decrementa en uno el contador y lo
        Sty  -2,X                       ; guarda en su posición respectiva en la
        Bra  Decre_Timers_BaseT         ; tabla, luego reinicia el ciclo.


;******************************************************************************
;    XVII.II. Subrutina Decre_Timers
;******************************************************************************
;       Decrementa, por entrada de la tabla los timers de cada una de las
; unidades de tiempo.
;******************************************************************************
Decre_Timers:
        Ldab 0,X                        ; Carga el contador X-ésimo de una de
                                        ; las tablas de contadores del usuario.
        Cmpb #0                         ; Si es cero, se salta.
        Beq  Decre_Timers_Siguiente
        Cmpb  #$FF                      ; Si se llega al final de la tabla se
        Bne  Decre_Timers_Restar        ; retorna, caso contrario se decrementa
        Rts                             ; el contador.
Decre_Timers_Siguiente:
        Inx                             ; Selecciona el siguiente elemento de la
        Bra  Decre_Timers               ; tabla y reinicia el ciclo
Decre_Timers_Restar:
        Decb                            ; Decrementa el contador y lo salva en
        Stab 0,X                        ; la tabla respectiva
        Bra  Decre_Timers_Siguiente
;*******************************************************************************
;    XVII. Máquina de tiempos. Final
;*******************************************************************************
