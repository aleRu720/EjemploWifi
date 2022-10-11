#include "mbed.h"
#include "wifi.h"
#include "config.h"

#define     RINGBUFFLENGTH      256

#define     GENERALINTERVAL     100

#define     SECUENCEHB          0x0015

#define     MASKHB              0x0F

#define     ALIVEAUTOINTERVAL   60000

/**
 * @brief Enumeración de la MEF para decodificar el protocolo
 * 
 */
typedef enum{
    START,
    HEADER_1,
    HEADER_2,
    HEADER_3,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eProtocolo;

_eProtocolo estadoProtocolo;

/**
 * @brief Enumeración de la lista de comandos
 * 
 */
typedef enum{
        ACK=0x0D,
        GETALIVE=0xF0,
        STARTCONFIG=0xEE,
        OTHERS
}_eID;

/**
 * @brief estructura de datos del Wifi para configurar la conexion
 * 
 */
wifiData myWifiData;


/**
 * @brief Estructura de datos para el puerto serie
 * 
 */
typedef struct{
    uint8_t timeOut;         //!< TiemOut para reiniciar la máquina si se interrumpe la comunicación
    uint8_t indexStart;      //!< Indice para saber en que parte del buffer circular arranca el ID
    uint8_t cheksumRx;       //!< Cheksumm RX
    uint8_t indexWriteRx;    //!< Indice de escritura del buffer circular de recepción
    uint8_t indexReadRx;     //!< Indice de lectura del buffer circular de recepción
    uint8_t indexWriteTx;    //!< Indice de escritura del buffer circular de transmisión
    uint8_t indexReadTx;     //!< Indice de lectura del buffer circular de transmisión
    uint8_t bufferRx[RINGBUFFLENGTH];   //!< Buffer circular de recepción
    uint8_t bufferTx[RINGBUFFLENGTH];   //!< Buffer circular de transmisión
}_sDato ;

 _sDato datosComSerie, datosComWifi;


/**
 * @brief Unión para descomponer/componer datos mayores a 1 byte
 * 
 */
typedef union {
    float f32;
    int32_t i32;
    uint32_t ui32;
    uint16_t ui16[2];
    uint8_t ui8[4];
    int8_t i8[4];
}_udat;

_udat myWord;


typedef union {
    struct{
        uint8_t bit7:1;
        uint8_t bit6:1;
        uint8_t bit5:1;
        uint8_t bit4:1;
        uint8_t bit3:1;
        uint8_t bit2:1;
        uint8_t bit1:1;
        uint8_t bit0:1;
    }individualFlags;
    uint8_t allFlags;
}_bFlags;

_bFlags myFlags;


/*************************************************************************************************/
/* Prototipo de Funciones */


/**
 * @brief Función que se llama en la interrupción de recepción de datos
 * Cuando se llama la función se leen todos los datos que llagaron.
 */
void onDataRx(void);

/**
 * @brief Decodifica las tramas que se reciben 
 * La función decodifica el protocolo para saber si lo que llegó es válido.
 * Utiliza una máquina de estado para decodificar el paquete
 */
void decodeProtocol(_sDato *);

/**
 * @brief Procesa el comando (ID) que se recibió
 * Si el protocolo es correcto, se llama a esta función para procesar el comando
 */
void decodeData(_sDato *);

/**
 * @brief Envía los datos a la PC puerto Serie
 * La función consulta si el puerto serie está libre para escribir, si es así envía 1 byte y retorna
 */
void sendData(_sDato *);

/**
 * @brief  Función Hearbeat
 * Ejecuta las tareas del hearbeat
 * 
 * @param generalTime variable para almacenar el tiempo
 */
void hearbeatTask(uint32_t *generalTime);


/**
 * @brief Rutina para revisar los buffers de comunicación, decodificar y transmitar según sea necesario
 * 
 * @param datosCom Puntero a la estructura de datos del buffer
 * @param source flag que indica el módulo a donde transmitir
 */
void comunicationsTask(_sDato *datosCom, uint8_t source);

void aliveAutoTask(uint32_t *aliveAutoTime);

void autoConnectWifi(void);


/*****************************************************************************************************/
/* Configuración del Microcontrolador */

DigitalOut HEARBEAT(PC_13); //!< Defino la salida del led

RawSerial pcCom(PA_9,PA_10,115200); //!< Configuración del puerto serie, la velocidad (115200) tiene que ser la misma en QT

Timer miTimer; //!< Timer general


/**
 * @brief Instanciación de la clase Wifi, le paso como parametros el buffer de recepción, el indice de 
 * escritura para el buffer de recepción y el tamaño del buffer de recepción
 */
Wifi myWifi(datosComWifi.bufferRx,&datosComWifi.indexWriteRx, sizeof(datosComWifi.bufferRx));

/*****************************************************************************************************/
/*********************************  Función Principal ************************************************/
int main()
{

    uint32_t generalTime=0, aliveAutoTime=0;

    miTimer.start();

    pcCom.attach(&onDataRx,RawSerial::RxIrq);

    myWifi.initTask();

    autoConnectWifi();
    
    while(true)
    {

        myWifi.taskWifi();
        hearbeatTask(&generalTime);
        comunicationsTask(&datosComSerie,true);
        comunicationsTask(&datosComWifi,false);
        aliveAutoTask(&aliveAutoTime);        
    }
    return 0;
}





/*****************************************************************************************************/
/************  MEF para decodificar el protocolo serie ***********************/
void decodeProtocol(_sDato *datosCom)
{
    static uint8_t nBytes=0;
    uint8_t indexWriteRxCopy=datosCom->indexWriteRx;

    while (datosCom->indexReadRx!=indexWriteRxCopy)
    {
        switch (estadoProtocolo) {
            case START:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='U'){
                    estadoProtocolo=HEADER_1;
                    datosCom->cheksumRx=0;
                }
                break;
            case HEADER_1:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='N')
                   estadoProtocolo=HEADER_2;
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (datosCom->bufferRx[datosCom->indexReadRx++]=='E')
                    estadoProtocolo=HEADER_3;
                else{
                    datosCom->indexReadRx--;
                   estadoProtocolo=START;
                }
                break;
        case HEADER_3:
            if (datosCom->bufferRx[datosCom->indexReadRx++]=='R')
                estadoProtocolo=NBYTES;
            else{
                datosCom->indexReadRx--;
               estadoProtocolo=START;
            }
            break;
            case NBYTES:
                datosCom->indexStart=datosCom->indexReadRx;
                nBytes=datosCom->bufferRx[datosCom->indexReadRx++];
               estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (datosCom->bufferRx[datosCom->indexReadRx++]==':'){
                   estadoProtocolo=PAYLOAD;
                    datosCom->cheksumRx ='U'^'N'^'E'^'R'^ nBytes^':';
                }
                else{
                    datosCom->indexReadRx--;
                    estadoProtocolo=START;
                }
                break;
            case PAYLOAD:
                if (nBytes>1){
                    datosCom->cheksumRx ^= datosCom->bufferRx[datosCom->indexReadRx++];
                }
                nBytes--;
                if(nBytes<=0){
                    estadoProtocolo=START;
                    if(datosCom->cheksumRx == datosCom->bufferRx[datosCom->indexReadRx]){
                        decodeData(datosCom); 
                    }
                }
               
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
}


/*****************************************************************************************************/
/************  Función para procesar el comando recibido ***********************/
void decodeData(_sDato *datosCom)
{
    #define POSID   2
    #define POSDATA 3
    wifiData *wifidataPtr;
    uint8_t *ptr; 
    uint8_t auxBuffTx[50], indiceAux=0, cheksum, sizeWifiData, indexBytesToCopy=0, numBytesToCopy=0;
    
    auxBuffTx[indiceAux++]='U';
    auxBuffTx[indiceAux++]='N';
    auxBuffTx[indiceAux++]='E';
    auxBuffTx[indiceAux++]='R';
    auxBuffTx[indiceAux++]=0;
    auxBuffTx[indiceAux++]=':';

    switch (datosCom->bufferRx[datosCom->indexStart+POSID]) {
        case GETALIVE:
            auxBuffTx[indiceAux++]=GETALIVE;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;         
            break;
        case STARTCONFIG: //Inicia Configuración del wifi 
            auxBuffTx[indiceAux++]=STARTCONFIG;
            auxBuffTx[indiceAux++]=ACK;
            auxBuffTx[NBYTES]=0x03;     
            myWifi.resetWifi();
            sizeWifiData =sizeof(myWifiData);
            indexBytesToCopy=datosCom->indexStart+POSDATA;
            wifidataPtr=&myWifiData;

            if ((RINGBUFFLENGTH - indexBytesToCopy)<sizeWifiData){
                numBytesToCopy=RINGBUFFLENGTH-indexBytesToCopy;
                memcpy(wifidataPtr,&datosCom->bufferRx[indexBytesToCopy], numBytesToCopy);
                indexBytesToCopy+=numBytesToCopy;
                sizeWifiData-=numBytesToCopy;
                ptr= (uint8_t *)wifidataPtr + numBytesToCopy;
                memcpy(ptr,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
            }else{
                memcpy(&myWifiData,&datosCom->bufferRx[indexBytesToCopy], sizeWifiData);
            }
            myWifi.configWifi(&myWifiData);
            break;
        
        default:
            auxBuffTx[indiceAux++]=0xDD;
            auxBuffTx[NBYTES]=0x02;
            break;
    }
   cheksum=0;
    for(uint8_t a=0 ;a < indiceAux ;a++)
    {
        cheksum ^= auxBuffTx[a];
        datosCom->bufferTx[datosCom->indexWriteTx++]=auxBuffTx[a];
    }
        datosCom->bufferTx[datosCom->indexWriteTx++]=cheksum;

}


/*****************************************************************************************************/
/************  Función para enviar los bytes hacia la pc ***********************/
void sendData(_sDato *datosCom)
{
    if(pcCom.writeable())
        pcCom.putc(datosCom->bufferTx[datosCom->indexReadTx++]);

}


/*****************************************************************************************************/
/************  Función para hacer el hearbeats ***********************/
void hearbeatTask(uint32_t *generalTime)
{
    static uint8_t indexHb=0;
    if ((miTimer.read_ms()-*generalTime)>=GENERALINTERVAL){
            *generalTime=miTimer.read_ms();       
        HEARBEAT.write( (~SECUENCEHB) & (1<<indexHb));
        indexHb++;
        indexHb &=MASKHB;  
    }   
}


void comunicationsTask(_sDato *datosCom, uint8_t source){
    if(datosCom->indexReadRx!=datosCom->indexWriteRx ){
            decodeProtocol(datosCom);
    }

    if(datosCom->indexReadTx!=datosCom->indexWriteTx){
        if(source)
            sendData(datosCom);
        else
            myWifi.writeWifiData(&datosCom->bufferTx[datosCom->indexReadTx++],1);   
    } 
}


void aliveAutoTask(uint32_t *aliveAutoTime){
    if(myWifi.isWifiReady()){
        if((miTimer.read_ms()-*aliveAutoTime)>=ALIVEAUTOINTERVAL){
            *aliveAutoTime=miTimer.read_ms();
            datosComWifi.bufferRx[datosComWifi.indexWriteRx+2]=GETALIVE;
            datosComWifi.indexStart=datosComWifi.indexWriteRx;
            decodeData(&datosComWifi);
        }
    }else{
        *aliveAutoTime=0;
    }
}

/**********************************************************************************/
/* Servicio de Interrupciones*/

void onDataRx(void)
{
    while (pcCom.readable())
    {
        datosComSerie.bufferRx[datosComSerie.indexWriteRx++]=pcCom.getc();
    }
}
/* FIN Servicio de Interrupciones*/
/**********************************************************************/

/**********************************AUTO CONNECT WIF*********************/

void autoConnectWifi(){
    #ifdef AUTOCONNECTWIFI
        memcpy(&myWifiData.cwmode, dataCwmode, sizeof(myWifiData.cwmode));
        memcpy(&myWifiData.cwdhcp,dataCwdhcp, sizeof(myWifiData.cwdhcp) );
        memcpy(&myWifiData.cwjap,dataCwjap, sizeof(myWifiData.cwjap) );
        memcpy(&myWifiData.cipmux,dataCipmux, sizeof(myWifiData.cipmux) );
        memcpy(&myWifiData.cipstart,dataCipstart, sizeof(myWifiData.cipstart) );
        memcpy(&myWifiData.cipmode,dataCipmode, sizeof(myWifiData.cipmode) );
        memcpy(&myWifiData.cipsend,dataCipsend, sizeof(myWifiData.cipsend) );
        myWifi.configWifi(&myWifiData);
    #endif
}