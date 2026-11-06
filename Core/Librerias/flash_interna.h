/*
 * flash_interna.h
 *
 *  Adaptada de la libreria del proyecto DOBBY.
 *
 *  Guardado de datos en la FLASH interna del STM32H5 (sectores de 8 KB,
 *  grabacion de a 16 bytes).
 *
 *  STM32H562RG = 1 MB de flash, dos bancos de 512 KB:
 *
 *   0x08000000 +------------------------+
 *              | programa               |
 *              | ...libre...            |
 *   0x080FE000 +------------------------+ FLASH_PARAM_DIR (banco 2, ultimo sector)
 *              | PARAMETROS (8 KB)      |
 *   0x08100000 +------------------------+ fin
 *
 *  OJO: los tamanos van fijos a proposito. Las macros FLASH_SIZE y FLASH_BANK_SIZE
 *  del HAL leen la zona de solo lectura 0x08FFF80C, y este proyecto tiene el ICACHE
 *  encendido sin la region MPU no cacheable que tiene DOBBY: leerla puede dar HardFault.
 *  El .ld de este proyecto dice 2048K (CubeMX): por eso Flash_guardar revisa con los
 *  simbolos del linker que el programa no llegue a la zona de parametros.
 */

#ifndef LIBRERIAS_FLASH_INTERNA_H_
#define LIBRERIAS_FLASH_INTERNA_H_

#include "main.h"

//como usar
/*
   //guardar una struct (borra el sector y graba) -> 1 = ok, 0 = error
   Flash_guardar(FLASH_PARAM_DIR, FLASH_PARAM_TAM, &mis_datos, sizeof(mis_datos));

   //leer: la flash se lee como memoria normal
   memcpy(&mis_datos, (const void *)FLASH_PARAM_DIR, sizeof(mis_datos));

   //verificar integridad de los datos con un CRC
   uint32_t crc = Flash_crc32(&mis_datos, bytes);

   Grabar SOLO con el robot detenido (borrar 8 KB tarda algunos ms).
 */

#define FLASH_CHIP_TAM     (1024u * 1024u)        //STM32H562RG: 1 MB
#define FLASH_BANCO_TAM    (FLASH_CHIP_TAM / 2u)  //dos bancos iguales
#define FLASH_PARAM_TAM    (8u * 1024u)           //multiplo de 8 KB

#define FLASH_FIN          (FLASH_BASE + FLASH_CHIP_TAM)
#define FLASH_PARAM_DIR    (FLASH_FIN - FLASH_PARAM_TAM)
#define FLASH_ZONA_INICIO  (FLASH_PARAM_DIR)      //inicio de todo lo reservado

uint8_t  Flash_zona_libre(void);                  //1 = el programa no toca la zona reservada
uint8_t  Flash_guardar(uint32_t dir, uint32_t tam_zona, const void *datos, uint32_t bytes);
uint32_t Flash_crc32(const void *datos, uint32_t bytes);

#endif /* LIBRERIAS_FLASH_INTERNA_H_ */
