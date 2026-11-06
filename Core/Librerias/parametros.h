/*
 * parametros.h
 *
 *  Adaptada de la libreria del proyecto DOBBY.
 *
 *  Guarda los parametros del robot en la FLASH interna (libreria flash_interna).
 *  Al arrancar los lee; si no hay datos validos carga los valores por defecto.
 */

#ifndef LIBRERIAS_PARAMETROS_H_
#define LIBRERIAS_PARAMETROS_H_

#include "main.h"

//como usar
/*
 1) En main.c, al arrancar (despues de MX_ICACHE_Init):
      if (Param_init()) { //cargo de la flash }  else { //valores por defecto }

 2) Usar los valores en cualquier parte:  PARAM.kgiro, PARAM.vgiro, PARAM.vmax

 3) Cambiar valores en RAM (rechazan lo que este fuera de rango):
      Param_set_kgiro(0.95f);
      Param_set_vgiro(300);
      Param_set_vmax(250);

 4) Grabar SOLO con el robot detenido:  Param_guardar();
 */

//cambiar si se agregan/quitan campos a la struct: invalida lo guardado y carga los de defecto
#define PARAM_VERSION    2

#define PARAM_KGIRO_DEF  1.0f     //1 = sin correccion
#define PARAM_KGIRO_MIN  0.5f
#define PARAM_KGIRO_MAX  2.0f
#define PARAM_VGIRO_DEF  500      //500 = no limita el giro
#define PARAM_VGIRO_MIN  0
#define PARAM_VGIRO_MAX  500
#define PARAM_VMAX_DEF   500      //500 = no limita la velocidad
#define PARAM_VMAX_MIN   0
#define PARAM_VMAX_MAX   500

typedef struct {
    uint32_t marca;        //PARAM_MARCA si hay datos validos
    uint32_t version;      //PARAM_VERSION

    float    kgiro;        //correccion entre motores: <1 frena el izquierdo, >1 frena el derecho
    int32_t  vgiro;        //giro maximo, en us respecto al neutro de 1500 (0 a 500)
    int32_t  vmax;         //tope de cada motor, en us respecto al neutro de 1500 (0 a 500)

    uint32_t crc;          //SIEMPRE el ultimo campo
} PARAMETROS;

extern PARAMETROS PARAM;

uint8_t Param_init(void);              //1 = cargo de flash, 0 = cargo por defecto
uint8_t Param_leer_flash(void);        //1 = cargo de flash, 0 = sin datos validos (PARAM no cambia)
void    Param_defecto(void);           //valores por defecto en RAM
uint8_t Param_guardar(void);           //1 = grabo bien, 0 = error
uint8_t Param_guardado(void);          //1 = lo que hay en RAM es igual a lo grabado en flash
uint8_t Param_set_kgiro(float k);      //1 = aceptado, 0 = fuera de rango (no cambia nada)
uint8_t Param_set_vgiro(int32_t v);    //1 = aceptado, 0 = fuera de rango (no cambia nada)
uint8_t Param_set_vmax(int32_t v);     //1 = aceptado, 0 = fuera de rango (no cambia nada)

#endif /* LIBRERIAS_PARAMETROS_H_ */
