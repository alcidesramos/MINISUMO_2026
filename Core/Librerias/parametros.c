/*
 * parametros.c
 *
 *  Adaptada de la libreria del proyecto DOBBY. Ver parametros.h
 */

#include "parametros.h"
#include "flash_interna.h"
#include <string.h>
#include <stddef.h>

#define PARAM_MARCA   0x50A02026u

PARAMETROS PARAM;


//el !(dentro del rango) tambien atrapa nan
static uint8_t Param_kgiro_ok(float k)
{
  return ((k >= PARAM_KGIRO_MIN) && (k <= PARAM_KGIRO_MAX)) ? 1 : 0;
}

static uint8_t Param_vgiro_ok(int32_t v)
{
  return ((v >= PARAM_VGIRO_MIN) && (v <= PARAM_VGIRO_MAX)) ? 1 : 0;
}

static uint8_t Param_vmax_ok(int32_t v)
{
  return ((v >= PARAM_VMAX_MIN) && (v <= PARAM_VMAX_MAX)) ? 1 : 0;
}


//valores por defecto (cuando la flash esta vacia o los datos no son validos)
void Param_defecto(void)
{
  memset(&PARAM, 0, sizeof(PARAM));
  PARAM.kgiro = PARAM_KGIRO_DEF;
  PARAM.vgiro = PARAM_VGIRO_DEF;
  PARAM.vmax  = PARAM_VMAX_DEF;
}


uint8_t Param_set_kgiro(float k)
{
  if (!Param_kgiro_ok(k)) return 0;
  PARAM.kgiro = k;
  return 1;
}


uint8_t Param_set_vgiro(int32_t v)
{
  if (!Param_vgiro_ok(v)) return 0;
  PARAM.vgiro = v;
  return 1;
}


uint8_t Param_set_vmax(int32_t v)
{
  if (!Param_vmax_ok(v)) return 0;
  PARAM.vmax = v;
  return 1;
}


//crc de toda la struct menos el campo crc
static uint32_t Param_calcula_crc(const PARAMETROS *p)
{
  return Flash_crc32(p, offsetof(PARAMETROS, crc));
}


//1 = los datos son validos
static uint8_t Param_valido(const PARAMETROS *p)
{
  return (p->marca == PARAM_MARCA) &&
         (p->version == PARAM_VERSION) &&
         (p->crc == Param_calcula_crc(p)) &&
         Param_kgiro_ok(p->kgiro) &&
         Param_vgiro_ok(p->vgiro) &&
         Param_vmax_ok(p->vmax);
}


//carga PARAM desde la flash. 1 = cargo. 0 = no hay datos validos: PARAM queda como estaba
uint8_t Param_leer_flash(void)
{
  PARAMETROS leido;
  memcpy(&leido, (const void *)FLASH_PARAM_DIR, sizeof(leido));

  if (!Param_valido(&leido)) return 0;
  PARAM = leido;
  return 1;
}


//al arrancar: la flash, o los valores por defecto si no hay datos validos
uint8_t Param_init(void)
{
  if (Param_leer_flash()) return 1;
  Param_defecto();
  return 0;
}


//graba PARAM en la flash. Llamar SOLO con el robot detenido.
uint8_t Param_guardar(void)
{
  PARAM.marca   = PARAM_MARCA;
  PARAM.version = PARAM_VERSION;
  PARAM.crc     = Param_calcula_crc(&PARAM);

  //borra el sector de parametros, graba y verifica (1 = ok, 0 = error)
  return Flash_guardar(FLASH_PARAM_DIR, FLASH_PARAM_TAM, &PARAM, sizeof(PARAM));
}


//1 = lo que hay en RAM es igual a lo grabado en flash
uint8_t Param_guardado(void)
{
  PARAMETROS leido;
  memcpy(&leido, (const void *)FLASH_PARAM_DIR, sizeof(leido));

  return Param_valido(&leido) && (leido.kgiro == PARAM.kgiro) && (leido.vgiro == PARAM.vgiro) && (leido.vmax == PARAM.vmax);
}
