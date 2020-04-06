#ifndef __LCD_ILI9341__
#define __LCD_ILI9341__

#include <stm32f10x.h>
#include "pinmacro.h"

#if 0
  LCD_SPI_SPEED (SPI_CR1_BR_2, SPI_CR1_BR_1, SPI_CR1_BR_0)
  LCD_TP_SPEED  (SPI_CR1_BR_2, SPI_CR1_BR_1, SPI_CR1_BR_0)
  LCD_RST (GPIO)
  LCD_CMD (GPIO)
  LCD_DOUT (GPIO)
  LCD_SCK (GPIO)
  LCD_CS (GPIO)
  LCD_DIN (GPIO)
  LCD_TP (GPIO)
  LCD_IRQ (GPIO)
  LCD_MIR_X (0/1)
  LCD_MIR_Y (0/1)
  LCD_ORIENT_V (0/1)
  LCD_ML (0/1)
  LCD_RGB (0/1)
  LCD_MH (0/1)
#endif
//LCD_SPI
#ifndef LCD_SPI_SPEED
  #define LCD_SPI_SPEED 0
#endif
#ifndef LCD_TP_SPEED
  #define LCD_TP_SPEED SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0
#endif
#ifndef LCD_RST
  #define LCD_RST   A,2,0,GPIO_PP50
#endif
#ifndef LCD_CMD
  #define LCD_CMD   A,3,0,GPIO_PP50
#endif
#ifndef LCD_DOUT
  #define LCD_DOUT  A,7,1,GPIO_APP50
#endif
#ifndef LCD_SCK
  #define LCD_SCK   A,5,1,GPIO_APP50
#endif
#ifndef LCD_CS
  #define LCD_CS    A,4,0,GPIO_PP50
#endif
#ifndef LCD_DIN
  #define LCD_DIN   A,6,1,GPIO_HIZ //not yet
#endif
#ifndef LCD_TP
  #define LCD_TP    //not yet
#endif
#ifndef LCD_IRQ
  #define LCD_IRQ   //not yet
#endif

//LCD_orientation
#ifndef LCD_MIR_X
  #define LCD_MIR_X 0
#endif
#ifndef LCD_MIR_Y
  #define LCD_MIR_Y 1
#endif
#ifndef LCD_ORIENT_V
  #define LCD_ORIENT_V 1
#endif
#ifndef LCD_ML
  #define LCD_ML 0
#endif
#ifndef LCD_RGB
  #define LCD_RGB 1
#endif
#ifndef LCD_MH
  #define LCD_MH 0
#endif

#if LCD_ORIENT_V
  #define LCD_MAXX 239
  #define LCD_MAXY 319
#else
  #define LCD_MAXX 319
  #define LCD_MAXY 239
#endif

//переключение на передачу 8-битных данных
#define lcd_size_8()  do{\
    SPI1->CR1 = SPI_CR1_MSTR | LCD_SPI_SPEED | SPI_CR1_SPE\
    | SPI_CR1_SSI | SPI_CR1_SSM;\
  }while(0)
//переключение на передачу 16-битных данных (удобно для цвета)
#define lcd_size_16() do{\
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_DFF | LCD_SPI_SPEED\
    | SPI_CR1_SPE | SPI_CR1_SSI | SPI_CR1_SSM;\
  }while(0)
//передача 1 слова (8 или 16 бит) по SPI1 и прием ответа
uint16_t lcd_send(uint16_t data);
//ожидание пока SPI1 освободится для изменения настроек
#define lcd_wait() do{ while(!(SPI1->SR & SPI_SR_TXE)){} while(SPI1->SR & SPI_SR_BSY){} }while(0)
//передача команды и переключение для передачи данных
//!!!должно завершаться lcd_stop()!!!
void lcd_start(uint8_t cmd);
//конец передачи данных
#define lcd_stop() do{lcd_wait(); GPO_OFF(LCD_CS);}while(0)
//передача только команды
#define lcd_cmd(cmd) do{lcd_start(cmd); lcd_stop();}while(0)
//передача данных (с явным переключением команда->данные)
void lcd_data(uint8_t data);
//передача 16-битного слова двумя порциями по 8 бит
void lcd_word(uint16_t data);
//ограничение области рисование и ожидание приема цветов
//!!!должно завершаться lcd_end_area()!!!
void lcd_begin_area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
//завершение рисования в области
#define lcd_end_area() lcd_stop()
//очистка дисплея (с ожиданием завершения)
void lcd_clr(uint16_t color);
//перевод из 3 байт r,g,b в код цвета для дисплея
#if LCD_RGB==1
#define rgb2col(r,g,b) (((r<<8)&0b1111100000000000)|((g<<3)&0b0000011111100000)|(b>>3))
#else
#define rgb2col(r,g,b) (((b<<8)&0b1111100000000000)|((g<<3)&0b0000011111100000)|(r>>3))
#endif
//инициализация дисплея
void lcd_init();
//начало передачи цветов по DMA
void lcd_dma_init(uint16_t mem[], uint16_t size);
void lcd_dma_restart(uint16_t mem[], uint16_t size);
//отключение DMA
void lcd_dma_deinit();
//завершена ли передача по DMA?
#define lcd_dma_finish() (DMA1->ISR & DMA_ISR_TCIF3)
//ожидание завершения передачи по DMA
#define lcd_dma_wait() do{}while(!lcd_dma_finish())


#endif
