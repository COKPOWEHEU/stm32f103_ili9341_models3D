#include "lcd_ili9341.h"

void lcd_sleep(uint32_t time){
  while(time--){asm volatile ("nop");}
}
void lcd_spi_init(){
  GPIO_config(LCD_RST);
  GPIO_config(LCD_CMD);
  GPIO_config(LCD_DOUT);
  GPIO_config(LCD_SCK);
  GPIO_config(LCD_CS);
//  GPIO_config(LCD_DIN);
//  GPIO_config(LCD_TP);
//  GPIO_config(LCD_IRQ);
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  lcd_size_8();
  SPI1->CR2 = SPI_CR2_SSOE;
  SPI1->DR = 0;
}

uint16_t lcd_send(uint16_t data){
  uint16_t res;
  while(!(SPI1->SR & SPI_SR_TXE)){}
  res = SPI1->DR;
  SPI1->DR = data;
  return res;
}

void lcd_start(uint8_t cmd){
  lcd_wait();
  GPO_ON(LCD_CMD);
  GPO_ON(LCD_CS);
  SPI1->DR = cmd;
  lcd_wait();
  GPO_OFF(LCD_CMD);
}

void lcd_data(uint8_t data){
  lcd_wait();
  GPO_OFF(LCD_CMD);
  GPO_ON(LCD_CS);
  SPI1->DR = data;
  lcd_wait();
  GPO_OFF(LCD_CS);
}

void lcd_word(uint16_t data){
  lcd_send((uint8_t)(data>>8));
  lcd_send((uint8_t)data);
}

void lcd_begin_area(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
  lcd_start(0x2A); lcd_word(y1); lcd_word(y2); lcd_stop();
  lcd_start(0x2B); lcd_word(x1); lcd_word(x2); lcd_stop();
  lcd_start(0x2C);
}

void lcd_clr(uint16_t color){
  uint32_t i,w=240,h=320,n;
  n = w*h;
  lcd_begin_area(0,0,w-1,h-1);
  
  lcd_size_16();
  
  for(i=0;i<n;i++){
    lcd_send(color);
  }
  lcd_stop();
  lcd_size_8();
}

void lcd_init(){
  GPIO_config(LCD_CS);
  GPIO_config(LCD_CMD);
  GPIO_config(LCD_RST);
  lcd_spi_init();
  
  GPO_ON(LCD_RST);
  lcd_sleep(100000);
  GPO_OFF(LCD_RST);
  lcd_cmd(0x01);
  lcd_sleep(100000);
  lcd_start(0xCB); lcd_send(0x39); lcd_send(0x2C); lcd_send(0x00);
                   lcd_send(0x34); lcd_send(0x02); lcd_stop();
  lcd_start(0xCF); lcd_send(0x00); lcd_send(0xC1); lcd_send(0x30); lcd_stop();
  lcd_start(0xE8); lcd_send(0x85); lcd_send(0x00); lcd_send(0x78); lcd_stop();
  lcd_start(0xEA); lcd_send(0x00); lcd_send(0x00); lcd_stop();
  lcd_start(0xED); lcd_send(0x64); lcd_send(0x03); lcd_send(0x12); lcd_send(0x81); lcd_stop();
  lcd_start(0xB6); lcd_send(0x08); lcd_send(0x82); lcd_send(0x27); lcd_stop();
  lcd_start(0xE0); lcd_send(0x0F); lcd_send(0x31); lcd_send(0x2B); lcd_send(0x0C);
                   lcd_send(0x0E); lcd_send(0x08); lcd_send(0x4E); lcd_send(0xF1);
                   lcd_send(0x37); lcd_send(0x07); lcd_send(0x10); lcd_send(0x03);
                   lcd_send(0x0E); lcd_send(0x09); lcd_send(0x00); lcd_stop();
  lcd_start(0xE1); lcd_send(0x00); lcd_send(0x0E); lcd_send(0x14); lcd_send(0x03);
                   lcd_send(0x11); lcd_send(0x07); lcd_send(0x31); lcd_send(0xC1);
                   lcd_send(0x48); lcd_send(0x08); lcd_send(0x0F); lcd_send(0x0C);
                   lcd_send(0x31); lcd_send(0x36); lcd_send(0x0F); lcd_stop();
  lcd_cmd(0xF7); lcd_data(0x20);
  lcd_cmd(0xC0); lcd_data(0x23);
  lcd_cmd(0xC1); lcd_data(0x10);
  lcd_cmd(0xC5); lcd_data(0x3e); lcd_data(0x28);
  lcd_cmd(0xC7); lcd_data(0x86);
  lcd_cmd(0x3A); lcd_data(0x55);
  lcd_cmd(0xB1); lcd_data(0x00); lcd_data(0x18);
  lcd_cmd(0xF2); lcd_data(0x00);
  lcd_cmd(0x26); lcd_data(0x01);
  lcd_start(0x36);
  lcd_send((LCD_MIR_Y<<7)|(LCD_MIR_X<<6)|(LCD_ORIENT_V<<5)|(LCD_ML<<4)|(LCD_RGB<<3)|(LCD_MH<<2));
  lcd_stop();
  
  lcd_cmd(0x11);
  lcd_sleep(100000);
  lcd_cmd(0x29);
  //SPI1_wait();
}
void lcd_dma_init(uint16_t mem[], uint16_t size){
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_DFF | LCD_SPI_SPEED | SPI_CR1_SSI | SPI_CR1_SSM;
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  
  DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR));
  DMA1_Channel3->CMAR = (uint32_t)mem;
  DMA1_Channel3->CNDTR = size;
  DMA1_Channel3->CCR = DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_CCR1_PL_1 | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0;
  DMA1_Channel3->CCR |= DMA_CCR1_EN;
  
  SPI1->CR2 |= SPI_CR2_TXDMAEN;
  DMA1->IFCR = DMA_IFCR_CTCIF3;
  SPI1->CR1 |= SPI_CR1_SPE;
}
void lcd_dma_restart(uint16_t mem[], uint16_t size){
  DMA1_Channel3->CCR &=~DMA_CCR1_EN;
  DMA1->IFCR = DMA_IFCR_CTCIF3;
  
  DMA1_Channel3->CMAR = (uint32_t)mem;
  DMA1_Channel3->CNDTR = size;
  DMA1_Channel3->CCR = DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_CCR1_PL_1 | DMA_CCR1_PSIZE_0 | DMA_CCR1_MSIZE_0;
  DMA1_Channel3->CCR |= DMA_CCR1_EN;
  
  SPI1->CR1 |= SPI_CR1_SPE;
}
void lcd_dma_deinit(){
  SPI1->CR2 &=~SPI_CR2_TXDMAEN;
  DMA1->IFCR = DMA_IFCR_CTCIF3;
  DMA1_Channel3->CCR &=~DMA_CCR1_EN;
  SPI1->CR1 &=~SPI_CR1_SPE;
  lcd_size_8();
}
