/*
 * w25qxx.h
 *
 *  Created on: 26 Jul 2024
 *      Author: apple
 */

#ifndef MAIN_W25QXX_H_
#define MAIN_W25QXX_H_



#include "driver/spi_master.h"


#define CONFIG_MISO_GPIO     19
#define CONFIG_MOSI_GPIO     23
#define CONFIG_SCLK_GPIO     18
#define CONFIG_CS_GPIO       5


#define CMD_WRITE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS_R1    0x05
#define CMD_READ_STATUS_R2    0x35
#define CMD_WRITE_STATUS_R    0x01 // Unimplemented
#define CMD_PAGE_PROGRAM      0x02
#define CMD_QUAD_PAGE_PROGRAM 0x32 // Unimplemented
#define CMD_BLOCK_ERASE64KB   0xd8
#define CMD_BLOCK_ERASE32KB   0x52
#define CMD_SECTOR_ERASE      0x20
#define CMD_CHIP_ERASE        0xC7
#define CMD_ERASE_SUPPEND     0x75 // Unimplemented
#define CMD_ERASE_RESUME      0x7A // Unimplemented
#define CMD_POWER_DOWN        0xB9
#define CMD_HIGH_PERFORM_MODE 0xA3 // Unimplemented
#define CMD_CNT_READ_MODE_RST 0xFF // Unimplemented
#define CMD_RELEASE_PDOWN_ID  0xAB // Unimplemented
#define CMD_MANUFACURER_ID    0x90
#define CMD_READ_UNIQUE_ID    0x4B
#define CMD_JEDEC_ID          0x9f

#define CMD_READ_DATA         0x03
#define CMD_READ_DATA4B       0x13
#define CMD_FAST_READ         0x0B
#define CMD_FAST_READ4B       0x0C
#define CMD_READ_DUAL_OUTPUT  0x3B // Unimplemented
#define CMD_READ_DUAL_IO      0xBB // Unimplemented
#define CMD_READ_QUAD_OUTPUT  0x6B // Unimplemented
#define CMD_READ_QUAD_IO      0xEB // Unimplemented
#define CMD_WORD_READ         0xE3 // Unimplemented

#define SR1_BUSY_MASK	0x01
#define SR1_WEN_MASK	0x02

typedef struct {
	bool _4bmode;
	spi_device_handle_t _SPIHandle;
} w25qxx_t;




void w25qxx_dump(char *id, int ret, uint8_t *data, int len);
void w25qxx_init(w25qxx_t * dev);
esp_err_t w25qxx_readStatusReg1(w25qxx_t * dev, uint8_t * reg1);
esp_err_t w25qxx_readStatusReg2(w25qxx_t * dev, uint8_t * reg2);
esp_err_t w25qxx_readUniqieID(w25qxx_t * dev, uint8_t * id);
esp_err_t w25qxx_readManufacturer(w25qxx_t * dev, uint8_t * id);
bool w25qxx_IsBusy(w25qxx_t * dev);
esp_err_t w25qxx_powerDown(w25qxx_t * dev);
esp_err_t w25qxx_WriteEnable(w25qxx_t * dev);
esp_err_t w25qxx_WriteDisable(w25qxx_t * dev);
uint16_t w25qxx_read(w25qxx_t * dev, uint32_t addr, uint8_t *buf, uint16_t n);
uint16_t w25qxx_fastread(w25qxx_t * dev, uint32_t addr, uint8_t *buf, uint16_t n);
bool w25qxx_eraseSector(w25qxx_t * dev, uint16_t sect_no, bool flgwait);
bool w25qxx_erase64Block(w25qxx_t * dev, uint16_t blk_no, bool flgwait);
bool w25qxx_erase32Block(w25qxx_t * dev, uint16_t blk_no, bool flgwait);
bool w25qxx_eraseAll(w25qxx_t * dev, bool flgwait);
int16_t w25qxx_pageWrite(w25qxx_t * dev, uint16_t sect_no, uint16_t inaddr, uint8_t* buf, int16_t n);
void dump(uint8_t *dt, int n);
void write_text_to_flash(w25qxx_t *dev, uint32_t start_addr, const char *text) ;
void read_text_from_flash(w25qxx_t *dev, uint32_t start_addr, char *buffer, size_t buffer_len);
esp_err_t write_audio_data_to_flash(w25qxx_t *dev, uint32_t start_addr, uint8_t *audio_data, size_t data_len);
esp_err_t read_audio_data_from_flash(w25qxx_t *dev, uint32_t start_addr, size_t data_len);


#endif /* MAIN_W25QXX_H_ */
