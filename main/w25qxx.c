/*
 * w25qxx.c
 *
 *  Created on: 26 Jul 2024
 *      Author: apple
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "esp_spiffs.h"


#include "w25qxx.h"

#define TAG "w25qxx"
#define _DEBUG_	0


#define HOST_ID SPI2_HOST
//#define HOST_ID SPI3_HOST

//static const int SPI_Command_Mode = 0;
//static const int SPI_Data_Mode = 1;
static const int SPI_Frequency = 1000000;



void w25qxx_dump(char *id, int ret, uint8_t *data, int len)
{
	int i;
	printf("[%s] = %d\n",id, ret);
	for(i=0;i<len;i++) {
		printf("%0x ",data[i]);
		if ( (i % 10) == 9) printf("\n");
	}
	printf("\n");
}


void w25qxx_init(w25qxx_t * dev)
{
  ESP_LOGI(TAG, "MISO_GPIO=%d", CONFIG_MISO_GPIO);
  ESP_LOGI(TAG, "MOSI_GPIO=%d", CONFIG_MOSI_GPIO);
  ESP_LOGI(TAG, "SCLK_GPIO=%d", CONFIG_SCLK_GPIO);
  ESP_LOGI(TAG, "CS_GPIO=%d", CONFIG_CS_GPIO);

	esp_err_t ret;

	//gpio_pad_select_gpio( CONFIG_CS_GPIO );
	gpio_reset_pin( CONFIG_CS_GPIO );
	gpio_set_direction( CONFIG_CS_GPIO, GPIO_MODE_OUTPUT );
	gpio_set_level( CONFIG_CS_GPIO, 0 );

	spi_bus_config_t spi_bus_config = {
		.sclk_io_num = CONFIG_SCLK_GPIO,
		.mosi_io_num = CONFIG_MOSI_GPIO,
		.miso_io_num = CONFIG_MISO_GPIO,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1
	};

	ret = spi_bus_initialize( HOST_ID, &spi_bus_config, SPI_DMA_CH_AUTO );
	if(_DEBUG_)ESP_LOGI(TAG, "spi_bus_initialize=%d",ret);
	assert(ret==ESP_OK);

	spi_device_interface_config_t devcfg;
	memset( &devcfg, 0, sizeof( spi_device_interface_config_t ) );
	devcfg.clock_speed_hz = SPI_Frequency;
	devcfg.spics_io_num = CONFIG_CS_GPIO;
	devcfg.queue_size = 7;
	devcfg.mode = 0;

	spi_device_handle_t handle;
	ret = spi_bus_add_device( HOST_ID, &devcfg, &handle);
	if(_DEBUG_)ESP_LOGI(TAG, "spi_bus_add_device=%d",ret);
	assert(ret==ESP_OK);
	dev->_SPIHandle = handle;
	dev->_4bmode = false;
#if CONFIG_4B_MODE
	ESP_LOGW(TAG, "4-Byte Address Mode");
	dev->_4bmode = true;
#endif
}

//
// Get status register 1
// reg1(out):Value of status register 1
//
esp_err_t w25qxx_readStatusReg1(w25qxx_t * dev, uint8_t * reg1)
{
	spi_transaction_t SPITransaction;
	uint8_t data[2];
	data[0] = CMD_READ_STATUS_R1;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if(_DEBUG_)ESP_LOGI(TAG, "w25qxx_readStatusReg1=%x",data[1]);
	*reg1 = data[1];
	return ret;
}

//
// Get status register 2
// reg2(out):Value of status register 2
//
esp_err_t w25qxx_readStatusReg2(w25qxx_t * dev, uint8_t * reg2)
{
	spi_transaction_t SPITransaction;
	uint8_t data[2];
	data[0] = CMD_READ_STATUS_R2;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if(_DEBUG_)ESP_LOGI(TAG, "w25qxx_readStatusReg2=%x",data[1]);
	*reg2 = data[1];
	return ret;
}

//
// Get Unique ID
// id(out):Unique ID 8 bytes	
//
esp_err_t w25qxx_readUniqieID(w25qxx_t * dev, uint8_t * id)
{
	spi_transaction_t SPITransaction;
	uint8_t data[13];
	data[0] = CMD_READ_UNIQUE_ID;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 13 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if(_DEBUG_)w25qxx_dump("readUniqieID", ret, data, 13);
	memcpy(id, &data[5], 8);
	return ret ;
}

//
// Get JEDEC ID(Manufacture, Memory Type,Capacity)
// d(out):Stores 3 bytes of Manufacture, Memory Type, Capacity
//
esp_err_t w25qxx_readManufacturer(w25qxx_t * dev, uint8_t * id)
{
	spi_transaction_t SPITransaction;
	uint8_t data[4];
	data[0] = CMD_JEDEC_ID;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 4 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if(_DEBUG_)w25qxx_dump("readManufacturer", ret, data, 4);
	memcpy(id, &data[1], 3);
	return ret ;
}

//
// Check during processing such as writing
// Return value: true:processing false:idle
//
bool w25qxx_IsBusy(w25qxx_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[2];
	data[0] = CMD_READ_STATUS_R1;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 2 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;
	if( (data[1] & SR1_BUSY_MASK) != 0) return true;
	return false;
}


//
// Power down 
//
esp_err_t w25qxx_powerDown(w25qxx_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[1];
	data[0] = CMD_POWER_DOWN;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 1 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	return ret;
}


//
// Write permission setting
//
esp_err_t w25qxx_WriteEnable(w25qxx_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[1];
	data[0] = CMD_WRITE_ENABLE;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 1 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	return ret;
}


//
// Write-protected setting
//
esp_err_t w25qxx_WriteDisable(w25qxx_t * dev)
{
	spi_transaction_t SPITransaction;
	uint8_t data[1];
	data[0] = CMD_WRITE_DISABLE;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 1 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	return ret;
}

//
// Read data
// addr(in):Read start address
//          3 Bytes Address Mode : 24 Bits 0x000000 - 0xFFFFFF
//          4 Bytes Address Mode : 32 Bits 0x00000000 - 0xFFFFFFFF
// n(in):Number of read data
//
uint16_t w25qxx_read(w25qxx_t * dev, uint32_t addr, uint8_t *buf, uint16_t n)
{ 
	spi_transaction_t SPITransaction;
	uint8_t *data;
	data = (uint8_t *)malloc(n+5);
	size_t offset;
	if (dev->_4bmode) {
		data[0] = CMD_READ_DATA4B;
		data[1] = (addr>>24) & 0xFF; // A31-A24
		data[2] = (addr>>16) & 0xFF; // A23-A16
		data[3] = (addr>>8) & 0xFF; // A15-A08
		data[4] = addr & 0xFF; // A07-A00
		offset = 5;
	} else {
		data[0] = CMD_READ_DATA;
		data[1] = (addr>>16) & 0xFF; // A23-A16
		data[2] = (addr>>8) & 0xFF; // A15-A08
		data[3] = addr & 0xFF; // A07-A00
		offset = 4;
	}
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (n+offset) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	memcpy(buf, &data[offset], n);
	free(data);
	if (ret != ESP_OK) return 0;
	return n;
}

//
// Fast read data
// addr(in):Read start address
//          3 Bytes Address Mode : 24 Bits 0x000000 - 0xFFFFFF
//          4 Bytes Address Mode : 32 Bits 0x00000000 - 0xFFFFFFFF
// n(in):Number of read data
//
uint16_t w25qxx_fastread(w25qxx_t * dev, uint32_t addr, uint8_t *buf, uint16_t n)
{
	spi_transaction_t SPITransaction;
	uint8_t *data;
	data = (uint8_t *)malloc(n+6);
	size_t offset;
	if (dev->_4bmode) {
		data[0] = CMD_FAST_READ4B;
		data[1] = (addr>>24) & 0xFF; // A31-A24
		data[2] = (addr>>16) & 0xFF; // A23-A16
		data[3] = (addr>>8) & 0xFF; // A15-A08
		data[4] = addr & 0xFF; // A07-A00
		data[5] = 0; // Dummy
		offset = 6;
	} else {
		data[0] = CMD_FAST_READ;
		data[1] = (addr>>16) & 0xFF; // A23-A16
		data[2] = (addr>>8) & 0xFF; // A15-A08
		data[3] = addr & 0xFF; // A07-A00
		data[4] = 0; // Dummy
		offset = 5;
	}
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (n+offset) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	esp_err_t ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	memcpy(buf, &data[offset], n);
	free(data);
	if (ret != ESP_OK) return 0;
	return n;
}


//
// Erasing data in 4kb space units
// sect_no(in):Sector number(0 - 2048)
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 30ms and up to 400ms.
// The upper 11 bits of the 23 bits of the address correspond to the sector number.
// The lower 12 bits are the intra-sectoral address.
//
// 補足:
// データシートでは消去に通常 30ms 、最大400msかかると記載されている
// アドレス23ビットのうち上位 11ビットがセクタ番号の相当する。
// 下位12ビットはセクタ内アドレスとなる。
//
bool w25qxx_eraseSector(w25qxx_t * dev, uint16_t sect_no, bool flgwait)
{
	spi_transaction_t SPITransaction;
	uint8_t data[4];
	uint32_t addr = sect_no;
	addr<<=12;

	// Write permission setting
	esp_err_t ret;
	ret = w25qxx_WriteEnable(dev);
	if (ret != ESP_OK) return false;

	data[0] = CMD_SECTOR_ERASE;
	data[1] = (addr>>16) & 0xff;
	data[2] = (addr>>8) & 0xff;
	data[3] = addr & 0xff;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 4 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;

	// Busy check
	while( w25qxx_IsBusy(dev) & flgwait) {
		vTaskDelay(1);
	}
	return true;
}

//
// Erasing data in 64kb space units
// blk_no(in):Block number(0 - 127)
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 150ms and up to 1000ms.
// The upper 7 bits of the 23 bits of the address correspond to the block.
// The lower 16 bits are the address in the block.
//
// 補足:
// データシートでは消去に通常 150ms 、最大1000msかかると記載されている
// アドレス23ビットのうち上位 7ビットがブロックの相当する。下位16ビットはブロック内アドレスとなる。
//
bool w25qxx_erase64Block(w25qxx_t * dev, uint16_t blk_no, bool flgwait)
{
	spi_transaction_t SPITransaction;
	uint8_t data[4];
	uint32_t addr = blk_no;
	addr<<=16;

	// Write permission setting
	esp_err_t ret;
	ret = w25qxx_WriteEnable(dev);
	if (ret != ESP_OK) return false;

	data[0] = CMD_BLOCK_ERASE64KB;
	data[1] = (addr>>16) & 0xff;
	data[2] = (addr>>8) & 0xff;
	data[3] = addr & 0xff;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 4 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;

	// Busy check
	while( w25qxx_IsBusy(dev) & flgwait) {
		vTaskDelay(1);
	}
	return true;
}

//
// Erasing data in 32kb space units
// blk_no(in):Block number(0 - 255)
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 120ms and up to 800ms.
// The upper 8 bits of the 23 bits of the address correspond to the block.
// The lower 15 bits are the in-block address.
//
// 補足:
// データシートでは消去に通常 120ms 、最大800msかかると記載されている
// アドレス23ビットのうち上位 8ビットがブロックの相当する。下位15ビットはブロック内アドレスとなる。
//
bool w25qxx_erase32Block(w25qxx_t * dev, uint16_t blk_no, bool flgwait)
{
	spi_transaction_t SPITransaction;
	uint8_t data[4];
	uint32_t addr = blk_no;
	addr<<=15;

	// Write permission setting
	esp_err_t ret;
	ret = w25qxx_WriteEnable(dev);
	if (ret != ESP_OK) return false;

	data[0] = CMD_BLOCK_ERASE32KB;
	data[1] = (addr>>16) & 0xff;
	data[2] = (addr>>8) & 0xff;
	data[3] = addr & 0xff;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 4 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;

	// Busy check
	while( w25qxx_IsBusy(dev) & flgwait) {
		vTaskDelay(1);
	}
	return true;
}


//
// Erase all data
// flgwait(in):true:Wait for complete / false:No wait for complete
// Return value: true:success false:fail
//
// Note:
// The data sheet states that erasing usually takes 15s and up to 30s.
//
// 補足:
// データシートでは消去に通常 15s 、最大30sかかると記載されている
//
bool w25qxx_eraseAll(w25qxx_t * dev, bool flgwait)
{
	spi_transaction_t SPITransaction;
	uint8_t data[1];

	// Write permission setting
	esp_err_t ret;
	ret = w25qxx_WriteEnable(dev);
	if (ret != ESP_OK) return false;

	data[0] = CMD_CHIP_ERASE;
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = 1 * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return false;

	// Busy check
	while( w25qxx_IsBusy(dev) & flgwait) {
		vTaskDelay(1);
	}
	return true;
}

//
// Page write
// sect_no(in):Sector number(0x00 - 0x7FF) 
// inaddr(in):In-sector address(0x00-0xFFF)
// data(in):Write data
// n(in):Number of bytes to write(0～256)
//
int16_t w25qxx_pageWrite(w25qxx_t * dev, uint16_t sect_no, uint16_t inaddr, uint8_t* buf, int16_t n)
{
	if (n > 256) return 0;
	spi_transaction_t SPITransaction;
	uint8_t *data;

	uint32_t addr = sect_no;
	addr<<=12;
	addr += inaddr;

	// Write permission setting
	esp_err_t ret;
	ret = w25qxx_WriteEnable(dev);
	if (ret != ESP_OK) return 0;

	// Busy check
	if (w25qxx_IsBusy(dev)) return 0;  

	data = (unsigned char*)malloc(n+4);
	data[0] = CMD_PAGE_PROGRAM;
	data[1] = (addr>>16) & 0xff;
	data[2] = (addr>>8) & 0xff;
	data[3] = addr & 0xFF;
	memcpy( &data[4], buf, n );
	memset( &SPITransaction, 0, sizeof( spi_transaction_t ) );
	SPITransaction.length = (n+4) * 8;
	SPITransaction.tx_buffer = data;
	SPITransaction.rx_buffer = data;
	ret = spi_device_transmit( dev->_SPIHandle, &SPITransaction );
	free(data);
	assert(ret==ESP_OK);
	if (ret != ESP_OK) return 0;

	// Busy check
	while( w25qxx_IsBusy(dev) ) {
		vTaskDelay(1);
	}
	return n;
}



void dump(uint8_t *dt, int n)
{
	uint16_t clm = 0;
	uint8_t data;
	uint8_t sum;
	uint8_t vsum[16];
	uint8_t total =0;
	uint32_t saddr =0;
	uint32_t eaddr =n-1;

	printf("----------------------------------------------------------\n");
	uint16_t i;
	for (i=0;i<16;i++) vsum[i]=0;  
	uint32_t addr;
	for (addr = saddr; addr <= eaddr; addr++) {
		data = dt[addr];
		if (clm == 0) {
			sum =0;
			printf("%05"PRIx32": ",addr);
		}

		sum+=data;
		vsum[addr % 16]+=data;

		printf("%02x ",data);
		clm++;
		if (clm == 16) {
			printf("|%02x \n",sum);
			clm = 0;
		}
	}
	printf("----------------------------------------------------------\n");
	printf("       ");
	for (i=0; i<16;i++) {
		total+=vsum[i];
		printf("%02x ",vsum[i]);
	}
	printf("|%02x \n\n",total);
}


void write_text_to_flash(w25qxx_t *dev, uint32_t start_addr, const char *text) {
    // Calculate the length of the text data
    size_t data_len = strlen(text);
    
    // Calculate the number of pages required
    size_t page_size = 256;
    size_t total_pages = (data_len + page_size - 1) / page_size;

    // Erase required sectors
    size_t sector_size = 4096;
    size_t total_sectors = (data_len + sector_size - 1) / sector_size;
    for (size_t sector = 0; sector < total_sectors; sector++) {
        uint32_t sector_addr = start_addr + sector * sector_size;
        if (!w25qxx_eraseSector(dev, sector_addr, true)) {
            ESP_LOGE(TAG, "Failed to erase sector at address ");
            return;
        }
    }

    // Write the data page by page
    for (size_t page = 0; page < total_pages; page++) {
        uint32_t page_addr = start_addr + page * page_size;
        size_t bytes_to_write = (data_len > page_size) ? page_size : data_len;
        if (w25qxx_pageWrite(dev, page_addr,0, (uint8_t *)(text + page * page_size), bytes_to_write) != bytes_to_write) {
            ESP_LOGE(TAG, "Failed to write data at address ");
            return;
        }
        data_len -= bytes_to_write;
    }

    ESP_LOGI(TAG, "Successfully wrote text data to flash memory");
}

// Function to read text data from flash memory
void read_text_from_flash(w25qxx_t *dev, uint32_t start_addr, char *buffer, size_t buffer_len) {
    // Calculate the length of the text data
    size_t data_len = buffer_len - 1; // Leave space for null terminator

    // Calculate the number of pages required
    size_t page_size = 256;
    size_t total_pages = (data_len + page_size - 1) / page_size;

    // Read the data page by page
    for (size_t page = 0; page < total_pages; page++) {
        uint32_t page_addr = start_addr + page * page_size;
        size_t bytes_to_read = (data_len > page_size) ? page_size : data_len;
        if (w25qxx_read(dev, page_addr, (uint8_t *)(buffer + page * page_size), bytes_to_read) != bytes_to_read) {
            ESP_LOGE(TAG, "Failed to read data at address ");
            return;
        }
        data_len -= bytes_to_read;
    }

    buffer[buffer_len - 1] = '\0'; // Null terminate the buffer
    ESP_LOGI(TAG, "Successfully read text data from flash memory");
}

esp_err_t write_audio_data_to_flash(w25qxx_t *dev, uint32_t start_addr, uint8_t *audio_data, size_t data_len) {
    size_t page_size = 256;
    size_t pages_per_sector = 4096 / page_size;
    
    // Erase the sector first
    uint32_t sector_addr = start_addr;
   if (!w25qxx_eraseSector(dev, sector_addr, true)) {
        ESP_LOGE(TAG, "Failed to erase sector at address %" PRIu32 "\n", sector_addr);
        return ESP_FAIL;
    }
    
    // Write the data page by page within the current sector
    for (size_t page = 0; page < pages_per_sector; page++) {
        uint32_t page_addr = page * page_size;
        size_t bytes_to_write = page_size;

        // Writing data to the page
        if (w25qxx_pageWrite(dev, sector_addr >> 12, page_addr, audio_data + (page * page_size), bytes_to_write) != bytes_to_write) {
            ESP_LOGE(TAG, "Failed to write data at page %d of sector %" PRIu32 "\n", page, sector_addr >> 12);
            return ESP_FAIL;
        }
        
        //ESP_LOGI(TAG, "Data written successfully at page %d of sector %" PRIu32 "\n", page, sector_addr >> 12);
    }
       
    return ESP_OK;
}



esp_err_t read_audio_data_from_flash(w25qxx_t *dev, uint32_t start_addr, size_t data_len) {
    size_t page_size = 256;
    size_t total_pages = (data_len + page_size - 1) / page_size;

    for (size_t page = 0; page < total_pages; page++) {
        uint32_t page_addr = start_addr + page * page_size;
        uint8_t buffer[page_size];
        memset(buffer, 0, page_size);

        int len = w25qxx_read(dev, page_addr, buffer, page_size);
        if (len != page_size) {
            ESP_LOGE(TAG, "Failed to read data at address %" PRIu32, page_addr);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Read Data: addr = %" PRIu32 " (Page %d)\n", page_addr, page);
        dump(buffer, page_size);
    }

    ESP_LOGI(TAG, "Successfully read audio data from flash memory");
    return ESP_OK;
}
