# ESP32-S3 ICM42688 Sensor Read Example

A basic example to demonstrate reading data from the ICM42688 6-axis motion sensor (accelerometer + gyroscope) using ESP32-S3 via SPI communication.

## Hardware Configuration
The following GPIO pins are used for SPI communication and interrupt:

| Signal       | GPIO Pin |
|--------------|----------|
| SPI MISO     | 13       |
| SPI MOSI     | 11       |
| SPI SCLK     | 12       |
| SPI CS       | 10       |
| IMU INT Pin  | 14       |

**Physical Connection:**
- Ensure proper 3.3V power supply and ground connections to the ICM42688
- Connect all SPI signals (MISO, MOSI, SCLK, CS) between ESP32-S3 and ICM42688
- Connect interrupt pin if using data-ready interrupts

## Key Features
- SPI initialization and configuration for ESP32-S3
- Basic ICM42688 register read/write operations
- Sensor initialization sequence
- Data readout of accelerometer and gyroscope values
- Interrupt handling configuration (optional)

## Configuration Steps
1. **SPI Bus Setup**:
   ```c
   spi_bus_config_t buscfg = {
       .miso_io_num = SPI_MISO_IO,
       .mosi_io_num = SPI_MOSI_IO,
       .sclk_io_num = SPI_SCLK_IO,
       .quadwp_io_num = -1,
       .quadhd_io_num = -1
   };