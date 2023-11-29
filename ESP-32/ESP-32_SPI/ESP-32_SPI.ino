void spi_init(SPIClass * spi);
void spi_Tx(SPIClass *spi, byte data);
void spi_Rx(SPIClass *spi);


static const int spiClk = 1000000; // 1 MHz
SPIClass * Input_SPI = NULL; //uninitalised pointers to Input_SPI objects
#define SCK 5
#define MOSI 18
#define MISO 19
#define SS 4

void spi_init(SPIClass * spi)
{
  spi->begin(SCK, MISO, MOSI, SS);
  //attachInterrupt(SS, spi_interupt, FALLING);
  pinMode(spi->pinSS(), OUTPUT);
}

void spi_Tx(SPIClass *spi, byte data)
{
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  digitalWrite(spi->pinSS(), LOW); //pull SS slow to prep other end for transfer
  spi->transfer(data);
  digitalWrite(spi->pinSS(), HIGH); //pull ss high to signify end of data transfer
  spi->endTransaction();
}

void spi_Rx(SPIClass *spi)
{
  spi->beginTransaction(SPISettings(spiClk, MSBFIRST, SPI_MODE0));
  byte data = spi->transfer(NULL);
  spi->endTransaction();
  esp_task_wdt_reset();
  write_mqtt(mqttClient, send_topic, data);
}

void spi_interupt()
{
  spi_Rx(Input_SPI);
}
