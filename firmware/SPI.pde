//***********************************************************************//
// SPI slave code adapted from:                                          //
// http://dorkbotpdx.org/blog/feurig/arduino_code_for_thought_spi_slave  //
//***********************************************************************//

/*
* Based on Atmel application note avr151.
* Donald Delmar Davis, Tempus Dictum, Inc.
*/

#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SS 10

void SPI_Init(void);
unsigned char SPI_ReadWrite(unsigned char data);
unsigned char SPI_Read(void);

#define SPI_DONTCARE (0x00)
unsigned char theByte = SPI_DONTCARE;

// init as SPI-Master
void SPI_MASTER_Init(void) {
  pinMode(SPI_SCK,OUTPUT);
  pinMode(SPI_MOSI,OUTPUT);
  pinMode(SPI_SS,OUTPUT);
  pinMode(SPI_MISO,INPUT);
  digitalWrite(SPI_SS, HIGH);
  // INIT interface, Master, set clock rate fck/4
  SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR0)|(0<<SPR1);
  // enable double rate
  //SPSR = (1<<SPI2X); // we will now gain fck/2 instead of fck/4
}

void SPI_SLAVE_Init(void) {
  pinMode(SPI_SCK,INPUT);
  pinMode(SPI_MOSI,INPUT);
  pinMode(SPI_SS,INPUT);
  pinMode(SPI_MISO,OUTPUT);

  // INIT interface, Master, set clock rate fck/4
  SPCR = (1<<SPE)|(0<<SPR0)|(0<<SPR1);
  // enable double rate
  //SPSR = (1<<SPI2X); // we will now gain fck/2 instead of fck/4
  // clear SPI interrupt flag by reading SPSR and SPDR
  theByte = SPSR;
  theByte = SPDR;
}

unsigned char SPI_ReadWrite(unsigned char data) {
  // set data to send into SPI data register
  SPDR = data;
  // Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));
  // return data read from SPI (if any)
  return SPDR;
}

unsigned char SPI_Read(void) {
return SPI_ReadWrite(SPI_DONTCARE);
}

// USAGE:
/*
void setup() {
  SPI_SLAVE_Init();
}

void loop(){
  digitalWrite(2,theByte & (1<<0));
  digitalWrite(3,theByte & (1<<1));
  digitalWrite(4,theByte & (1<<2));
  digitalWrite(5,theByte & (1<<3));
  digitalWrite(6,theByte & (1<<4));
  digitalWrite(7,theByte & (1<<5));
  digitalWrite(8,theByte & (1<<6));
  digitalWrite(9,theByte & (1<<7));

  theByte=SPI_ReadWrite(theByte);
}
*/
