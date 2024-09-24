// Definizioni Pin
#define PINB_COMMAND_DATA 1
#define PINB_MOSI 3
#define PINB_SCK 5
#define PINB_SS 2
//
#define RED    0xF800

#define ILI9341_CASET  0x2A   // Column Address Set
#define ILI9341_PASET  0x2B   // Page Address Set
#define ILI9341_RAMWR  0x2C   // Memory Write
#define ILI9341_PWCTR1 0xC0 ///< Power Control 1
#define ILI9341_PWCTR2 0xC1 ///< Power Control 2
#define ILI9341_VMCTR1 0xC5 ///< VCOM Control 1
#define ILI9341_VMCTR2 0xC7 ///< VCOM Control 2
#define ILI9341_MADCTL 0x36   ///< Memory Access Control
#define ILI9341_VSCRSADD 0x37 ///< Vertical Scrolling Start Address
#define ILI9341_PIXFMT 0x3A   ///< COLMOD: Pixel Format Set
#define ILI9341_FRMCTR1 0xB1 ///< Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_DFUNCTR 0xB6 ///< Display Function Control
#define ILI9341_GAMMASET 0x26 ///< Gamma Set
#define ILI9341_GMCTRP1 0xE0 ///< Positive Gamma Correction
#define ILI9341_GMCTRN1 0xE1 ///< Negative Gamma Correction
#define ILI9341_SLPOUT 0x11 ///< Sleep Out
#define ILI9341_DISPON 0x29   ///< Display ON
#define LUNGHEZZA 320
#define LARGHEZZA 240
//Vettore dei comandi per l'inizializzazione dello schermo
static const uint8_t PROGMEM initcmd[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  ILI9341_PWCTR1  , 1, 0x23,             // Power control VRH[5:0]
  ILI9341_PWCTR2  , 1, 0x10,             // Power control SAP[2:0];BT[3:0]
  ILI9341_VMCTR1  , 2, 0x3e, 0x28,       // VCM control
  ILI9341_VMCTR2  , 1, 0x86,             // VCM control2
  ILI9341_MADCTL  , 1, 0x48,             // Memory Access Control
  ILI9341_VSCRSADD, 1, 0x00,             // Vertical scroll zero
  ILI9341_PIXFMT  , 1, 0x55,
  ILI9341_FRMCTR1 , 2, 0x00, 0x18,
  ILI9341_DFUNCTR , 3, 0x08, 0x82, 0x27, // Display Function Control
  0xF2, 1, 0x00,                         // 3Gamma Function Disable
  ILI9341_GAMMASET , 1, 0x01,             // Gamma curve selected
  ILI9341_GMCTRP1 , 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, // Set Gamma
    0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  ILI9341_GMCTRN1 , 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, // Set Gamma
    0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  ILI9341_SLPOUT  ,1, 0x80,                // Exit Sleep
  ILI9341_DISPON  ,2, 0x80,                // Display on
  0x00                                   // End of list
};
// Termine definizioni

void setup() {
    SPI_MasterOpen(16);  // Inizializza SPI con divisore di clock 16
    ILI9341_Init();     // Inizializza il display
    ILI_drawPixelAtCenter();
    ILI_drawPixel(120,160,RED);
}

/* Metodo per inizializzare l'interfaccia SPI**/
void SPI_MasterOpen(uint8_t clkdiv){
  //Configurazione MOSI, SCK, SS 
  DDRB = (1 << DDB3) | (1<<DDB5) | (1 << DDB2);
  //Abilitazione SPI , configurazione mcu come Master,
  // campionamento fronte di salita ,idle clock=basso)
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPHA) | (0 << CPOL);
  // Gestione del divisore di clock
  if (clkdiv == 2 || clkdiv == 8 || clkdiv == 32) {
    SPSR |= (1 << SPI2X);  // Imposta SPI2X per divisori con doppia velocità
  } else {
    SPSR &= ~(1 << SPI2X);  // Disabilita SPI2X per gli altri divisori
  }
  // Gestione dei bit SPR1 e SPR0 per i restatni divisori
  switch (clkdiv) {
    case 2:  // 2 = SPI2X = 1, SPR1 = 0, SPR0 = 0
    case 4:  // 4 = SPI2X = 0, SPR1 = 0, SPR0 = 0
      SPCR &= ~((1 << SPR1) | (1 << SPR0));  // SPR1 = 0, SPR0 = 0
      break;
    case 8:  // 8 = SPI2X = 1, SPR1 = 0, SPR0 = 1
    case 16: // 16 = SPI2X = 0, SPR1 = 0, SPR0 = 1
      SPCR = (SPCR & ~(1 << SPR1)) | (1 << SPR0);  // SPR1 = 0, SPR0 = 1
      break;
    case 32: // 32 = SPI2X = 1, SPR1 = 1, SPR0 = 0
    case 64: // 64 = SPI2X = 0, SPR1 = 1, SPR0 = 0
      SPCR = (SPCR & ~(1 << SPR0)) | (1 << SPR1);  // SPR1 = 1, SPR0 = 0
      break;
    case 128: // 128 = SPI2X = 0, SPR1 = 1, SPR0 = 1
      SPCR |= (1 << SPR1) | (1 << SPR0);  // SPR1 = 1, SPR0 = 1
      break;
    default:
      // Default a 4 se clkdiv non è valido
      SPCR &= ~((1 << SPR1) | (1 << SPR0));  // SPR1 = 0, SPR0 = 0
      SPSR &= ~(1 << SPI2X);  
      break;
  }
}

/**Metodo per trasmettere un dato senza attendere il completamento della trasmissione**/
void SPI_MasterTx(uint8_t data){
  SPDR = data;
}

/** Metodo per trasmettere un dato con attesa termine trasmissione(POLLING)*/
void masterTransmit(uint8_t data) {
  SPDR = data;
  while(!(SPSR & (1 << SPIF)));
}

/** Metodo restituisce true se la trasmissione è stata completata*/
boolean SPI_MasterReady(){
  return SPSR & (1 << SPIF);  // Verifica se SPIF è settato a 1; SPIF = SPI Interrupt flag
 }



/**Inizializzazione schermo**/
void ILI9341_Init() {
  // Invia la sequenza di inizializzazione 
  ILI_sendCmd(0x01); // Reset dello schermo poichè non fornito nella simulazione da wokwi
  delay(200);
  uint8_t cmd, x, numArgs;          // Comando, ritardo(settimo bit), numero argomenti comando
  const uint8_t *addr = initcmd;    // Puntatore ai comandi per l'inizializzazione dello schermo
  uint8_t *dataAddr;                //Puntatore temporaneo per i dati associati al comando
  while ((cmd = pgm_read_byte(addr++)) > 0) { // fino a quando ci sono dei comandi
    //Serial.print("Comando: ");
    //Serial.println(cmd,HEX);
    
    dataAddr = addr; 
    x = pgm_read_byte(addr++);          
    numArgs = x & 0x7F;                 // estrai il numero di argomenti associati al comando
    sendCommand(cmd, dataAddr+1, numArgs);
    addr += numArgs;
    if (x & 0x80)
      delay(150);
  }
 
}


/* Tale metodo serve per inviare un comando che ha piu dati ad esso associati*/
void sendCommand(uint8_t cmd, uint8_t *dati, uint8_t numDati){ // 1 dato = byte
 if ((1 << PINB_SS) & PINB){      // Se il pin è alto 
    PORTB |= (1 << 2);            // Seleziona lo slave
  }
    
  ILI_sendCmd(cmd);             //Invia il comando
  if(cmd == 0x11 || cmd == 0x29){
      delay(200);             // è necessario un ritardo dopo il comando di sleep out 
    }                         // e del comando display on
      
  for (int i = 0; i < numDati; i++) {
      ILI_sendData(pgm_read_byte(dati)); // Send the data bytes
      dati++;
    }

}

/*Metodo per inviare un comando*/
void ILI_sendCmd(uint8_t cmd) {
  PORTB &= ~((1 << PINB_COMMAND_DATA) | (1 << PINB_SS ));  // Attivo la linea comando, e seleziono lo slave
  masterTransmit(cmd);
  PORTB |= (1 << PINB_SS);
}


/*Metodo per inviare un dato associato al comando*/
void ILI_sendData(uint8_t data) {
  PORTB |= (1 << PINB_COMMAND_DATA);   // Modalità dati
  PORTB &= (~(1 << PINB_SS));          // Linea SS bassa seleziono lo slave
  masterTransmit(data);
  PORTB |= (1 << PINB_SS);
}


/**Metodo per trasmettere un dato da 16 bit allo schemro**/
void ILI_sendData16(uint16_t data){
  PORTB |= (1 << PINB_COMMAND_DATA);  // Modalità dati
  PORTB &= (~( 1 << PINB_SS));        // Linea SS bassa seleziono lo slave
  ILI_sendData(data >> 8);             // Invio gli 8 bit piu significativi
  ILI_sendData(data & 0xFF);             // Invio gli 8 bit meno significativi
}



// Funzione per impostare una finestra di disegno (area) sul display
void ILI_setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  // Imposta la finestra Y (colonne)
  ILI_sendCmd(ILI9341_CASET);     // Column Address Set
  ILI_sendData16(x0);             // Inizio colonna
  ILI_sendData16(x1);             // Fine colonna
  // Imposta la finestra X (righe)
  ILI_sendCmd(ILI9341_PASET);     // Page Address Set
  ILI_sendData16(y0);             // Inizio riga
  ILI_sendData16(y1);             // Fine riga
  
}

// Funzione per disegnare un pixel in una posizione specifica
void ILI_drawPixel(uint16_t x, uint16_t y, uint16_t color) {
  // Imposta la finestra di disegno come un singolo pixel
  ILI_setAddrWindow(x, y, x+1, y+1);
  // Invia il comando per iniziare a scrivere in memoria
  ILI_sendCmd(ILI9341_RAMWR);
  // Invia il colore del pixel
  ILI_sendData16(color);
}

// Funzione per disegnare un pixel al centro dello schermo
void ILI_drawPixelAtCenter() {
  uint16_t centerY = LUNGHEZZA / 2;   
  uint16_t centerX = LARGHEZZA / 2;     

  // Disegna un pixel rosso al centro dello schermo
  ILI_drawPixel(centerX, centerY, RED);
}






void loop() {
 
  }
