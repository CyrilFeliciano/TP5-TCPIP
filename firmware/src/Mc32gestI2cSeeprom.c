//--------------------------------------------------------
// Mc32gestI2cEEprom.C
//--------------------------------------------------------
// Gestion I2C de la SEEPROM du MCP79411 (Solution exercice)
//	Description :	Fonctions pour EEPROM MCP79411
//
//	Auteur 		: 	C. HUBER
//      Date            :       26.05.2014
//	Version		:	V1.0
//	Compilateur	:	XC32 V1.31
// Modifications :
//
/*--------------------------------------------------------*/



#include "Mc32gestI2cSeeprom.h"
#include "Mc32_I2cUtilCCS.h"




// Definitions du bus (pour mesures)
// #define I2C-SCK  SCL2/RA2      PORTAbits.RA2   pin 58
// #define I2C-SDA  SDa2/RA3      PORTAbits.RA3   pin 59




// Initialisation de la communication I2C et du MCP79411
// ------------------------------------------------

void I2C_InitMCP79411(void)
{
   bool Fast = true;
   i2c_init( Fast );
} //end I2C_InitMCP79411

// Ecriture d'un bloc dans l'EEPROM du MCP79411 
void I2C_WriteSEEPROM(void *SrcData, uint32_t EEpromAddr, uint16_t NbBytes)
{
    uint8_t i = 0;  // Variable pour la boucle de la page
    uint8_t y = 0;  // Variable pour la boucle du nombre de page
    uint8_t *i2cData = SrcData; // Pointeur qui prend l'adresse de SrcData pour le prendre comme un tableau
    uint8_t NbBytesPage = 0;    // Nombre de bytes dans la page
    // Boucle qui s'exécute pour chaque page
    for(y = 0; y <= (NbBytes/8); y++)
    {
        // Test si on est à la dernière page
        if(y == (NbBytes/8))
        {
            // La boucle de la page s'exécute le nombre de fois qu'il y a à écrire
            NbBytesPage = NbBytes - 8*(y);
        }
        else
        {
            // La boucle de la page s'exécute 8 fois
            NbBytesPage = 8;
        }
        // Test si on peut écrire (ACK)
        do
        {
            i2c_start();
        }while(!i2c_write(MCP79411_EEPROM_W));
        // Écriture de l'adresse
        i2c_write((uint8_t)EEpromAddr + (y * 8));
        // Boucle d'écriture d'une page
        for(i = 0; i < NbBytesPage; i++)
        {
           i2c_write(i2cData[i+(y*8)]);
        }
        i2c_stop(); // Stop après chaque page
    }
} // end I2C_WriteSEEPROM



// Lecture d'un bloc dans l'EEPROM du MCP79411
void I2C_ReadSEEPROM(void *DstData, uint32_t EEpromAddr, uint16_t NbBytes)
{
    uint8_t i = 0;
    uint8_t *i2cData = DstData;
    // Test si on peut écrire (ACK)
    do
    {
        i2c_start();
    }while(!i2c_write(MCP79411_EEPROM_W));
    // Écriture de l'adresse
    i2c_write((uint8_t)EEpromAddr);
    // Restart
    i2c_reStart();
    // Demande de lecture
    i2c_write(MCP79411_EEPROM_R);
    // Lecture du nombre de Bytes demandés
    for(i = 0; i < NbBytes - 1; i++)
    {
        i2cData[i] = i2c_read(1);
    }
    // Finir par une lecture sans ACK
   i2cData[i] = i2c_read(0);
   i2c_stop();  // Arret
} // end I2C_ReadSEEPROM
   





 



