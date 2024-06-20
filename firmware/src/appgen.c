/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    appgen.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "appgen.h"
#include "Mc32DriverLcd.h"
#include "Mc32gestI2cSeeprom.h"
#include "GesPec12.h"
#include "MenuGen.h"
#include "Generateur.h"
#include "Mc32gest_SerComm.h"
#include "app.h"
#include "Mc32gestSpiDac.h"
#include "driver/tmr/drv_tmr_static.h" //driver timer static

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APPGEN_DATA appgenData;



// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APPGEN_Initialize ( void )

  Remarks:
    See prototype in appgen.h.
 */

void APPGEN_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appgenData.state = APPGEN_STATE_INIT;
    
    appgenData.newIp = false;

    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APPGEN_Tasks ( void )

  Remarks:
    See prototype in appgen.h.
 */

void APPGEN_Tasks ( void )
{
     S_ParamGen structInterAPP;    //Déclaration de la variable accueillant temporairement les valeurs de pParam
     static bool changementDebranchement = false;
     static uint16_t attenteAdrrIP = 0;
     
    /* Check the application's current state. */
    switch ( appgenData.state )
    {
        
        /* Application's initial state. */
        case APPGEN_STATE_INIT:
        {
            //Initialisation du LCD
            lcd_init();
            //Allumage de la backlight du LCD
            lcd_bl_on();
            //Initialisation du bus I2C
            I2C_InitMCP79411();
            // Init SPI DAC
            SPI_InitLTC2604();
            //Appel de la fonction d'initialisation du PEC12
            Pec12Init();
            //Appel de la fonction d'initialisation du menu
            structInterAPP = MENU_Initialize(&LocalParamGen);
            //Appel de la fonction d'initialisation de gensig
            GENSIG_Initialize(&LocalParamGen,&structInterAPP);
            //Appel de la fonction changeant l'amplitude la forme et l'offset du signal
            GENSIG_UpdateSignal(&LocalParamGen);
            //Appel de la fonction changeant la fréquence du signal
            GENSIG_UpdatePeriode(&LocalParamGen);
            //Mise des deux structures aux mêmes valeurs
            RemoteParamGen = LocalParamGen; 
            //Démarrage des timers
            DRV_TMR1_Start();
            DRV_TMR0_Start();
            //Mise de l'état de la machine d'état en mode APPGEN_STATE_WAIT
            APP_Gen_UpdateState(APPGEN_STATE_WAIT);
            
            break;
        }

        case APPGEN_STATE_SERVICE_TASKS:
        {
            // Si il y a une nouvelle adresse IP
            if(appgenData.newIp)
            {
                // Incrémente le compteur de temps d'attente pour la nouvelle adresse IP
                attenteAdrrIP++;   
                // Allume le rétroéclairage de l'écran LCD
                lcd_bl_on();
                // Positionne le curseur sur la ligne 2, colonne 6 de l'écran LCD
                lcd_gotoxy(6, 2);
                // Affiche "Adr. IP" sur l'écran LCD
                printf_lcd("Adr. IP");
                // Positionne le curseur sur la ligne 3, colonne 1 de l'écran LCD
                lcd_gotoxy(1, 3);
                // Affiche la nouvelle adresse IP stockée dans appgenData.str sur l'écran LCD
                lcd_put_string_ram(appgenData.str);
                // Réinitialise la durée d'inactivité de Pec12
                Pec12.InactivityDuration = 0;

                // Si le temps d'attente pour la nouvelle adresse IP dépasse une certaine limite
                if(attenteAdrrIP > NOUVELLE_ADRESSE_AFFICHAGE)
                {
                    // Réinitialise le drapeau indiquant une nouvelle adresse IP
                    appgenData.newIp = false;
                    // Sélectionne le mode de menu avec les paramètres locaux
                    MENU_SelectMode(&LocalParamGen, true);
                    // Réinitialise le compteur de temps d'attente
                    attenteAdrrIP = 0;
                }
            }
            else
            {
                // Bascule l'état de la LED BSP_LED_2 pour indiquer le temps d'interruption
                BSP_LEDToggle(BSP_LED_2);

                // Si la connexion du client TCP est établie
                if(etatTCPIP == true)
                {
                    // Allume le rétroéclairage de l'écran LCD
                    lcd_bl_on();                    
                    // Indique un changement de débranchement
                    changementDebranchement = true;
                    // Met à jour les signaux avec les paramètres distants
                    GENSIG_UpdateSignal(&RemoteParamGen);
                    // Met à jour la période avec les paramètres distants
                    GENSIG_UpdatePeriode(&RemoteParamGen);

                    // Si une demande de sauvegarde a été effectuée, afficher le menu de sauvegarde
                    if(tcpStatSave == true)
                    {
                        MENU_DemandeSave();
                    }
                    else
                    {
                        // Exécute le menu avec les paramètres distants
                        MENU_Execute(&RemoteParamGen, false);
                    }
                }
                else  // Si la connexion du client TCP n'est pas établie, mettre à jour les signaux avec les paramètres locaux
                {
                    // Si un changement de débranchement a été détecté
                    if(changementDebranchement == true)
                    {
                        // Sélectionne le mode de menu avec les paramètres locaux
                        MENU_SelectMode(&LocalParamGen, 1);
                        // Réinitialise le drapeau de changement de débranchement
                        changementDebranchement = false;
                    } 

                    // Exécute le menu avec les paramètres locaux
                    MENU_Execute(&LocalParamGen, true);
                    // Met à jour les signaux avec les paramètres locaux
                    GENSIG_UpdateSignal(&LocalParamGen);
                    // Met à jour la période avec les paramètres locaux
                    GENSIG_UpdatePeriode(&LocalParamGen);
                }     
            }
            // Met à jour l'état de l'application pour attendre
            APP_Gen_UpdateState(APPGEN_STATE_WAIT);
            break;
        }
        
        case APPGEN_STATE_WAIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}


// *****************************************************************************
// Fonction :
//    void APPGEN_DispNewAddress(IPV4_ADDR ipAddr)
//
// Résumé :
//    Affiche une nouvelle adresse IP sur l'écran LCD.
//
// Description :
//    Cette fonction met à jour l'affichage de l'écran LCD avec une nouvelle
//    adresse IP fournie en paramètre. Elle définit également un drapeau pour
//    indiquer qu'il y a une nouvelle adresse IP. Toutes les lignes de l'écran
//    LCD sont effacées avant d'afficher la nouvelle adresse.
//
// Paramètres :
//    - ipAddr : Nouvelle adresse IP de type IPV4_ADDR à afficher.
//
// Retourne :
//    - Rien (void).
// *****************************************************************************

void APPGEN_DispNewAddress (IPV4_ADDR ipAddr)
{
    // Déclare une variable pour compter les lignes à effacer sur l'écran LCD
    uint8_t compteurClearLine;
    // Définit le drapeau indiquant qu'il y a une nouvelle adresse IP à afficher
    appgenData.newIp = true;
    // Boucle pour effacer chaque ligne de l'écran LCD
    for(compteurClearLine = 1; compteurClearLine <= MAX_NBR_LINE; compteurClearLine++)
    {
        // Efface la ligne actuelle de l'écran LCD
        lcd_ClearLine(compteurClearLine);
    }
    // Formate l'adresse IP dans une chaîne de caractères et la stocke dans appgenData.str
    sprintf(appgenData.str, "IP : %d.%d.%d.%d", ipAddr.v[0], ipAddr.v[1], ipAddr.v[2], ipAddr.v[3]);
}

// *****************************************************************************
// Fonction :
//    void APP_Gen_UpdateState(APP_GEN_STATES NewState)
//
// Résumé :
//    Met à jour l'état de l'application avec une nouvelle valeur.
//
// Description :
//    Cette fonction met à jour l'état de l'application avec la nouvelle valeur
//    fournie. La mise à jour est effectuée directement sur la variable d'état
//    globale `app_genData.state`.
//
// Paramètres :
//    - NewState : Nouvelle valeur de l'état de l'application.
//
// Retourne :
//    - Rien (void).
// *****************************************************************************
void APP_Gen_UpdateState(APPGEN_STATES NewState)
{
    // Met à jour l'état de l'application avec la nouvelle valeur
    appgenData.state = NewState;
}

// *****************************************************************************
// Fonction :
//    void MENU_DemandeSave(void)
//
// Résumé :
//    Affiche un message de confirmation de sauvegarde.
//
// Description :
//    Cette fonction affiche un message de confirmation de sauvegarde sur un écran LCD.
//    Lors du premier appel, elle efface plusieurs lignes de l'écran. Ensuite, elle
//    affiche "Sauvegarde OK" pendant un certain nombre d'appels avant de réinitialiser
//    l'état de sauvegarde.
//
// Paramètres :
//    - Aucun.
//
// Retourne :
//    - Rien (void).
// *****************************************************************************
void MENU_DemandeSave(void)
{
    static bool compteurPremierPassage = true;  // Indique si c'est le premier passage dans la fonction
    static uint8_t comptAffichageSauvegarde = 0;  // Compteur pour l'affichage du message de sauvegarde
    uint8_t indexClearLine = 0;  // Index pour effacer les lignes de l'écran
 
    if (compteurPremierPassage == true)
    {
        // Efface les premières lignes de l'écran LCD
        for (indexClearLine = 0; indexClearLine < 5; indexClearLine++)
        {
            lcd_ClearLine(indexClearLine);
        }
        compteurPremierPassage = false;  // Marque la fin du premier passage
    }
    else if (comptAffichageSauvegarde < 100)
    {
        // Affiche "Sauvegarde OK" à une position spécifique sur l'écran LCD
        lcd_gotoxy(4, 2);
        printf_lcd("Sauvegarde OK");
        comptAffichageSauvegarde++;  // Incrémente le compteur d'affichage
    }
    else
    {
        // Réinitialise le compteur d'affichage et l'état de sauvegarde
        comptAffichageSauvegarde = 0;
        tcpStatSave = false;
        compteurPremierPassage = true;
    }
}
/*******************************************************************************
 End of File
 */
