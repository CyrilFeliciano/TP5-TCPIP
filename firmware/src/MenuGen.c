// Tp3  manipulation MenuGen avec PEC12
// C. HUBER  10/02/2015 pour SLO2 2014-2015
// Fichier MenuGen.c
// Gestion du menu  du générateur
// Traitement cyclique à 10 ms

#include <stdint.h>                   
#include <stdbool.h>
#include "MenuGen.h"
#include "Mc32DriverLcd.h"
#include "GesPec12.h"
#include "Generateur.h"
#include "Mc32Debounce.h"
#include "bsp.h"
#include "DefMenuGen.h"
#include "Mc32gestI2cSeeprom.h"
#include "appgen.h"

//Déclaration du tableau contanant les caractères à afficher pour le paramètre "forme"
    static const char MenuShapes[4][21] = {"Sinus", "Triangle", "DentDeScie", "Carre"};

/**
 * Function name :MENU_Initialize
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Initialise les paramètres du programme en lisant les données depuis la mémoire flash et affiche un message d'initialisation sur un écran LCD.
 * 
 * Cette fonction initialise les paramètres du programme en lisant les données depuis la mémoire flash. Elle affiche un message d'initialisation sur 
 * l'écran LCD pour indiquer le démarrage du programme et informer l'utilisateur sur l'origine des données chargées (mémoire flash ou valeurs par défaut).
 * 
 * @param pParam Pointeur vers la structure S_ParamGen pour stocker les paramètres initialisés.
 * 
 * @return  La structure S_ParamGen contenant les paramètres initialisés.
 */

S_ParamGen MENU_Initialize(S_ParamGen *pParam)
{
    //Déclaration de la structure acceillant les valeurs lues dans la flash
    S_ParamGen structInter; 
    //Lecture des données dans la flash et enregistrement dans la structure 
    I2C_ReadSEEPROM(&structInter,MCP79411_EEPROM_BEG,sizeof(S_ParamGen));
    //Affichage de l'initialisation du programme 
    lcd_gotoxy(1,1);
    printf_lcd("TP5 IpGen 23-24");
    lcd_gotoxy(1,2);
    printf_lcd("Feliciano");
    lcd_gotoxy(1,3);
    printf_lcd("Bignens");
    //Si des données sont présentes dans la flash nous affichons "Données en mémoire", sinon "Données par défaut"
    if (structInter.Magic == MAGIC)
    {
        lcd_gotoxy(1,4);
        printf_lcd("Donnees en memoire");
    }
    else
    {
        lcd_gotoxy(1,4);
        printf_lcd("Donnees par defaut");
    } 
    //Retour de la structure contenant les données lues dans la flash
    return structInter;
}

/**
 * Function name :MENU_Execute
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Permet à l'utilisateur de sélectionner, ajuster et enregistrer les valeurs des paramètres via un écran LCD et des boutons.
 * 
 * Cette fonction gère l'interface utilisateur permettant de naviguer entre les différents paramètres, d'ajuster leurs valeurs à l'aide
 * d'un encodeur rotatif et de les enregistrer dans la mémoire flash. Elle utilise des indications visuelles sur l'écran LCD pour guider 
 * l'utilisateur dans le processus de sélection, d'ajustement et d'enregistrement des valeurs des paramètres.
 * 
 * @param pParam Pointeur vers la structure S_ParamGen contenant les paramètres à ajuster et enregistrer.
 * 
 * @return Aucune valeur de retour.
 */

void MENU_Execute(S_ParamGen *pParam, bool local)
{
    //Déclaration des variables et des structures internes à la fonction 
    static int8_t menuCounter = MENUCOUNTERMIN;     //Variable de comptage pour la position du curseur dans le menu
    static int8_t menuMode = SELECTMODE;          //Variable de l'état dans lequel se trouve le menu (selection du mode ou sélection de la valeur)
    static int8_t isSaved = NOSAVEMODE;         //Variable déterminant si l'utilisateur est en train d'enregistrer les valeurs ou non
    static uint8_t firstTime = true;          //Variable servant à savoir si cette fonction est appelée pour le première fois ou non
    static uint8_t lineClearedLcd;          //Variable de comptage pour l'effacement de l'affichage du LCD lors du premier appel de la fonction
    static S_ParamGen NewValue;           //Structure contenant les valeurs intermédiaires de pParam avant que l'utilisateur ait appuyé sur OK
    
    
    if(local == true)
    {
        //Première fois que la fonction est appelée?
        if(firstTime == true)
        {
            //Boucle permettant d'effacer l'affichage du LCD
            for(lineClearedLcd = 1; lineClearedLcd < 5; lineClearedLcd++)
            {
                lcd_ClearLine(lineClearedLcd);
            }
            //Lors du premier appel de la fonction NewValue reprend les valeurs de pParam
            NewValue = *pParam;       
            //Appel de la fonction d'affichage du LCD permettant à l'utilisateur de choisir le paramètre à modifier
            MENU_SelectMode(pParam, MODEFORME);
            //Toggle de la valeur de firstTime afin que le programme ne rentre plus dans ce "if"
            firstTime = false;
        }
        //Appel de la fonction de debounce pour le bouton de sauvegarde S9
        DoDebounce(&DescrS9,S_OK);

        //Est-ce que S9 a été pressé?
        if(DebounceIsPressed(&DescrS9))
        {
            //Nous alluomons la backlight du LCD au cas ou celle-ci serait éteinte (en cas d'inactivité)
            lcd_bl_on(); 
            //Indication d'une activité de l'utilisateur
            Pec12.NoActivity = false;
            Pec12.InactivityDuration = 0;
            //Est-ce que l'utilisateur est déjà dans le mode de sauvegarde des données?
            if(isSaved == NOSAVEMODE)
            {
                //Remise à zéro du flag du bouton S9
                DebounceClearPressed(&DescrS9);
                //Boucle permettant d'effacer l'affichage du LCD
                for(lineClearedLcd = 1; lineClearedLcd < 5; lineClearedLcd++)
                {
                    lcd_ClearLine(lineClearedLcd);
                }
                //Affichage du mode de sauvegarde du système 
                lcd_gotoxy(1,2);
                printf_lcd("    Sauvegarde ?");
                lcd_gotoxy(1,3);
                printf_lcd("    (appui long)");
                //Le programme passe en mode de sauvegarde des données
                isSaved = SAVEMODE;
            }
        }
       //Est-ce que l'utilisateur n'est pas dans le mode de sauvegarde des données?
        if(isSaved == NOSAVEMODE)
        {
            //Est ce l'utilisateur est dans le mode de sélection du paramètre?
            if(menuMode == SELECTMODE)
            {
                //Est ce que l'utilisateur incrémente le paramètre à l'aide du PEC12?
                if(Pec12IsPlus())
                {
                    //Remise à zéro du flag de l'incrémentation du PEC12
                    Pec12ClearPlus();
                    //Est-ce que le curseur est au bout du menu?
                    if(menuCounter < MENUCOUNTERMAX)
                    { 
                        //Incrémentation du paramètre choisi
                        menuCounter ++;
                    }

                    else
                    { 
                        //Rebouclage du paramètre choisi
                        menuCounter = MENUCOUNTERMIN;
                    }
                    //Appel de la fonction d'affichage du LCD permettant à l'utilisateur de choisir le paramètre à modifier
                    MENU_SelectMode(&LocalParamGen, menuCounter);
                }
                //Est ce que l'utilisateur décrémente le paramètre à l'aide du PEC12?
                else if(Pec12IsMinus())
                {
                    //Remise à zéro du flag de la décrémentation du PEC12
                    Pec12ClearMinus();
                    //Est-ce que le curseur est au bout du menu?
                    if(menuCounter > MENUCOUNTERMIN)
                    { 
                        //Décrémentation du paramètre choisi
                        menuCounter --;
                    }

                    else
                    {   //Rebouclage du paramètre choisi
                        menuCounter = MENUCOUNTERMAX;
                    }
                    //Appel de la fonction d'affichage du LCD permettant à l'utilisateur de choisir le paramètre à modifier
                    MENU_SelectMode(&LocalParamGen, menuCounter);
                }

                //Est ce que l'utilisateur a appuyé sur OK?
                if(Pec12IsOK())
                {
                    //Remise à zéro du flag du OK
                    Pec12ClearOK();
                    //Enregistrement dans la structure OldValue des valeurs actuelles de pParam
                    OldValue.Forme = pParam->Forme;
                    OldValue.Frequence = pParam->Frequence;
                    OldValue.Amplitude = pParam->Amplitude;
                    OldValue.Offset = pParam->Offset;
                    //Actualisation de la structure Newvalue en fonction des valeurs choisies par l'utilisateur
                    NewValue = MENU_SelectValue(NewValue, menuCounter);
                    //Changement du mode de fonctionnement du programme pour le mode de sélection de la valeur
                    menuMode = SELECTVALUE;
                }
            }
            //Est ce l'utilisateur est dans le mode de la valeur des paramètres?
            else if(menuMode == SELECTVALUE)
            {
                //Actualisation de la structure Newvalue en fonction des valeurs choisies par l'utilisateur
                NewValue = MENU_SelectValue(NewValue, menuCounter);
                //Est ce que l'utilisateur a appuyé sur OK (validation de la valeur reglée)?
                if(Pec12IsOK())
                {
                    //Remise à zéro du flag du OK
                    Pec12ClearOK();
                    //Actualisation de la structure pParam avec la nouvelle valeur reglée par l'utilisateur
                    switch (menuCounter)
                    {
                        case MODEFORME:
                            pParam->Forme = NewValue.Forme;
                            break;
                        case MODEFREQ:
                            pParam->Frequence = NewValue.Frequence;
                            break;
                        case MODEAMPL:
                            pParam->Amplitude = NewValue.Amplitude;
                            break;
                        case MODEOFFSET:
                            pParam->Offset = NewValue.Offset;
                            break;
                        default:
                            break;
                    }
                    //Appel de la fonction changeant l'amplitude la forme et l'offset du signal
                    GENSIG_UpdateSignal(pParam);
                    //Appel de la fonction changeant la fréquence du signal
                    GENSIG_UpdatePeriode(pParam);
                    //Appel de la fonction d'affichage du LCD permettant à l'utilisateur de choisir le paramètre à modifier
                    MENU_SelectMode(&LocalParamGen, menuCounter);
                    //Changement du mode de fonctionnement du programme pour le mode de sélection du paramètre
                    menuMode = SELECTMODE; 
                }
                //Est ce que l'utilisateur a appuyé sur OK (annulation de la valeur reglée)?
                else if (Pec12IsESC())
                {
                    //Remise à zéro du flag du ESC
                    Pec12ClearESC();
                    //Effacement de la ligne du paramètre que l'utilisateur était en train de regler
                    lcd_ClearLine(menuCounter);
                    //pParam revient à sa OldValue et NewValue reprend la valeur de pParam
                    switch (menuCounter)
                    {
                        case MODEFORME:
                            pParam->Forme = OldValue.Forme;
                            NewValue.Forme = pParam->Forme;
                            break;
                        case MODEFREQ:
                            pParam->Frequence = OldValue.Frequence;
                            NewValue.Frequence = pParam->Frequence;
                            break;
                        case MODEAMPL:
                            pParam->Amplitude = OldValue.Amplitude;
                            NewValue.Amplitude = pParam->Amplitude;
                            break;
                        case MODEOFFSET:
                            pParam->Offset = OldValue.Offset;
                            NewValue.Offset = pParam->Offset;
                            break;
                        default:
                            break;
                    }
                    //Appel de la fonction d'affichage du LCD permettant à l'utilisateur de choisir le paramètre à modifier
                    MENU_SelectMode(&LocalParamGen, menuCounter);
                    //Changement du mode de fonctionnement du programme pour le mode de sélection du paramètre
                    menuMode = SELECTMODE;
                }
            }
        }
        //Si l'utilisateur est déjà dans le mode d'enregistrement des valeurs
        else
        {
            //Appel de la fonction enregistrant les valeurs dans la mémoire flash
            isSaved = MENU_SaveValues(&LocalParamGen, menuCounter);
        }
    }
    else
    {   
        lcd_gotoxy(1,1);
        printf_lcd("#Forme = %s    ", MenuShapes[pParam->Forme]);
        lcd_gotoxy(1,2);
        printf_lcd("#Freq [Hz] = %4d", pParam->Frequence);
        lcd_gotoxy(1,3);
        printf_lcd("#Ampl [mV] = %5d  ", pParam->Amplitude);
        lcd_gotoxy(1,4);
        printf_lcd("#Offset [mV] = %5d", pParam->Offset); 
    }
}

/**
 * Function name :MENU_SelectMode
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Affiche les paramètres actuels sur un écran LCD avec une indication visuelle de la sélection de mode.
 * 
 * Cette fonction affiche les valeurs actuelles des paramètres, notamment la forme, la fréquence, 
 * l'amplitude et l'offset, sur un écran LCD. Elle utilise un astérisque (*) pour indiquer visuellement le mode 
 * sélectionné par l'utilisateur.
 * 
 * @param  pParam Pointeur vers la structure S_ParamGen contenant les paramètres actuels.
 * @param  selectModeMenuCounter Compteur de menu de sélection pour indiquer visuellement le mode sélectionné.
 * 
 * @return  Aucune valeur de retour.
 */

void MENU_SelectMode(S_ParamGen *pParam, int8_t selectModeMenuCounter)
{
        
    //Actualisation de l'affichage du LCD
    lcd_gotoxy(1,1);
    printf_lcd(" Forme = %s     ", MenuShapes[pParam->Forme]);
    lcd_gotoxy(1,2);
    printf_lcd(" Freq [Hz] = %4d", pParam->Frequence);
    lcd_gotoxy(1,3);
    printf_lcd(" Ampl [mV] = %5d  ", pParam->Amplitude);
    lcd_gotoxy(1,4);
    printf_lcd(" Offset [mV] = %5d", pParam->Offset); 
    
    //Affichage de l'astérisque à l'endroit sélectionné par l'utilisateur
    lcd_gotoxy(1,selectModeMenuCounter);
    printf_lcd("*");
}

/**
 * Function name :MENU_SelectValue
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Sélectionne et ajuste les valeurs des paramètres à l'aide d'un encodeur rotatif (PEC12) et les affiche sur un écran LCD.
 * 
 * Cette fonction permet à l'utilisateur de sélectionner et d'ajuster les valeurs des paramètres à l'aide d'un encodeur rotatif. 
 * Elle gère les différents modes de sélection pour chaque paramètre, notamment la forme, la fréquence, l'amplitude et l'offset, 
 * en fonction du compteur de menu de sélection. Les valeurs sélectionnées sont affichées en temps réel sur l'écran LCD.
 * 
 * @param  selectValueNewValue Structure contenant les nouvelles valeurs des paramètres à ajuster.
 * @param  selectValueMenuCounter Compteur de menu de sélection pour naviguer entre les différents paramètres.
 * 
 * @return  La structure selectValueNewValue mise à jour avec les valeurs ajustées.
 */

S_ParamGen MENU_SelectValue(S_ParamGen selectValueNewValue, int8_t selectValueMenuCounter)
{
    //Déclaration du tableau contanant les caractères à afficher pour le paramètre "forme"
   // static const char MenuShapes[4][21] = {"Sinus", "Triangle", "DentDeScie", "Carre"};
    //Swicth case pour inrémenter ou décrémenter la valeur du paramètre sélectionné
    switch (selectValueMenuCounter)
        {
            case MODEFORME:
                //Est ce que l'utilisateur incrémente le paramètre à l'aide du PEC12?
                if(Pec12IsPlus())
                {
                    //Remise à zéro du flag de l'incrémentation du PEC12
                    Pec12ClearPlus();
                    //Est ce que la valeur du paramètre est en dessous de sa valeur max?
                    if(selectValueNewValue.Forme < FORMEMAX)
                    {
                        //Incrémentation de la forme
                        selectValueNewValue.Forme ++;
                    }
                    else
                    {
                        //Rebouclage de la forme
                        selectValueNewValue.Forme = FORMEMIN;
                    }
                    //Effacement de la ligne de la forme sur le LCD
                    lcd_ClearLine(1);
                }
                //Est ce que l'utilisateur décrémente le paramètre à l'aide du PEC12?
                else if(Pec12IsMinus())
                {
                    //Remise à zéro du flag de la décrémentation du PEC12
                    Pec12ClearMinus();
                    //Est ce que la valeur du paramètre est en dessus de sa valeur max?
                    if(selectValueNewValue.Forme > FORMEMIN)
                    {
                        //Décrémentation de la forme
                        selectValueNewValue.Forme --;
                    }
                    else
                    {
                        //Rebouclage de la forme
                        selectValueNewValue.Forme = FORMEMAX;
                    }
                    //Effacement de la ligne de la forme sur le LCD
                    lcd_ClearLine(1);
                } 
                //Affichage de la valeur règlée sur la ligne "forme"
                lcd_gotoxy(1,1);
                printf_lcd("?Forme = %s", MenuShapes[selectValueNewValue.Forme]);
                
            break;

            case MODEFREQ:
                //Est ce que l'utilisateur incrémente le paramètre à l'aide du PEC12?
                if(Pec12IsPlus())
                {
                    //Remise à zéro du flag de l'incrémentation du PEC12
                    Pec12ClearPlus();
                    //Est ce que la valeur du paramètre est en dessous de sa valeur max?
                    if(selectValueNewValue.Frequence < FREQMAX)
                    {
                        //Incrémentation de la fréquence
                        selectValueNewValue.Frequence = selectValueNewValue.Frequence + FREQSTEP;
                    }
                    else
                    {
                        //Rebouclage de la fréquence 
                        selectValueNewValue.Frequence = FREQMIN;
                    }
                    //Effacement de la ligne de la fréquence sur le LCD
                    lcd_ClearLine(2);
                }
                //Est ce que l'utilisateur décrémente le paramètre à l'aide du PEC12?
                else if(Pec12IsMinus())
                {
                    //Remise à zéro du flag de la décrémentation du PEC12
                    Pec12ClearMinus();
                    //Est ce que la valeur du paramètre est en dessus de sa valeur max?
                    if(selectValueNewValue.Frequence > FREQMIN)
                    {
                        //Décrémentation de la fréquence
                        selectValueNewValue.Frequence = selectValueNewValue.Frequence - FREQSTEP;
                    }
                    else
                    {
                        //Rebouclage de la fréquence
                        selectValueNewValue.Frequence = FREQMAX;
                    }
                    //Effacement de la ligne de la fréquence sur le LCD
                    lcd_ClearLine(2);
                }
                //Affichage de la valeur règlée sur la ligne "fréquence"
                lcd_gotoxy(1,2);
                printf_lcd("?Freq [Hz] = %4d", selectValueNewValue.Frequence);
            break;

            case MODEAMPL:
                //Est ce que l'utilisateur incrémente le paramètre à l'aide du PEC12?
                if(Pec12IsPlus())
                {
                    //Remise à zéro du flag de l'incrémentation du PEC12
                    Pec12ClearPlus();
                    //Est ce que la valeur du paramètre est en dessous de sa valeur max?
                    if(selectValueNewValue.Amplitude < AMPLMAX)
                    {
                        //Incrémentation de l'amplitude
                        selectValueNewValue.Amplitude = selectValueNewValue.Amplitude + AMPLSTEP;
                    }
                    else
                    {
                        //Rebouclage de l'amplitude
                        selectValueNewValue.Amplitude = AMPLMIN;
                    }
                    //Effacement de la ligne de l'amplitude sur le LCD
                    lcd_ClearLine(3);
                }
                //Est ce que l'utilisateur décrémente le paramètre à l'aide du PEC12?
                else if(Pec12IsMinus())
                {
                    //Remise à zéro du flag de la décrémentation du PEC12
                    Pec12ClearMinus();
                    //Est ce que la valeur du paramètre est en dessus de sa valeur max?
                    if(selectValueNewValue.Amplitude > AMPLMIN)
                    {
                        //Décrémentation de l'amplitude
                        selectValueNewValue.Amplitude = selectValueNewValue.Amplitude - AMPLSTEP;
                    }
                    else
                    {
                        //Rebouclage de l'amplitude
                        selectValueNewValue.Amplitude = AMPLMAX;
                    }
                    //Effacement de la ligne de l'amplitude sur le LCD
                    lcd_ClearLine(3);
                }
                //Affichage de la valeur règlée sur la ligne "amplitude"
                lcd_gotoxy(1,3);
                printf_lcd("?Ampl [mV] = %5d", selectValueNewValue.Amplitude);
            break;

            case MODEOFFSET:
                
                if(Pec12IsPlus() && (selectValueNewValue.Offset < OFFSETMAX))
                {
                    //Remise à zéro du flag de l'incrémentation du PEC12
                    Pec12ClearPlus();
                    //Incrémentation de l'offset
                    selectValueNewValue.Offset = selectValueNewValue.Offset + OFFSETSTEP;
                    //Effacement de la ligne de l'offset sur le LCD
                    lcd_ClearLine(4);      
                }
                
                else if(Pec12IsMinus() && (selectValueNewValue.Offset > OFFSETMIN))
                {
                    //Remise à zéro du flag de la décrémentation du PEC12
                    Pec12ClearMinus();
                    //Décrémentation de l'offset
                    selectValueNewValue.Offset = selectValueNewValue.Offset - OFFSETSTEP;
                    //Effacement de la ligne de l'offset sur le LCD
                    lcd_ClearLine(4);      
                }
                //Affichage de la valeur règlée sur la ligne "offset"
                lcd_gotoxy(1,4);
                printf_lcd("?Offset [mV] = %5d", selectValueNewValue.Offset );  
            break;
            
            default:
                break;

        }
    //Retour de la structure NewValue
    return(selectValueNewValue);
}

/**
 * Function name :MENU_SaveValues
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Enregistre les valeurs des paramètres dans la mémoire non volatile (NVM) après confirmation de l'utilisateur.
 * 
 * Cette fonction enregistre les valeurs des paramètres dans la NVM lorsque l'utilisateur confirme l'action en appuyant 
 * sur un bouton spécifique pendant une certaine durée. Elle fournit un retour visuel sur un écran (LCD) pour les 
 * opérations de sauvegarde réussies ou annulées.
 * 
 * @param  pParam Pointeur vers la structure S_ParamGen contenant les valeurs des paramètres.
 * @param  saveValuesMenuCounter Compteur pour naviguer dans le menu des valeurs à sauvegarder.
 * 
 * @return État du mode de sauvegarde (SAVEMODE si en mode de sauvegarde, NOSAVEMODE sinon).
 */

int8_t MENU_SaveValues(S_ParamGen *pParam, int8_t saveValuesMenuCounter)
{
    //Déclaration des variables internes à la fonction 
    static uint8_t saveCounter = 0;     //Variable comptant le temps durant lequel l'utilisateur appuie sur S9
    static uint8_t saveMode = false;        //Variable indiquant si l'utilisateur a choisi de sauvegarder les valeurs ou non
    static uint8_t saveDisplayCounter = 0;      //Variable comptant le temps durant lequel la confirmation de la sauvegarde doit être affiché
   
    //Si l'utilisateur n'a pas encore confirmé ou annulé la sauvegarde
    if(saveMode == false)
    {
        //Est-ce que S9 est pressé?
        if(DebounceIsPressed(&DescrS9))
        {
            //Incrémentation du compteur du temps de pression sur S9
            saveCounter++;
            //Détection d'un flanc sur le bouton S9
            if(DebounceGetInput(&DescrS9))
            {
                //Remise à zéro du flag du bouton S9
                DebounceClearPressed(&DescrS9);
                //Est-ce que le bouton S9 a été pressé plus de 500ms?
                if(saveCounter >= SAVECOUNTERMAX)
                {
                    //Enregistrement des valeurs de pParam dans la mémoire flash
                    //NVM_WriteBlock((uint32_t*)pParam, sizeof(S_ParamGen));
                    I2C_WriteSEEPROM((uint32_t*)pParam, MCP79411_EEPROM_BEG, sizeof(S_ParamGen));
                    //Effacement des lignes d'affichage de la sauvegarde sur le LCD
                    lcd_ClearLine(2);
                    lcd_ClearLine(3);
                    //Remise à zéro du compteur de pression sur S9
                    saveCounter = 0;
                    //Changement du mode de la sauvegarde afin d'afficher la confiamation de l'enregistrement des valeurs
                    saveMode = true;
                    //Affichage de la confirmation de la sauvegarde
                    lcd_gotoxy(1,2);
                    printf_lcd("    Sauvegarde OK"); 
                }
                else
                {
                    //Effacement des lignes d'affichage de la sauvegarde sur le LCD
                    lcd_ClearLine(2);
                    lcd_ClearLine(3);
                    //Remise à zéro du compteur de pression sur S9
                    saveCounter = 0;
                    //Changement du mode de la sauvegarde afin d'afficher la l'annulation de l'enregistrement des valeurs
                    saveMode = true;
                    //Affichage de la confirmation de la sauvegarde
                    lcd_gotoxy(1,2);
                    printf_lcd(" Sauvegarde ANNULEE"); 
                }            
            } 
        }
        //Retour de l'état de l'état de la sauvegarde
        return(SAVEMODE);
    }
    else if((saveMode != false) && (saveDisplayCounter < SAVEDISPLAYTIME))
    {
        //Incrémentation du compteur de l'affichage de la sauvagarde
        saveDisplayCounter++;
        //Retour de l'état de l'état de la sauvegarde
        return(SAVEMODE);
    }
    else
    {
        //Effacement de la ligne affichant si la valeur a été sauvegardée ou non
        lcd_ClearLine(2);
        //Remise à zéro du compteur de l'affichage de la sauvagarde
        saveDisplayCounter = 0;
        //Remise à zéro du mode de la sauvegarde
        saveMode = false;
        //Appel de la fonction d'affichage du LCD permettant à l'utilisateur de choisir le paramètre à modifier
        MENU_SelectMode(&LocalParamGen, saveValuesMenuCounter);
        //Retour de l'état de l'état de la sauvegarde
        return(NOSAVEMODE);
    }
}


