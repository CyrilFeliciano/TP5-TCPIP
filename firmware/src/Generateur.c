// Canevas manipulation GenSig avec menu

// C. HUBER  09/02/2015

// Fichier Generateur.C

// Gestion  du générateur
 
// Prévu pour signal de 40 echantillons
 
// Migration sur PIC32 30.04.2014 C. Huber
 
 
#include "Generateur.h"
#include "DefMenuGen.h"
#include "Mc32gestSpiDac.h"
#include "app.h"
#include "Mc32DriverLcd.h"
#include <stdint.h>
#include <stdbool.h>
 

static uint16_t formeSignal [MAX_ECH];
 
/**
 * Function name :GENSIG_Initialize
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Initialise les paramètres du générateur de signal.
 * 
 * Cette fonction initialise les paramètres du générateur de 
 * signal en fonction des valeurs lues dans la mémoire flash. Si aucune valeur n'est trouvée en mémoire,
 * elle initialise les paramètres avec des valeurs par défaut.
 * 
 * @param pParam Pointeur vers la structure contenant les paramètres du générateur de signal.
 * @param structInterGENSIG Pointeur vers la structure contenant les valeurs lues dans la mémoire flash.
 * 
 * @return Aucune valeur de retour.
 */

void  GENSIG_Initialize(S_ParamGen *pParam, S_ParamGen *structInterGENSIG)
{
    //Est-ce que la valeur enregistrée dans la flash correspond au champ magic?
    if (structInterGENSIG->Magic == MAGIC)
    {
        //Enregistrement des valeurs de la flash dans pParam
        *pParam = *structInterGENSIG;
    }
    else
    {
        //Initialisation des paramètres de base de la structure pParam
        pParam->Forme = SignalSinus;
        pParam->Frequence = 100;
        pParam->Amplitude = 10000;
        pParam->Offset = 0;
        pParam->Magic = MAGIC;  
    } 
}

/**
 * Function name :GENSIG_UpdatePeriode
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Met à jour la période du signal en fonction de la fréquence.
 * 
 * Cette fonction met à jour la période du signal en fonction de la fréquence spécifiée dans les paramètres du générateur de signal.
 * 
 * @param pParam Pointeur vers la structure contenant les paramètres du générateur de signal.
 * 
 * @return Aucune valeur de retour.
 */

void  GENSIG_UpdatePeriode(S_ParamGen *pParam)
{
    //Déclaration de la variable stockant la conversion du nombre de pas de la fréquence 
    static uint16_t nbrPasFrequence = false;
    //Conversion de la valeur de la fréquence pour le timer 3
    nbrPasFrequence = ((TRANSFORMATION_FREQUENCE/pParam->Frequence)-1)+0.5;
    //Changement de la valeur de recharge du timer 3
    PLIB_TMR_Period16BitSet(TMR_ID_3, nbrPasFrequence);

}
 
/**
 * Function name :GENSIG_UpdateSignal
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Met à jour la forme du signal en fonction des paramètres spécifiés.
 * 
 * Cette fonction met à jour la forme du signal en fonction des paramètres spécifiés tels que la forme, l'amplitude et l'offset.
 * 
 * @param pParam Pointeur vers la structure contenant les paramètres du générateur de signal.
 * 
 * @return Aucune valeur de retour.
 */

void GENSIG_UpdateSignal(S_ParamGen *pParam)
{
    //Déclaration des variables et du tableau internes à la fonction 
    uint8_t indexEchantillons;
    int16_t Offset;
    uint16_t amplitude_en_mV;
    const uint16_t tbSinus[100] = {
    53, 56, 59, 62, 65, 68, 71, 74, 77, 79, 82, 84, 86, 89, 90, 92, 94, 95, 96, 98, 
    98, 99, 100, 100, 100, 100, 100, 99, 98, 98, 96, 95, 94, 92, 90, 89, 86, 
    84, 82, 79, 77, 74, 71, 68, 65, 62, 59, 56, 53, 50, 47, 44, 41, 38, 35, 
    32, 29, 26, 23, 21, 18, 16, 14, 11, 10, 8, 6, 5, 4, 2, 2, 1, 0, 0, 0, 0, 
    1, 1, 2, 2, 3, 4, 6, 8, 10, 11, 14, 16, 18, 21, 23, 26, 29, 32, 35, 38, 
    41, 44, 47, 50};
    //Multiplication de l'offset par -1 afin d'inverser l'effet de l'offset
    Offset = pParam->Offset * INVERSION;
    //Conversion de l'amplitude saisie par l'utilisateur en mV
    amplitude_en_mV = (pParam ->Amplitude / MAX_ECH) * DEUX ;
    
    //Boucle permettant de parcourir toutes les cases du tableau formeSignal
    for(indexEchantillons = 0; indexEchantillons < MAX_ECH; indexEchantillons++)
    {
        //Action selon la forme choisie
        switch (pParam->Forme) 
        {
            case SignalSinus:
                //Calcul des 100 échantillons du sinus
                formeSignal[indexEchantillons] = (tbSinus[indexEchantillons] - MOITIE_ECH) * amplitude_en_mV + MOITIE_STEP_AMPLITUDE + Offset;
                break;

            case SignalCarre:
                if (indexEchantillons < MAX_ECH * DIV_PAR_DEUX)
                {
                    //Calcul des 50 échantillons du signal carré négatif
                    formeSignal[indexEchantillons] = MOITIE_STEP_AMPLITUDE - ((amplitude_en_mV * MAX_ECH)* DIV_PAR_DEUX - Offset);
                }
                else
                {
                    //Calcul des 50 échantillons du signal carré positif
                    formeSignal[indexEchantillons] = MOITIE_STEP_AMPLITUDE + ((amplitude_en_mV * MAX_ECH)* DIV_PAR_DEUX + Offset);
                }
                break;

            case SignalDentDeScie:
                //Calcul des 100 échantillons du signal en dent de scie
                formeSignal[indexEchantillons] = Offset + (indexEchantillons - MOITIE_ECH) * amplitude_en_mV + MOITIE_STEP_AMPLITUDE;
                break;

            case SignalTriangle:
                if((MAX_ECH * DIV_PAR_DEUX) < indexEchantillons)
                {
                    //Calcul des 50 échantillons du signal triangle positif
                    formeSignal[indexEchantillons] = amplitude_en_mV * (MAX_ECH - DEUX * (indexEchantillons - QUART_ECH)) + Offset + MOITIE_STEP_AMPLITUDE; 
                }
                else
                {
                    //Calcul des 50 échantillons du signal triangle négatif
                    formeSignal[indexEchantillons] = amplitude_en_mV * (DEUX * (indexEchantillons - QUART_ECH)) + Offset + MOITIE_STEP_AMPLITUDE;
                }
                break;
                
            default:
                break;
        }
        //Est-ce que la valeur du signal dépasse l'amplitude maximum négative?
        if ((formeSignal[indexEchantillons] > MAX_STEP_AMPLITUDE) && (Offset < false)) 
        {
            //Valeur du signal bloquée à zéro
            formeSignal[indexEchantillons] = false;
        }
        //Est-ce que la valeur du signal dépasse l'amplitude maximum positive?
        else if ((formeSignal[indexEchantillons] > MAX_STEP_AMPLITUDE) && (Offset > false))
        {
            //Valeur du signal bloquée à sa valeur maximum (20000)
            formeSignal[indexEchantillons] = MAX_STEP_AMPLITUDE;
        }
        //Conversion des valeurs présente dans les cases du tableau pour le DAC
        formeSignal[indexEchantillons] = (MAXSTEP_DAC * formeSignal [indexEchantillons]/ MAX_STEP_AMPLITUDE);
    }
}
 
 /**
 * Function name :GENSIG_Execute
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Exécute le générateur de signal en envoyant les échantillons au DAC.
 * 
 * Cette fonction exécute le générateur de signal en envoyant les échantillons stockés dans le tableau formeSignal au DAC via le bus SPI.
 * 
 * @return Aucune valeur de retour.
 */

void  GENSIG_Execute(void)
{
   //Déclaration de la variable permettant de connaitre l'échantillon en cours de traitement
   static uint16_t EchNb = 0;
   //Ecriture de l'échantillon dans le DAC via le bus SPI
   SPI_WriteToDac(0, formeSignal[EchNb]); 
   //Incrémentation de la variable permettant de connaitre l'échantillon en cours de traitement
   EchNb++;
   //Remise à zéro de EchNb quand celui-ci a atteint sa valeur maximale
   EchNb = EchNb % MAX_ECH;
}
