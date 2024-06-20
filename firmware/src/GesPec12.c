// GesPec12.c  Canevas pour réalisation  
// C. HUBER    09/02/2015

// Fonctions pour la gestion du Pec12
//
//
// Principe : Il est nécessaire d'appeler cycliquement la fonction ScanPec12
//            avec un cycle de 1 ms
//
//  Pour la gestion du Pec12, il y a 9 fonctions à disposition :
//       Pec12IsPlus       true indique un nouveau incrément
//       Pec12IsMinus      true indique un nouveau décrément
//       Pec12IsOK         true indique action OK
//       Pec12IsESC        true indique action ESC
//       Pec12NoActivity   true indique abscence d'activité sur PEC12
//  Fonctions pour quittance des indications
//       Pec12ClearPlus    annule indication d'incrément
//       Pec12ClearMinus   annule indication de décrément
//       Pec12ClearOK      annule indication action OK
//       Pec12ClearESC     annule indication action ESC
//
//
//---------------------------------------------------------------------------


// définitions des types qui seront utilisés dans cette application

#include "GesPec12.h"
#include "Mc32Debounce.h"
#include "Mc32DriverLcd.h"
#include "MenuGen.h"

// Descripteur des sinaux
S_SwitchDescriptor DescrA;
S_SwitchDescriptor DescrB;
S_SwitchDescriptor DescrPB;

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Principe utilisation des fonctions
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//
// ScanPec12 (bool ValA, bool ValB, bool ValPB)
//              Routine effectuant la gestion du Pec12
//              recoit la valeur des signaux et du boutons
//
// s'appuie sur le descripteur global.
// Après l'appel le descripteur est mis à jour

// Comportement du PEC12
// =====================

// Attention 1 cran génère une pulse complète (les 4 combinaisons)
// D'ou traitement uniquement au flanc descendant de B

// Dans le sens horaire CW:
//     __________                      ________________
// B:            |____________________|
//     ___________________                       _________
// A:                     |_____________________|                    

// Dans le sens anti-horaire CCW:
//     ____________________                      _________
// B:                      |____________________|
//     __________                       __________________
// A:            |_____________________|        

/**
 * Function name :ScanPec12
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Analyse l'état des signaux provenant d'un encodeur rotatif (PEC12) et d'un bouton poussoir, gère l'antirebond et détecte les actions de l'utilisateur.
 * 
 * Cette fonction analyse l'état des signaux provenant d'un encodeur rotatif (PEC12) et d'un bouton poussoir, effectue le traitement de l'antirebond et détecte 
 * les actions de l'utilisateur telles que les rotations dans le sens horaire ou anti-horaire, les pressions et les relâchements sur le bouton poussoir. 
 * Elle gère également l'activation et la désactivation de la rétroéclairage de l'écran LCD en fonction de l'activité de l'utilisateur.
 * 
 * @param ValA État du signal A de l'encodeur rotatif.
 * @param ValB État du signal B de l'encodeur rotatif.
 * @param ValPB État du bouton poussoir de l'encodeur rotatif.
 * 
 * @return  Aucune valeur de retour.
 */

void ScanPec12 (bool ValA, bool ValB, bool ValPB)
{    
   // Traitement de l'antirebond sur A, B et PB
   DoDebounce (&DescrA, ValA);
   DoDebounce (&DescrB, ValB);
   DoDebounce (&DescrPB, ValPB);
   //Est-ce qu'il y a un flanc descendant sur le signal B?
   if(DebounceIsPressed(&DescrB))
   {
       //Est-ce qu'il y a une activité de l'utilisateur sur le bouton PEC12?
       if(Pec12.NoActivity == true)
       {
            //Allumage de la backlight du LCD
            lcd_bl_on(); 
            //Changement de l'état de la variable d'activité sur le PEC12
            Pec12.NoActivity = false;
       }
       //Test de l'état du signal A
       if(DebounceGetInput(&DescrA))
       {
           //Incrément sens horaire détecté 
           Pec12.Dec = false;
           Pec12.Inc = true;
       }
       
       else 
       {
           //Incrément sens anti-horaire détecté 
           Pec12.Dec = true;
           Pec12.Inc = false;
       }
       //Remise à zéro de la variable d'inactivité sur le PEC12
       Pec12.InactivityDuration = 0;
   }
   //Est-ce qu'il y a un flanc descendant sur le signal PB?
   if(DebounceIsPressed(&DescrPB))
   {
       //Est-ce qu'il y a une activité de l'utilisateur sur le bouton PEC12?
       if(Pec12.NoActivity == true)
       {
           //Allumage de la backlight du LCD
            lcd_bl_on(); 
            //Changement de l'état de la variable d'activité sur le PEC12
            Pec12.NoActivity = false;
       }
       //Incrémentation de la durée de pression sur le PEC12
       Pec12.PressDuration++;
       //Test de l'état du signal PB
       if(DebounceGetInput(&DescrPB))
       {//Remise à zéro du flag du bouton S9
           
           DebounceClearPressed(&DescrPB);
           //Est ce que le PEC12 a été pressé plus de 500ms?
           if(Pec12.PressDuration >= PRESSDURATIONESC)
           {
               //Action escape detectée
               Pec12.ESC = true;
               Pec12.OK = false;
               //Remise à zéro de la durée de pression sur le PEC12
               Pec12.PressDuration = 0;
           }
           else
           {
               //Action escape detectée
               Pec12.OK = true;
               Pec12.ESC = false;
               //Remise à zéro de la durée de pression sur le PEC12
               Pec12.PressDuration = 0;
           }            
       }
       //Remise à zéro de la variable d'inactivité sur le PEC12
       Pec12.InactivityDuration = 0;
   }
   //Est-ce qu'il y a une inactivité de l'utilisateur durant plus de 5sec?
   if(Pec12.InactivityDuration < INACTIVITYDURATIONMAX)
   {
       //Incrémentation à zéro de la variable d'inactivité sur le PEC12
       Pec12.InactivityDuration++;
   }
   else
   {
       // Inactivité de l'utilisateur detectée
       Pec12.NoActivity = true;
       //Remise à zéro de la variable d'inactivité sur le PEC12
       Pec12.InactivityDuration = 0;
       //Extinction de la backlight du LCD
       lcd_bl_off();
   }
   
   //Clear des flag de la fonction
   DebounceClearPressed(&DescrA);
   DebounceClearPressed(&DescrB);
   DebounceClearReleased(&DescrPB);

 } // ScanPec12

/**
 * Function name :Pec12Init
 * @author Antonio Do Carmo & Cyril Feliciano 
 * @date 21.03.2024
 *
 * @brief Initialise les descripteurs de touches et la structure de gestion de l'encodeur rotatif PEC12.
 * 
 * Cette fonction effectue l'initialisation des descripteurs de touches pour les signaux A, B et PB de 
 * l'encodeur rotatif PEC12, ainsi que l'initialisation de la structure de gestion associée à cet encodeur. 
 * Elle met tous les événements de l'encodeur et les indicateurs de l'activité à zéro, prêts à être mis à jour 
 * par les fonctions de gestion des événements.
 *  
 * @return  Aucune valeur de retour.
 */

void Pec12Init (void)
{
   // Initialisation des descripteurs de touches Pec12
   DebounceInit(&DescrA);
   DebounceInit(&DescrB);
   DebounceInit(&DescrPB);
   
   // Init de la structure PEc12
    Pec12.Inc = false;             // événement incrément  
    Pec12.Dec = false;             // événement décrément 
    Pec12.OK = false;              // événement action OK
    Pec12.ESC = false;             // événement action ESC
    Pec12.NoActivity = false;      // Indication d'activité
    Pec12.PressDuration = false;   // Pour durée pression du P.B.
    Pec12.InactivityDuration = false; // Durée inactivité
  
 } // Pec12Init





//       Pec12IsPlus       true indique un nouveau incrément
bool Pec12IsPlus    (void) {
   return (Pec12.Inc);
}

//       Pec12IsMinus      true indique un nouveau décrément
bool Pec12IsMinus    (void) {
   return (Pec12.Dec);
}

//       Pec12IsOK         true indique action OK
bool Pec12IsOK    (void) {
   return (Pec12.OK);
}

//       Pec12IsESC        true indique action ESC
bool Pec12IsESC    (void) {
   return (Pec12.ESC);
}

//       Pec12NoActivity   true indique abscence d'activité sur PEC12
bool Pec12NoActivity    (void) {
   return (Pec12.NoActivity);
}

//  Fonctions pour quittance des indications
//       Pec12ClearPlus    annule indication d'incrément
void Pec12ClearPlus   (void) {
   Pec12.Inc = 0;
}

//       Pec12ClearMinus   annule indication de décrément
void Pec12ClearMinus   (void) {
   Pec12.Dec = 0;
}

//       Pec12ClearOK      annule indication action OK
void Pec12ClearOK   (void) {
   Pec12.OK = 0;
}

//       Pec12ClearESC     annule indication action ESC
void Pec12ClearESC   (void) {
   Pec12.ESC = 0;
}

void Pec12ClearInactivity   (void) {
  Pec12.NoActivity = 0;
  Pec12.InactivityDuration = 0;
}


