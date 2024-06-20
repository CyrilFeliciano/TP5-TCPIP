#ifndef MenuGen_h
#define MenuGen_h

// Tp3  manipulation MenuGen avec PEC12
// C. HUBER  03.02.2016
// Fichier MenuGen.h
// Gestion du menu  du générateur
// Traitement cyclique à 1 ms du Pec12


#include <stdint.h>
#include "DefMenuGen.h"
#include "Mc32Debounce.h"

//Déclaration de la valeur des define

#define SELECTMODE 0
#define SELECTVALUE 1
#define MENUCOUNTERMAX 4
#define MENUCOUNTERMIN 1
#define MODEFORME 1
#define MODEFREQ 2
#define MODEAMPL 3
#define MODEOFFSET 4
#define FORMEMAX 3
#define FORMEMIN 0
#define FREQMAX 2000
#define FREQMIN 20
#define FREQSTEP 20
#define AMPLMAX 10000
#define AMPLMIN 0
#define AMPLSTEP 100
#define OFFSETMAX 5000
#define OFFSETMIN -5000
#define OFFSETSTEP 100
#define SAVEMODE 1
#define NOSAVEMODE 0
#define SAVECOUNTERMAX 50
#define SAVEDISPLAYTIME 200

//Déclaration des strucures

S_SwitchDescriptor DescrS9;
S_ParamGen OldValue;

// Prototype des fonctions 

S_ParamGen MENU_Initialize(S_ParamGen *pParam);
void MENU_Execute(S_ParamGen *pParam, bool local);
void MENU_SelectMode(S_ParamGen *pParam, int8_t selectModeMenuCounter);
S_ParamGen MENU_SelectValue(S_ParamGen selectValueNewValue, int8_t selectValueMenuCounter);
int8_t MENU_SaveValues(S_ParamGen *pParam, int8_t saveValuesMenuCounter);

#endif




  
   







