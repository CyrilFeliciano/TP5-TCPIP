#ifndef Generateur_h
#define Generateur_h

// TP3 MenuGen 2016
// C. HUBER  03.02.2016
// Fichier Generateur.h
// Prototypes des fonctions du générateur  de signal
#include "DefMenuGen.h"

//Déclaration de la valeur des define

#define MAX_ECH 100
#define MOITIE_ECH 50
#define QUART_ECH 25
#define DIV_PAR_DEUX 0.5
#define DEUX 2
#define INVERSION -1
#define MOITIE_STEP_AMPLITUDE 10000
#define MAX_STEP_AMPLITUDE 20000
#define MAXSTEP_DAC 65535
#define TRANSFORMATION_FREQUENCE 800000

// Prototype des fonctions 

void  GENSIG_Initialize(S_ParamGen *pParam, S_ParamGen *structInter);
void  GENSIG_UpdatePeriode(S_ParamGen *pParam);
void  GENSIG_UpdateSignal(S_ParamGen *pParam);
void  GENSIG_Execute(void);


#endif