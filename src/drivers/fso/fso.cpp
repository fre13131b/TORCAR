/***************************************************************************

    file                 : fso.cpp
    created              : Fri Dec 30 10:56:19 GMT 2011
    copyright            : (C) 2012 Frederic Socquet

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>



#include "CDriver.h"


#define TRACE_FSO printf ("In %s\n",__FUNCTION__);fflush(stdout);
#define BUFSIZE 20
#define NBBOTS 3

static char *g_botname[NBBOTS];
static CDriver *g_CDriver[NBBOTS];


static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newRace(int index, tCarElt* car, tSituation *s); 
static int  pitcmd(int index, tCarElt* car, tSituation *s);
static void drive(int index, tCarElt* car, tSituation *s); 
static void endRace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
fso(tModInfo *modInfo) 
{
    char buffer[BUFSIZE];
    int i;
    TRACE_FSO;

    /* clear all structures */
    memset(modInfo, 0, 10*sizeof(tModInfo));

    for (i = 0; i < NBBOTS; i++) {
        sprintf(buffer, "fso %d", i+1);
        g_botname[i] = strdup(buffer);	  /* store pointer to string */
        modInfo[i].name    = g_botname[i];  /* name of the module (short) */
        modInfo[i].desc    = strdup(buffer); /* description of the module (can be long) */
        modInfo[i].fctInit = InitFuncPt;  /* init function */
        modInfo[i].gfId    = ROB_IDENT;	  /* supported framework version */
        modInfo[i].index   = i;           /* indices from 0 to 9 */
    }
    return 0;
}

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
   tRobotItf *l_itf = (tRobotItf *)pt;

    /* create robot instance for index */
    g_CDriver[index] = new CDriver(index);

    l_itf->rbNewTrack = initTrack; /* Give the robot the track view called */
    l_itf->rbNewRace  = newRace;   /* Start a new race */
    l_itf->rbDrive    = drive;     /* Drive during race */
    l_itf->rbPitCmd   = pitcmd;    /* Pit commands */
    l_itf->rbEndRace  = endRace;   /* End of the current race */
    l_itf->rbShutdown = shutdown;  /* Called before the module is unloaded */
    l_itf->index      = index;     /* Index used if multiple interfaces */
    return 0;


} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    g_CDriver[index]->initTrack(track, carHandle, carParmHandle, s);
}

/* Start a new race. */
static void  
newRace(int index, tCarElt* car, tSituation *s) 
{ 
	g_CDriver[index]->newRace(car, s);
}

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{

	g_CDriver[index]->drive(car, s);

}
/* End of the current race */

/* Pitstop callback */
static int pitcmd(int index, tCarElt* car, tSituation *s)
{
    return g_CDriver[index]->pitCommand(car, s);
}


static void
endRace(int index, tCarElt *car, tSituation *s)
{
	g_CDriver[index]->endRace(car, s);
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
	free(g_botname[index]);
    delete g_CDriver[index];
}

