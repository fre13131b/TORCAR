/***************************************************************************

    file                 : driver.h
    created              : Thu Dec 20 01:20:19 CET 2002
    copyright            : (C) 2002-2004 Bernhard Wymann
    email                : berniw@bluewin.ch
    version              : $Id: driver.h,v 1.12.2.1 2008/11/09 17:50:19 berniw Exp $

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>
#include <portability.h>


class CDriver {
	public:
		CDriver(int index);
		~CDriver();

		// Callback functions called from TORCS.
		void initTrack  (tTrack* t, void *carHandle, void **carParmHandle, tSituation *s);
		void newRace    (tCarElt* car, tSituation *s);
		void drive      (tCarElt* car, tSituation *s);
		int  pitCommand (tCarElt* car, tSituation *s);
		void endRace    (tCarElt* car, tSituation *s);

	private:
		// Utility functions.
		bool isStuck(tCarElt* car);
        void update(tCarElt* car, tSituation *s);
		float getAllowedSpeed(tTrackSeg *segment);
		float getAccel(tCarElt* car);
		float getDistToSegEnd(tCarElt* car);
        float getBrake(tCarElt* car);


		// Per robot global data.
		int   m_stuck;
		int   m_index;
		float m_speedangle;		// the angle of the speed vector relative to trackangle, > 0.0 points to right.
		float m_trackangle ;
		float m_angle ;
		int   m_MAX_UNSTUCK_COUNT ;


        /* class constants */
        static const float m_MAX_UNSTUCK_ANGLE;
        static const float m_UNSTUCK_TIME_LIMIT;
		static const float m_MAX_UNSTUCK_SPEED;
		static const float m_MIN_UNSTUCK_DIST;

		static const float m_G;
		static const float m_FULL_ACCEL_MARGIN;



		// Track variables.
		tTrack* m_track;
};

#endif // _DRIVER_H_
