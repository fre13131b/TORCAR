/***************************************************************************

    file                 : CDriver.cpp
    created              : 1 dec 2012
    copyright            : (C) 2012 Frederic Socquet
    email                : frederic.socquet@gmail.com
    version              :

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#include "CDriver.h"


const float CDriver::m_MAX_UNSTUCK_ANGLE = 15.0/180.0*PI;		/* [radians] */
const float CDriver::m_UNSTUCK_TIME_LIMIT = 2.0;				/* [s] */


const float CDriver::m_MAX_UNSTUCK_SPEED = 5.0;   /* [m/s] */
const float CDriver::m_MIN_UNSTUCK_DIST = 3.0;    /* [m] */

const float CDriver::m_G = 9.81;                  /* [m/(s*s)] */
const float CDriver::m_FULL_ACCEL_MARGIN = 1.0;   /* [m/s] */

CDriver::CDriver ( int index )
{
 m_index = index ;
}


CDriver::~CDriver ( )
{
}

/* Called for every track change or new race. */
void CDriver::initTrack(tTrack* t, void *carHandle, 
                       void **carParmHandle, tSituation *s)
{
    m_track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void CDriver::newRace(tCarElt* car, tSituation *s)
{
    m_MAX_UNSTUCK_COUNT = int(m_UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    m_stuck = 0;
}

/* Drive during race. */
void CDriver::drive(tCarElt* car, tSituation *s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        car->ctrl.steer = -m_angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.5; // 50% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {

        float steerangle = m_angle - car->_trkPos.toMiddle/car->_trkPos.seg->width;
        car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = 4;
        car->ctrl.brakeCmd = getBrake(car);
        if (car->ctrl.brakeCmd == 0.0) {
            car->ctrl.accelCmd = getAccel(car);
        } else {
            car->ctrl.accelCmd = 0.0;
        }

    }
}

/* Set pitstop commands. */
int CDriver::pitCommand(tCarElt* car, tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void CDriver::endRace(tCarElt *car, tSituation *s)
{
}

/* Update my private data every timestep */
void CDriver::update(tCarElt* car, tSituation *s)
{
    m_trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    m_angle = m_trackangle - car->_yaw;
    NORM_PI_PI(m_angle);
}



/* Check if I'm stuck */
bool CDriver::isStuck(tCarElt* car)
{
    if (fabs(m_angle) > m_MAX_UNSTUCK_ANGLE &&
        car->_speed_x < m_MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > m_MIN_UNSTUCK_DIST) {
        if (m_stuck > m_MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*m_angle < 0.0) {
            return true;
        } else {
            m_stuck++;
            return false;
        }
    } else {
        m_stuck = 0;
        return false;
    }
}


/* Compute the allowed speed on a segment */
float CDriver::getAllowedSpeed(tTrackSeg *segment)
{
    if (segment->type == TR_STR) {
        return FLT_MAX;
    } else {
        float mu = segment->surface->kFriction;
        return sqrt(mu*G*segment->radius);
    }
}
/* Compute the length to the end of the segment */
float CDriver::getDistToSegEnd(tCarElt* car)
{
    if (car->_trkPos.seg->type == TR_STR) {
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    } else {
        return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
    }

}

/* Compute fitting acceleration */
float CDriver::getAccel(tCarElt* car)
{
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (allowedspeed > car->_speed_x + m_FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
    }
}

float CDriver::getBrake(tCarElt* car)
{
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = segptr->surface->kFriction;
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);

	// maxlookaheddist is the distance we have to check (formula with special case v2 = 0).

    float lookaheaddist = getDistToSegEnd(car);

    // lookaheaddist holds the distance we have already checked. First we check if we need to brake for a speed limit on the end of the current segment.

    float allowedspeed = getAllowedSpeed(segptr);
    //if (allowedspeed < car->_speed_x) return 1.0;
    // Compute the allowed speed on the current segment. We check our speed, and if we are too fast we brake, else we continue with the algorithm. Here you can improve the return value, it's a bit tough to brake full (e. g. make it dependent on the speed difference).

    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {

		// The first line moves segptr to the next segment. The guard of the loop checks if we have already checked far enough.

        allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x) {

		// Compute the allowed speed on the *segptr segment. If the allowed speed is smaller than the current speed, we need to investigate further.

            float allowedspeedsqr = allowedspeed*allowedspeed;
            float brakedist = (currentspeedsqr - allowedspeedsqr) / (2.0*mu*G);

		// Here we compute the braking distance according to the formula above.

            if (brakedist > lookaheaddist) {

		// Here the magic check is done. If the required distance to brake is greater than the current distance we need to brake. This works because the simulation timestep is small, so we fail the point in the worst case with ~2.0 meters. So to fix that you can add always 2 meters to the brakedist, or better a speed dependent value.

                return 1.0;
            }
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0.0;
}