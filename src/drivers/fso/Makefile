##############################################################################
#
#    file                 : Makefile
#    created              : Fri Dec 30 10:56:19 GMT 2011
#    copyright            : (C) 2002 Frederic Socquet
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = fso
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp CDriver.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-base_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-base_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
