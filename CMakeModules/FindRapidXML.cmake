# - Try to find RapidXML lib
# Once done this will define
#
#  RapidXML_FOUND - system has eigen lib
#  RapidXML_INCLUDE_DIR - the eigen include directory

# Copyright (c) 2010, Manfred Kroehnert, <mkroehnert _at_ users _dot_ sourceforge _dot_ net>
# Redistribution and use is allowed according to the terms of the BSD license.

IF  (RapidXML_INCLUDE_DIR)
    SET(RapidXML_FOUND TRUE)
ELSE (RapidXML_INCLUDE_DIR)
    FIND_PATH(RapidXML_INCLUDE_DIR NAMES rapidxml.hpp
        HINTS
        ${INCLUDE_INSTALL_DIR}
        ${KDE4_INCLUDE_DIR}
    )

    INCLUDE(FindPackageHandleStandardArgs)

    FIND_PACKAGE_HANDLE_STANDARD_ARGS(RapidXML DEFAULT_MSG RapidXML_INCLUDE_DIR )

    MARK_AS_ADVANCED(RapidXML_INCLUDE_DIR)
ENDIF (RapidXML_INCLUDE_DIR)

