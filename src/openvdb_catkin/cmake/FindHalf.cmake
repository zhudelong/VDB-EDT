# Copyright (c) 2013 Esteban Tovagliari

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

#  Find Half
#  Find Half headers and libraries.
#
#  HALF_INCLUDE_DIRS - where to find Half includes.
#  HALF_LIBRARIES    - List of libraries when using Half.
#  HALF_FOUND        - True if Half found.

# Look for the header file.
FIND_PATH( HALF_INCLUDE_DIR NAMES OpenEXR/half.h)

# Look for the libraries.
FIND_LIBRARY( HALF_LIBRARY NAMES Half)

# handle the QUIETLY and REQUIRED arguments and set HALF_FOUND to TRUE if
# all listed variables are TRUE
INCLUDE( FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS( HALF DEFAULT_MSG HALF_LIBRARY
                                                    HALF_INCLUDE_DIR
                                                    )
# Copy the results to the output variables.
IF( HALF_FOUND)
    SET( HALF_LIBRARIES ${HALF_LIBRARY})
    SET( HALF_INCLUDE_DIRS ${HALF_INCLUDE_DIR})
ELSE()
    SET( HALF_LIBRARIES)
    SET( HALF_INCLUDE_DIRS)
ENDIF()

MARK_AS_ADVANCED( HALF_LIBRARY HALF_INCLUDE_DIR)