# Check ae-numeric sources for forbidden extended integer types.
# Usage: cmake -P check_no_int128.cmake

set(ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
file(GLOB_RECURSE SOURCES
  "${ROOT}/ae-numeric/*.[hH]"
  "${ROOT}/ae-numeric/*.[cC][pP][pP]"
  "${ROOT}/tests/*.[hH]"
  "${ROOT}/tests/*.[cC][pP][pP]"
)

set(FORBIDDEN_PATTERNS
  "__int128"
  "__SIZEOF_INT128__"
)

set(FOUND FALSE)
foreach(src ${SOURCES})
  file(READ "${src}" contents)
  foreach(pat ${FORBIDDEN_PATTERNS})
    string(FIND "${contents}" "${pat}" idx)
    if(NOT idx EQUAL -1)
      message(SEND_ERROR "Forbidden token '${pat}' found in ${src}")
      set(FOUND TRUE)
    endif()
  endforeach()
endforeach()

if(FOUND)
  message(FATAL_ERROR "128-bit integer types/macros are not allowed")
else()
  message(STATUS "No forbidden 128-bit integer tokens found")
endif()
