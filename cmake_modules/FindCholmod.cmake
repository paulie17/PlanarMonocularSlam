# Cholmod lib usually requires linking to a blas and lapack library.
# It is up to the user of this module to find a BLAS and link to it.

if (CHOLMOD_INCLUDES AND CHOLMOD_LIBRARIES)
  set(CHOLMOD_FIND_QUIETLY TRUE)
endif ()

find_path(CHOLMOD_INCLUDES
  NAMES
  cholmod.h
  PATHS
  $ENV{CHOLMODDIR}
  ${INCLUDE_INSTALL_DIR}
  PATH_SUFFIXES
  suitesparse
  ufsparse
)

find_library(CHOLMOD_LIBRARIES cholmod PATHS $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})

if(CHOLMOD_LIBRARIES)

  get_filename_component(CHOLMOD_LIBDIR ${CHOLMOD_LIBRARIES} PATH)

  find_library(AMD_LIBRARY amd PATHS ${CHOLMOD_LIBDIR} $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})
  if (AMD_LIBRARY)
    set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARIES} ${AMD_LIBRARY})
  else ()
    set(CHOLMOD_LIBRARIES FALSE)
  endif ()

endif()

if(CHOLMOD_LIBRARIES)

  find_library(COLAMD_LIBRARY colamd PATHS ${CHOLMOD_LIBDIR} $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})
  if (COLAMD_LIBRARY)
    set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARIES} ${COLAMD_LIBRARY})
  else ()
    set(CHOLMOD_LIBRARIES FALSE)
  endif ()

endif()

if(CHOLMOD_LIBRARIES)

  find_library(CAMD_LIBRARY camd PATHS ${CHOLMOD_LIBDIR} $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})
  if (CAMD_LIBRARY)
    set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARIES} ${CAMD_LIBRARY})
  else ()
    set(CHOLMOD_LIBRARIES FALSE)
  endif ()

endif()

if(CHOLMOD_LIBRARIES)

  find_library(CCOLAMD_LIBRARY ccolamd PATHS ${CHOLMOD_LIBDIR} $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})
  if (CCOLAMD_LIBRARY)
    set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARIES} ${CCOLAMD_LIBRARY})
  else ()
    set(CHOLMOD_LIBRARIES FALSE)
  endif ()

endif()

if(CHOLMOD_LIBRARIES)

  find_library(CHOLMOD_METIS_LIBRARY metis PATHS ${CHOLMOD_LIBDIR} $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})
  if (CHOLMOD_METIS_LIBRARY)
    set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARIES} ${CHOLMOD_METIS_LIBRARY})
  endif ()

endif()

if(CHOLMOD_LIBRARIES)

  find_library(SUITESPARSE_LIBRARY SuiteSparse PATHS ${CHOLMOD_LIBDIR} $ENV{CHOLMODDIR} ${LIB_INSTALL_DIR})
  if (SUITESPARSE_LIBRARY)
    set(CHOLMOD_LIBRARIES ${CHOLMOD_LIBRARIES} ${SUITESPARSE_LIBRARY})
  endif ()
  
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Cholmod DEFAULT_MSG
                                  CHOLMOD_INCLUDES CHOLMOD_LIBRARIES)

mark_as_advanced(CHOLMOD_INCLUDES CHOLMOD_LIBRARIES AMD_LIBRARY COLAMD_LIBRARY SUITESPARSE_LIBRARY CAMD_LIBRARY CCOLAMD_LIBRARY CHOLMOD_METIS_LIBRARY)