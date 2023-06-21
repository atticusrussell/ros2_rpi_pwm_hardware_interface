include(ExternalProject)
ExternalProject_Add(pigpio
  GIT_REPOSITORY    https://github.com/joan2937/pigpio.git
  GIT_TAG           v79
  CMAKE_ARGS        -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> -Wno-dev
)

ExternalProject_Get_Property(pigpio INSTALL_DIR)

set(pigpio_INCLUDE_DIR ${INSTALL_DIR}/include)
set(pigpio_LIBRARIES ${INSTALL_DIR}/lib/libpigpio.so ${INSTALL_DIR}/lib/libpigpiod_if2.so)

mark_as_advanced(pigpio_INCLUDE_DIR pigpio_LIBRARIES)
