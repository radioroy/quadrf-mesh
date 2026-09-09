find_package(SoapySDR REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3F REQUIRED fftw3f)

add_library(FFTW3::fftw3f INTERFACE IMPORTED)
target_include_directories(FFTW3::fftw3f INTERFACE ${FFTW3F_INCLUDE_DIRS})
target_link_libraries(FFTW3::fftw3f INTERFACE ${FFTW3F_LIBRARIES})
