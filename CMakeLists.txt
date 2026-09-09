cmake_minimum_required(VERSION 3.16)
project(quadrf-mesh VERSION 0.1.0 LANGUAGES CXX)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/debian/changelog")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/debian/changelog" _changelog_contents)
    string(REGEX MATCH "quadrf-mesh \\(([0-9]+\\.[0-9]+\\.[0-9]+)" _match "${_changelog_contents}")
    if(CMAKE_MATCH_1)
        set(QUADRF_MESH_RELEASE_VERSION "${CMAKE_MATCH_1}")
    endif()
endif()
if(NOT QUADRF_MESH_RELEASE_VERSION)
    set(QUADRF_MESH_RELEASE_VERSION "${PROJECT_VERSION}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(GNUInstallDirs)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Dependencies.cmake")

add_compile_options(-Wall -Wextra)

# ARM64 (aarch64) defaults ON; override with -DPHY_USE_NEON=OFF for scalar evaluation.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
    set(_phy_neon_default ON)
else()
    set(_phy_neon_default OFF)
endif()
option(PHY_USE_NEON "Use ARM NEON for DSP kernels" ${_phy_neon_default})

set(PHY_JTAG_PATH "/usr/bin/quadrf-jtag" CACHE FILEPATH "Path to QuadRF jtag CLI")

add_library(quadrf_air_ipc INTERFACE)
target_include_directories(quadrf_air_ipc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(phy_dsp STATIC
    src/dsp/fft_engine.cpp
    src/dsp/chirp_generator.cpp
    src/dsp/dechirper.cpp
    src/dsp/peak_detector.cpp
    src/dsp/resampler.cpp
)
target_include_directories(phy_dsp PUBLIC include)
target_link_libraries(phy_dsp PUBLIC FFTW3::fftw3f)
if(PHY_USE_NEON)
    # phy_lora's SymbolDemod fold shares the same flag as the DSP atoms.
    target_compile_definitions(phy_dsp PRIVATE PHY_USE_NEON)
endif()

add_library(phy_lora STATIC
    src/lora/coding.cpp
    src/lora/modulator.cpp
    src/lora/symbol_demod.cpp
    src/lora/demodulator.cpp
    src/lora/receiver.cpp
    src/lora/transmitter.cpp
)
target_include_directories(phy_lora PUBLIC include)
target_link_libraries(phy_lora PUBLIC phy_dsp)

# Mesh packet header and hop helpers (integer operations, decoupled from DSP).
add_library(phy_mesh STATIC
    src/mesh/packet.cpp
)
target_include_directories(phy_mesh PUBLIC include)

# StreamAPI framing (0x94 0xC3). No protobuf, no DSP.
add_library(phy_stream STATIC
    src/stream/framer.cpp
)
target_include_directories(phy_stream PUBLIC include)

# PhoneAPI bridge: legacy PTY support.
find_package(OpenSSL REQUIRED)
add_library(phy_bridge STATIC
    src/bridge/pb_wire.cpp
    src/bridge/crypto.cpp
    src/bridge/phone_api.cpp
    src/bridge/session.cpp
)
target_include_directories(phy_bridge PUBLIC include)
target_link_libraries(phy_bridge PUBLIC phy_mesh phy_stream OpenSSL::Crypto)

add_library(phy_radio STATIC
    src/radio/radio_session.cpp
    src/radio/stream_pair.cpp
    src/radio/rf_frontend.cpp
)
target_include_directories(phy_radio PUBLIC include)
target_link_libraries(phy_radio PUBLIC SoapySDR)
target_compile_definitions(phy_radio PUBLIC PHY_JTAG_PATH="${PHY_JTAG_PATH}")

function(phy_add_app target)
    add_executable(${target} apps/${target}.cpp)
    target_link_libraries(${target} PRIVATE phy_radio phy_dsp)
    set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/apps")
endfunction()

phy_add_app(rx_tone_sanity)
phy_add_app(tone_loopback)
phy_add_app(clock_coherence_test)
phy_add_app(digital_loopback_test)
phy_add_app(digital_loopback_probe)
target_sources(digital_loopback_probe PRIVATE src/util/debug_dumper.cpp)

phy_add_app(lora_loopback)
target_link_libraries(lora_loopback PRIVATE phy_lora)
target_sources(lora_loopback PRIVATE src/util/debug_dumper.cpp)

add_executable(chirp_offline tests/chirp_offline.cpp)
target_link_libraries(chirp_offline PRIVATE phy_dsp)
set_target_properties(chirp_offline PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

add_executable(lora_offline_test tests/lora_offline_test.cpp)
target_link_libraries(lora_offline_test PRIVATE phy_lora phy_mesh)
if(PHY_USE_NEON)
    target_compile_definitions(lora_offline_test PRIVATE PHY_USE_NEON)
endif()
set_target_properties(lora_offline_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

add_executable(bridge_offline_test tests/bridge_offline_test.cpp)
target_link_libraries(bridge_offline_test PRIVATE phy_bridge)
set_target_properties(bridge_offline_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

add_executable(air_ipc_offline_test tests/air_ipc_offline_test.cpp)
target_link_libraries(air_ipc_offline_test PRIVATE quadrf_air_ipc phy_mesh)
target_include_directories(air_ipc_offline_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(air_ipc_offline_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

add_executable(meshtastic_air_ipc_consumer_test
    integrations/meshtastic/backend/air_ipc_consumer_smoke.cpp)
target_link_libraries(meshtastic_air_ipc_consumer_test PRIVATE quadrf_air_ipc)
target_include_directories(meshtastic_air_ipc_consumer_test PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")
set_target_properties(meshtastic_air_ipc_consumer_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests")

# Product binary: air-frame IPC ↔ QuadRF LoRa PHY.
add_executable(quadrf-lora-phy apps/quadrf_lora_phy.cpp)
target_link_libraries(quadrf-lora-phy PRIVATE phy_radio phy_lora quadrf_air_ipc phy_bridge)
set_target_properties(quadrf-lora-phy PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/apps")

pkg_check_modules(SDL2 REQUIRED sdl2)
add_executable(quadrf-mesh-monitor apps/quadrf_mesh_monitor.cpp)
target_include_directories(quadrf-mesh-monitor PRIVATE ${SDL2_INCLUDE_DIRS} apps)
target_link_libraries(quadrf-mesh-monitor PRIVATE ${SDL2_LIBRARIES} pthread)
target_compile_definitions(quadrf-mesh-monitor PRIVATE
    QUADRF_MESH_VERSION="v${QUADRF_MESH_RELEASE_VERSION}"
)
set_target_properties(quadrf-mesh-monitor PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/apps")

if(BUILD_TESTING)
    add_test(NAME air-ipc-v1 COMMAND air_ipc_offline_test)
    add_test(NAME meshtastic-air-ipc-consumer COMMAND meshtastic_air_ipc_consumer_test)
    add_test(NAME mesh-bridge COMMAND bridge_offline_test)
    add_test(NAME lora-scalar COMMAND lora_offline_test)
    add_test(NAME chirp-offline COMMAND chirp_offline)
endif()

install(TARGETS quadrf-lora-phy quadrf-mesh-monitor RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES include/quadrf/air_ipc.hpp
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/quadrf")
