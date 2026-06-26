# harp-fx/device — the reverb HARP §8.8 audio.fx EFFECT device daemon (harp-fx-deviced).
#
# Reuses harp's generic daemon main + protocol from HARP_ROOT (no harp source copied),
# and drops in the reverb engine seam (reverb_engine.c) — an engine.c REPLACEMENT. Mirrors
# jetson-synth/device/CMakeLists.txt; the difference is that THIS harp tree keeps the audio
# threads INSIDE engine.c (fused with the synth voice pool), so reverb_engine.c carries its
# own effect-simplified copy of host_paced_loop/audio_thread/audio_stop instead of linking a
# separate device/audio_loop.c. engine.c itself is therefore NOT linked (reverb_engine.c
# supplies every symbol the daemon/protocol need from it).
#
# Build (software backend, any POSIX host):
#   cmake -S device -B device/build -DHARP_ROOT=/Users/jak/src/harp
#   cmake --build device/build
#
# Set HARP_ROOT to the harp checkout (feat/audio-in-transport — the §8.8 seam).

cmake_minimum_required(VERSION 3.16)
project(harp_fx_device C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)

if(NOT DEFINED HARP_ROOT)
  set(HARP_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../harp")
endif()
if(NOT EXISTS "${HARP_ROOT}/device/device.h")
  message(FATAL_ERROR "HARP_ROOT (${HARP_ROOT}) is not a harp checkout — pass -DHARP_ROOT=/path/to/harp")
endif()
message(STATUS "harp-fx-deviced: HARP_ROOT=${HARP_ROOT}")

find_package(Threads REQUIRED)

# --- harpcore (protocol primitives), from the harp checkout ---
add_library(fx_harpcore STATIC
  ${HARP_ROOT}/core/src/cbor.c   ${HARP_ROOT}/core/src/sha256.c  ${HARP_ROOT}/core/src/frame.c
  ${HARP_ROOT}/core/src/object.c ${HARP_ROOT}/core/src/store.c   ${HARP_ROOT}/core/src/envelope.c
  ${HARP_ROOT}/core/src/link.c   ${HARP_ROOT}/core/src/audio.c
  ${HARP_ROOT}/core/src/plat_posix.c) # harp_now_ns (§8.3.1 fence bound)
target_include_directories(fx_harpcore PUBLIC ${HARP_ROOT}/core/include)

# --- the reverb device daemon ---
# harp-deviced.c is the generic daemon (transport + audio_rtp_emit + session loop);
# session.c/state.c are the §5/§9/§11 protocol; panel.c the front-panel socket; rtp.c the
# §8.7 RTP packetizer. reverb_engine.c is the engine seam (engine.c REPLACEMENT) — it
# provides g_params/the evq/the engine_* seam AND the audio thread, reading a->fx_in → wet.
add_executable(harp-fx-deviced
  ${HARP_ROOT}/device/harp-deviced.c   # main() + transports + audio_rtp_emit
  ${HARP_ROOT}/device/session.c        # §5/§9/§11 protocol handlers + session loop
  ${HARP_ROOT}/device/state.c          # engine <-> content-addressed objects
  ${HARP_ROOT}/device/panel.c          # POSIX front-panel unix-socket API (Electra rides this)
  ${HARP_ROOT}/host/rtp.c              # §8.7 RTP packetization
  ${HARP_ROOT}/host/sock_io.c          # §8.7/§16 eth control-socket io (recv/send + pre-hello deadline)
  reverb_engine.c                      # the reverb engine seam (engine.c replacement, §8.8 audio.fx)
)
target_include_directories(harp-fx-deviced PRIVATE
  ${HARP_ROOT}/device ${HARP_ROOT}/host ${HARP_ROOT}/core/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../src) # reverb_engine_core.h (the shared DSP)

# NPARAMS: the harp default is 12 (device.h). The linked harp lib sources (session.c/state.c,
# which size the param bank by NPARAMS) AND reverb_engine.c MUST agree — so we leave the
# default 12 everywhere and do NOT override it. (If the reverb ever needs >12 controls, set
# -DNPARAMS=N here AND it propagates to every TU below, like jetson's 17.)

target_link_libraries(harp-fx-deviced fx_harpcore Threads::Threads m)

# --- fx-audio-test: the §8.8 verification host (harness/fx_audio_test.c) ---
# Paces dry audio through harp-fx-deviced as an effect and proves the WET return is bit-exact
# to the reverb oracle (reverb_engine_core.h). Reuses harp's host client + control-socket io.
add_executable(fx-audio-test
  ${CMAKE_CURRENT_SOURCE_DIR}/../harness/fx_audio_test.c
  ${HARP_ROOT}/host/client.c
  ${HARP_ROOT}/host/sock_io.c)
target_include_directories(fx-audio-test PRIVATE
  ${HARP_ROOT}/host ${HARP_ROOT}/core/include ${CMAKE_CURRENT_SOURCE_DIR}/../src)
target_link_libraries(fx-audio-test fx_harpcore Threads::Threads m)

# On Linux (Kria) harp-deviced.c references the USB-FFS gadget transport (device/ffs.c). The
# reverb device is Ethernet/TCP-only, so ffs.c is NOT linked — reverb_engine.c stubs
# harp_ffs_serve / harp_ffs_audio_{in,out}_fd to satisfy the link (see its tail).
