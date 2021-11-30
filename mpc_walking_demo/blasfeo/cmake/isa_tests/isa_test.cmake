# This function will prepare a test for a target (specified by the
# variable TEST_TARGET) and execute it. This consists of compiling
# a C file with associated assembly files using the compile flags,
# and then executing the result to see if it runs correctly.
#
# The assembly files contain an exemplar instruction for the ISA,
# so if they fail to run it means the specific ISA is not supported.
#
# The results of the test are stored as the variables
#  CHK_TARGET_BUILD - True if the target built without error
#  CHK_TARGET_RUN   - True if the test ran without error
function(TestForISA)
  # For Intel Haswell, test for if the AVX2 and FMA ISAs work
  set(CMP_CHECK_X64_INTEL_HASWELL
      TEST_AVX2
      TEST_FMA
      )

  # For Intel Sandy Bridge, test for if the AVX ISA works
  set(CMP_CHECK_X64_INTEL_SANDY_BRIDGE
      TEST_AVX
      )

  # For Intel Core, test for if the SSE3 ISA works
  set(CMP_CHECK_X64_INTEL_CORE
      TEST_SSE3
      )

  # For AMD Bulldozer, test for if the AVX and FMA ISAs work
  set(CMP_CHECK_X64_AMD_BULLDOZER
      TEST_AVX
      TEST_FMA
      )

  # For the Cortex A57, test for if the VFPv4 and NEONv2 ISAs work
  set(CMP_CHECK_ARMV8A_ARM_CORTEX_A57
      TEST_VFPv4
      TEST_NEONv2
      )

  # For the Cortex A53, test for if the VFPv4 and NEONv2 ISAs work
  set(CMP_CHECK_ARMV8A_ARM_CORTEX_A53
      TEST_VFPv4
      TEST_NEONv2
      )

  # For the Cortex A15, test for if the VFPv3 and NEON ISAs work
  set(CMP_CHECK_ARMV7A_ARM_CORTEX_A15
      TEST_VFPv3
      TEST_NEON
      )

  # For the Cortex A7, test for if the VFPv3 and NEON ISAs work
  set(CMP_CHECK_ARMV7A_ARM_CORTEX_A7
      TEST_VFPv3
      TEST_NEON
      )

  # For the Cortex A9, test for if the VFPv3 and NEON ISAs work
  set(CMP_CHECK_ARMV7A_ARM_CORTEX_A9
      TEST_VFPv3
      TEST_NEON
      )

  # The main source file to test with
  set(CMP_CHECK_SRCS
      ${PROJECT_SOURCE_DIR}/cmake/isa_tests/isa_test.c
      )

  set(C_DEFS_CHK "")

  # Add the assembly test files and the compile definitions
  foreach(CHECK ${CMP_CHECK_${TEST_TARGET}})
      list( APPEND CMP_CHECK_SRCS ${PROJECT_SOURCE_DIR}/cmake/isa_tests/${CHECK}.S )
      list( APPEND C_DEFS_CHK "-D${CHECK}" )
  endforeach()

  string( REPLACE ";" "" C_DEFS_CHK "${C_DEFS_CHK}" )

  # Populate the flags to use for the testing
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${C_FLAGS_TARGET_${TEST_TARGET}}")
  set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${ASM_FLAGS_TARGET_${TEST_TARGET}}")

  if(${BLASFEO_CROSSCOMPILING})
    set(CHK_TARGET_RUN_${TEST_TARGET} "1")

    # Only tell CMake to compile the files, not link them since we are doing cross-compilation
    if (${CMAKE_VERSION} VERSION_EQUAL "3.6.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.6")
      set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    elseif()
      set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")
    endif()

    try_compile( CHK_TARGET_BUILD_${TEST_TARGET}                   # Variable to save the build result to
                 "${CMAKE_BINARY_DIR}/compilerTest/${TEST_TARGET}" # Directory to compile in
                 SOURCES ${CMP_CHECK_SRCS}                         # Source to compile
                 CMAKE_FLAGS
                   "-DCOMPILE_DEFINITIONS=${C_DEFS_CHK}"
                 OUTPUT_VARIABLE CHK_OUTPUT${TEST_TARGET}
                )
  else()
    try_run( CHK_TARGET_RUN_${TEST_TARGET}                     # Variable to save the run result to
             CHK_TARGET_BUILD_${TEST_TARGET}                   # Variable to save the build result to
             "${CMAKE_BINARY_DIR}/compilerTest/${TEST_TARGET}" # Directory to compile in
             SOURCES ${CMP_CHECK_SRCS}                         # Source to compile
             CMAKE_FLAGS
              "-DCOMPILE_DEFINITIONS=${C_DEFS_CHK}"
             OUTPUT_VARIABLE CHK_OUTPUT${TEST_TARGET}
            )
  endif()

  if(${CHK_TARGET_BUILD_${TEST_TARGET}})
    set(CHK_TARGET_BUILD TRUE PARENT_SCOPE)

    if(${CHK_TARGET_RUN_${TEST_TARGET}} STREQUAL "0")
      set(CHK_TARGET_RUN TRUE PARENT_SCOPE)
    else()
      set(CHK_TARGET_RUN FALSE PARENT_SCOPE)
    endif()

  else()
    set(CHK_TARGET_BUILD FALSE PARENT_SCOPE)
    set(CHK_TARGET_OUTPUT ${CHK_OUTPUT${TEST_TARGET}} PARENT_SCOPE)
  endif()

endfunction()
