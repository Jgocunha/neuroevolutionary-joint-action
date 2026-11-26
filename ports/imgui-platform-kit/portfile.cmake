# To move to a newer imgui-platform-kit, change REF to its commit, set SHA512 to 0 and
# install once: vcpkg's error message prints the real SHA512 to paste here. Update
# "version-date" in vcpkg.json to that commit's date.
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Jgocunha/imgui-platform-kit
    REF 366f9f7e988f92b86ceee80ca0e80aecb55c8f61
    SHA512 47257df8f9ed2a3b72b4910e8efc3f1863cd31be19a5a8b2eb180069fb6ff5b6d08cbd4af5f1808ab725f8fa5317dbd560490ff5acc1d81cc351a689034fdddc
    HEAD_REF main
)

# The library exports no symbols, so it can only be linked statically.
vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

# The kit's CMakeLists stops unless VCPKG_ROOT is set in the environment, and vcpkg
# clears that variable for port builds on Windows.
set(ENV{VCPKG_ROOT} "${VCPKG_ROOT_DIR}")

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}/imgui-platform-kit"
    OPTIONS -DIMGUI_PLATFORM_KIT_BUILD_TESTS=OFF
)
vcpkg_cmake_install()
vcpkg_cmake_config_fixup()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include" "${CURRENT_PACKAGES_DIR}/debug/share")
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
