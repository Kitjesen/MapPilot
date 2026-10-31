# One GTSAM release for the online and saved-map optimizers.
file(READ "${CMAKE_CURRENT_LIST_DIR}/../locks/gtsam.json" _lingtu_gtsam_lock)
string(JSON LINGTU_GTSAM_VERSION GET "${_lingtu_gtsam_lock}" version)
find_package(GTSAM ${LINGTU_GTSAM_VERSION} EXACT CONFIG REQUIRED)
