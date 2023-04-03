add_library(tfm_app_rot_partition_sc STATIC)

target_include_directories(tfm_app_rot_partition_sc
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    PRIVATE
        ${CMAKE_BINARY_DIR}/secure_controller
)
target_include_directories(tfm_partitions
    INTERFACE
        ${CMAKE_BINARY_DIR}/secure_controller
)

target_sources(tfm_app_rot_partition_sc
    PRIVATE
        secure_controller.c
        secure_timer.c
        serial2002.c
)

# The generated sources
target_sources(tfm_app_rot_partition_sc
    PRIVATE
        $<$<BOOL:${TFM_PSA_API}>:
            ${CMAKE_BINARY_DIR}/secure_controller/auto_generated/intermedia_tfm_secure_controller.c>
)
target_sources(tfm_partitions
    INTERFACE
        $<$<BOOL:${TFM_PSA_API}>:
            ${CMAKE_BINARY_DIR}/secure_controller/auto_generated/load_info_tfm_secure_controller.c>
)

target_link_libraries(tfm_app_rot_partition_sc
    PRIVATE
        tfm_secure_api
        psa_interface
        secure_fw
        platform_s
        tfm_psa_rot_partition_crypto
        tfm_sprt
)

target_link_libraries(tfm_partitions
    INTERFACE
        tfm_app_rot_partition_sc
)

target_compile_definitions(tfm_partition_defs
    INTERFACE
    TFM_PARTITION_SECURE_CONTROLLER
)
