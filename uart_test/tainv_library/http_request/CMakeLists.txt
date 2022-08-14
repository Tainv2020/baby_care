set(app_src lib_http_request.c)
set(pre_req nvs_flash)
list(APPEND pre_req protocol_examples_common)

idf_component_register(SRCS ${app_src}
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
