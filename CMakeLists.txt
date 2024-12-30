FILE(GLOB_RECURSE sources ./*.*)
idf_component_register(
    SRCS ${sources} 
    INCLUDE_DIRS .
    PRIV_REQUIRES NMEA2000
)