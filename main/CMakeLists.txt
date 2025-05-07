set(srcs 
    "main.cpp"
    "user.cpp"
    "uncannyEyes.cpp"
)

set(include_dirs 
    "."
    "graphics"
)

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES arduino Adafruit_BusIO Adafruit-GFX-Library Adafruit-ST7735-Library
) 