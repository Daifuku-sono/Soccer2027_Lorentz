cmake_minimum_required(VERSION 3.13)


set(PICO_SDK_PATH "C:/Users/Sonog/OneDrive/ドキュメント/pico-sdk")


# Pico SDKの読み込み準備
include(pico_sdk_import.cmake)

# プロジェクト名
project(Soccer2027_Lorentz C CXX ASM)

# SDKの初期化
pico_sdk_init()

# 実行ファイルの設定（プログラムを構成するファイルを並べる）
add_executable(Soccer2027
    main.cpp
    Ball/Ball.cpp
)


target_link_libraries(Soccer2027 
    pico_stdlib 
    hardware_gpio
)


pico_enable_stdio_usb(Soccer2027 1)
pico_enable_stdio_uart(Soccer2027 0)


pico_add_extra_outputs(Soccer2027)
