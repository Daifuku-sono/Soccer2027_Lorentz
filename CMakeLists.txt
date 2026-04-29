cmake_minimum_required(VERSION 3.13)

# Pico SDKの読み込み準備
include(pico_sdk_import.cmake)

# プロジェクト名と使用言語の設定
project(Soccer2027_Lorentz C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pico SDKの初期化
pico_sdk_init()

# 実行ファイル（プログラム）の定義
# ※もしメインのファイル名がmain.cppではない場合は書き換えてください
add_executable(Soccer2027
    main.cpp
    Ball/Ball.cpp
)

# 使用するPicoのライブラリをリンク
target_link_libraries(Soccer2027 
    pico_stdlib 
    hardware_gpio
)

# printfでfloat（%f）を使えるようにする設定
pico_set_printf_float(Soccer2027 1)

# シリアル通信（printf）をUSB経由で出力する設定
pico_enable_stdio_usb(Soccer2027 1)
pico_enable_stdio_uart(Soccer2027 0)

# Picoに書き込むための .uf2 ファイルを生成する
pico_add_extra_outputs(Soccer2027)
