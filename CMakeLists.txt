cmake_minimum_required(VERSION 3.22)

project(ManipulatorClone VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64")

add_subdirectory(JUCE)

juce_add_plugin(Pitch Shift Plugin
    COMPANY_NAME "ConnorVE"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD TRUE
    PLUGIN_MANUFACTURER_CODE YNAM
    PLUGIN_CODE MANP
    FORMATS AU VST3
    PRODUCT_NAME "Pitch Shift"
)

target_sources(Pitch Shift Plugin PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp
)

target_compile_definitions(Pitch Shift Plugin PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
)

target_link_libraries(Pitch Shift Plugin PRIVATE
    juce::juce_audio_utils
)
