cmake_minimum_required(VERSION 3.22)

project(Pitch VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64")

add_subdirectory(JUCE)

juce_add_plugin(Pitch
    COMPANY_NAME "Connor"
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS FALSE
    COPY_PLUGIN_AFTER_BUILD TRUE
    PLUGIN_MANUFACTURER_CODE CNOR
    PLUGIN_CODE PTCH
    FORMATS AU VST3
    PRODUCT_NAME "Pitch"
)
juce_generate_juce_header(Pitch)

target_sources(Pitch
    PRIVATE
        Source/PluginProcessor.cpp
        Source/PluginEditor.cpp
        Source/PluginProcessor.h
        Source/PluginEditor.h
)

target_compile_definitions(Pitch
    PRIVATE
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
    JUCE_IGNORE_VST3_MISMATCHED_PARAMETER_ID_WARNING=1
)

target_link_libraries(Pitch
    PRIVATE
        juce::juce_audio_utils
        juce::juce_dsp
        juce::juce_audio_processors
)
