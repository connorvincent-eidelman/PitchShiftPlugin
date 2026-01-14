#include "PluginEditor.h"
#include "PluginProcessor.h"

PitchShiftPluginAudioProcessorEditor::
PitchShiftPluginAudioProcessorEditor(
    PitchShiftPluginAudioProcessor& p)
    : AudioProcessorEditor(&p),
      audioProcessor(p)
{
    setSize(600, 320);

    stereoWidthSlider.setSliderStyle(juce::Slider::Rotary);
    stereoWidthSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    stereoWidthSlider.setPopupDisplayEnabled(true, false, this);
    addAndMakeVisible(stereoWidthSlider);

    stereoWidthAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "stereoWidth", stereoWidthSlider);
    stereoLabel.setText("Width", juce::dontSendNotification);
    stereoLabel.setJustificationType(juce::Justification::centred);
    stereoLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    stereoLabel.setFont(juce::Font(12.0f, juce::Font::bold));
    addAndMakeVisible(stereoLabel);
    
    smearSlider.setSliderStyle(juce::Slider::Rotary);
    smearSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    smearSlider.setPopupDisplayEnabled(true, false, this);
    addAndMakeVisible(smearSlider);

    smearAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "smear", smearSlider);
    smearLabel.setText("Smear", juce::dontSendNotification);
    smearLabel.setJustificationType(juce::Justification::centred);
    smearLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    smearLabel.setFont(juce::Font(12.0f, juce::Font::bold));
    addAndMakeVisible(smearLabel);
    
    // Pitch slider
    pitchSlider.setSliderStyle(juce::Slider::Rotary);
    pitchSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    pitchSlider.setPopupDisplayEnabled(true, false, this);
    addAndMakeVisible(pitchSlider);
    pitchAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "pitch", pitchSlider);
    pitchLabel.setText("Pitch (st)", juce::dontSendNotification);
    pitchLabel.setJustificationType(juce::Justification::centred);
    pitchLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    pitchLabel.setFont(juce::Font(12.0f, juce::Font::bold));
    addAndMakeVisible(pitchLabel);

    // Formant slider
    formantSlider.setSliderStyle(juce::Slider::Rotary);
    formantSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    formantSlider.setPopupDisplayEnabled(true, false, this);
    addAndMakeVisible(formantSlider);
    formantAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "formant", formantSlider);
    formantLabel.setText("Formant (st)", juce::dontSendNotification);
    formantLabel.setJustificationType(juce::Justification::centred);
    formantLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    formantLabel.setFont(juce::Font(12.0f, juce::Font::bold));
    addAndMakeVisible(formantLabel);

    // Overdrive Drive knob
    overdriveDriveSlider.setSliderStyle(juce::Slider::Rotary);
    overdriveDriveSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    overdriveDriveSlider.setPopupDisplayEnabled(true, false, this);
    overdriveDriveSlider.setRange(0.0, 30.0, 0.01);
    addAndMakeVisible(overdriveDriveSlider);
    overdriveDriveAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "overdriveDrive", overdriveDriveSlider);
    overdriveDriveLabel.setText("Drive", juce::dontSendNotification);
    overdriveDriveLabel.setJustificationType(juce::Justification::centred);
    overdriveDriveLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    overdriveDriveLabel.setFont(juce::Font(12.0f, juce::Font::bold));
    addAndMakeVisible(overdriveDriveLabel);

    // Overdrive Mix knob
    overdriveMixSlider.setSliderStyle(juce::Slider::Rotary);
    overdriveMixSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    overdriveMixSlider.setPopupDisplayEnabled(true, false, this);
    overdriveMixSlider.setRange(0.0, 1.0, 0.01);
    addAndMakeVisible(overdriveMixSlider);
    overdriveMixAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "overdriveMix", overdriveMixSlider);
    overdriveMixLabel.setText("Mix", juce::dontSendNotification);
    overdriveMixLabel.setJustificationType(juce::Justification::centred);
    overdriveMixLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    overdriveMixLabel.setFont(juce::Font(12.0f, juce::Font::bold));
    addAndMakeVisible(overdriveMixLabel);

    // Overdrive Stacks slider (horizontal integer slider)
    overdriveStacksSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    overdriveStacksSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 60, 20);
    overdriveStacksSlider.setPopupDisplayEnabled(true, false, this);
    overdriveStacksSlider.setNumDecimalPlacesToDisplay(0);
    overdriveStacksSlider.setRange(0, 32, 1);
    addAndMakeVisible(overdriveStacksSlider);
    overdriveStacksAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "overdriveStacks", overdriveStacksSlider);
    overdriveStacksLabel.setText("Stacks", juce::dontSendNotification);
    overdriveStacksLabel.setJustificationType(juce::Justification::centred);
    overdriveStacksLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    overdriveStacksLabel.setFont(juce::Font(10.0f));
    addAndMakeVisible(overdriveStacksLabel);

    // Smooth Grains micro-slider (between Pitch and Formant)
    smoothGrainsSlider.setSliderStyle(juce::Slider::LinearHorizontal);
    smoothGrainsSlider.setTextBoxStyle(juce::Slider::NoTextBox, false, 0, 0);
    smoothGrainsSlider.setPopupDisplayEnabled(true, false, this);
    addAndMakeVisible(smoothGrainsSlider);
    smoothGrainsAttachment = std::make_unique<juce::AudioProcessorValueTreeState::SliderAttachment>(
        audioProcessor.apvts, "smoothGrains", smoothGrainsSlider);
    smoothGrainsLabel.setText("Smooth", juce::dontSendNotification);
    smoothGrainsLabel.setJustificationType(juce::Justification::centred);
    smoothGrainsLabel.setColour(juce::Label::textColourId, juce::Colours::white);
    smoothGrainsLabel.setFont(juce::Font(10.0f));
    addAndMakeVisible(smoothGrainsLabel);

    // debug HUD
    debugLabel.setColour(juce::Label::textColourId, juce::Colours::lime);
    debugLabel.setJustificationType(juce::Justification::topLeft);
    addAndMakeVisible(debugLabel);
    startTimerHz(30); // 30 Hz HUD refresh
}

PitchShiftPluginAudioProcessorEditor::
~PitchShiftPluginAudioProcessorEditor() {}

void PitchShiftPluginAudioProcessorEditor::paint(
    juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);

    g.setColour(juce::Colours::white);
    g.setFont(15.0f);
    g.drawFittedText(
        "Pitch Shift",
        getLocalBounds(),
        juce::Justification::centred,
        1);
}

void PitchShiftPluginAudioProcessorEditor::resized() {
    auto area = getLocalBounds().reduced(24);
    auto leftArea = area.removeFromLeft(area.getWidth() / 2);
    // arrange 3x2 grid so we can add Drive and Mix knobs
    auto areaW = area.getWidth();
    auto areaH = area.getHeight();
    int cols = 3;
    int rows = 2;
    int cellW = areaW / cols;
    int cellH = (areaH * 7) / 10 / rows; // reserve bottom area for stacks slider

    juce::Rectangle<int> r00(area.getX(), area.getY(), cellW, cellH);
    juce::Rectangle<int> r01(area.getX() + cellW, area.getY(), cellW, cellH);
    juce::Rectangle<int> r02(area.getX() + 2 * cellW, area.getY(), cellW, cellH);
    juce::Rectangle<int> r10(area.getX(), area.getY() + cellH, cellW, cellH);
    juce::Rectangle<int> r11(area.getX() + cellW, area.getY() + cellH, cellW, cellH);
    juce::Rectangle<int> r12(area.getX() + 2 * cellW, area.getY() + cellH, cellW, cellH);

    auto knobPad = 8;
    auto k00 = r00.reduced(knobPad);
    auto k01 = r01.reduced(knobPad);
    auto k02 = r02.reduced(knobPad);
    auto k10 = r10.reduced(knobPad);
    auto k11 = r11.reduced(knobPad);
    auto k12 = r12.reduced(knobPad);

    // top row: Width, Smear, Drive
    stereoWidthSlider.setBounds(k00);
    smearSlider.setBounds(k01);
    overdriveDriveSlider.setBounds(k02);
    // bottom row: Pitch, Formant, Mix
    pitchSlider.setBounds(k10);
    formantSlider.setBounds(k11);
    overdriveMixSlider.setBounds(k12);

    int labelH = 18;
    stereoLabel.setBounds(k00.withY(std::max(0, k00.getY() - labelH)).withHeight(labelH));
    smearLabel.setBounds(k01.withY(std::max(0, k01.getY() - labelH)).withHeight(labelH));
    overdriveDriveLabel.setBounds(k02.withY(std::max(0, k02.getY() - labelH)).withHeight(labelH));
    pitchLabel.setBounds(k10.withY(std::max(0, k10.getY() - labelH)).withHeight(labelH));
    formantLabel.setBounds(k11.withY(std::max(0, k11.getY() - labelH)).withHeight(labelH));
    overdriveMixLabel.setBounds(k12.withY(std::max(0, k12.getY() - labelH)).withHeight(labelH));

    // position Smooth Grains slider between bottom-left and bottom-center knobs
    auto smoothArea = juce::Rectangle<int>((k10.getCentreX() + k11.getCentreX())/2 - 80, k10.getBottom() - 36, 160, 16);
    smoothGrainsSlider.setBounds(smoothArea);
    smoothGrainsLabel.setBounds(smoothArea.withY(smoothArea.getY() - 14).withHeight(12));

    // stacks slider across bottom area
    auto stacksArea = juce::Rectangle<int>(area.getX() + 40, area.getBottom() - 48, areaW - 80, 20);
    overdriveStacksSlider.setBounds(stacksArea);
    overdriveStacksLabel.setBounds(stacksArea.withY(stacksArea.getY() - 14).withHeight(12));

    debugLabel.setBounds(10, 10, getWidth() - 20, 100);
}

void PitchShiftPluginAudioProcessorEditor::timerCallback()
{
    auto text = audioProcessor.consumeDebugText();
    if (!text.isEmpty())
        debugLabel.setText(text, juce::dontSendNotification);
}
