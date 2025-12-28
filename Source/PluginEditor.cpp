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
    // arrange 2x2 grid: top-left Width, top-right Smear, bottom-left Pitch, bottom-right Formant
    auto areaW = area.getWidth();
    auto areaH = area.getHeight();
    int cellW = areaW / 2;
    int cellH = areaH / 2;

    juce::Rectangle<int> r00(area.getX(), area.getY(), cellW, cellH);
    juce::Rectangle<int> r01(area.getX() + cellW, area.getY(), cellW, cellH);
    juce::Rectangle<int> r10(area.getX(), area.getY() + cellH, cellW, cellH);
    juce::Rectangle<int> r11(area.getX() + cellW, area.getY() + cellH, cellW, cellH);

    auto knobPad = 8;
    auto k00 = r00.reduced(knobPad);
    auto k01 = r01.reduced(knobPad);
    auto k10 = r10.reduced(knobPad);
    auto k11 = r11.reduced(knobPad);

    stereoWidthSlider.setBounds(k00);
    smearSlider.setBounds(k01);
    pitchSlider.setBounds(k10);
    formantSlider.setBounds(k11);

    int labelH = 18;
    stereoLabel.setBounds(k00.withY(std::max(0, k00.getY() - labelH)).withHeight(labelH));
    smearLabel.setBounds(k01.withY(std::max(0, k01.getY() - labelH)).withHeight(labelH));
    pitchLabel.setBounds(k10.withY(std::max(0, k10.getY() - labelH)).withHeight(labelH));
    formantLabel.setBounds(k11.withY(std::max(0, k11.getY() - labelH)).withHeight(labelH));

    // position Smooth Grains slider centered between bottom knobs
    auto smoothArea = juce::Rectangle<int>((k10.getCentreX() + k11.getCentreX())/2 - 80, k10.getBottom() - 36, 160, 16);
    smoothGrainsSlider.setBounds(smoothArea);
    smoothGrainsLabel.setBounds(smoothArea.withY(smoothArea.getY() - 14).withHeight(12));
}
