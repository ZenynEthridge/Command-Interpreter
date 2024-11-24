#include <string>
#include <iostream>

struct PinSetEvent {
    int pinNumber;
    int pinMode;
};

bool operator==(PinSetEvent const& lhs, PinSetEvent const& rhs) {
    return lhs.pinNumber == rhs.pinNumber && lhs.pinMode == rhs.pinMode;
}

struct PinWriteEvent {
    int pinNumber;
    int valueWritten;
};

bool operator==(PinWriteEvent const& lhs, PinWriteEvent const& rhs) {
    return lhs.pinNumber == rhs.pinNumber && lhs.valueWritten == rhs.valueWritten;
}

struct ParseInterpreterResult {
    bool wiringPiSetUp;
    std::vector<PinSetEvent> pinSetEvents;
    std::vector<PinWriteEvent> pinWriteEvents;
};

bool operator==(ParseInterpreterResult const& lhs, ParseInterpreterResult const& rhs) {
    return lhs.wiringPiSetUp == rhs.wiringPiSetUp && lhs.pinSetEvents == rhs.pinSetEvents && lhs.pinWriteEvents == rhs.pinWriteEvents;
}

std::ostream& operator<<(std::ostream& os, ParseInterpreterResult const& object) {
    os << "parseInterpreterResult with:" << std::endl <<  "wiringPiSetUp: " << object.wiringPiSetUp << std::endl;
    if (!object.pinSetEvents.empty()) {
        os << "pinSetEvents: " << std::endl;
        for (auto pinSetEvent : object.pinSetEvents) {
            os << "pin " << pinSetEvent.pinNumber << " set to mode " << pinSetEvent.pinMode << std::endl;
        }
    }

    if (!object.pinWriteEvents.empty()) {
        os << "pinWriteEvents: " << std::endl;
        for (auto pinWriteEvent : object.pinWriteEvents) {
            os << "value " << pinWriteEvent.valueWritten << " written to pin " << pinWriteEvent.pinNumber << std::endl;
        }
    }
    return os;
}


ParseInterpreterResult parseInterpreterOutput(const std::string& output) {
    ParseInterpreterResult parseResult;
    auto index = output.find("[Mock]") + 7;
    parseResult.wiringPiSetUp = (output.substr(index, 21) == "wiringPi GPIO set up!");
    index = output.find("[Mock]", index) + 7;
    while (index <= output.size() && output.substr(index, 8) == "pinMode:") {
        parseResult.pinSetEvents.push_back(PinSetEvent{std::stoi(output.substr(index + 13, 1)), std::stoi(output.substr(index + 27, 1))});
        index = output.find("[Mock]", index) + 7;
    }
    while (index <= output.size() && output.substr(index, 9) == "pwmWrite:") {
        parseResult.pinWriteEvents.push_back(PinWriteEvent{std::stoi(output.substr(index + 14, 1)), std::stoi(output.substr(index + 21, output.find('\n') - index - 1))});
        index = output.find("[Mock]", index) + 7;
    }
    return parseResult;
}
