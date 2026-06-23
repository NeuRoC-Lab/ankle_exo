#pragma once

#include <Arduino.h>


class SerialMotorControl {
public:
    SerialMotorControl(Stream& serial, MotorCmd& cmd)
        : m_serial(serial),
          m_cmd(cmd)
    {}

    void update();

private:
    Stream& m_serial;
    MotorCmd& m_cmd;
    String m_line;

    void handleLine(String line);
    void applyParam(const String& name, float value);
    void printCurrentCommand();
};

void SerialMotorControl::update() {
    while (m_serial.available()) {
        char c = m_serial.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            handleLine(m_line);
            m_line = "";
        } else {
            m_line += c;
        }
    }
}

void SerialMotorControl::handleLine(String line) {
    line.trim();
    line.toLowerCase();

    if (!line.startsWith("set ")) {
        return;
    }

    String tok[12];
    int n = 0;

    while (line.length() && n < 12) {
        int sp = line.indexOf(' ');

        if (sp < 0) {
            tok[n++] = line;
            break;
        }

        tok[n++] = line.substring(0, sp);
        line = line.substring(sp + 1);
        line.trim();
    }

    int count = n - 1;

    if (count <= 0 || count % 2 != 0) {
        m_serial.println("usage: set pos vel kp kd trq 1.0 0.0 10.0 0.5 0.0");
        return;
    }

    int num_params = count / 2;
    int param_start = 1;
    int value_start = 1 + num_params;

    for (int i = 0; i < num_params; i++) {
        applyParam(
            tok[param_start + i],
            tok[value_start + i].toFloat()
        );
    }

    printCurrentCommand();
}

void SerialMotorControl::applyParam(const String& name, float value) {
    if (name == "pos") {
        m_cmd.position = value;
    } else if (name == "vel") {
        m_cmd.velocity = value;
    } else if (name == "trq") {
        m_cmd.torque = value;
    } else if (name == "kp") {
        m_cmd.kp = value;
    } else if (name == "kd") {
        m_cmd.kd = value;
    } else if (name == "id") {
        m_cmd.can_id = (uint32_t)value;
    } else {
        m_serial.print("bad param: ");
        m_serial.println(name);
    }
}

void SerialMotorControl::printCurrentCommand() {
    m_serial.print("cmd id=");
    m_serial.print(m_cmd.can_id);

    m_serial.print(" pos=");
    m_serial.print(m_cmd.position, 4);

    m_serial.print(" vel=");
    m_serial.print(m_cmd.velocity, 4);

    m_serial.print(" kp=");
    m_serial.print(m_cmd.kp, 4);

    m_serial.print(" kd=");
    m_serial.print(m_cmd.kd, 4);

    m_serial.print(" trq=");
    m_serial.println(m_cmd.torque, 4);
}